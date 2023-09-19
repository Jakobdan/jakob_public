#General car code/ setup.

import board
import busio
import digitalio
import storage
import adafruit_sdcard
import time
from adafruit_mpl3115a2 import MPL3115A2
import neopixel
import adafruit_ssd1306
import displayio
import math
import analogio
import adafruit_lsm303_accel
import adafruit_tcs34725
import adafruit_lsm303dlh_mag
from adafruit_esp32spi import adafruit_esp32spi
import adafruit_esp32spi.adafruit_esp32spi_socket as socket

import adafruit_requests as requests
import supervisor

from adafruit_io.adafruit_io import IO_HTTP, AdafruitIO_RequestError
import rotaryio
import pwmio

from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService
from adafruit_airlift.esp32 import ESP32
import gc
gc.collect()
#Bluetooth

esp32 = ESP32()
adapter = esp32.start_bluetooth()
ble = BLERadio(adapter)
uart = UARTService()
advertisement = ProvideServicesAdvertisement(uart)



#GLOBAL breytur
global last_mode_pressed, last_estop_pressed, last_plus_pressed, last_minus_pressed
last_mode_pressed = 0
last_estop_pressed = 0
last_plus_pressed = 0
last_minus_pressed = 0

global wheel_diameter, obs_stop_dist, obs_go_dist, pwm_ramp_speed
wheel_diameter = 4.4 #cm
obs_stop_dist = 30 #cm
obs_go_dist = 50 #cm
pwm_ramp_speed = 15 #cm/s

global last_time, last_distance_cm_left, last_distance_cm_right
last_time = time.monotonic()
last_distance_cm_left = 0
last_distance_cm_right = 0

global desired_speed_cm_left, previous_error_left, errors_list_left, integral_left
desired_speed_cm_left = 10 
previous_error_left = 0
errors_list_left = []
integral_left = 0

global desired_speed_cm_right, previous_error_right, errors_list_right, integral_right
desired_speed_cm_right = 10  
previous_error_right = 0
errors_list_right = []
integral_right = 0

global desired_radius
desired_radius = 100 #cm
#FASTAR
global DEBOUNCE_TIME,INCREMENTS_PER_REV
DEBOUNCE_TIME = 0.1  # 50ms
INCREMENTS_PER_REV = 700 #700inc per rev
# Initialize PID constants
Kp = 0.005
Ki = 0.0005
Kd = 0.0005

def estop_switch(switch, mode): #estop mode self contained function
    global last_mode_pressed
    if not switch.value:
        estop_menu = True
        while estop_menu:
            try:
                motor_speed(0,0,0,0)
            except:
                print('Error with stopping motors')
            estop_mode = 0 
            estop_mode_menu_nr = 1
            while estop_mode == 0:
                display.fill(0)
                display.text(f'Estop in mode {mode}',0,0,1)
                display.text('Restart?',0,20,1)
                display.show()
                mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
                if mode_is_pressed:
                    display.fill(0)

                    display.text('RESTARTING...',60,20,1)
                    display.show()
                    time.sleep(0.5)
                    display.fill(0)
                    display.show()
                    supervisor.reload()
                estop_mode = menu_cycle(estop_mode, estop_mode_menu_nr)
            while estop_mode == 1:
                display.fill(0)
                display.text(f'Estop in mode {mode}',0,0,1)
                display.text('Back to last mode?',0,20,1)
                display.show()
                mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
                if mode_is_pressed:
                    estop_menu = False
                    estop_mode = None
                    return None
                estop_mode = menu_cycle(estop_mode, estop_mode_menu_nr)

def estop_crash(mode): #estop crash self contained function
    try:
        motor_speed(0,0,0,0)
    except:
        print('Error with stopping motors')
    try:
        
        display.fill(0)
        display.text(f'ERROR: {mode}',0,0,1)
        display.text('Estop: Crash',0,20,1)
        display.show()
    except:
        print('Display error, restarting...')
        time.sleep(1)
        supervisor.reload()
    global last_mode_pressed
    while True:
        mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
        if mode_is_pressed: #resets the process once the reset/mode value has been input (only mode button is set up right now
            display.fill(0)
            display.text('RESTARTING',60,20,1)
            display.show()
            time.sleep(0.5)
            display.fill(0)
            display.show()
            supervisor.reload()

def change_variable_over_bluetooth(current_value, variable_name, uart):
    """Allow a variable to be changed over Bluetooth.

    Args:
    - current_value: The current value of the variable.
    - variable_name: A string representing the name of the variable.
    - uart: The UARTService instance for Bluetooth communication.
    
    Returns:
    - The new value for the variable if it was changed. Otherwise, returns the original value.
    """
    while True:
        ble.start_advertising(advertisement)
        print("Waiting for Bluetooth connection...")
        
        while not ble.connected:
            pass

        print("Bluetooth connected!")
        break

    # Print current value to Bluetooth device
    print('start')
    uart.write(f"{variable_name} is now {current_value}\n".encode('utf-8'))
    
    # Wait for a command from the Bluetooth device
    buffer = bytearray()
    while True:
        time.sleep(0.1)
        data = uart.read(uart.in_waiting or 1)
        
        if data:
            buffer.extend(data)
            if b'\n' in buffer:  # Check for newline indicating end of command
            
                command = buffer.decode('utf-8').strip()
                print(command)
                
                # Check if command matches expected format
                if command.startswith(variable_name + "="):
                    try:
                        # Extract new value and convert to appropriate type
                        new_value = type(current_value)(command.split('=')[1])
                        uart.write(f"Setting {variable_name} to {new_value} \n".encode('utf-8'))
                        return new_value
                    except ValueError:
                        uart.write(f"Invalid value for {variable_name}\n".encode('utf-8'))
                else:
                    uart.write(f"Unknown command. Expected format: {variable_name}=VALUE\n".encode('utf-8'))
                buffer = bytearray()  # Clear buffer for next command

    return current_value  # If no valid command received, return original value

def IR_INIT_test():
    print("running code")
    global IR_sensor
    IR_sensor = analogio.AnalogIn(board.A0)
    an_val = IR_sensor.value
    distance_start = 4.1061*pow(10,-8)*pow(an_val,2)-0.0044*an_val+133.4935
    display.fill(0)
    display.text('Running IR checker',0,0,1)
    display.show()
    IR_Test_Time_start = time.monotonic()
    error_code = 101
    while time.monotonic()-IR_Test_Time_start < 0.2:
        distance = 4.1061*pow(10,-8)*pow(an_val,2)-0.0044*an_val+133.4935
        if distance_start - distance > 5 or distance_start - distance < -5:
            print('IR error')
            crash_code = 102
            estop_crash(crash_code)
        estop_switch(Estop_pin, error_code)
        time.sleep(0.1)
    print('IR Success!')

def Component_Test():
    try:
        #i2c
        global i2c, display
        i2c = board.I2C()
        # Display
        displayio.release_displays()
        display_width = 128
        display_height = 32
        display = adafruit_ssd1306.SSD1306_I2C(display_width, display_height, i2c, addr=0x3c)
    except:
        crash_code = 105
        estop_crash(crash_code)
    try:
        #Initialize Wifi
        esp32_cs = DigitalInOut(board.ESP_CS)
        esp32_ready = DigitalInOut(board.ESP_BUSY)
        esp32_reset = DigitalInOut(board.ESP_RESET)

        spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
        esp = adafruit_esp32spi.ESP_SPIcontrol(spi, esp32_cs, esp32_ready, esp32_reset)
        socket.set_interface(esp)
        requests.set_socket(socket, esp)
    except:
        #Initialize Wifi
        crash_code = 100
        print('Wifi failed')
    try:
        #Initialize buttons and potensiometers
        global pot_pin1, pot_pin2
        pot_pin1 = analogio.AnalogIn(board.A1)
        pot_pin2 = analogio.AnalogIn(board.A2)

        global Mode_pin, Estop_pin, plus_pin, minus_pin
        Mode_pin = digitalio.DigitalInOut(board.D7)
        Mode_pin.direction = digitalio.Direction.INPUT
        Mode_pin.pull = digitalio.Pull.UP

        Estop_pin = digitalio.DigitalInOut(board.D8)
        Estop_pin.direction = digitalio.Direction.INPUT
        Estop_pin.pull = digitalio.Pull.UP

        plus_pin = digitalio.DigitalInOut(board.D9)
        plus_pin.direction = digitalio.Direction.INPUT
        plus_pin.pull = digitalio.Pull.UP

        minus_pin = digitalio.DigitalInOut(board.D12)
        minus_pin.direction = digitalio.Direction.INPUT
        minus_pin.pull = digitalio.Pull.UP
    except:
        #Initialize buttons and potensiometers
        crash_code = 1011
        estop_crash(crash_code)

    # IR sensor
    IR_INIT_test()

    # Acel sensor
    try:
        global sensor_acc, sensor_mag
        sensor_acc = adafruit_lsm303_accel.LSM303_Accel(i2c)
        sensor_mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)
    except:
        crash_code = 103
        estop_crash(crash_code)

    # Color sensor
    try:
        global color_sensor
        color_sensor = adafruit_tcs34725.TCS34725(i2c)
    except:
        crash_code = 104
        estop_crash(crash_code)

    #SD card, not required, but can be used
    
    try:
        spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
        cs = digitalio.DigitalInOut(board.D10)
        cs.direction = digitalio.Direction.OUTPUT
        cs.value = True
        sdcard = adafruit_sdcard.SDCard(spi, cs)
        vfs = storage.VfsFat(sdcard)
        storage.mount(vfs, "/sd")
        print("SD card mounted successfully!") #we can implement an SD icon, should we need/want to
    except Exception as e:
        print("Failed to mount SD card:", e)

    #Motor and/or altimeter component test (add at some point)
    #Encoders
    try:
        global Motor_pinV1, Motor_pinV2, Motor_pinH1, Motor_pinH2
        global encoder_left, last_position_left, encoder_right, last_position_right
        Motor_pinV1 = pwmio.PWMOut(board.D5)
        Motor_pinV2 = pwmio.PWMOut(board.D4)
        

        Motor_pinH1 = pwmio.PWMOut(board.D11)
        Motor_pinH2 = pwmio.PWMOut(board.D10)
        
        encoder_left = rotaryio.IncrementalEncoder(board.D2, board.D3)
        last_position_left = encoder_left.position

        encoder_right = rotaryio.IncrementalEncoder(board.D0, board.D1)
        last_position_right = encoder_right.position
         
    except:
        crash_code = 105
        estop_crash(crash_code)
    try: 
        # Temp/pressure sensor
        global sensor
        sensor = MPL3115A2(i2c)
    except:
        crash_code = 105
        estop_crash(crash_code)

    try:
        #Neopixel
        global pixels
        pixels = neopixel.NeoPixel(board.NEOPIXEL, 1)
    except:
        crash_code = 105
        estop_crash(crash_code)
    
def CONF_MENU(): 
    print('Confic')
    main_menu_cycle = 1
    menu_active = True
    number_of_menus = 5
    
    while menu_active == True:

        while main_menu_cycle == 1: #Wheel diameter 
            time.sleep(0.1)
            display.fill(0)
            display.text('Change diameter?',0,0,1)
            display.show()

            global last_mode_pressed
            mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
            if mode_is_pressed:
                global wheel_diameter
                time.sleep(0.1)
                display.fill(0)
                display.text('Diameter up/down?',0,0,1)
                in_diameter_menu = True

                while in_diameter_menu == True:
                    time.sleep(0.1)
                    mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
                    if mode_is_pressed:
                        in_diameter_menu = False
                    wheel_diameter = diameter_change(wheel_diameter)

                    display.fill(0)
                    display.text('Diameter up/down?',0,0,1)
                    display.text('{:.1f}cm'.format(wheel_diameter),0,20,1)
                    display.show()
                    print('{:.1f}cm'.format(wheel_diameter))

                    estop_switch(Estop_pin, error_code)
                estop_switch(Estop_pin, error_code)
            main_menu_cycle = menu_cycle(main_menu_cycle, number_of_menus) #changes menu
            estop_switch(Estop_pin, error_code)
        
        while main_menu_cycle == 2: #Stop distance
            display.fill(0)
            display.text('Ch stop dis?',0,0,1)
            display.show()
            global obs_stop_dist
            mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
            if mode_is_pressed:
                
                in_obs_menu = True
                while in_obs_menu == True:
                    time.sleep(0.1)
                    mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
                    if mode_is_pressed:
                        in_obs_menu = False
                    obs_stop_dist = obs_stop_dist_change(obs_stop_dist)

                    display.fill(0)
                    display.text('Stop dis up/down',0,0,1)
                    display.text('{:.1f}cm'.format(obs_stop_dist),0,20,1)
                    display.show()
                    estop_switch(Estop_pin, error_code)
                estop_switch(Estop_pin, error_code)
            main_menu_cycle = menu_cycle(main_menu_cycle, number_of_menus) #changes menu
            
        while main_menu_cycle == 3: #Go distance 
            display.fill(0)
            display.text('Ch go dis?',0,0,1)
            display.show()
            global obs_go_dist
            mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
            if mode_is_pressed:
                in_obs_go_menu = True
                while in_obs_go_menu == True:
                    time.sleep(0.1)
                    mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
                    if mode_is_pressed:
                        in_obs_go_menu = False
                        
                    obs_go_dist = obs_go_dist_change(obs_go_dist)
                    display.fill(0)
                    display.text('Go dis up/down',0,0,1)
                    display.text('{:.1f}cm'.format(obs_go_dist),0,20,1)
                    display.show()

                    estop_switch(Estop_pin, error_code)
                estop_switch(Estop_pin, error_code)
            main_menu_cycle = menu_cycle(main_menu_cycle, number_of_menus) #changes menu
        
        while main_menu_cycle == 4: #PWM ramp speed
            display.fill(0)
            display.text('Ch pwm speed?',0,0,1)
            display.show()
            global pwm_ramp_speed
            mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
            if mode_is_pressed:
                in_pwm_ramp_speed_menu = True
                while in_pwm_ramp_speed_menu == True:
                    time.sleep(0.1)
                    mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
                    if mode_is_pressed:
                        in_pwm_ramp_speed_menu = False
                        
                    pwm_ramp_speed = pwm_ramp_speed_change(pwm_ramp_speed)
                    display.fill(0)
                    display.text('Ramp speed up/down',0,0,1)
                    display.text('{:.1f}cm'.format(pwm_ramp_speed),0,20,1)
                    display.show()
                    estop_switch(Estop_pin, error_code)
                estop_switch(Estop_pin, error_code)
            main_menu_cycle = menu_cycle(main_menu_cycle, number_of_menus) #changes menu
        
        while main_menu_cycle == 5: # Curve radius
            display.fill(0)
            display.text('Ch curve radius?',0,0,1)
            display.show()
            global desired_radius
            mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
            if mode_is_pressed:
                in_curve_radius_menu = True
                while in_curve_radius_menu == True:
                    time.sleep(0.1)
                    mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
                    if mode_is_pressed:
                        in_curve_radius_menu = False
                        
                    desired_radius = curve_radius_change(desired_radius)
                    display.fill(0)
                    display.text('Curve radius up/down',0,0,1)
                    display.text('{:.1f}cm'.format(desired_radius),0,20,1)
                    display.show()

                    estop_switch(Estop_pin, error_code)
                estop_switch(Estop_pin, error_code)
            main_menu_cycle = menu_cycle(main_menu_cycle, number_of_menus) #changes menu
                
        while main_menu_cycle == 0: #Exit settings
            time.sleep(0.1)
            display.fill(0)
            display.text('Exit menu?',0,0,1)
            display.show()
            main_menu_cycle = menu_cycle(main_menu_cycle, number_of_menus) #changes menu
            mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
            if mode_is_pressed: #confirms the config settings
                main_menu_cycle = None #change to something thats nott assigned in while loop
                menu_active = False
            estop_switch(Estop_pin, error_code)
            
    return wheel_diameter, obs_stop_dist, obs_go_dist, pwm_ramp_speed, desired_radius

def menu_cycle(menu, number_of_menus): #Function for changing main menu prompt
    if menu == 10:
        return menu
    global last_plus_pressed
    global last_minus_pressed
    
    plus_is_pressed, last_plus_pressed = plus_pin_pressed(last_plus_pressed)
    minus_is_pressed, last_minus_pressed = minus_pin_pressed(last_minus_pressed)
    if plus_is_pressed:
        menu = (menu + 1)  # ensures the menu cycle
        if menu > number_of_menus:
            menu = 0
    elif minus_is_pressed:
        menu = menu - 1
        if menu < 0:
            menu = number_of_menus
        
    time.sleep(0.1)
    return menu

#CONFIG CHANGES
def diameter_change(wheel_diameter):
    wheel_lower = 1 #cm
    wheel_upper = 8 #cm
    global last_plus_pressed, last_minus_pressed
    
    plus_is_pressed, last_plus_pressed = plus_pin_pressed(last_plus_pressed)
    minus_is_pressed, last_minus_pressed = minus_pin_pressed(last_minus_pressed)
    if plus_is_pressed: #changes diameter
        wheel_diameter += 0.1
        if wheel_diameter > wheel_upper:
            wheel_diameter = wheel_lower

    elif minus_is_pressed: #changes diameter
        wheel_diameter -= 0.1
        if wheel_diameter < wheel_lower:
            wheel_diameter = wheel_upper
    return wheel_diameter

def obs_stop_dist_change(obs_stop_dist):
    obs_stop_dist_lower = 20 #cm
    obs_stop_dist_upper = 40 #cm
    global last_plus_pressed, last_minus_pressed
    
    plus_is_pressed, last_plus_pressed = plus_pin_pressed(last_plus_pressed)
    minus_is_pressed, last_minus_pressed = minus_pin_pressed(last_minus_pressed)
    if plus_is_pressed: #changes diameter
        obs_stop_dist += 5
        if obs_stop_dist > obs_stop_dist_upper:
            obs_stop_dist = obs_stop_dist_lower

    elif minus_is_pressed: #changes diameter
        obs_stop_dist -=  5
        if obs_stop_dist < obs_stop_dist_lower:
            obs_stop_dist = obs_stop_dist_upper
    return obs_stop_dist

def obs_go_dist_change(obs_go_dist):
    obs_go_dist_lower = 30 #cm
    obs_go_dist_upper = 60 #cm
    global last_plus_pressed, last_minus_pressed
    
    plus_is_pressed, last_plus_pressed = plus_pin_pressed(last_plus_pressed)
    minus_is_pressed, last_minus_pressed = minus_pin_pressed(last_minus_pressed)
    
    if plus_is_pressed: #changes diameter
        obs_go_dist += 5
        if obs_go_dist > obs_go_dist_upper:
            obs_go_dist = obs_go_dist_lower

    elif minus_is_pressed: #changes diameter
        obs_go_dist -=  5
        if obs_go_dist < obs_go_dist_lower:
            obs_go_dist = obs_go_dist_upper
    return obs_go_dist

def pwm_ramp_speed_change(pwm_ramp_speed):
    pwm_ramp_speed_lower = 10 #cm
    pwm_ramp_speed_upper = 20 #cm
    global last_plus_pressed, last_minus_pressed
    
    plus_is_pressed, last_plus_pressed = plus_pin_pressed(last_plus_pressed)
    minus_is_pressed, last_minus_pressed = minus_pin_pressed(last_minus_pressed)
    
    if plus_is_pressed: #changes diameter
        pwm_ramp_speed += 1
        if pwm_ramp_speed > pwm_ramp_speed_upper:
            pwm_ramp_speed = pwm_ramp_speed_lower

    elif minus_is_pressed: #changes diameter
        pwm_ramp_speed -=  1
        if pwm_ramp_speed < pwm_ramp_speed_lower:
            pwm_ramp_speed = pwm_ramp_speed_upper
    return pwm_ramp_speed

def curve_radius_change(curve_radius):
    curve_radius_lower = 20 #cm
    curve_radius_upper = 100 #cm
    global last_plus_pressed, last_minus_pressed
    
    plus_is_pressed, last_plus_pressed = plus_pin_pressed(last_plus_pressed)
    minus_is_pressed, last_minus_pressed = minus_pin_pressed(last_minus_pressed)
    
    if plus_is_pressed: #changes radius
        curve_radius += 10
        if curve_radius > curve_radius_upper:
            curve_radius = curve_radius_lower

    elif minus_is_pressed: #changes radius
        curve_radius -=  10
        if pwm_ramp_speed < curve_radius_lower:
            curve_radius = curve_radius_upper
    return curve_radius


#BUTTON FUNCTIONS
def mode_pin_pressed(last_mode_pressed):
    current_time =time.monotonic()
    
    button_on = bool(not Mode_pin.value and current_time - last_mode_pressed > DEBOUNCE_TIME)
    if button_on:
        last_mode_pressed = time.monotonic()
    else:
        last_mode_pressed = last_mode_pressed
    return button_on,last_mode_pressed

def plus_pin_pressed(last_plus_pressed):
    current_time =time.monotonic()
    button_on = bool(not plus_pin.value and current_time - last_plus_pressed > DEBOUNCE_TIME)
    if button_on:
        last_plus_pressed = time.monotonic()
    else:
        last_plus_pressed = last_plus_pressed
    return button_on,last_plus_pressed

def minus_pin_pressed(last_minus_pressed):
    current_time = time.monotonic()
    button_on = bool(not minus_pin.value and current_time - last_minus_pressed > DEBOUNCE_TIME)
    if button_on:
        last_minus_pressed = time.monotonic()
    else:
        last_minus_pressed = last_minus_pressed
    return button_on,last_minus_pressed

# CALCULATIVE FUNCTIONS
def dominant_color(threshold = 20, return_position=False):
    rgb = color_sensor.color_rgb_bytes
    max_val = max(rgb)
    
    if max_val < threshold:
        return "Off track"

    labels = ["Red", "Green", "Blue"] if not return_position else ["Left", "Center", "Right"]

    dominant_index = rgb.index(max_val)
    return labels[dominant_index]

def clear_disp(x1,x2,y1,y2):

    for i in range(x1,x2,1):
        for j in range(y1,y2,1):
            display.pixel(i,j,0)

def IR_process():
    an_val = IR_sensor.value
    distance = 4.1061*pow(10,-8)*pow(an_val,2)-0.0044*an_val+133.4935
    display.text(" ",0,0,0)
    display.text("{0:1.0f}cm".format(distance),0,0,1)
    print("{0:1.0f}cm".format(distance),end="   ")
    distance_checker(distance)

def distance_checker(distance):
    if distance < 30:
        display.text('Clear',50,0,0)
        display.text('OBS',50,0,1)
        pixels[0] = (255, 0, 0)
    else:
        display.text('OBS',50,0,0)
        display.text('Clear',50,0,1)
        pixels[0] = (0, 0, 0)

def pos_to_cm(position):
    return -3.1415*(wheel_diameter) * position / INCREMENTS_PER_REV

def read_IR_sensor():
    an_val = IR_sensor.value
    return 4.1061*pow(10,-8)*pow(an_val,2)-0.0044*an_val+133.4935
    
def obsticle(dist_to_obs):
    return bool(read_IR_sensor() < dist_to_obs)
 
def motor_speed(V1,V2,H1,H2):
    Motor_pinV1.duty_cycle = V1
    Motor_pinV2.duty_cycle = V2
    Motor_pinH1.duty_cycle = H1
    Motor_pinH2.duty_cycle = H2
    
def read_speed():
    global last_distance_cm_left,last_distance_cm_right,encoder_left,encoder_right, last_time, delta_time
    
    delta_time = time.monotonic()- last_time
    
    speed_left =  (pos_to_cm(encoder_left.position) - last_distance_cm_left)/ delta_time
    speed_right = (-pos_to_cm(encoder_right.position) - last_distance_cm_right)/ delta_time
    
    last_distance_cm_left = pos_to_cm(encoder_left.position)
    last_distance_cm_right = -pos_to_cm(encoder_right.position)
    
    last_time = time.monotonic()
    return speed_left, speed_right

def color_to_pwm():
    pf = round(0.20*65535)
    track_positions = ["Off track", "Left", "Center", "Right"]
    pwm_configs = [
        (pf, 0, 0, pf),  # Off track
        (0, pf, pf, 0),  # Left
        (0, pf, 0, pf),  # Center
        (pf, 0, 0, pf)   # Right
    ]
    track_pos = dominant_color(return_position=True)
    idx = track_positions.index(track_pos)
    

    return pwm_configs[idx]


mode = 0
error_code = 1

while True:
  
    while mode == 0: #INIT (comp test)

        
        Component_Test()
        
        error_code = 105
        estop_switch(Estop_pin, error_code)
        
        display.fill(0)
        display.text('Initializing:',0,20,1)
        display.text('Finalizing',0,0,1)
        display.show()
        time.sleep(0.2)

        
        # Estop button handling
        error_code = 106
        estop_switch(Estop_pin, error_code)
        mode = 1
    
    while mode == 1: #Start
        estop_switch(Estop_pin, error_code)
        start_mode = 1
        start_mode_menu_nr = 3 #Only 2 becouse there is no exit option
        while start_mode == 1:
            display.fill(0)
            display.text('Mode Start',0,0,1)
            display.text('CONFIC?',0,20,1)
            display.show()
            
            mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
            if mode_is_pressed:
                mode = 2
                start_mode = None
            start_mode = menu_cycle(start_mode,start_mode_menu_nr)
            estop_switch(Estop_pin, error_code)
            time.sleep(0.01)
        while start_mode == 2:
            display.fill(0)
            display.text('Mode Start',0,0,1)
            display.text('TEST?',0,20,1)
            display.show()
            mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
            if mode_is_pressed:
                mode = 3
                start_mode = None
            
            start_mode = menu_cycle(start_mode,start_mode_menu_nr)
            estop_switch(Estop_pin, error_code)
            time.sleep(0.01)
        while start_mode == 3:
            display.fill(0)
            display.text('Mode Start',0,0,1)
            display.text('RUN?',0,20,1)
            display.show()
            mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
            if mode_is_pressed:
                mode = 4
                start_mode = None
                
            start_mode = menu_cycle(start_mode,start_mode_menu_nr)
            time.sleep(0.01)
        while start_mode == 0:
            display.fill(0)
            display.text('Mode Start',0,0,1)
            display.text('Restart?',0,20,1)
            display.show()
            mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
            if mode_is_pressed:
                display.fill(0)
                display.text('RESTARTING',60,20,1)
                display.show()
                time.sleep(0.1)
                display.fill(0)
                display.show()
                supervisor.reload()
                
            start_mode = menu_cycle(start_mode,start_mode_menu_nr)
            estop_switch(Estop_pin, error_code)
            time.sleep(0.01)
      
    while mode == 2: #CONF
        #Prófa að sleppa þessu, ætti að vera global og uppfærast sjálfkrafa.
        wheel_diameter, obs_stop_dist, obs_go_dist,pwm_ramp_speed, desired_radius  = CONF_MENU()
        wheel_diameter = change_variable_over_bluetooth(wheel_diameter, "Wheel diameter", uart)
        print(wheel_diameter)
        
        # Estop button handling
        estop_switch(Estop_pin, error_code)
        mode = 1

    while mode == 3: #TEST
        current_time = time.monotonic()
        test_mode = 1
        test_mode_menu_nr = 3
        while test_mode == 1:
            display.fill(0)
            display.text('TEST:',0,0,1)
            display.text('DRV OL FW?',0,20,1)
            display.show()
            mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
            if mode_is_pressed:
                mode = 10
                test_mode = None
                
            test_mode = menu_cycle(test_mode,test_mode_menu_nr)
            estop_switch(Estop_pin, error_code)
            time.sleep(0.01)
        while test_mode == 2:
            display.fill(0)
            display.text('TEST:',0,0,1)
            display.text('PID?',0,20,1)
            display.show()
            mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
            if mode_is_pressed:
                mode = 15
                test_mode = None
                
            test_mode = menu_cycle(test_mode,test_mode_menu_nr)
            estop_switch(Estop_pin, error_code)
            time.sleep(0.01)
        while test_mode == 3:
            display.fill(0)
            display.text('TEST:',0,0,1)
            display.text('PWM ramp?',0,20,1)
            display.show()
            mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
            if mode_is_pressed:
                
                mode = 12
                test_mode = None
            
            test_mode = menu_cycle(test_mode,test_mode_menu_nr)
            estop_switch(Estop_pin, error_code)
            time.sleep(0.01)
        while test_mode == 0:
            display.fill(0)
            display.text('TEST:',0,0,1)
            display.text('Exit TEST?',0,20,1)
            display.show()
            
            mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
            if mode_is_pressed:
                mode = 1
                test_mode = None
            test_mode = menu_cycle(test_mode,test_mode_menu_nr)
            estop_switch(Estop_pin, error_code)
            time.sleep(0.01)
    
    while mode == 4: #RUN
        current_time = time.monotonic()
        estop_switch(Estop_pin, error_code)
        display.fill(0)
        display.text('RUN',0,0,1)
        display.text('Start?',0,20,1)
        display.show()
        mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
        if mode_is_pressed:
            disp_time = time.monotonic()
            last_pwm_configs = (0,0,0,0)
            while True:
                estop_switch(Estop_pin, error_code)
                pwm_configs = color_to_pwm()
                if not last_pwm_configs == pwm_configs:
                    motor_speed(*pwm_configs)
                else:
                    motor_speed(*pwm_configs)
                last_pwm_configs = pwm_configs
                if time.monotonic()-disp_time>0.1:
                    display.fill(0)
                    display.text('RUN',0,0,1)
                    display.text(str(dominant_color()),0,15,1)
                    
                    display.show()
                    disp_time = time.monotonic()
                time.sleep(0.01)
        time.sleep(0.01)
                
    while mode == 10: #Test:DRV OL FW
        display.fill(0)
        display.text('DRV OL FW',60,20,1)
        display.show()
        i=0
        last_position_left = pos_to_cm(encoder_left.position)
        last_position_right = pos_to_cm(encoder_left.position)
        jj = 100
        disp_time = time.monotonic()
        time.sleep(0.1)
        while (pos_to_cm(encoder_left.position)-last_position_left) < 100:
            estop_switch(Estop_pin, error_code)
            #Update display
            if time.monotonic()-disp_time>0.5:
                display.fill(0)
                display.text('DRV OL FW',60,20,1)
                display.text(str(pos_to_cm(encoder_left.position)),0,0,1)
                display.text(str(pos_to_cm(encoder_left.position)),0,0,1)
                display.show()
                disp_time = time.monotonic()
                
            if obsticle(obs_stop_dist):
                
                for j in range(i,0,1):
                    estop_switch(Estop_pin, error_code)
                    motor_speed(0,round(j*655.35),0,round(j*655.35))
                    time.sleep(0.1)
                while obsticle(obs_go_dist):
                    estop_switch(Estop_pin, error_code)
                    print('Waiting',end='')
                    print(read_IR_sensor())
                    motor_speed(0,0,0,0)
                    if time.monotonic()-disp_time>0.1:
                        display.fill(0)
                        display.text('DRV OL FW',60,20,1)
                        display.text('OBS',0,0,1)
                        display.show()
                        disp_time = time.monotonic()
                    time.sleep(0.1)
                i=0
                    
                    
            
            i += 10
            
            if pos_to_cm(encoder_left.position)-last_position_left > 70:
                i = (340 - 10/3*(pos_to_cm(encoder_left.position)-last_position_left))
                print(i)
            
            if i>100:
                i=100
            motor_speed(0,round(i*655.35),0,round(i*655.35))
                
            time.sleep(0.01)
            
        motor_speed(0,0,0,0)
        mode = 3
    
    while mode == 11: # Test:PID:Fw
        estop_switch(Estop_pin, error_code)
        display.fill(0)
        display.text('PID',60,20,1)
        display.show()
        
        disp_time = time.monotonic()
        current_time = time.monotonic()
        previous_error_time = time.monotonic()
        time.sleep(0.1)
        while True:
            estop_switch(Estop_pin, error_code)
            distance_cm_left = pos_to_cm(-encoder_left.position)
            distance_cm_right = pos_to_cm(encoder_right.position)
            
            current_speed_cm_left, current_speed_cm_right = read_speed()
            
            #print((current_speed_cm_left,current_speed_cm_right,))
            # Calculate the error

            error_left = desired_speed_cm_left - current_speed_cm_left
            error_right = desired_speed_cm_right - current_speed_cm_right
            
            errors_list_left.append(error_left)  # Append the new error to the list
            errors_list_right.append(error_right)  # Append the new error to the list
            
            integral_left = sum(errors_list_left) 
            integral_right = sum(errors_list_right) 
            
            
            derivative_left = (error_left - previous_error_left)/(time.monotonic()-previous_error_time)/delta_time
            derivative_right = (error_right - previous_error_right)/(time.monotonic()-previous_error_time)/delta_time
            
            # PID output
            print((Kp * error_left), (Ki * integral_left), (Kd * derivative_left))
            Power_left = (Kp * error_left) + (Ki * integral_left) + (Kd * derivative_left)
            Power_right = (Kp * error_right) + (Ki * integral_right) + (Kd * derivative_right)
            # Convert Power to PWM
            pwm_value_left = round(Power_left * 65535)
            pwm_value_right = round(Power_right * 655.35)

            # Ensure PWM value is within bounds
            if pwm_value_left > 65535:
                pwm_value_left = 65535
            elif pwm_value_left < 0:
                pwm_value_left = 0
                
            if pwm_value_right > 65535:
                pwm_value_right = 65535
            elif pwm_value_right < 0:
                pwm_value_right = 0

            

            previous_error_left = error_left
            previous_error_right = error_right
            previous_error_time = time.monotonic()

            time.sleep(0.5)  # Delay for a tenth of a second (or adjust as needed)
            if time.monotonic()-disp_time >= 0.1:
                
                display.fill(0)
                display.text('PID',60,20,1)
                display.text('{:.1f}cm/s {:.1f}cm/s'.format(current_speed_cm_left,current_speed_cm_right),0,0,1)
                display.show()
                disp_time = time.monotonic()
            motor_speed(0,pwm_value_left,0,pwm_value_right)
             
    while mode == 12: # Test:PWM ramp up L4B3
        estop_switch(Estop_pin, error_code)
        display.fill(0)
        display.text('PWM RAMP',0,0,1)
        display.show()
        
        disp_time = time.monotonic()
        
        power = 30 #Starts at 30 avoid starting too slow
        speeds = (0,0) 
        while (speeds[0]+speeds[1])/2< pwm_ramp_speed:
            estop_switch(Estop_pin, error_code)
            time.sleep(0.05)
            motor_speed(0,round(power*655.35),0,round(power*655.35))
            power= power +1
            speeds = read_speed()
            print(speeds)
            if time.monotonic()-disp_time >= 0.1:
                display.fill(0)
                display.text('{:.1f}cm/s {:.1f}cm/s'.format(speeds[0],speeds[1]),0,20,1)
                display.show()
                disp_time = time.monotonic()
            top_i = power
        
        for i in range(0,50,1): #only t0 50 to cutoff at low speed
            estop_switch(Estop_pin, error_code)
            time.sleep(0.05)
            motor_speed(0,round(top_i/100*(100-i)*655.35),0,round(top_i/100*(100-i)*655.35))
            speeds = read_speed()
            if time.monotonic()-disp_time >= 0.1:
                display.fill(0)
                display.text('{:.1f}cm/s {:.1f}cm/s'.format(speeds[0],speeds[1]),0,20,1)
                display.show()
                disp_time = time.monotonic()
            
        motor_speed(0,0,0,0)
        mode = 3
            
    while mode == 13: # Test:PID:CurveR 
    
        display.fill(0)
        display.text('PID: CurveR',60,20,1)
        display.show()
        
        disp_time = time.monotonic()
        current_time = time.monotonic()
        previous_error_time = time.monotonic()
        time.sleep(0.1)
        while True:
            desired_speed_cm_left_analog = desired_speed_cm_left*pot_pin1.value/65535
            desired_speed_cm_right_analog = desired_speed_cm_right*pot_pin2.value/65535
            distance_cm_left = pos_to_cm(-encoder_left.position)
            distance_cm_right = pos_to_cm(encoder_right.position)
            
            current_speed_cm_left, current_speed_cm_right = read_speed()
            
            #print((current_speed_cm_left,current_speed_cm_right,))
            # Calculate the error

            error_left = desired_speed_cm_left - current_speed_cm_left
            error_right = desired_speed_cm_right - current_speed_cm_right
            
            errors_list_left.append(error_left)  # Append the new error to the list
            errors_list_right.append(error_right)  # Append the new error to the list
            
            integral_left = sum(errors_list_left) 
            integral_right = sum(errors_list_right) 
            
            
            derivative_left = (error_left - previous_error_left)/(time.monotonic()-previous_error_time)/delta_time
            derivative_right = (error_right - previous_error_right)/(time.monotonic()-previous_error_time)/delta_time
            
            # PID output
            
            print(((Kp * error_left), (Ki * integral_left), (Kd * derivative_left)))
            Power_left = (Kp * error_left) + (Ki * integral_left) + (Kd * derivative_left)
            Power_right = (Kp * error_right) + (Ki * integral_right) + (Kd * derivative_right)
            # Convert Power to PWM
            
            pwm_value_left = round(Power_left * 65535)
            pwm_value_right = round(Power_right * 65535)

            # Ensure PWM value is within bounds
            if pwm_value_left > 65535:
                pwm_value_left = 65535
            elif pwm_value_left < 0:
                pwm_value_left = 0
                
            if pwm_value_right > 65535:
                pwm_value_right = 65535
            elif pwm_value_right < 0:
                pwm_value_right = 0

            

            previous_error_left = error_left
            previous_error_right = error_right
            previous_error_time = time.monotonic()

            time.sleep(0.5)  # Delay for a tenth of a second (or adjust as needed)
            if time.monotonic()-disp_time >= 0.1:
                
                display.fill(0)
                display.text('PID CurveR',0,0,1)
                display.text('{:.1f}cm/s {:.1f}cm/s'.format(current_speed_cm_left,current_speed_cm_right),0,10,1)
                display.text('Radius: {:.1f}cm'.format(20*pot_pin1.value/65535),0,20,1)
                
                display.show()
                disp_time = time.monotonic()
            motor_speed(0,pwm_value_left,0,1000)
            
    while mode == 14: # Test:PID:CurveL 
    
        display.fill(0)
        display.text('PID: CurveL',60,20,1)
        display.show()
        
        disp_time = time.monotonic()
        current_time = time.monotonic()
        previous_error_time = time.monotonic()
        time.sleep(0.1)
        while True:
            
            desired_speed_cm_left_analog= desired_speed_cm_left*pot_pin1.value/65535
            desired_speed_cm_right_analog= desired_speed_cm_right*pot_pin2.value/65535
            
            distance_cm_left = pos_to_cm(-encoder_left.position)
            distance_cm_right = pos_to_cm(encoder_right.position)
            
            current_speed_cm_left, current_speed_cm_right = read_speed()
            
            print((current_speed_cm_left,current_speed_cm_right,))
            # Calculate the error

            error_left = desired_speed_cm_left_analog - current_speed_cm_left
            error_right = desired_speed_cm_right_analog - current_speed_cm_right
            
            errors_list_left.append(error_left)  # Append the new error to the list
            errors_list_right.append(error_right)  # Append the new error to the list
            
            integral_left = sum(errors_list_left) 
            integral_right = sum(errors_list_right) 
            
            
            derivative_left = (error_left - previous_error_left)/(time.monotonic()-previous_error_time)/delta_time
            derivative_right = (error_right - previous_error_right)/(time.monotonic()-previous_error_time)/delta_time
            
            # PID output
            Power_left = (Kp * error_left) + (Ki * integral_left) + (Kd * derivative_left)
            Power_right = (Kp * error_right) + (Ki * integral_right) + (Kd * derivative_right)
            # Convert Power to PWM
            
            pwm_value_left = round(Power_left * 65535)
            pwm_value_right = round(Power_right * 65535)*pot_pin2.value/65535

            # Ensure PWM value is within bounds
            if pwm_value_left > 65535:
                pwm_value_left = 65535
            elif pwm_value_left < 0:
                pwm_value_left = 0
                
            if pwm_value_right > 65535:
                pwm_value_right = 65535
            elif pwm_value_right < 0:
                pwm_value_right = 0

            

            previous_error_left = error_left
            previous_error_right = error_right
            previous_error_time = time.monotonic()

            time.sleep(0.5)  # Delay for a tenth of a second (or adjust as needed)
            if time.monotonic()-disp_time >= 0.1:
                
                display.fill(0)
                display.text('PID: CurveL',60,20,1)
                display.text('{:.1f}cm/s {:.1f}cm/s'.format(current_speed_cm_left,current_speed_cm_right),0,0,1)
                display.show()
                disp_time = time.monotonic()
            motor_speed(0,pwm_value_left,0,pwm_value_right)
                   
    while mode == 15: # Test:PID
        pid_mode = 1
        print('PID select')
        
        while pid_mode == 1: #PID: FW?
            display.fill(0)
            display.text('PID: FW?',0,0,1)
            display.show()
            
            mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
            if mode_is_pressed:
                mode = 11
                pid_mode = None
            pid_mode = menu_cycle(pid_mode,4)
            time.sleep(0.01)
        while pid_mode == 2: #PID: CurveR?
            display.fill(0)
            display.text('PID: CurveR?',0,0,1)
            display.show()
            mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
            if mode_is_pressed:
                mode = 13
                pid_mode = None
            pid_mode = menu_cycle(pid_mode,4)
            time.sleep(0.01)
        while pid_mode == 3: #PID: CurveL?
            display.fill(0)
            display.text('PID: CurveL?',0,0,1)
            display.show()
            mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
            if mode_is_pressed:
                mode = 14
                pid_mode = None
            pid_mode = menu_cycle(pid_mode,5)
            time.sleep(0.01)
        while pid_mode == 4: #PID: Exit?
            display.fill(0)
            display.text('PID: Exit?',0,0,1)
            display.show()
            mode_is_pressed,last_mode_pressed = mode_pin_pressed(last_mode_pressed)
            if mode_is_pressed:
                mode = 3
                pid_mode = None
            pid_mode = menu_cycle(pid_mode,5)
            time.sleep(0.01)
    
    
    
