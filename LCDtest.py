import sys

sys.path.append('../')
import RPi.GPIO as GPIO
import rgb1602
import time
import smbus
import lgpio as sbc
import signal
import os
import sys, getopt

# Configuration
PWM_GPIO_NR = 12  # PWM gpio number used to drive PWM fan (gpio18 = pin 12)
WAIT_TIME = 0.1  # [s] Time to wait between each refresh
PWM_FREQ = 10000  # [Hz] 10kHz for Noctua PWM control

# Configurable temperature and fan speed
MIN_TEMP = 65
MAX_TEMP = 95
FAN_LOW = 20
FAN_HIGH = 100
FAN_OFF = 0
FAN_MAX = 100

# logging and metrics (enable = 1)
VERBOSE = 1
NODE_EXPORTER = 0

# parse input arguments
try:
    opts, args = getopt.getopt(sys.argv[1:], "hv",
                               ["min-temp=", "max-temp=", "fan-low=", "fan-high=", "wait-time=", "help", "pwm-gpio=",
                                "pwm-freq=", "verbose", "node-exporter"])
except getopt.GetoptError:
    print(
        'fan.py [--min-temp=40] [--max-temp=70] [--fan-low=20] [--fan-high=100] [--wait-time=1] [--pwm-gpio=18] [--pwm-freq=10000] [--node-exporter] [-v|--verbose] [-h|--help]')
    sys.exit(2)
for opt, arg in opts:
    if opt in ("-h", "--help"):
        print(
            'fan.py [--min-temp=40] [--max-temp=70] [--fan-low=20] [--fan-high=100] [--wait-time=1] [--pwm-gpio=18] [--pwm-freq=10000] [--node-exporter] [-v|--verbose] [-h|--help]')
        sys.exit()
    elif opt in ("-v", "--verbose"):
        VERBOSE = 1
    elif opt in ("--min-temp"):
        MIN_TEMP = int(arg)
    elif opt in ("--max-temp"):
        MAX_TEMP = int(arg)
    elif opt in ("--fan-low"):
        FAN_LOW = int(arg)
    elif opt in ("--fan-high"):
        FAN_HIGH = int(arg)
    elif opt in ("--wait-time"):
        WAIT_TIME = int(arg)
    elif opt in ("--pwm-gpio"):
        PWM_GPIO_NR = int(arg)
    elif opt in ("--pwm-freq"):
        PWM_FREQ = int(arg)
    elif opt in ("--node-exporter"):
        NODE_EXPORTER = 1
print("")
print("MIN_TEMP:", MIN_TEMP)
print("MAX_TEMP:", MAX_TEMP)
print("FAN_LOW:", FAN_LOW)
print("FAN_HIGH:", FAN_HIGH)
print("WAIT_TIME:", WAIT_TIME)
print("PWM_GPIO_NR:", PWM_GPIO_NR)
print("PWM_FREQ:", PWM_FREQ)
print("VERBOSE:", VERBOSE)
print("NODE_EXPORTER:", NODE_EXPORTER)
print("")


####from other script
# # Get CPU's temperature
def getCpuTemperature():
    res = read_temp_sensor();
    # temp = (float(res)/1000)
    print("original reading: ", res)
    # print("whack reading: ", temp)
    # return temp
    return res


# Set fan speed
def setFanSpeed(speed, temp):
    sbc.tx_pwm(fan, PWM_GPIO_NR, PWM_FREQ, speed, pulse_offset=0, pulse_cycles=0)

    # print fan speed and temperature
    if VERBOSE == 1:
        print("fan speed: ", int(speed), "    temp: ", temp)

    # write fan metrics to file for node-exporter/prometheus
    if NODE_EXPORTER == 1:
        # Save a reference to the original standard output
        original_stdout = sys.stdout
        with open('/var/lib/node_exporter/fan-metrics.prom', 'w') as f:
            # Change the standard output to the file we created.
            sys.stdout = f
            print('raspberry_fan_speed ', speed)
            print('raspberry_fan_temp ', temp)
            print('raspberry_fan_min_temp ', MIN_TEMP)
            print('raspberry_fan_max_temp ', MAX_TEMP)
            print('raspberry_fan_fan_low ', FAN_LOW)
            print('raspberry_fan_fan_high ', FAN_HIGH)
            print('raspberry_fan_wait_time ', WAIT_TIME)
            print('raspberry_fan_pwm_gpio ', PWM_GPIO_NR)
            print('raspberry_fan_freq ', PWM_FREQ)
            # Reset the standard output to its original value
            sys.stdout = original_stdout
            f.close()

    return ()


# Handle fan speed
def handleFanSpeed():
    temp = getCpuTemperature()

    # Turn off the fan if temperature is below MIN_TEMP
    if temp < MIN_TEMP:
        setFanSpeed(FAN_OFF, temp)
        return int(FAN_OFF)

    # Set fan speed to MAXIMUM if the temperature is above MAX_TEMP
    elif temp > MAX_TEMP:
        setFanSpeed(FAN_MAX, temp)
        return int(FAN_MAX)

    # Caculate dynamic fan speed
    else:
        step = (FAN_HIGH - FAN_LOW) / (MAX_TEMP - MIN_TEMP)
        delta = temp - MIN_TEMP
        speed = FAN_LOW + (round(delta) * step)
        setFanSpeed(speed, temp)
        return int(speed)


# Handle manual speed
def handleFanSpeedMan(ManualSpeed):
    temp = getCpuTemperature()
    setFanSpeed(ManualSpeed, temp)
    print(ManualSpeed)
    return int(ManualSpeed)


def read_temp_sensor():
    # Get I2C bus
    bus = smbus.SMBus(1)
    bus.write_byte(0x40, 0xF3)

    time.sleep(0.3)
    # SI7021 address, 0x40 Read data 2 bytes, Temperature
    data0 = bus.read_byte(0x40)
    data1 = bus.read_byte(0x40)
    # Convert the data and output it
    celsTemp = ((data0 * 256 + data1) * 175.72 / 65536.0) - 46.85
    fahrTemp = celsTemp * 1.8 + 32
    # print("Sensor temp: " + str(fahrTemp))
    return (fahrTemp)


lcd = rgb1602.RGB1602(16, 2)
GPIO.setmode(GPIO.BCM)
# Define keys
lcd_key = 0
key_in = 0

btnRIGHT = 0
btnUP = 1
btnDOWN = 2
btnLEFT = 3
btnSELECT = 4

GPIO.setup(16, GPIO.IN)
GPIO.setup(17, GPIO.IN)
GPIO.setup(18, GPIO.IN)
GPIO.setup(19, GPIO.IN)
GPIO.setup(20, GPIO.IN)


# Read the key value
def read_LCD_buttons():
    key_in16 = GPIO.input(16)
    key_in17 = GPIO.input(17)
    key_in18 = GPIO.input(18)
    key_in19 = GPIO.input(19)
    key_in20 = GPIO.input(20)

    # select button
    if (key_in16 == 1):
        return btnSELECT

    # up button
    if (key_in17 == 1):
        return btnUP

    # down button
    if (key_in18 == 1):
        return btnDOWN

    # left button
    if (key_in19 == 1):
        return btnLEFT

    # right button
    if (key_in20 == 1):
        return btnRIGHT


# Delay between each button press input
# Current delay is 0.2 seconds
def button_delay(oldepoch):
    if (time.time() - oldepoch >= 0.2):
        return True
    return False


# Default phase2 Display
def menu_main(curTemp, autoFan, manualSpeed):
    lcd.clear()
    lcd.setCursor(0, 0)
    lcd.printout("Temp: %.2f F" % curTemp)

    # Handle fan speed every WAIT_TIME sec
    # Also check to see if manual speed has been enabled
    if (autoFan):
        speed = handleFanSpeed()
    else:
        speed = handleFanSpeedMan(manualSpeed)
    lcd.setCursor(0, 1)
    disp = "Speed: " + str(speed) + "%"
    lcd.printout(disp)


# Phase 2 menu display function
def menu_modedisplay(menuupdate, autoFan, manualSpeed, lastPress):
    lcd.clear()
    lcd.setCursor(0, 0)

    # Menu 1 screen will tell which fan state is on Auto or Manual
    lcd.printout("Fan Control:")
    lcd.setCursor(0, 1)
    if (autoFan):
        lcd.printout("Auto")
    else:
        lcd.printout("Manual: " + str(manualSpeed) + "%")

    if (menuupdate):
        lcd.clear()
        print("Entering Fan Mode Update")
        lcd.setCursor(0, 0)
        lcd.printout("Set Control:")
        lcd.setCursor(0, 1)

        #Keep track of changing mode or fan speed
        selectmode = True

        while (True):
            lcd.clear()
            if (autoFan):
                lcd.printout("Auto")
            else:
                lcd.printout("Manual: " + str(manualSpeed) + "%")

            lcd_key = read_LCD_buttons()

            if(lcd_key is not None and button_delay(lastPress)):
                if(selectmode):
                    #inverse mode when either button are pressed
                    if(lcd_key == btnRIGHT or lcd_key == btnLEFT):
                        autoFan = not autoFan
                    if(lcd_key == btnSELECT):
                        selectmode = False

                #Elif to check if auto fan or manual update is selected
                elif(not autoFan):
                    # Increment by 1 if left/right is pressed
                    # Increment by 10.0 if up/down is pressed
                    if (lcd_key is not None and button_delay(lastPress)):
                        if (lcd_key == btnRIGHT):
                            manualSpeed += 1
                        elif (lcd_key == btnLEFT):
                            manualSpeed -= 1
                        elif (lcd_key == btnUP):
                            manualSpeed += 10
                        elif (lcd_key == btnDOWN):
                            manualSpeed -= 10
                        elif (lcd_key == btnSELECT):
                            print("Manual Speed Selected")
                            break
                        print("New speed selected")
                        lastPress = time.time()
            time.sleep(WAIT_TIME)
        print("New Fan Controls Set")
        if(autoFan):
            print("Mode: Auto")
        else:
            print("Mode: Manual" + str(manualSpeed) + "%")
        lcd.clear()
        lcd.setCursor(0, 0)
        lcd.printout("Fan Control Set:")
        lcd.setCursor(0, 1)
        if (autoFan):
            lcd.printout("Auto")
        else:
            lcd.printout("Manual: " + str(manualSpeed) + "%")
        time.sleep(2)
        lcd.clear()
        return [autoFan, manualSpeed]









# Display current target/goal temperature and give option to update to new one
def menu_goaltemp(menuupdate, desTemp, lastPress):
    lcd.clear()
    lcd.setCursor(0, 0)
    lcd.printout("Target Temp:")
    lcd.setCursor(0, 1)
    lcd.printout("%.1f F" % desTemp)

    if (menuupdate):
        print("Editing Target Temp")
        lcd.setCursor(0, 0)
        lcd.printout("Set Target Temp:")
        lcd.setCursor(0, 1)

        while (True):
            # format second line to always have 4 characters in the float XX.X
            lcd.printout("%.1f" % desTemp + " F")

            lcd_key = read_LCD_buttons()  # Reading keys
            # Increment by 0.1 if left/right is pressed
            # Increment by 1.0 if up/down is pressed
            if(lcd_key is not None and button_delay(lastPress)):
                if (lcd_key == btnRIGHT):
                    desTemp += 0.1
                elif (lcd_key == btnLEFT):
                    desTemp -= 0.1
                elif (lcd_key == btnUP):
                    desTemp += 1.0
                elif (lcd_key == btnDOWN):
                    desTemp -= 1.0
                    # Select button sets temperature and changes display green to confirm
                elif (lcd_key == btnSELECT):
                    lcd.clear()
                    print("Target Temp Select: " + str(desTemp))
                    lcd.setRGB(124, 252, 0)
                    lcd.setCursor(0, 0)
                    lcd.printout("Target Temp Set:")
                    lcd.setCursor(0, 1)
                    lcd.printout("%.1f" % desTemp + " F")
                    time.sleep(2)
                    lcd.clear()
                    lastPress = time.time()
                    return desTemp
                lastPress = time.time()
                print("Desired Temp Update: " + str(desTemp))





lcd.setCursor(0, 0)
# Initial color is blue
lcd.setRGB(30, 144, 255)

# Note up to only 32 characters 16 on each line
# I am unsure if the \n escape character works properly with it right now if not it can be manually circumvented

# Current assumption is that it does NOT automatically wrap to the next line and \n must be used

# placeholder temperature
curTemp = 72.0
desTemp = 70.0

# Keep track of last button press
lastPress = time.time()
print("Entered Phase 1")
print("Default temp: " + str(curTemp))
print("Default target temp: " + str(desTemp))

# while loop to set initial temperature target
while True:
    lcd.setCursor(0, 0)
    lcd.printout("Set Target Temp:")
    lcd.setCursor(0, 1)

    # format second line to always have 4 characters in the float XX.X
    lcd.printout("%.1f" % desTemp + " F")

    lcd_key = read_LCD_buttons()  # Reading keys
    # Increment by 0.1 if left/right is pressed
    # Increment by 1.0 if up/down is pressed
    if (lcd_key is not None and button_delay(lastPress)):
        if (lcd_key == btnRIGHT):
            desTemp += 0.1
        elif (lcd_key == btnLEFT):
            desTemp -= 0.1
        elif (lcd_key == btnUP):
            desTemp += 1.0
        elif (lcd_key == btnDOWN):
            desTemp -= 1.0
            # Select button sets temperature and changes display green to confirm
        elif (lcd_key == btnSELECT):
            lcd.clear()
            print("Target Temp Select: " + str(desTemp))
            lcd.setRGB(124, 252, 0)
            lcd.setCursor(0, 0)
            lcd.printout("Target Temp Set:")
            lcd.setCursor(0, 1)
            lcd.printout("%.1f" % desTemp + " F")
            time.sleep(2)
            lcd.clear()
            print("Phase 1 Complete")
            break

        lastPress = time.time()
        print("Desired Temp Update: " + str(desTemp))




# try:
# Setup GPIO pin
fan = sbc.gpiochip_open(0)
sbc.gpio_claim_output(fan, PWM_GPIO_NR)
setFanSpeed(FAN_LOW, MIN_TEMP)

# Menu state correlates to each possible menu screen
menustate = 0

# Keep track if a current menu is being edited/updated
menuupdate = False

# Keep track of current time to time out if no updates happen after certain time
timetrack = time.time()


# Keep track of time for phase 2 menu
def menu_timeout(oldepoch, menustate):
    # Current timeout time is 10 seconds will return to main menu afterwards
    if (menustate != 0):
        if (time.time() - oldepoch >= 10):
            print("Menu time up going back to home")
            return 0
    return menustate


# Boolean to keep track if manual or auto fan control is on
# Default value is True
autoFan = True
# int/percentage value to keep track of manual speed starts at 100%
manualSpeed = 100
print("Entered Phase 2")

# while loop to read and update temperature
while True:
    # lcd.clear()

    # Read and update current read temperature
    curTemp = read_temp_sensor()
    print("menu state is " + str(menustate))
    # Change color of LCD depending if temperature is higher than desired
    if (curTemp > desTemp):
        lcd.setRGB(255, 0, 0)
    else:
        lcd.setRGB(124, 252, 0)

    # Check which menu to display
    if (menustate == 0):
        menu_main(curTemp, autoFan, desTemp)
    elif (menustate == 1):
        result = menu_modedisplay(menuupdate, autoFan, manualSpeed, time.time())
        if(result is not None):
            autoFan = result[0]
            manualSpeed = result[1]
            menuupdate = False
        menustate = menu_timeout(timetrack, menustate)
    elif (menustate == 2):
        tempval = menu_goaltemp(menuupdate, desTemp, time.time())
        if(tempval is not None):
            desTemp = tempval
        menustate = menu_timeout(timetrack, menustate)
    else:
        menu_main(curTemp, autoFan, desTemp)

    lcd_key = read_LCD_buttons()  # Reading keys
    if (not menuupdate and button_delay(lastPress)):
        # Go to next menu up
        if (lcd_key == btnRIGHT):
            print("Going to next screen")
            if (menustate == 2):
                menustate = 0
            else:
                menustate += 1
            # Begin timeout timer
            timetrack = time.time()

        # Go to previous menu
        elif (lcd_key == btnLEFT):
            print("Going to prev screen")
            if (menustate == 0):
                menustate = 2
            else:
                menustate -= 1
            # Begin timeout timer
            timetrack = time.time()
        # Set edit menu state to be true if not on main
        elif (lcd_key == btnSELECT and menustate > 0):
            print("Entering edit mode")
            menuupdate = True

        lastPress = time.time()


    time.sleep(WAIT_TIME)

# except KeyboardInterrupt: # trap a CTRL+C keyboard interrupt
#     setFanSpeed(FAN_LOW,MIN_TEMP)
