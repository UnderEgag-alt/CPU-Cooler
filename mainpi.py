import sys
sys.path.append('../')
import RPi.GPIO as GPIO
import rgb1602
import time
import smbus



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
    print("Sensor temp: " + str(fahrTemp))
    return(fahrTemp)

lcd = rgb1602.RGB1602(16,2)
GPIO.setmode(GPIO.BCM)
# Define keys
lcd_key     = 0
key_in  = 0

btnRIGHT  = 0
btnUP     = 1
btnDOWN   = 2
btnLEFT   = 3
btnSELECT = 4

GPIO.setup(16, GPIO.IN)
GPIO.setup(17, GPIO.IN)
GPIO.setup(18, GPIO.IN)
GPIO.setup(19, GPIO.IN)
GPIO.setup(20, GPIO.IN)


#Read the key value
def read_LCD_buttons():
  key_in16 = GPIO.input(16)
  key_in17 = GPIO.input(17)
  key_in18 = GPIO.input(18)
  key_in19 = GPIO.input(19)
  key_in20 = GPIO.input(20)

#select button
  if (key_in16 == 1):
    return btnSELECT

  #up button
  if (key_in17 == 1):
    return btnUP

  #down button
  if (key_in18 == 1):
    return btnDOWN

  #left button
  if (key_in19 == 1):
    return btnLEFT

  #right button
  if (key_in20 == 1):
    return btnRIGHT

lcd.setCursor(0,0)
#Initial color is blue
lcd.setRGB(30,144,255)

#placeholder temperature
curTemp = 72.0
desTemp = 70.0


#Note up to only 32 characters 16 on each line
#I am unsure if the \n escape character works properly with it right now if not it can be manually circumvented

#Current assumption is that it does NOT automatically wrap to the next line and \n must be used
butPress = False

#while loop to set initial temperature target
while True:
    lcd.setCursor(0,0)
    lcd.printout("Set target temp:")
    lcd.setCursor(0,1)



    #format second line to always have 4 characters in the float XX.X
    lcd.printout("%.1f" % desTemp + " F")

    # Delay between each button press currently 1 second
    if (butPress):
        print("Desired Temp Update: " + str(desTemp))
        butPress = False
        time.sleep(0.2)

    lcd_key = read_LCD_buttons()  # Reading keys
    #Increment by 0.1 if left/right is pressed
    #Increment by 1.0 if up/down is pressed
    if (lcd_key == btnRIGHT):
        desTemp += 0.1
        butPress = True
    elif (lcd_key == btnLEFT):
        desTemp -= 0.1
        butPress = True
    elif (lcd_key == btnUP):
        desTemp += 1.0
        butPress = True
    elif (lcd_key == btnDOWN):
        desTemp -= 1.0
        butPress = True

    #Select button sets temperature and changes display green to confirm
    elif (lcd_key == btnSELECT):
        lcd.clear()
        print("Target Temp Select: " + str(desTemp))
        lcd.setRGB(124, 252, 0)
        lcd.setCursor(0, 0)
        lcd.printout("Target Temp Set:")
        lcd.setCursor(0,1)
        lcd.printout("%.1f" % desTemp + " F")
        time.sleep(2)
        lcd.clear()
        break




#while loop to read and update temperature
while True:
  lcd.setCursor(0,0)
  lcd.printout("Current Temp:")

  #Read and update current read temperature
  curTemp = read_temp_sensor()
  lcd.setCursor(0,1)
  lcd.printout("%.2f F" % curTemp)

  #lcd_key = read_LCD_buttons()  #  Reading keys
  if (curTemp > desTemp):
      lcd.setRGB(255, 0, 0)
  else:
      lcd.setRGB(124, 252, 0)




"""
    #Read the button presses then display accordingly
  if (lcd_key == btnRIGHT):
    lcd.print("RIGHT ")
  elif (lcd_key == btnLEFT):
    lcd.print("LEFT  ")
  elif (lcd_key == btnUP):
    lcd.print("UP    ")
  elif (lcd_key == btnDOWN):
    lcd.print("DOWN  ")
  elif (lcd_key == btnSELECT):
    lcd.print("SELECT")
"""

