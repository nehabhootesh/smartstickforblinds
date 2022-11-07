#!/usr/bin/python

# Import required Python libraries
import multiprocessing
import os
import RPi.GPIO as GPIO
import pyttsx3
import time
import time
import paho.mqtt.client as mqtt
import RPi. GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.IN)
#### setting up MQTT #########
mqttc = mqtt.Client()
mqttc.connect("broker.hivemq.com", 1883,60)
mqttc.loop_start()

# We will be using the BCM GPIO numbering
GPIO.setmode(GPIO.BOARD)

# Select which GPIOs you will use
GPIO_BUZZER1 = 33
GPIO_TRIGGER1 = 35
GPIO_ECHO1 = 37
#for 2nd ultrasonic
GPIO_BUZZER2 = 36
GPIO_TRIGGER2 = 38
GPIO_ECHO2 = 40
# Set BUZZER to OUTPUT mode
GPIO.setup(GPIO_BUZZER1, GPIO.OUT)
# Set TRIGGER to OUTPUT mode
GPIO.setup(GPIO_TRIGGER1, GPIO.OUT)
# Set ECHO to INPUT mode
GPIO.setup(GPIO_ECHO1, GPIO.IN)

# Set BUZZER to OUTPUT mode
GPIO.setup(GPIO_BUZZER2, GPIO.OUT)
# Set TRIGGER to OUTPUT mode
GPIO.setup(GPIO_TRIGGER2, GPIO.OUT)
# Set ECHO to INPUT mode
GPIO.setup(GPIO_ECHO2, GPIO.IN)

# Measures the distance between a sensor and an obstacle and returns the measured value
def distance1():
  # Send 10 microsecond pulse to TRIGGER
  GPIO.output(GPIO_TRIGGER1, True) # set TRIGGER to HIGH
  time.sleep(0.00001) # wait 10 microseconds
  GPIO.output(GPIO_TRIGGER1, False) # set TRIGGER back to LOW
 
  # Create variable start and assign it current time
  start = time.time()
  # Create variable stop and assign it current time
  stop = time.time()
  # Refresh start value until the ECHO goes HIGH = until the wave is send
  while GPIO.input(GPIO_ECHO1) == 0:
    start = time.time()
 
  # Assign the actual time to stop variable until the ECHO goes back from HIGH to LOW = the wave came back
  while GPIO.input(GPIO_ECHO1) == 1:
    stop = time.time()
 
  # Calculate the time it took the wave to travel there and back
  measuredTime1 = stop - start
  # Calculate the travel distance by multiplying the measured time by speed of sound
  distanceBothWays = measuredTime1 * 33112 # cm/s in 20 degrees Celsius
  # Divide the distance by 2 to get the actual distance from sensor to obstacle
  distance1 = distanceBothWays / 2

  # Print the distance to see if everything works correctly
  print("Distance : {0:5.1f}cm".format(distance1))
  # Return the actual measured distance
  return distance1

def distance2():
  # Send 10 microsecond pulse to TRIGGER
  GPIO.output(GPIO_TRIGGER2, True) # set TRIGGER to HIGH
  time.sleep(0.00001) # wait 10 microseconds
  GPIO.output(GPIO_TRIGGER2, False) # set TRIGGER back to LOW
 
  # Create variable start and assign it current time
  start = time.time()
  # Create variable stop and assign it current time
  stop = time.time()
  # Refresh start value until the ECHO goes HIGH = until the wave is send
  while GPIO.input(GPIO_ECHO2) == 0:
    start = time.time()
 
  # Assign the actual time to stop variable until the ECHO goes back from HIGH to LOW = the wave came back
  while GPIO.input(GPIO_ECHO2) == 1:
    stop = time.time()
 
  # Calculate the time it took the wave to travel there and back
  measuredTime2 = stop - start
  # Calculate the travel distance by multiplying the measured time by speed of sound
  distanceBothWays = measuredTime2 * 33112 # cm/s in 20 degrees Celsius
  # Divide the distance by 2 to get the actual distance from sensor to obstacle
  distance2 = distanceBothWays / 2

  # Print the distance to see if everything works correctly
  print("Distance : {0:5.1f}cm".format(distance2))
  # Return the actual measured distance
  return distance2

# Calculates the frequency of beeping depending on the measured distance
def beep_freq1():
  # Measure the distance
  dist = distance1()
  # If the distance is bigger than 50cm, we will not beep at all
  if dist > 50:
    return -1
  # If the distance is between 50 and 30 cm, we will beep once a second
  elif dist <= 50 and dist >=30:
    return 1
  # If the distance is between 30 and 20 cm, we will beep every twice a second
  elif dist < 30 and dist >= 20:
    return 0.5
  # If the distance is between 20 and 10 cm, we will beep four times a second
  elif dist < 20 and dist >= 10:
    return 0.25
  # If the distance is smaller than 10 cm, we will beep constantly
  else:
    return 0

# Calculates the frequency of beeping depending on the measured distance
def beep_freq2():
  # Measure the distance
  dist = distance2()
  # If the distance is bigger than 50cm, we will  beep constantly
  if dist > 50:
    return 0
  # If the distance is between 50 and 30 cm, we will beep four times a second
  elif dist <= 50 and dist >=30:
    return 0.25
  # If the distance is between 30 and 20 cm, we will beep every twice a second
  elif dist < 30 and dist >= 20:
    return 0.5
  # If the distance is between 20 and 10 cm, we will beep once a second
  elif dist < 20 and dist >= 10:
    return 1
  # If the distance is smaller than 10 cm, we will beep constantly
  else:
    return -1

# Main function
def main():
  try:
    import time
    # Repeat till the program is ended by the user
    while True:
      Data=GPIO.input(11)
      if (Data > 0):
        import serial
        import time
        import string
        import pynmea2
        port="/dev/ttyAMA0"
        ser=serial.Serial(port, baudrate=9600, timeout=0.5)
        dataout = pynmea2.NMEAStreamReader()
        newdata=ser.readline()

        if newdata[0:6] == "$GPRMC":
                newmsg=pynmea2.parse(newdata)
                lat=newmsg.latitude
                lng=newmsg.longitude
                gps = "Latitude=" + str(lat) + "and Longitude=" + str(lng)
                print(gps)
                (result,mid) = mqttc.publish("paho/location",gps,2)
                print("Switch is pressed ")
                time.sleep(1)
      # Get the beeping frequency
      freq1 = beep_freq1()
      # No beeping
      if freq1 == -1:
        GPIO.output(GPIO_BUZZER1, False)
        time.sleep(0.25)
      # Constant beeping
      elif freq1 == 0:
        engine = pyttsx3.init()
        # convert this text to speech
        text = "Front obstacle"
        engine.say(text)
# play the speech
        engine.runAndWait()
        GPIO.output(GPIO_BUZZER1, True)
        time.sleep(0.25)
      # Beeping on certain frequency
      else:
        GPIO.output(GPIO_BUZZER1, True)
        time.sleep(0.2) # Beep is 0.2 seconds long
        GPIO.output(GPIO_BUZZER1, False)
        time.sleep(freq1) 
        # Pause between beeps = beeping frequency
      freq2 = beep_freq2()
      # No beeping
      if freq2 == -1:
        GPIO.output(GPIO_BUZZER2, False)
        time.sleep(0.25)
      # Constant beeping
      elif freq2 == 0:
        GPIO.output(GPIO_BUZZER2, True)
        time.sleep(0.25)
      # Beeping on certain frequency
      else:
        GPIO.output(GPIO_BUZZER2, True)
        time.sleep(0.2) # Beep is 0.2 seconds long
        GPIO.output(GPIO_BUZZER2, False)
        time.sleep(freq2)
        # mqttc.loop_stop()
        # mqttc.disconnect()
  # If the program is ended, stop beeping and cleanup GPIOs
  except KeyboardInterrupt:
    GPIO.output(GPIO_BUZZER1, False)
    GPIO.output(GPIO_BUZZER2, False)
    GPIO.cleanup()
  

# Run the main function when the script is executed
if __name__ == "__main__":
    main()