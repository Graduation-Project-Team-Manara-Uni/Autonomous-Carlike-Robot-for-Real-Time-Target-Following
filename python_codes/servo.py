from adafruit_servokit import ServoKit
import time
import controller
ang=controller.psid
kit = ServoKit(channels=16)
kit.servo[0].angle=50
time.sleep(2)
kit.servo[0].angle = 95  # Try channel 0 first
time.sleep(2)
kit.servo[0].angle = 115
time.sleep(2)
kit.servo[0].angle=85
time.sleep(2)

