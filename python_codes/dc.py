from adafruit_pca9685 import PCA9685
import board
import busio
import time
import controller
V=controller.Vdc
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 1000  # Set PWM frequency (e.g., 60Hz for servos)

# Set channel 0 duty cycle to 50% (approx 2048/4096)
pca.channels[0].duty_cycle = 0x0000  # 16-bit value, 0xFFFF = full on
pca.channels[1].duty_cycle = 0x0000  # 16-bit value, 0xFFFF = full on
time.sleep(2)
