import RPi.GPIO as GPIO
import time

PIN_RED = 12  # 赤のGPIO（BCM番号）

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_RED, GPIO.OUT)

try:
    print("RED ON")
    GPIO.output(PIN_RED, GPIO.HIGH)  # MOSFET ON → 赤LED点灯
    time.sleep(10)                   # 10秒間つけっぱなし

    print("RED OFF")
    GPIO.output(PIN_RED, GPIO.LOW)   # 消灯
    time.sleep(1)

finally:
    GPIO.cleanup()
