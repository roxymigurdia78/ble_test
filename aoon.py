import RPi.GPIO as GPIO
import time

PIN_BLUE = 19  # 青のGPIO（BCM番号）

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_BLUE, GPIO.OUT)

try:
    print("BLUE ON")
    GPIO.output(PIN_BLUE, GPIO.HIGH)  # MOSFET ON → 青LED点灯
    time.sleep(10)                    # 10秒間つけっぱなし

    print("BLUE OFF")
    GPIO.output(PIN_BLUE, GPIO.LOW)   # 消灯
    time.sleep(1)

finally:
    GPIO.cleanup()
