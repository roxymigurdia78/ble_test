import RPi.GPIO as GPIO
import time

PIN = 13  # 緑：GPIO13（BCM番号）

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN, GPIO.OUT)

try:
    print("GREEN ON")
    GPIO.output(PIN, GPIO.HIGH)  # ゲートを3.3Vにする → MOSFET ON → LED点灯のはず
    time.sleep(10)               # 10秒間つけっぱなし

    print("GREEN OFF")
    GPIO.output(PIN, GPIO.LOW)   # 消灯
    time.sleep(1)

finally:
    GPIO.cleanup()
