import sys
import RPi.GPIO as GPIO
import time

# ピン配置 (BCM番号)
RED_PIN = 12
GREEN_PIN = 13
BLUE_PIN = 19

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(RED_PIN, GPIO.OUT)
    GPIO.setup(GREEN_PIN, GPIO.OUT)
    GPIO.setup(BLUE_PIN, GPIO.OUT)

def all_off():
    GPIO.output(RED_PIN, GPIO.LOW)
    GPIO.output(GREEN_PIN, GPIO.LOW)
    GPIO.output(BLUE_PIN, GPIO.LOW)

def main():
    if len(sys.argv) < 2:
        return

    command = sys.argv[1].lower()
    setup()

    if command == "blue":
        all_off()
        GPIO.output(BLUE_PIN, GPIO.HIGH)
    elif command == "red":
        all_off()
        GPIO.output(RED_PIN, GPIO.HIGH)
    elif command == "green":
        all_off()
        GPIO.output(GREEN_PIN, GPIO.HIGH)
    elif command == "off":
        all_off()
    
    # 状態を保持するため、Pythonスクリプトは終了するが
    # GPIOの状態はCleanupしない限り維持される
    # (終了時に GPIO.cleanup() を呼ばないのがポイント)

if __name__ == "__main__":
    main()
