import RPi.GPIO as GPIO
import time
import colorsys  # HSV → RGB 変換に使う（標準ライブラリ）

# GPIO (BCM番号)
RED = 12
GREEN = 13
BLUE = 19

FREQ = 500  # PWM周波数(Hz)

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(RED, GPIO.OUT)
    GPIO.setup(GREEN, GPIO.OUT)
    GPIO.setup(BLUE, GPIO.OUT)

    pwm_r = GPIO.PWM(RED, FREQ)
    pwm_g = GPIO.PWM(GREEN, FREQ)
    pwm_b = GPIO.PWM(BLUE, FREQ)

    pwm_r.start(0)
    pwm_g.start(0)
    pwm_b.start(0)

    return pwm_r, pwm_g, pwm_b

def set_color(pwm_r, pwm_g, pwm_b, r, g, b):
    """r,g,b は 0〜100 (%)"""
    pwm_r.ChangeDutyCycle(r)
    pwm_g.ChangeDutyCycle(g)
    pwm_b.ChangeDutyCycle(b)

def main():
    pwm_r, pwm_g, pwm_b = setup()

    # LEDごとの明るさ補正（明るさバランス悪かったらここで調整）
    SCALE_R = 100
    SCALE_G = 100
    SCALE_B = 100

    try:
        hue = 0.0  # 0.0〜1.0 をぐるぐる回す
        while True:
            # 彩度S=1, 明度V=1 で「一番鮮やかな色」
            r_f, g_f, b_f = colorsys.hsv_to_rgb(hue, 1.0, 1.0)  # 0.0〜1.0 のfloat

            # 0〜100% に変換してスケール
            r = min(100, r_f * 100 * SCALE_R / 100)
            g = min(100, g_f * 100 * SCALE_G / 100)
            b = min(100, b_f * 100 * SCALE_B / 100)

            set_color(pwm_r, pwm_g, pwm_b, r, g, b)

            # hue を少しずつ増やして一周させる
            hue += 0.002   # ここを小さく→ゆっくり、大きく→早く
            if hue >= 1.0:
                hue -= 1.0

            time.sleep(0.01)  # ここを変えて滑らかさ・スピード調整

    except KeyboardInterrupt:
        pass
    finally:
        pwm_r.stop()
        pwm_g.stop()
        pwm_b.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
