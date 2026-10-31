#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Омни-платформа на EV3 (ev3dev2), 3 колеса, гироскоп.
#
# Колёса (вид сверху, перёд вверху):
#
#     B (левое переднее)     C (правое переднее)
#                  \         /
#                   \       /
#                      A (заднее)
#
# Направление езды в градусах: 0 = вперёд, 90 = вправо, 180 = назад, -90 = влево
# Поворот: по часовой плюс, против часовой минус.
#
# Курс платформы хранится в heading_target и всегда удерживается.
# turn_by(90) просто сдвигает курс на 90, а езда держит этот курс.
# heading_off() выключает удержание, heading_on() включает обратно.

import time
import math

from ev3dev2.motor import MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import GyroSensor

# ---------------------------------------------------------------------
# НАСТРОЙКИ
# ---------------------------------------------------------------------

# Если колесо крутится не в ту сторону, поставь -1
DIR_REAR = 1
DIR_FRONT_LEFT = 1
DIR_FRONT_RIGHT = 1

# Подстройка силы колёс. Если колесо слабее остальных, увеличь его коэффициент.
# Например, при езде вбок передние колёса получают вдвое меньше мощности, чем заднее,
# и на малой мощности могут крутиться хуже, чем надо.
GAIN_REAR = 1.0
GAIN_FRONT_LEFT = 1.0
GAIN_FRONT_RIGHT = 1.0

HOLD_KP = 1.5       # коэффициент удержания курса при езде
HOLD_KI = 1.0       # интегральная часть: убирает постоянный увод корпуса при езде
INTEGRAL_MAX = 15   # ограничение интеграла, чтобы он не раскачивал платформу
RAMP_TIME = 0.4     # плавный разгон при езде, секунды (0 = без разгона)
TURN_KP = 1.2       # коэффициент поворота на месте
TURN_MIN = 12       # минимальная мощность поворота
TURN_MAX = 60       # максимальная мощность поворота
TURN_TOLERANCE = 2  # допустимая ошибка угла в градусах

heading_target = 0      # куда платформа должна смотреть (градусы)
heading_hold = True     # True = курс удерживается

motor_rear = MediumMotor(OUTPUT_A)
motor_front_left = MediumMotor(OUTPUT_B)
motor_front_right = MediumMotor(OUTPUT_C)

gyro = GyroSensor(INPUT_1)


# ---------------------------------------------------------------------
# ГИРОСКОП И КУРС
# ---------------------------------------------------------------------

def reset_gyro():
    # Обнулить угол и курс. Платформа должна стоять неподвижно.
    global heading_target
    gyro.mode = 'GYRO-RATE'
    gyro.mode = 'GYRO-ANG'
    time.sleep(0.5)
    heading_target = 0


def get_angle():
    # Текущий угол. Датчик по часовой даёт минус, поэтому знак переворачиваем.
    return -gyro.angle


def angle_diff(target, current):
    # Разница углов в диапазоне от -180 до 180 (плюс = надо повернуть по часовой)
    diff = target - current
    while diff > 180:
        diff = diff - 360
    while diff < -180:
        diff = diff + 360
    return diff


def heading_on():
    # Включить удержание курса. Курсом становится текущий угол.
    global heading_hold, heading_target
    heading_target = get_angle()
    heading_hold = True


def heading_off():
    # Выключить удержание курса и остановить моторы
    global heading_hold
    heading_hold = False
    stop()


# ---------------------------------------------------------------------
# МОТОРЫ
# ---------------------------------------------------------------------

def stop():
    motor_rear.off(brake=True)
    motor_front_left.off(brake=True)
    motor_front_right.off(brake=True)


def drive(vx, vy, w):
    # vx: вправо (+) / влево (-)
    # vy: вперёд (+) / назад (-)
    # w:  поворот по часовой (+) / против часовой (-)
    # Все значения в процентах мощности.
    # Колёса стоят под 120 градусов: 0.5 это cos(60), 0.866 это sin(60)
    rear = vx - w
    front_left = -0.5 * vx - 0.866 * vy - w
    front_right = -0.5 * vx + 0.866 * vy - w

    # Подстройка силы колёс
    rear = rear * GAIN_REAR
    front_left = front_left * GAIN_FRONT_LEFT
    front_right = front_right * GAIN_FRONT_RIGHT

    # Если что-то больше 100, уменьшаем все пропорционально
    biggest = max(abs(rear), abs(front_left), abs(front_right))
    if biggest > 100:
        scale = 100.0 / biggest
        rear = rear * scale
        front_left = front_left * scale
        front_right = front_right * scale

    motor_rear.on(rear * DIR_REAR)
    motor_front_left.on(front_left * DIR_FRONT_LEFT)
    motor_front_right.on(front_right * DIR_FRONT_RIGHT)


def drive_angle(direction, speed, w=0):
    # Ехать в направлении direction (0 = вперёд, 90 = вправо) со скоростью speed
    rad = math.radians(direction)
    drive(speed * math.sin(rad), speed * math.cos(rad), w)


# ---------------------------------------------------------------------
# УДЕРЖАНИЕ КУРСА
# ---------------------------------------------------------------------

def hold_step():
    # Один шаг удержания курса на месте (П-регулятор).
    # Возвращает мощность поворота, 0 = курс уже в пределах допуска.
    error = angle_diff(heading_target, get_angle())

    if abs(error) <= TURN_TOLERANCE:
        stop()
        return 0

    w = TURN_KP * error

    # не больше TURN_MAX и не меньше TURN_MIN, иначе на малых ошибках колёса не трогаются
    if w > TURN_MAX:
        w = TURN_MAX
    if w < -TURN_MAX:
        w = -TURN_MAX
    if abs(w) < TURN_MIN:
        if error > 0:
            w = TURN_MIN
        else:
            w = -TURN_MIN

    drive(0, 0, w)
    return w


def hold_heading(seconds=None):
    # Держать курс заданное время. Без аргумента держит бесконечно.
    # Используй вместо time.sleep между командами.
    start = time.time()
    while seconds is None or time.time() - start < seconds:
        if heading_hold:
            hold_step()
        else:
            stop()
        time.sleep(0.01)


def settle(timeout=5):
    # Доворачивать до курса, пока ошибка не станет меньше допуска
    start = time.time()
    while time.time() - start < timeout:
        if hold_step() == 0:
            break
        time.sleep(0.01)
    stop()


# ---------------------------------------------------------------------
# КОМАНДЫ ДВИЖЕНИЯ
# ---------------------------------------------------------------------

def turn_by(delta):
    # Повернуться на delta градусов. Курс сдвигается на delta
    # (от прошлого курса, а не от того, где встали, поэтому ошибки не копятся).
    # Если удержание выключено, считаем от текущего угла.
    global heading_target
    if heading_hold:
        heading_target = heading_target + delta
    else:
        heading_target = get_angle() + delta
    settle()


def move_for_time(direction, speed, seconds):
    # Ехать в направлении direction заданное время, держа курс.
    # В конце доворачиваем до курса. Если удержание выключено, едем без поправок.
    if not heading_hold:
        drive_angle(direction, speed, 0)
        time.sleep(seconds)
        stop()
        return

    start = time.time()
    last = start
    integral = 0
    while time.time() - start < seconds:
        now = time.time()
        dt = now - last
        last = now

        error = angle_diff(heading_target, get_angle())

        # Интеграл ошибки: если корпус постоянно уводит в одну сторону,
        # он понемногу нарастает и гасит этот увод.
        integral = integral + error * dt
        if integral > INTEGRAL_MAX:
            integral = INTEGRAL_MAX
        if integral < -INTEGRAL_MAX:
            integral = -INTEGRAL_MAX

        # Плавный разгон: в начале платформу сильнее всего уводит в сторону,
        # потому что колёса разгоняются не одновременно.
        ramp = 1.0
        if RAMP_TIME > 0:
            ramp = (now - start) / RAMP_TIME
            if ramp > 1:
                ramp = 1.0
        current_speed = speed * ramp

        w = HOLD_KP * error + HOLD_KI * integral

        # Если корпус довернуло на угол, направление езды поправляем на столько же
        # в обратную сторону. Тогда путь остаётся прямым относительно комнаты.
        drive_angle(direction + error, current_speed, w)
        time.sleep(0.01)
    stop()
    settle()


def turn_and_go(turn, seconds, speed=50, direction=0):
    # Повернуться на turn градусов и ехать seconds секунд.
    # Примеры:
    #   turn_and_go(90, 2)       повернуться на 90 вправо и ехать вперёд 2 секунды
    #   turn_and_go(0, 3, 40)    ехать вперёд 3 секунды на скорости 40
    #   turn_and_go(-90, 0)      только повернуться на 90 влево
    turn_by(turn)
    if seconds > 0:
        move_for_time(direction, speed, seconds)


def check_strafe(speed=40, seconds=2):
    # Диагностика езды вправо БЕЗ удержания курса.
    # Печатает угол корпуса и реальную скорость колёс (градусы в секунду).
    # При чистом движении вбок передние колёса должны крутиться ровно вдвое
    # медленнее заднего и в обратную сторону. Угол должен оставаться около 0.
    reset_gyro()
    start = time.time()
    drive_angle(90, speed)
    while time.time() - start < seconds:
        print("t={:.1f} angle={} rear={} FL={} FR={}".format(
            time.time() - start,
            get_angle(),
            motor_rear.speed * DIR_REAR,
            motor_front_left.speed * DIR_FRONT_LEFT,
            motor_front_right.speed * DIR_FRONT_RIGHT))
        time.sleep(0.2)
    stop()



SPEED_CM_S_PER_PCT = 0.5   # калибровка: см/с на 1% мощности
W_PER_DEG_S = 0.4          # калибровка: % мощности поворота на 1 град/с


def move_circle(radius, speed, degrees=360, clockwise=True,
                face_tangent=False, start_dir=0):
    # Ехать по дуге радиуса radius (см) на скорости speed (%).
    # face_tangent=False: корпус держит курс, вращается направление езды.
    # face_tangent=True: нос смотрит по касательной, вращается сам курс.
    global heading_target
    sign = 1 if clockwise else -1
    v = speed * SPEED_CM_S_PER_PCT
    omega = math.degrees(v / radius)      # град/с
    seconds = degrees / omega

    start = time.time()
    last = start
    integral = 0
    while time.time() - start < seconds:
        now = time.time()
        dt = now - last
        last = now
        t = now - start

        error = angle_diff(heading_target, get_angle())
        integral = integral + error * dt
        integral = max(-INTEGRAL_MAX, min(INTEGRAL_MAX, integral))
        w = HOLD_KP * error + HOLD_KI * integral

        if face_tangent:
            heading_target = heading_target + sign * omega * dt
            w = w + sign * omega * W_PER_DEG_S
            direction = start_dir
            ramp = 1.0
        else:
            direction = start_dir + sign * omega * t
            ramp = min(1.0, t / RAMP_TIME) if RAMP_TIME > 0 else 1.0

        drive_angle(direction + error, speed * ramp, w)
        time.sleep(0.01)
    stop()
    settle()



def orbit(radius, speed, degrees=360, clockwise=True, front_to_center=True):
    # Кружить вокруг точки, держа нос на центр (или от центра).
    # radius: расстояние от центра платформы до точки, см.
    if clockwise == front_to_center:
        side = -90     # по часовой с носом в центр, или против часовой с носом наружу
    else:
        side = 90
    move_circle(radius, speed, degrees,
                clockwise=clockwise, face_tangent=True, start_dir=side)

# ---------------------------------------------------------------------
# ЗАПУСК
# ---------------------------------------------------------------------

if __name__ == "__main__":
    try:
        # check_strafe()   # диагностика езды вправо (раскомментируй, а остальное закомментируй)

        reset_gyro()


        reset_gyro()
        orbit(50, 30)                      # круг радиусом 40 см по часовой, нос в центр
        orbit(50, 30, clockwise=False)     # обратно

        # слайд-квадрат: корпус всё время смотрит в одну сторону (курс 0),
        # а едем вперёд, вправо, назад, влево
       # turn_and_go(0, 1.5, 100, 0)      # вперёд
        

        #turn_and_go(0, 2, 80, 90 + 45)     # вправо
        #turn_and_go(0, 2, 80, -90) 


       # turn_and_go(0, 1.5, 100, 180)    # назад

       # turn_and_go(0, 3, 80, -90)    # влево
    
        # обычный квадрат с поворотами
        # for i in range(4):
        #     turn_and_go(90, 1.5, 40)

        # после квадрата бесконечно держим курс (Stop чтобы выйти)
        while True:
            hold_heading(1)
            print("angle = {}, target = {}".format(get_angle(), heading_target))
    finally:
        stop()