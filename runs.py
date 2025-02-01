# pylint: disable=C0114
# pylint: disable=C0116
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Color, Direction, Port, Stop
from pybricks.tools import hub_menu
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# Hardware definitions =================================================================
WHEEL_DIAMETER = 62.4

hub = PrimeHub()
left_wheel = Motor(Port.C)
right_wheel = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
left_arm = Motor(Port.E)
right_arm = Motor(Port.D)
run_sensor = ColorSensor(Port.F)
map_sensor = ColorSensor(Port.A)
wheels = DriveBase(
    left_wheel, right_wheel, wheel_diameter=WHEEL_DIAMETER, axle_track=129.4
)
wheels.use_gyro(True)
pi = 3.1415926535898
RUN_RED = Color(h=339, s=85, v=94)
RUN_GREEN = Color(h=154, s=77, v=52)
DARK_BLUE_MAT = Color(h=210, s=37, v=32)
RIZZ_BLACK = Color(h=189, s=27, v=31)
RUN_YELLOW = Color(h=41, s=68, v=100)
whale_blue = Color(h=202, s=62, v=60)
sea_blue = Color(h=203, s=67, v=57)
RUN_COLORS = [Color.BLACK, RUN_RED, RUN_YELLOW, RUN_GREEN, Color.WHITE]

COLOR_LIST = [
    Color.BLACK,
    Color.WHITE,
    Color.BLUE,
    Color.GREEN,
    Color.RED,
    Color.YELLOW,
    Color.WHITE,
    DARK_BLUE_MAT,
    RIZZ_BLACK,
    RUN_YELLOW,
    whale_blue,
    sea_blue,
]


map_sensor.detectable_colors(COLOR_LIST)
run_sensor.detectable_colors(RUN_COLORS)
# print(run_sensor.hsv)


# Functions ============================================================================
def gyro_abs(target, base_speed, kp=0.16, then: Stop = Stop.HOLD):
    STOP_RANGE = 0.05
    while not (
        hub.imu.heading() - STOP_RANGE < target < hub.imu.heading() + STOP_RANGE
    ):
        error = (target - (hub.imu.heading() % 360)) % 360
        speed = base_speed + (error * kp)
        direction = "left" if error > 180 else "right"
        if direction == "left":
            left_wheel.dc(-speed)
            right_wheel.dc(speed)
        else:
            left_wheel.dc(speed)
            right_wheel.dc(-speed)

    if then == Stop.HOLD:
        left_wheel.hold()
        right_wheel.hold()
    else:
        wheels.brake()


def reset():
    hub.imu.reset_heading(0)
    right_wheel.reset_angle(0)
    left_wheel.reset_angle(0)
    right_arm.reset_angle(0)
    left_arm.reset_angle(0)
    wheels.settings(600, 300)


def deg_to_mm(deg: int | float) -> float:
    distance = (pi * WHEEL_DIAMETER * deg) / 360
    return distance


def gyro_follow_PID(
    target_distance, target_angle, base_speed=40, kp=1.6, ki=0.0025, kd=1
):
    right_wheel.reset_angle(0)
    left_wheel.reset_angle(0)
    right_wheel.dc(base_speed)
    left_wheel.dc(base_speed)
    error_sum = 0
    last_error = 0
    distance_traveled = (
        deg_to_mm(right_wheel.angle()) + deg_to_mm(left_wheel.angle())
    ) / 2
    while distance_traveled < target_distance:
        error = target_angle - hub.imu.heading()  # p
        error_sum += error  # i
        error_change = error - last_error  # d
        # fix:
        speed_change = error * kp + error_sum * ki - error_change * kd
        right_wheel.dc(base_speed - speed_change)
        left_wheel.dc(base_speed + speed_change)
        wait(5)
        last_error = error
        distance_traveled = (
            deg_to_mm(right_wheel.angle()) + deg_to_mm(left_wheel.angle())
        ) / 2


# print(map_sensor.hsv())


# Runs =================================================================================
def black_run():
    reset()
    hub.display.number(1)
    # wheels.settings(600, 600)
    # wheels.straight(-10000)

    wheels.settings(turn_rate=100)
    left_arm.run_angle(speed=100, rotation_angle=-90, wait=False)  # pick up stuff
    wheels.straight(-500)
    left_arm.run_angle(speed=100, rotation_angle=90)
    wheels.straight(-90)
    wheels.turn(angle=90 - 20.5)
    right_arm.run_angle(800, 500, wait=False)
    wheels.straight(185)  # M01
    right_arm.run_angle(800, -350)  # M04
    wheels.straight(-150)
    right_arm.run_angle(600, -100)
    wheels.turn(43)
    wheels.settings(900, 1000)  # M02
    wheels.straight(250)
    wheels.settings(300, 750)
    wheels.straight(-260)
    wheels.turn(58)
    right_arm.run_angle(250, 160)
    wheels.straight(80)
    wheels.turn(-15)
    # right_arm.run_angle(250, 450)
    right_arm.run_time(250, 1000)
    wheels.straight(55)
    wheels.straight(-110)
    left_wheel.run_angle(240, 120)
    wheels.settings(1000, 800)
    wheels.straight(-800)
    wheels.turn(180)


def red_run():

    reset()
    hub.display.number(2)
    wheels.settings(500, 250)
    wheels.straight(415)
    # gyro_abs(45, 25)
    wheels.turn(45)
    wheels.settings(500, 100)  # slows down so the thingy doesn't fall back
    wheels.straight(280)

    # Turn to trident
    left_wheel.run_angle(360, 165, then=Stop.NONE)
    wheels.settings(300, 600)
    wheels.straight(70)  # Drive to trident

    # Pick up trident
    left_arm.run_angle(speed=750, rotation_angle=-220)
    wait(500)
    left_arm.run_time(speed=150, time=1800)
    wait(350)
    wheels.straight(80)
    wheels.turn(-40)
    wait(350)
    wheels.settings(straight_speed=300)
    wheels.straight(10000, then=Stop.HOLD, wait=False)
    while map_sensor.color() != RIZZ_BLACK:
        pass
    wheels.stop()
    # wait(300)
    wheels.straight(100)
    wheels.settings(300, 600)
    wheels.turn(-90)
    wheels.straight(460)
    wheels.turn(-90)
    wheels.straight(240)
    wheels.straight(-205)
    wheels.turn(90)
    wheels.settings(1000, 8000)
    wheels.straight(350)
    wheels.settings(200, 200)
    wheels.straight(-120)
    # go home ._.
    wheels.settings(900, 800)

    wheels.turn(-90)
    wheels.straight(400)
    wheels.curve(480, -110, then=Stop.NONE)
    wheels.straight(100)
    # wheels.turn(35)
    # wheels.straight(75)
    # wheels.turn(-13)
    # wheels.straight(150)
    # finished angler fish m05
    # wheels.turn(92)
    # wheels.straight(130)
    # wheels.settings(200)
    # wheels.straight(50)
    # wheels.settings(250, 250)
    # right_arm.run_angle(600, -400)

    # wheels.settings(turn_rate=300, turn_acceleration=500)
    # wheels.turn(40)
    # wheels.straight(-200)
    # wheels.straight(500)
    # # Turn to anglerfish
    # print("turn to anglerfish")
    # # wheels.straight(15, Stop.NONE)
    # wheels.curve(180, -95, Stop.NONE)
    # wheels.straight(55, Stop.NONE)
    # wheels.curve(180, -80)

    # # Drive to anglerfish
    # print("drive to anglerfish")
    # wheels.straight(80)

    # # Drive to seabed sample
    # print("drive to seabed sample")
    # wheels.straight(-50)
    # wheels.turn(65)


# def run_angle_time_limit(motor, rotate_speed, rotate_angle, stop_type=Stop.HOLD, time_limit_ms = 3):
#     motor.run_angle(motor, rotate_speed, rotate_angle, stop_type, wait=False)
#     run_time = StopWatch()
#     print(motor.done())
#     motor.stop_type()


def yellow_run():
    reset()
    right_arm.hold()
    hub.display.number(3)
    wheels.settings(straight_speed=200, straight_acceleration=200)
    wheels.straight(280)
    right_arm.run_time(speed=-200, time=900)
    # get in position to grab boat -------
    right_arm.run_time(speed=200, time=1000)
    right_arm.hold()

    # --------------------------------------
    wheels.straight(440, then=Stop.NONE)
    wheels.straight(190)
    wheels.settings(straight_speed=60, straight_acceleration=60)
    wheels.straight(-150)

    wait(500)
    wheels.settings(straight_speed=150)
    right_arm.hold()
    wheels.straight(-150)
    wheels.settings(straight_speed=500, straight_acceleration=200)
    wheels.turn(-45)
    wheels.straight(170, wait=False)
    right_arm.run_time(speed=100, time=1000)
    right_arm.hold()
    wheels.settings(straight_speed=800, straight_acceleration=300)
    wheels.settings(straight_speed=900, straight_acceleration=800)
    wheels.curve(360, 80, then=Stop.NONE)
    wheels.curve(400, -70)


# while True:
#     print(print(map_sensor.hsv()))


def green_run():
    reset()
    hub.display.number(4)
    wheels.settings(300, 500)
    wheels.straight(445)  # pick up tamnoon
    # wheels.settings(turn_acceleration=60)
    wheels.straight(-20)
    # go to green circle
    wheels.turn(135)
    wheels.straight(50)
    wheels.turn(-80)
    wheels.straight(330)
    wheels.turn(-95)
    wheels.straight(210)

    # complete green circle
    right_arm.run_time(speed=-360, time=2000)
    right_arm.run_time(speed=600, time=2000)
    # leave the tamnoon in the circle
    wheels.straight(100)
    wheels.turn(-30)
    wheels.straight(100)
    wheels.settings(straight_speed=1000)
    wheels.curve(radius=-300, angle=-90, then=Stop.NONE)
    wheels.straight(-500)


def white_run():
    hub.display.number(5)
    wheels.settings(600, 600)
    wheels.straight(300)
    wheels.turn(50)
    wheels.straight(425)
    wheels.turn(34)
    wheels.settings(300)
    wheels.straight(10000, then=Stop.HOLD, wait=False)
    while map_sensor.reflection() > 20:
        pass
    wheels.stop()
    wheels.straight(60)
    right_arm.run_angle(speed=100, rotation_angle=50)
    wait(100)
    wheels.straight(-200)


def run_straight():
    wheels.settings(600, 600)
    wheels.straight(100000)


selected = run_sensor.color()
print(run_sensor.color())
if selected == Color.BLACK:
    black_run()
elif selected == Color.WHITE:
    white_run()
elif selected == RUN_RED:
    red_run()
elif selected == RUN_YELLOW:
    yellow_run()
elif selected == RUN_GREEN:
    green_run()


# selected = hub_menu("0", "1")
# if selected == "0":
#     if run_sensor.color() == Color.BLACK:
#         print(run_sensor.color())
#         black_run()
#     elif run_sensor.color() == Color(h=339, s=85, v=94):
#         red_run()
#         print(run_sensor.color())
#     elif run_sensor.color() == Color.YELLOW:
#         yellow_run()
#         print(run_sensor.color())

# if run_sensor.color() == Color.BLACK:
#     black_run()
# else:
#     run_straight()
