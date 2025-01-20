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
pi = 3.1416
RUN_RED = Color(h=339, s=85, v=94)
RUN_GREEN = Color(h=154, s=77, v=52)
DARK_BLUE_MAT = Color(h=210, s=37, v=32)
RIZZ_BLACK = Color(h=189, s=27, v=31)
RUN_COLORS = [Color.BLACK, RUN_RED, Color.YELLOW, RUN_GREEN, Color.WHITE]

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
    wheels.straight(65)
    wheels.turn(-15)
    right_arm.run_angle(250, 450)
    wheels.straight(40)
    wheels.straight(-110)
    left_wheel.run_angle(240, 120)
    wheels.straight(-1000)


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
    wheels.straight(100)
    wheels.settings(300, 600)
    wheels.turn(-90)
    wheels.straight(455)
    wheels.turn(-90)
    wheels.straight(240)
    wheels.straight(-205)
    wheels.turn(90)
    wheels.settings(1000, 1000)
    wheels.straight(350)
    wheels.settings(300, 600)
    wheels.straight(-250)


def yellow_run():
    reset()
    right_arm.hold()
    hub.display.number(3)
    wheels.settings(straight_speed=200, straight_acceleration=200)
    wheels.straight(220)
    # right_arm.run_angle_time_limit(rotate_speed=200, rotate_angle=-170)
    right_arm.run_time(-200, 1000)
    wheels.straight(300)
    wheels.straight(-20)
    right_arm.run_target(speed=200, rotation_angle=220)
    wheels.straight(400)
    right_arm.run_angle(speed=200, rotation_angle=-220)
    wheels.settings(200, 200)
    wheels.straight(-400)
    wheels.straight(700)
    print("done")


def green_run():
    reset()
    hub.display.number(4)
    wheels.settings(300, 600)
    wheels.straight(450)  # pick up tamnoon
    wheels.straight(-50)
    # go to green circle
    wheels.turn(135)
    wheels.turn(-90)
    wheels.straight(350)
    wheels.turn(-90)
    # wheels.straight(20)
    # complete green circle
    right_arm.run_time(speed=-360, time=1500)
    right_arm.run_time(speed=360, time=1500)
    # leave the tamnoon in the circle
    wheels.straight(285)


def white_run():
    hub.display.number(1)
    wheels.settings(600, 600)
    wheels.straight(300)
    wheels.turn(50)
    wheels.straight(425)
    wheels.turn(34)
    wheels.settings(300)
    wheels.straight(10000, then=Stop.HOLD, wait=False)
    while map_sensor.color() != RIZZ_BLACK:
        pass
    wheels.stop()
    wheels.straight(60)
    right_arm.run_angle(speed=100, rotation_angle=50)


def run_straight():
    wheels.settings(600, 600)
    wheels.straight()


selected = run_sensor.color()
print(run_sensor.color())
if selected == Color.BLACK:
    black_run()
elif selected == Color.WHITE:
    white_run()
elif selected == RUN_RED:
    red_run()
elif selected == Color.YELLOW:
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
