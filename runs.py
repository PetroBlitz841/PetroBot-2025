# pylint: disable=C0114, C0116, W0611, C0103, R0913, E1123, E1111

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Color, Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch


pi = 3.1416
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

# color definitions =================================================================

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


# Utility Functions ============================================================================
def gyro_abs(target, base_speed, kp=0.16, then: Stop = Stop.HOLD):
    """turns the robot to a given angle relative to the launch angle"""
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
    """converts the motor's degrees to the robot's distance"""
    hub.imu.reset_heading(0)
    right_wheel.reset_angle(0)
    left_wheel.reset_angle(0)
    right_arm.reset_angle(0)
    left_arm.reset_angle(0)
    wheels.settings(600, 300)


def deg_to_mm(deg: int | float) -> float:
    """converts the motor's degrees to the robot's distance"""
    distance = (pi * WHEEL_DIAMETER * deg) / 360
    return distance


def gyro_follow_PID(
    target_distance, target_angle, base_speed=40, kp=1.6, ki=0.0025, kd=1
):
    """function for driving straight
    kp - proportinal correction multiplyer
    ki - integral correction multiplyer
    kd - derivative correction multiplyer
    """
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
        # correct the error:
        speed_change = error * kp + error_sum * ki - error_change * kd
        right_wheel.dc(base_speed - speed_change)
        left_wheel.dc(base_speed + speed_change)
        wait(5)
        last_error = error
        distance_traveled = (
            deg_to_mm(right_wheel.angle()) + deg_to_mm(left_wheel.angle())
        ) / 2


# Runs =================================================================================
def black_run():
    reset()
    hub.display.number(1)

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
    wheels.settings(1000, 2000)  # M02
    wheels.straight(250)
    wheels.settings(300, 5000)
    wheels.straight(-260)
    wheels.turn(58)
    right_arm.run_angle(250, 160)
    wheels.straight(80)
    wheels.turn(-15)
    right_arm.run_time(250, 1000)
    wheels.straight(55)
    wheels.straight(-110)
    left_wheel.run_angle(240, 135)
    wheels.settings(1000, 800)
    wheels.straight(-800)
    wheels.turn(180)


def red_run():

    reset()
    hub.display.number(2)
    wheels.settings(500, 250)
    wheels.straight(380)
    wheels.turn(45)
    wheels.settings(500, 100)  # slows down so the thingy doesn't fall back
    wheels.straight(280)

    # Turn to trident
    left_wheel.run_angle(360, 165, then=Stop.NONE)
    wheels.settings(300, 600)
    wheels.straight(80)  # Drive to trident

    # Pick up trident
    left_arm.run_angle(speed=750, rotation_angle=-220)
    wait(500)
    left_arm.run_time(speed=150, time=1800)
    wait(350)
    wheels.straight(80)
    wheels.turn(-38)
    wait(350)
    wheels.settings(straight_speed=300)
    wheels.straight(1000, then=Stop.HOLD, wait=False)
    while map_sensor.color() != RIZZ_BLACK:
        pass
    left_wheel.hold()
    right_wheel.hold()
    wheels.straight(100)
    wheels.settings(300, 600)
    wheels.turn(-90)
    wheels.straight(460)

    # turn to dag haca

    wheels.turn(-90)

    wheels.straight(236)
    wheels.straight(-180)

    # #turn to tzollet
    wheels.turn(45)
    wheels.straight(170)
    wheels.turn(-45)
    wheels.straight(125)
    right_arm.run_angle(250, -90)
    right_arm.run_angle(250, 90)
    wheels.straight(40)
    right_arm.run_angle(250, -90)
    right_arm.run_angle(250, 90)
    right_wheel.run_angle(350, 160)
    wheels.straight(60)
    left_wheel.run_angle(350, 160)

    # wheels.straight(190)
    # right_arm.run_time(350, 300)
    # wheels.turn(-45)
    wheels.settings(1000, 750)
    # wheels.straight(270)
    right_arm.dc(5)
    wheels.curve(500, -90, then=Stop.NONE)
    wheels.straight(3000)


def yellow_run():
    reset()
    right_arm.hold()
    hub.display.number(3)

    wheels.settings(straight_speed=500, straight_acceleration=200)
    wheels.straight(280)
    right_arm.run_time(speed=-200, time=900)
    # get in position to grab boat:
    right_arm.run_time(speed=500, time=1000)
    right_arm.hold()

    # --------------------------------------
    wheels.straight(440, then=Stop.NONE)
    wheels.straight(190)
    wheels.settings(straight_speed=400, straight_acceleration=60)
    wheels.straight(-150)

    wait(500)
    # wheels.settings(straight_speed=400)
    right_arm.hold()
    wheels.straight(-170)
    wheels.settings(straight_speed=500, straight_acceleration=500)
    wheels.turn(-45)
    wheels.straight(170, wait=False)
    right_arm.run_time(speed=100, time=1000)
    right_arm.hold()
    wheels.settings(straight_speed=800, straight_acceleration=300)

    wheels.curve(390, 60)
    wheels.straight(-100)
    left_arm.run_angle(1000, -100)
    wheels.settings(1000, 2000)
    wheels.straight(1000)
    # wheels.curve(400, -70)
    # wheels.straight(2345)


def green_run():
    reset()
    right_arm.hold()
    hub.display.number(4)
    wheels.settings(straight_speed=500, straight_acceleration=400)
    right_arm.hold()
    wheels.straight(501)
    wheels.straight(-185)
    wheels.turn(-50)
    wheels.straight(260)
    wheels.turn(50)
    wheels.straight(170)
    wheels.turn(50)
    wheels.straight(630, Stop.HOLD)  # to wall
    right_arm.run_angle(speed=700, rotation_angle=700)
    wait(500)
    right_arm.run_angle(speed=-700, rotation_angle=1500, then=Stop.HOLD, wait=False)
    left_arm.run_angle(speed=1000, rotation_angle=900)
    left_arm.run_angle(speed=1000, rotation_angle=-900, wait=False)
    wheels.straight(-50)
    wheels.curve(radius=-180, angle=100)
    wheels.settings(straight_speed=200, straight_acceleration=200)
    wheels.straight(-350)
    wheels.straight(100)
    wheels.turn(10)
    wheels.straight(-150)
    wheels.straight(200)
    wheels.turn(85)
    wheels.settings(400, 400)
    wheels.straight(-400, Stop.NONE)
    wheels.curve(radius=-220, angle=100)


def white_run():
    hub.display.number(5)
    wheels.settings(600, 500)
    wheels.straight(340)
    wheels.turn(-50)
    wheels.straight(85)
    right_arm.run_angle(300, 180)
    wheels.straight(-50)
    right_arm.run_angle(300, -90)
    wheels.straight(280)
    wheels.turn(90)
    wheels.settings(300)
    wheels.straight(10000, then=Stop.HOLD, wait=False)
    while map_sensor.reflection() > 20:
        pass
    wheels.stop()
    wheels.straight(60)
    left_arm.run_angle(speed=100, rotation_angle=-90)
    wait(100)
    wheels.straight(-10, Stop.NONE)
    wheels.curve(-200, -25, Stop.NONE)
    wheels.settings(straight_speed=1000, straight_acceleration=900)
    wheels.straight(-400, Stop.NONE)
    # wheels.curve(-80, 90, Stop.HOLD)


def run_straight():
    wheels.settings(600, 600)
    wheels.straight(100000)


print(run_sensor.color())

# run selector =====================================================
selected = run_sensor.color()

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
