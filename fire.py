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
BLACK = Color(h=189, s=27, v=31)
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
    BLACK,
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


TARGET = 57


def follow_line_til_end(speed, kp, side):
    if side == "R":
        direction = 1
    else:
        direction = -1  # changing which wheel will add or subtruct the change
        left_wheel.reset_angle()  # reseting wheel angle for counting the distance we crossed
    while "1+1=3":
        while (
            map_sensor.reflection() < 25
        ):  # converting the angle of the wheel to mm and checking if we crossed the distance
            change = (
                (TARGET - map_sensor.reflection()) * kp * direction
            )  # calculating the change needed to add or subtruct from the wheels
            left_wheel.dc(speed + change)
            right_wheel.dc(speed - change)


# Runs =================================================================================
def black_run():
    """105 points"""
    reset()
    hub.display.number(1)
    wheels.settings(turn_rate=100, straight_speed=500, straight_acceleration=500)

    # pick up Coral (M03), Krill (M12) & Water sample (M14)
    left_arm.run_angle(speed=100, rotation_angle=-90, wait=False)
    wheels.straight(-490)
    left_arm.run_angle(speed=100, rotation_angle=90)
    # drive to Coral nursery(M01)
    wheels.straight(-90)
    wheels.turn(angle=90 - 20.5)
    right_arm.run_angle(800, 500, wait=False)
    # push Coral Buds (M01) & pick up Scuba Diver (M04)
    wheels.straight(185)
    right_arm.run_angle(800, -350)
    wheels.straight(-150)
    right_arm.run_angle(600, -120)
    # turn to Shark
    wheels.turn(43)
    wheels.settings(1000, 3000)
    # launch Shark (M02)
    wheels.straight(250)
    # drive to Coral Reef (M03)
    wheels.settings(300, 300)
    wheels.straight(-260)
    wheels.turn(58)
    right_arm.run_angle(250, 160)
    wheels.straight(80)
    # hang Scuba diver (M04) & press Coral reef (M03)
    wheels.turn(-15)
    right_arm.run_time(250, 1500)
    wheels.straight(55)
    # drive back to red launch area
    wheels.straight(-110)
    left_wheel.run_angle(240, 145)
    wheels.settings(1000, 2000)
    wheels.straight(-400, then=Stop.NONE)
    wheels.curve(-400, -90)


def red_run():
    reset()
    hub.display.number(2)
    wheels.settings(500, 450)
    wheels.straight(-130, then=Stop.NONE)
    wheels.curve(-340, -90)
    wheels.straight(-150)
    wait(100000)
    wheels.straight(200)
    wheels.curve(150, -90)
    wheels.straight(400)


def yellow_run():
    """85 points"""
    reset()
    right_arm.hold()
    hub.display.number(3)

    # drive towards Ship (M15)
    wheels.settings(straight_speed=500, straight_acceleration=200)
    wheels.straight(330)

    # drop Sample Collection (M14)
    right_arm.run_time(speed=-200, time=900)
    right_arm.run_time(speed=500, time=1000)
    right_arm.hold()

    # Rearrange Artificial Habitat (M08)
    wheels.straight(670)
    wheels.settings(straight_speed=400, straight_acceleration=60)
    wheels.straight(-160)
    wait(500)
    right_arm.hold()
    wheels.settings(700, 700)
    wheels.straight(-160)

    # drive towards Shark Habitat (M02)
    wheels.settings(straight_speed=500, straight_acceleration=500)
    wheels.turn(-45)
    wheels.straight(170, wait=False)
    # drop Shark
    right_arm.hold()
    right_arm.run_time(speed=100, time=1000)
    wheels.settings(straight_speed=800, straight_acceleration=1000)

    # drive to blue launch area
    wheels.curve(390, 60)
    wheels.straight(-100)
    left_arm.run_angle(1000, -100)
    wheels.settings(1000, 2000)
    wheels.straight(1000)


def green_run():
    """100 points"""
    reset()
    right_arm.hold()
    hub.display.number(4)
    original_settings = wheels.settings()
    wheels.settings(straight_speed=500, straight_acceleration=500)
    right_arm.hold()

    # release Unknown Creature (M09)
    wheels.straight(501)
    wheels.straight(-185)

    # drive towrds Sonar (M11) and Submersible (M10)
    wheels.turn(-50)
    wheels.straight(300)
    wheels.turn(50)
    wheels.straight(170)
    wheels.turn(50)
    wheels.straight(400, Stop.NONE)  # allign to wall
    wheels.drive(500, 0)
    wait(700)
    right_wheel.hold()
    left_wheel.hold()

    # Sonar
    right_arm.run_angle(speed=800, rotation_angle=-300)
    wait(500)
    right_arm.run_angle(speed=700, rotation_angle=1500, then=Stop.HOLD, wait=False)

    # Send Over the Submersible
    left_arm.run_angle(speed=1000, rotation_angle=900)
    left_arm.run_angle(speed=1000, rotation_angle=-900, wait=False)

    # return to blue launch area
    wheels.settings(500, 900)
    wheels.straight(-50)
    wheels.curve(radius=-190, angle=80, then=Stop.NONE)
    wheels.straight(-415)
    wheels.straight(380)
    wheels.curve(radius=-200, angle=-120)
    wheels.curve(radius=-150, angle=60, then=Stop.NONE)
    wheels.straight(-500)


def white_run():
    """75 points"""
    hub.display.number(5)
    wheels.settings(600, 500)
    left_arm.hold()

    # drive towards Cargo Ship (M13)
    wheels.straight(337)
    wheels.turn(-50)
    wheels.straight(85)
    # change Shipping Lanes
    right_arm.run_angle(280, 180)
    wheels.straight(-50)
    right_arm.run_angle(300, -110)
    # drive towards the Whale (M12)
    wheels.straight(300)
    wheels.turn(100)
    wheels.settings(300)
    wheels.straight(10000, then=Stop.HOLD, wait=False)
    while map_sensor.reflection() > 20:
        pass
    wheels.stop()
    wheels.straight(60)
    # Feed the Whale (M12) and leave Reef Segment (M03)
    left_arm.run_angle(speed=100, rotation_angle=-90)
    wait(100)

    # drive to the Cold Seep (M09)
    wheels.straight(-40, Stop.NONE)
    wheels.curve(-180, -35, Stop.NONE)
    wheels.settings(straight_speed=1000, straight_acceleration=900)
    wheels.straight(-500)


def run_straight():
    wheels.settings(600, 600)
    wheels.straight(100000)


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
