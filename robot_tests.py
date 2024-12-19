# pylint: disable=C0114
# pylint: disable=C0116
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Color, Direction, Port
from pybricks.tools import hub_menu
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# Hardware definitions =================================================================
hub = PrimeHub()
left_wheel = Motor(Port.C)
right_wheel = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
left_arm = Motor(Port.E)
right_arm = Motor(Port.D)
run_sensor = ColorSensor(Port.F)
map_sensor = ColorSensor(Port.A)
wheels = DriveBase(left_wheel, right_wheel, wheel_diameter=62.4, axle_track=129.4)
wheels.use_gyro(True)

COLOR_LIST = [
    Color.WHITE,
    Color.BLACK,
    Color.GREEN,
    Color(h=339, s=85, v=94),
    Color.BLUE,
    Color.YELLOW,
    Color.VIOLET,
    Color(h=339, s=85, v=94),
]

map_sensor.detectable_colors(COLOR_LIST)
run_sensor.detectable_colors(COLOR_LIST)
print(2)


# Functions ============================================================================
def gyro_abs(target, speed):
    while (
        not target > hub.imu.heading() - 0.01 or not target < hub.imu.heading() + 0.01
    ):  # the stoping range
        direction = (
            target - (hub.imu.heading() % 360)
        ) % 360  # calculating the direction of the turn
        if direction > 180:
            left_wheel.dc(-speed)
            right_wheel.dc(speed)
        else:
            left_wheel.dc(speed)
            right_wheel.dc(-speed)
    wheels.stop()


def gyro_straight(distance_mm, speed):

    distance_driven = 0
    wheels.settings(speed)
    while distance_driven <= distance_mm - 50:  # Check if the distance has been covered
        print(hub.imu.heading())
        # current_angle = hub.imu.heading()
        current_angle = 0

        # If the robot drifts from the straight line, correct by adjusting the motor speeds
        if current_angle == 0:  # Robot is turning to the right
            wheels.straight(50, wait=False)
            distance_driven += 50
        else:
            gyro_abs(0, 200)

    for _ in range(0, 10):
        wheels.straight(10)
    # Stop when the target distance is reached
    wheels.stop()


def reset():
    hub.imu.reset_heading(0)
    right_wheel.reset_angle(0)
    left_wheel.reset_angle(0)
    right_arm.reset_angle(0)
    left_arm.reset_angle(0)
    wheels.settings(600, 300)


# Runs =================================================================================
def black_run():
    reset()
    hub.display.number(1)
    left_arm.run_angle(speed=100, rotation_angle=-90, wait=False)  # pick up stuff
    wheels.straight(-500)
    left_arm.run_angle(speed=100, rotation_angle=90)
    wheels.straight(-90)
    wheels.turn(angle=90 - 20.5)
    right_arm.run_angle(300, 250, wait=False)
    wheels.straight(185)  # M01
    right_arm.run_angle(300, -200)  # M04
    wheels.straight(-150)
    right_arm.run_angle(300, -50)
    wheels.turn(43)
    wheels.settings(900, 1000)  # M02
    wheels.straight(250)
    wheels.settings(300, 750)
    wheels.straight(-260)
    wheels.turn(58)
    right_arm.run_angle(50, 65)
    wheels.straight(65)
    wheels.turn(-15)
    right_arm.run_angle(200, 350)
    wheels.straight(40)
    wheels.straight(-110)
    left_wheel.run_angle(240, 120)
    wheels.drive(-1000, 25)


def red_run():
    reset()
    hub.display.number(2)
    wheels.settings(600, 250)
    wheels.straight(435)
    wheels.turn(45)
    wheels.settings(straight_acceleration=400)
    wheels.straight(300)


def green_run():
    hub.display.number(3)
    wheels.settings()


def white_run():
    hub.display.number(1)
    wheels.settings()


def yellow_run():
    hub.display.number(1)
    wheels.settings()


selected = hub_menu("0", "1")
if selected == "0":
    if run_sensor.color() == Color.BLACK:
        print(run_sensor.color())
        black_run()
    elif run_sensor.color() == Color(h=339, s=85, v=94):
        red_run()
        print(run_sensor.color())
