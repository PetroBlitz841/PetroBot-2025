# pylint: disable=C0114
# pylint: disable=C0116

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Color, Direction, Port
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

run_colors = (Color.BLACK, Color.GREEN, Color.WHITE, Color.YELLOW, Color.RED)
run_sensor.detectable_colors(run_colors)

wheels.settings(turn_rate=100)


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


# Runs =================================================================================
def black_run():
    hub.imu.reset_heading(0)
    hub.display.number(1)
    wheels.settings(200, 200)
    wait(150)
    left_arm.run_angle(speed=100, rotation_angle=-90, wait=False)
    wheels.straight(-500)
    left_arm.run_angle(speed=100, rotation_angle=90)
    wheels.straight(-90)
    wheels.turn(
        angle=90 - 20.5,
    )
    right_arm.run_angle(300, 250)
    wheels.straight(175)
    right_arm.run_angle(300, -200)
    wheels.straight(-150)
    right_arm.run_angle(300, -50)
    wheels.turn(45)
    wheels.settings(900, 1000)
    wheels.straight(250)
    wheels.settings(300, 750)


def red_run():
    hub.display.number(1)
    wheels.settings()


def green_run():
    hub.display.number(1)
    wheels.settings()


def white_run():
    hub.display.number(1)
    wheels.settings()


def yellow_run():
    hub.display.number(1)
    wheels.settings()


color_options = {
    Color.BLACK: black_run,
    Color.GREEN: green_run,
    Color.WHITE: white_run,
    Color.YELLOW: yellow_run,
    Color.RED: red_run,
}
halifa = run_sensor.color()  # pylint: disable=E1111
# print("current: ", hub.battery.current(), "voltage: ", hub.battery.voltage())
color_options[halifa]()
