from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, hub_menu
from pybricks.tools import hub_menu


# Hardware definitions
hub = PrimeHub()
left_wheel = Motor(Port.C)
right_wheel = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
left_arm = Motor(Port.E)
right_arm = Motor(Port.D)
run_sensor = ColorSensor(Port.F)
map_sensor = ColorSensor(Port.A)
wheels = DriveBase(left_wheel, right_wheel, wheel_diameter=62.4, axle_track=10.6)

run_colors = (Color.BLACK, Color.GREEN, Color.WHITE, Color.YELLOW, Color.RED)
run_sensor.detectable_colors(run_colors)


# Runs =================================================================================
def black_run():
    hub.display.number(1)
    wheels.settings(300)

    wheels.straight(distance=200)
    wheels.straight(distance=-200)


def green_run():
    wheels.settings()
    pass


def white_run():
    wheels.settings()
    pass


def yellow_run():
    wheels.settings()
    pass


def red_run():
    wheels.settings()
    pass


color_options = {
    Color.BLACK: black_run,
    Color.GREEN: green_run,
    Color.WHITE: white_run,
    Color.YELLOW: yellow_run,
    Color.YELLOW: red_run,
}
halifa = run_sensor.color()
# print("current: ", hub.battery.current(), "voltage: ", hub.battery.voltage())
color_options[halifa]()
