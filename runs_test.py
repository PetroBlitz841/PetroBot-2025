# pylint: disable=C0114, C0116, W0611, C0103, R0913, E1123, E1111

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Color, Direction, Port, Stop
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

wheels.straight(50)
wheels.straight(-50)
wheels.straight(50)
