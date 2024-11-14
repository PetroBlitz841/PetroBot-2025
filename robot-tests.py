from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, hub_menu
from pybricks.tools import hub_menu

hub = PrimeHub()

left_wheel = Motor(Port.C)
right_wheel = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
left_arm = Motor(Port.E)
right_arm = Motor(Port.D)
fast_change_sensor = ColorSensor(Port.F)
map_sensor = ColorSensor(Port.A)

yosi_jr_jr = DriveBase(left_wheel, right_wheel, wheel_diameter=62.4, axle_track=10.6)

halifacolors = (Color.BLACK, Color.GREEN, Color.WHITE, Color.YELLOW, Color.RED)
fast_change_sensor.detectable_colors(halifacolors)

halifa = fast_change_sensor.color()
if halifa == Color.BLACK:
    hub.display.number(1)
    yosi_jr_jr.straight(distance=100)
    right_arm.turn(50)
elif halifa == Color.GREEN:
    hub.display.number(2)
    yosi_jr_jr.straight(distance=300)
elif halifa == Color.WHITE:
    hub.display.number(3)
    yosi_jr_jr.straight(distance=500)
elif halifa == Color.YELLOW:
    hub.display.number(4)
    yosi_jr_jr.straight(distance=600)
elif halifa == Color.RED:
    hub.display.number(5)
    yosi_jr_jr.straight(distance=700)
