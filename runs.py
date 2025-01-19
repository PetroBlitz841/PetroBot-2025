# pylint: disable=C0114
# pylint: disable=C0116
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Color, Direction, Port, Stop
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

RUN_RED = Color(h=339, s=85, v=94)
RUN_GREEN = Color(h=154, s=77, v=52)
RUN_COLORS = [Color.BLACK, RUN_RED, Color.YELLOW, RUN_GREEN]

COLOR_LIST = [
    Color.BLACK,
    Color.WHITE,
    Color.BLUE,
    Color.GREEN,
    Color.RED,
    Color.YELLOW,
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


print(map_sensor.hsv())


# Runs =================================================================================
def black_run():
    reset()
    hub.display.number(1)
    wheels.settings(600, 600)
    wheels.straight(-10000)

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
    wheels.straight(420)
    # gyro_abs(45, 25)
    wheels.turn(45)
    wheels.settings(500, 100)  # slows down so the thingy doesn't fall back
    wheels.straight(280)

    # Turn to trident
    left_wheel.run_angle(360, 165, then=Stop.NONE)
    wheels.settings(300, 600)
    wheels.straight(60)  # Drive to trident

    # Pick up trident
    left_arm.run_angle(speed=750, rotation_angle=-220)
    wait(500)
    left_arm.run_time(speed=150, time=1800)
    wait(350)
    wheels.straight(80)
    wheels.turn(-45)
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
    wheels.straight(450)
    wheels.turn(-90)
    wheels.straight(185)
    # wheels.turn(35)
    # wheels.straight(75)
    # wheels.turn(-13)
    # wheels.straight(150)
    # finished angler fish m05
    wheels.turn(92)
    wheels.straight(130)
    wheels.settings(200)
    wheels.straight(50)
    wheels.settings(250, 250)
    right_arm.run_angle(600, -400)

    wheels.settings(turn_rate=300, turn_acceleration=500)
    wheels.turn(40)
    wheels.straight(-200)
    wheels.straight(500)
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


def yellow_run():
    reset()
    hub.display.number(3)
    wheels.settings(800, 200)
    wheels.straight(300)
    right_arm.run_angle(200, -120)
    right_arm.run_angle(200, 120)
    wheels.straight(700)
    wheels.settings(200, 200)
    wheels.straight(-400)


def green_run():
    reset()
    hub.display.number(4)
    wheels.settings(600, 250)
    # wheels.straight(500)
    # wheels.straight(-40)
    # wheels.turn(-45)

    # wheels.straight(350)
    # wheels.turn(90)
    # wheels.straight(400)
    # wheels.turn(-150)
    # wheels.straight(20)
    # wheels.straight(-50)
    # wheels.turn(60)

    # wheels.straight(-100)
    wait(1000)
    right_arm.run_angle(400, -430)
    right_arm.run_angle(400, 800)
    wheels.straight(-5000)


def white_run():
    hub.display.number(1)
    wheels.settings()


def run_straight():
    wheels.settings(600, 600)
    wheels.straight(10000)


selected = run_sensor.color()
print(run_sensor.color())
if selected == Color.BLACK:
    black_run()
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
