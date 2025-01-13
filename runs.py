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
    wheels.straight(380)
    # gyro_abs(45, 25)
    wheels.turn(45)
    wheels.settings(500, 100)  # slows down so the thingy doesn't fall back
    wheels.straight(280)
    left_wheel.run_angle(360, 165, then=Stop.NONE)
    wheels.settings(300, 600)
    wheels.straight(65)
    left_arm.run_angle(750, -220)
    wait(500)
    left_arm.run_angle(250, 250)
    wait(350)

    wheels.straight(15, Stop.NONE)

    wheels.curve(180, -95, Stop.NONE)
    wheels.straight(40, Stop.NONE)
    wheels.curve(185, -90)

    wheels.straight(80)
    wheels.curve(100, -20)
    wheels.straight(30)
    right_wheel.run_angle(200, -220)
    wheels.straight(200)
    wheels.curve(500, 25)
    right_arm.run_angle(800, -350)
    # right_arm.run_angle(800, 350)

    # wheels.straight(50)
    # right_arm.run_angle(800, -350)

    # wheels.straight(200)
    # wheels.turn(-90)
    # wheels.straight(400)

    # wheels.straight(425, then=Stop.COAST_SMART)
    # right_wheel.hold()
    # left_wheel.run_angle(400, 230)  # gets to the ship

    # # wheels.settings(straight_speed=100)
    # # wheels.straight(300)
    # # wheels.settings(straight_speed=600)

    # start_angle = left_wheel.angle()  # drives into it
    # wheels.straight(250)
    # # left_wheel.run(182)
    # # right_wheel.run(180)
    # # while left_wheel.angle() < start_angle + 450:
    # #     pass
    # # left_wheel.hold()
    # # right_wheel.hold()

    # wheels.settings(150, 100)
    # wheels.straight(55)
    # # left_wheel.run_angle(300, 200, then=Stop.NONE)
    # wheels.turn(40, then=Stop.COAST)
    # # wheels.straight(50)
    # left_arm.run_until_stalled(150)  # takes the kilshon
    # left_arm.run_until_stalled(-150)

    # # wheels.turn(40)
    # # wheels.straight(150)
    # # left_wheel.run_angle(200)


def yellow_run():
    reset()
    hub.display.number(3)
    wheels.settings(800, 200)
    wheels.straight(300)
    right_arm.run_angle(200, -120)
    wheels.straight(700)


def green_run():
    reset()
    hub.display.number(4)
    wheels.settings(600, 250)
    wheels.straight(500)
    wheels.straight(-40)
    wheels.turn(-45)

    wheels.straight(350)
    wheels.turn(90)
    wheels.straight(400)
    wheels.turn(-150)
    wheels.straight(20)
    wheels.straight(-50)
    wheels.turn(60)

    wheels.straight(-100)


def white_run():
    hub.display.number(1)
    wheels.settings()


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
