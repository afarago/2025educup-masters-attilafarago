""" 
EDUCUP 2025 MASTERS Challenge

Attila Farago

"""

#region INIT_AND_UTILS
from pybricks.hubs import InventorHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

def exit():
    raise SystemExit()
    
hub = InventorHub()
me = Motor(Port.E)
mf = Motor(Port.F, Direction.COUNTERCLOCKWISE)
bot = DriveBase(mf, me, 60, 114) # 64mm wheel -> rubber compress factors to 60mm
print('bot.settings', bot.settings())
BOT_TURN_RATE=150
BOT_TURN_RATE_FAST=350
bot.settings(straight_speed=200+300, straight_acceleration=250, turn_rate=BOT_TURN_RATE, turn_acceleration=600)
botset = bot.settings()
mc = Motor(Port.C)
bot.use_gyro(True)
# hub.imu.settings(heading_correction=362) #362
print('bot.heading_control.target_tolerances', bot.heading_control.target_tolerances())
bot.heading_control.target_tolerances(position=8)
print('bot.distance_control.target_tolerances', bot.distance_control.target_tolerances())
bot.distance_control.target_tolerances(position=8)
print('bot.heading_control.pid', bot.heading_control.pid())

# bot.settings(straight_speed=1000)
# bot.heading_control.pid(ki=5)
# bot.straight(2000)
# exit()
# mc.run_time(-100, 1000)


def calibrate1():
    startrot = hub.imu.heading()
    bot.straight(100)
    for _ in range(4*4):
        bot.turn(90)
    endrot = hub.imu.heading()
    print((endrot-startrot)/16)

def wait_button():
    hub.system.set_stop_button([Button.BLUETOOTH])
    while not (hub.buttons.pressed()): pass
    button = hub.buttons.pressed()
    while (hub.buttons.pressed()): pass
    hub.system.set_stop_button([Button.CENTER])
    return list(button)[0]

ARM_MOTOR_DIRECTION = +1
def arm_up(wait=True):
    if wait:
        mc.run_until_stalled(ARM_MOTOR_DIRECTION*300, duty_limit=70)
        mc.run_time(ARM_MOTOR_DIRECTION*300, 100, Stop.HOLD)
        # print("arm is up")
    else:
        mc.run_time(ARM_MOTOR_DIRECTION*300, 1000, wait=False)

def arm_close_bottom():
    mc.run_angle(ARM_MOTOR_DIRECTION*300, 130, then=Stop.HOLD)

def arm_down(wait=True):
    if wait:
        mc.run_until_stalled(-ARM_MOTOR_DIRECTION*300, duty_limit=40)
        mc.run_time(ARM_MOTOR_DIRECTION*200, 200, Stop.COAST) # ease a bit
        # print("arm is down")
    else:
        mc.run_time(-ARM_MOTOR_DIRECTION*100, 1000, wait=False)        

def align(speed=200, ms=2000, turn=0):
    bot.use_gyro(False)
    bot.drive(speed, turn)
    sw = StopWatch()
    # while sw.time()<ms and not (me.stalled() and mf.stalled()):
    #     pass
    # wait(100)
    wait(ms)
    bot.stop()
    # while not hub.imu.stationary(): pass
    bot.use_gyro(True)
    wait(100)

def align_turn(degree):
    bot.settings(turn_rate=BOT_TURN_RATE_FAST)
    bot.turn(degree, Stop.COAST_SMART)
    bot.settings(turn_rate=BOT_TURN_RATE)
#endregion 

#region MAIN
sw_main=StopWatch()
sw_lap=StopWatch()
def lap(tag):
    print("TIME", tag, sw_lap.time()/1000, "\t", sw_main.time()/1000)
    sw_lap.reset()

def main():
    part1()
    lap(1)
    part2()
    lap(2)
    part3()
    lap(3)
    part4()
    lap(4)
    part5()
    lap(5)
    part6()
    lap(6)
    part6b()
    lap("6b")
    part7()
    lap(7)
#endregion

#region MISSION_CODES
def part1():
    # start, R2, cap1-2-3-4--5, stops before R1, at NORTH_WALL heading south

    bot.settings(straight_speed=200, turn_rate=100)

    bot.straight(20)
    arm_up()
    # bot.use_gyro(False)
    bot.straight(50)
    bot.turn(-90)

    bot.straight(140, Stop.NONE)
    bot.arc(-114, angle=90, then=Stop.NONE)
    # bot.curve(114, -90) # arc?
    bot.straight(170)
    # bot.turn(180, Stop.COAST_SMART)
    align_turn(180)
    align(-400, 1500)
    # at north wall front facing

    bot.settings(straight_speed=500, turn_rate=100)
    bot.arc(75, angle=90)
    bot.straight(300+580)
    # bot.turn(-90, Stop.COAST_SMART)

    # NORTH_WALL align
    # align_turn(-90)
    bot.turn(-90)
    align(-400, 1000)
    bot.stop()

def part2():
    # NORTH_WALL(with arm up, holding R2)->goto and collect R1, put R2 on it
    bot.settings(*botset)
    bot.settings(straight_speed=200, turn_rate=100)

    bot.arc(70, angle=90)
    # distance from R2 = 90mm
    bot.straight(55)
    arm_down()

    bot.straight(-30)
    bot.straight(60)
    arm_close_bottom() # close lightly to secure
    bot.straight(-40)

def part3():
    # R2->west wall align
    # bot.settings(*botset)
    # mc.hold()
    # bot.settings(turn_acceleration=200) # care for rocket cargo
    # bot.arc(-72, angle=-90, then=Stop.COAST_SMART)
    # bot.straight(-400)
    # bot.straight(130)

    # # WEST_WALL align to west wall
    # bot.turn(90)
    # align(-200, 2000)
    # bot.reset()

    # R2->west wall align+290 to east
    bot.settings(*botset)
    mc.hold()
    bot.settings(turn_acceleration=200) # care for rocket cargo

    bot.straight(-45, Stop.NONE)
    bot.turn(-90)

    # push fig back
    OFFSET=10
    bot.straight(365+OFFSET)
    bot.straight(-OFFSET)
    bot.turn(-90)


def part4():
    # WEST_WALL+290X->though the field (collect cap6) moving east, align north, then align east wall
    # leave R1+R2, leave caps, align to south wall
    bot.settings(*botset)
    mc.hold()
    bot.settings(turn_acceleration=200) # care for rocket cargo
    bot.settings(straight_speed=1000) # rush trough the field
    bot.heading_control.pid(ki=5)

    bot.straight(500-290, then=Stop.NONE)
    # bot.curve(200, -10, then=Stop.NONE)
    # bot.arc(-200, distance=50, then=Stop.NONE)
    bot.drive(500, -200)
    startangl = bot.angle()
    # nowait straign with small degree turn
    while bot.angle()-startangl>-11: pass
    bot.straight(1400)
    # pivots to left for no reason, yet its good for me :D

    bot.settings(straight_speed=400)
    # align to NORTH_WALL
    bot.settings(turn_rate=80) #slow turn
    bot.turn(90+10)
    bot.settings(turn_rate=BOT_TURN_RATE)
    align(-300, 1500) # north wall

    bot.straight(130) # to south

    bot.settings(turn_rate=80) #slow turn
    bot.turn(-90)
    bot.settings(turn_rate=BOT_TURN_RATE)
    
    # align to EAST_WALL forward / above rocket
    align(200, 1000)
    bot.straight(-120)
    arm_down() # open arm

    # leave caps
    bot.straight(-140)
    bot.turn(-90)
    bot.straight(-30)
    arm_up()
    
    # align SOUTH_WALL
    bot.straight(-710, Stop.NONE)
    align(-200, 1000)
    bot.heading_control.pid(ki=0)

def part5():
    # SOUTH_WALL->R3->R1+R2 placement->EAST_WALL
    bot.settings(*botset)
    # bot.settings(straight_speed=1000)
    bot.straight(440)
    # bot.turn(-90)
    bot.curve(-114, 90-15, Stop.NONE)
    
    # align to EAST_WALL
    align(-400, 1000)

    # turn to R3
    bot.straight(40)
    
    # armdown nowait prep
    mc.run_time(-ARM_MOTOR_DIRECTION*100, 1000, wait=False)
    bot.turn(-90)
    arm_down()

    # pick up R3
    bot.straight(120)
    arm_up()

    # go to R1+R2, put R3
    bot.turn(160)
    bot.straight(100)
    bot.turn(20)
    bot.straight(315+55)
    arm_down()

    # back-to east wall
    bot.straight(-300-50)
    arm_up(False)
    bot.turn(-90)
    align(-200, 1000)
    arm_up(True)

def part6():
    # EASTWALL->fig3,fig4,sodacan
    bot.settings(*botset)
    bot.settings(straight_speed=500, turn_rate=150)
    arm_up(False)
    # arm_up(True)
    ## todo nowait
    bot.straight(500, Stop.NONE)
    bot.curve(250, -45, Stop.NONE)
    bot.straight(50, Stop.NONE)
    bot.curve(400, 45)
    bot.stop()

    # align to south wall
    bot.turn(90)
    align(-200, 1000)

    arm_down(False)
    bot.straight(10)
    bot.turn(-90)
    arm_down(True)

    # pick up soda can
    bot.straight(150)
    arm_up()

def part6b(): 
    # sodacan->fig1,fig2
    bot.settings(*botset)
    bot.settings(straight_speed=300, turn_rate=150)
    bot.straight(50, Stop.NONE)
    bot.curve(300, 60, Stop.NONE)
    bot.curve(300, -60, Stop.NONE)
    bot.straight(150, Stop.NONE) 
    bot.stop()

    bot.turn(180+5)
    bot.settings(straight_speed=1000)
    bot.straight(500+1050)
    bot.turn(90-5)

    bot.straight(-550)
    bot.turn(180)
    # release soda can
    arm_down()

def part7():
    # sodacan@rocketcenter->university
    bot.settings(*botset)
    bot.settings(straight_speed=300, turn_rate=150)
    # bot.straight(-200, Stop.NONE)
    bot.curve(-120, -75, Stop.NONE)
    arm_up(False) # reduce friction
    bot.heading_control.pid(ki=5)
    bot.settings(straight_speed=1000, turn_rate=150)

    # bot.straight(-1100, Stop.NONE)
    # bot.stop() # no deceleration
    bot.straight(-1550, Stop.BRAKE)
    # bot.stop() # no deceleration
    wait(1000)
#endregion

main()