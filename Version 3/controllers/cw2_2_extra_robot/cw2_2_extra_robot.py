"""cw2_2 controller."""
# An implementation of an active decision-making architecture.
# Thymio robot confronts action selection problems and has autonomous solutions, acting accordingly.
# Robot needs all three resources to survive, water, energy and food.
# All resources start at the maximum value (100) and will decrease at different speeds to mimic real-life.
# Hunger decreases at a slower rate (0.002 per tick/10 times per second).
# Energy and water decrease at a faster rate (0.003 per tick/10 times per second).
# When consumming any resource the robot will stop moving and rise up the physiology variable up to 100.
# There are differences for each resource when consumming:
# Food: The hunger will rise at a possibly slower rate than other both resources (up to 0.06 per tick/10 times per second).
# Food: By every tick spent consuming food the robot will slow down (0.05 per tick/10 times per second), until the robot gets to the minimum speed of 1.
# Water: The thirst will rise at a possibly faster rate than hunger (up to 0.09 per tick/10 times per second).
# Energy: The energy will rise at a possibly faster rate than hunger (up to 0.09 per tick/10 times per second).
# Energy: By every tick spent consuming energy the robot will lose thirst (0.02 per tick/10 times per second).
# When any of these variables reach 0, the robot will die.  
# Based on trace_maker controller and task 1 controller.

from controller import Robot 
import random

# Hardware constants
DIST_SENS_COUNT = 7
DIST_SENS_FRONT_COUNT = 5
GRND_SENS_COUNT = 2
TS_NAMES = ('center','left','right','forward','backward')

# Global variable to hold motor speed
MOTOR_SPEED = 4

#Constants to keep a count for the behaviour_advance()
TURN_COUNT = 0
THRESH_TURN_COUNT = 300
TURN_RESET = 600

#Constant to keep a count for the behaviour_advance()
NAVI_COUNT = 0
THRESH_NAVI_COUNT = 3000
NAVI_RESET = 6000

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# initialisation functions
def init_actuators():
    """ Initialise motors, LEDs, pen """
    global motor_l, motor_r, m_spd_l, m_spd_r

    # Set up motors
    motor_l = robot.getMotor('motor.left')
    motor_r = robot.getMotor('motor.right')

    # Configure motors for velocity control
    motor_l.setPosition(float('inf'))
    motor_r.setPosition(float('inf'))

    # initialise variables for actuator values
    reset_actuator_values()


def init_sensors():
    """ Initialise distance sensors, ground sensors etc. """
    global ds, ds_val

    # Set up distance sensors
    ds = []
    for i in range(DIST_SENS_COUNT):
        s_name = 'prox.horizontal.{:d}'.format(i)
        # print(ds_name)
        ds.append(robot.getDistanceSensor(s_name))
        ds[i].enable(timestep)

    # Create array to store distance sensor readings
    ds_val = [0] * DIST_SENS_COUNT
    
    
def read_sensors():
    """ Read sensor values from hardware into variables """
    global ds, ds_val

    for i in range(DIST_SENS_COUNT):
        ds_val[i] = ds[i].getValue()
    

def reset_actuator_values():
    """ Reset motor & LED target variables to zero/off. """
    global m_spd_l, m_spd_r
    
    m_spd_l = 0
    m_spd_r = 0


def send_actuator_values():
    """ Write motor speed and LED colour variables to hardware.
        Called at the end of the Main loop, after actions have been calculated """
    global m_spd_l, m_spd_r

    motor_l.setVelocity(m_spd_l)
    motor_r.setVelocity(m_spd_r)

def behaviour_advance():
    """ Behavior to move forward avoiding obstacles.
        Returns left and right motor values. 
        Function is called everytime an object is detected ahead."""
    global ds_val, TURN_COUNT, NAVI_COUNT

    ret_l = MOTOR_SPEED
    ret_r = MOTOR_SPEED
    
    """Depending on turn_count value, different weights are given to the robot distance sensors.
       This was implemented to avoid the robot being stuck in the corners of the arena. This way,
       the robot can only get stuck for the seconds equivalent to the integer given to THRESH_TURN_COUNT (300).
       This means the robot will change weights every 300 ticks or 30 seconds.
       First set of weights favour the turning towards the right and second set to the left."""
    
    TURN_COUNT = TURN_COUNT + 1
    NAVI_COUNT = NAVI_COUNT + 1
    
    if NAVI_COUNT == NAVI_RESET:
        NAVI_COUNT = 0
    
    if TURN_COUNT == TURN_RESET:
        TURN_COUNT = 0
    
    if TURN_COUNT < THRESH_TURN_COUNT: 
        weights = (2,1,1,0.5,-2)
    else:
        weights = (2,-0.5,-1,-1,-2)
        
    count = 0
    
    for i in range(DIST_SENS_FRONT_COUNT):
        count += ds_val[i] * weights[i]

    if count>0:
        # Object to left, spin right
        ret_l = MOTOR_SPEED
        ret_r = -MOTOR_SPEED
    elif count<0:
        # Object to right, spin left
        ret_l = -MOTOR_SPEED
        ret_r = MOTOR_SPEED
    else: 
        if NAVI_COUNT < THRESH_NAVI_COUNT:
            ret_l = MOTOR_SPEED*0.87
            ret_r = MOTOR_SPEED
        else:
            ret_l = MOTOR_SPEED
            ret_r = MOTOR_SPEED*0.87
    # If count==0 use default values above
    
    return ret_l, ret_r
    
 
#
# Main entry point for code
# 

# Initialisation
init_actuators()
init_sensors()

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

        # Read the sensors:
        read_sensors()
    
        # Process sensor data here.
        reset_actuator_values()
        # Execute behaviours...
        m_spd_l, m_spd_r = behaviour_advance()
    
        # Send actuator commands:
        send_actuator_values()
    

# End of Main loop
# Exit & cleanup code.
# (none required)