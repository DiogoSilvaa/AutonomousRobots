"""cw2_1_2 controller"""
# An implementation of the trace-maker robot from Prescott & Ibbotson.
# Some changes for the Thymio 2 robot (e.g. only 2 ground sensors positioned differently).
# Thymio Robot will draw a meandering pattern.
# The biggest difference to the cw_2_1_1 controller is that
# the thigmotaxis behaviour is only executed immediately following the strophotaxis behaviour
# but is otherwise, disengaged.
# Based on trace_maker controller and cw2_1_1 controller.

from controller import Robot

# Hardware constants
DIST_SENS_COUNT = 7
DIST_SENS_FRONT_COUNT = 5
GRND_SENS_COUNT = 2

# Control constants
MOTOR_SPEED = 2 #default motor speed

SIDE_LEFT = 0 # to reference ground sensors
SIDE_RIGHT = 1
 
THRESH_WHITE = 770 #value to detect the floor in the ground sensors
THRESH_BLACK = 250 #value to detect the line in the ground sensors
CYCLE_CHANGE_ACTIVE = 1 #variable to identify the cycle has changed
CYCLE_CHANGE_INACTIVE = 0 #variable to identify the cycle has not changed
COUNT = 0 #variable to hold tick count
THRESH_DURATION = 2050 #constant to determine cycle duration
THRESH_APPROACH  = 425 #constant to determine maximum time to approach line before supressing.

# Globals used by behaviour_avoid_line()
AVOID_LINE_PERSIST_TIME = 0.4 # seconds
avoid_line_persist_until = 0

#Global variables used by behaviour_strophotaxis(duration)
turning_side = 0 #starting turning side is 0 (left).
stro_cycle_change = 0 #cycle change starts by being inactive (did not happen recently).

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

    # Initialise pen
    pen = robot.getPen("Thymio II pen")
    pen.write(True) # default True, set to False to disable pen
    pen.setInkColor(0x000000, 0.7) # colour (RGB) & opacity


def init_sensors():
    """ Initialise distance sensors, ground sensors etc. """
    global ds, ds_val, gs, gs_val

    # Set up distance sensors
    ds = []
    for i in range(DIST_SENS_COUNT):
        s_name = 'prox.horizontal.{:d}'.format(i)
        # print(ds_name)
        ds.append(robot.getDistanceSensor(s_name))
        ds[i].enable(timestep)

    # Create array to store distance sensor readings
    ds_val = [0] * DIST_SENS_COUNT

    # Set up ground sensors
    gs = []
    for i in range(GRND_SENS_COUNT):
        s_name = 'prox.ground.{:d}'.format(i)
        # print(ds_name) # uncomment to debug names
        gs.append(robot.getDistanceSensor(s_name))
        gs[i].enable(timestep)

    # Create array to store ground sensor readings
    gs_val = [0] * GRND_SENS_COUNT


def read_sensors():
    """ Read sensor values from hardware into variables """
    global ds, ds_val, gs, gs_val

    for i in range(DIST_SENS_COUNT):
        ds_val[i] = ds[i].getValue()

    for i in range(GRND_SENS_COUNT):
        gs_val[i] = gs[i].getValue()
    

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


def object_ahead():
    """ Utility function to return True if an object is detected by the front sensors
        or False otherwise. """
    global ds_val
    
    for i in range(DIST_SENS_FRONT_COUNT):
        if ds_val[i]>0:
            return True

    return False # Nothing detected in loop

#
#BEHAVIOUR FUNCTIONS
#

def behaviour_advance(side):
    """ Behavior to move forward, turning when encountering obstacles.
        Returns left and right motor values.
        Function takes as parameter the current side the robot is monitoring the ground sensor and moving towards 
        Function is called at every tick."""
        
    global ds_val

    ret_l = MOTOR_SPEED
    ret_r = MOTOR_SPEED

    # Have different centre weight depending on side
    if side == SIDE_LEFT:
        weights = (2,1,1,0.5,-2)
    else: # SIDE_RIGHT
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
        #There is no object, and it is free to move as it is defined to
        if side == SIDE_LEFT: 
        #If turning side is left, curves slightly to the left
            ret_l = MOTOR_SPEED*1.6
            ret_r = MOTOR_SPEED*2.3
        else: 
        # side == SIDE_RIGHT
        #If turning side is right, curves slightly to the right
            ret_l = MOTOR_SPEED*2.3
            ret_r = MOTOR_SPEED*1.6
    # If count==0 use default values above

    return ret_l,ret_r


def behaviour_approach_line(side):
    """ Behaviour to curve towards lines to the left/right.
        Returns left and right motor values.
        Function takes as parameter the current side the robot is monitoring the ground sensor and moving towards 
        Function is called at every tick."""
        
    global gs_val

    # If front distance sensors detect object this behaviour does nothing
    if object_ahead():
        return None,None

    ret_l = None # None means no output
    ret_r = None # won't inhibit other outputs

    # If ground sensor detects white, curve to sensor side
    if gs_val[side] > THRESH_WHITE:
        #if the strophotaxis cycle has changed:
        if stro_cycle_change == CYCLE_CHANGE_ACTIVE:
            #if the ticker count is still below the maximum time available for approaching the line:
            if COUNT < THRESH_APPROACH:
                if side==SIDE_LEFT:
                    # curve to left
                    ret_l = MOTOR_SPEED*0.1
                    ret_r = MOTOR_SPEED*1.9
                else: # SIDE_RIGHT
                    # curve to right
                    ret_l = MOTOR_SPEED*1.9
                    ret_r = MOTOR_SPEED*0.1
            #if the ticker count is beyond the threshold, function does nothing, leaving the movement determination to behaviour_advance()
            else:
                ret_l = None
                ret_r = None
                
    return ret_l,ret_r
    
    

def behaviour_avoid_line(side):
    """ Behaviour to turn away from lines to the left/right.
        Returns left,right motor values.
        Function takes as parameter the current side the robot is monitoring the ground sensor and moving towards 
        Function is called at every tick. """
        
    global gs_val, stro_cycle_change, avoid_line_persist_until

    # If front distance sensors detect object this behaviour does nothing
    if object_ahead():
        return None,None

    ret_l = None # None means no output
    ret_r = None

    now = robot.getTime() # current time

    if gs_val[side] < THRESH_BLACK:
    #returns 1 if the black line is being detected by the ground sensor, 0 otherwise
        #If it is detected, it will avoid the line for an extra 0.4 seconds (AVOID_LINE_PERSIST_TIME)
        avoid_line_persist_until = now + AVOID_LINE_PERSIST_TIME
        #Upon detecting the line,the strophotaxis cycle change will become inactive as the change was not recent enough
        stro_cycle_change = CYCLE_CHANGE_INACTIVE
    
    #If the current tick hasn't reached the tick avoid line persists until:
    if now < avoid_line_persist_until:
        if side==SIDE_LEFT:
            # turn to right
            ret_l = MOTOR_SPEED
            ret_r = -MOTOR_SPEED
        else: # SIDE_RIGHT
            # turn to left
            ret_l = -MOTOR_SPEED
            ret_r = MOTOR_SPEED
            
    return ret_l,ret_r
    
    

def behaviour_strophotaxis(side):
    """Behaviour to switch turning side at a certain interval
       Function takes as parameter the current side the robot is monitoring the ground sensor and moving towards 
       Function is called at every tick.
       Returns nothing in case a change is made to the cycle, just changes turning_side variable
       Otherwise, returns None to notify that no change was made to the turning side."""
       
    global turning_side, COUNT, stro_cycle_change, THRESH_DURATION
    
    #Increments the ticker count by 1 at every tick
    COUNT = COUNT + 1
    
    #If ticker count reaches cycle maximum duration:
    if COUNT == THRESH_DURATION:
        if side == SIDE_RIGHT:
            #resets ticker count to restart counting the new cycle ticks
            COUNT = 0
            #sets the cycle change to active as cycle was changed
            stro_cycle_change = CYCLE_CHANGE_ACTIVE
            #changes the side the robot is monitoring the ground sensors and turning to the right side
            turning_side = SIDE_LEFT
        elif side == SIDE_LEFT:
            #resets ticker count to restart counting the new cycle ticks
            COUNT = 0
            #sets the cycle change to active as cycle was changed
            stro_cycle_change = CYCLE_CHANGE_ACTIVE
            #changes the side the robot is monitoring the ground sensors and turning to the right side
            turning_side = SIDE_RIGHT
    else: 
    #There is no change in the cycle as the current cycle duration has not reached the threshold:
        return None

#
#COORDINATION FUNCTION
#
       
def coordination_subsumption(side):
    """ Coordinate behaviours using a subsumption architecture. 
        Function takes as parameter the current side the robot is monitoring the ground sensor and moving towards 
    """
    global m_spd_l, m_spd_r
    
    # Run all the sub-behaviours
    behaviour_strophotaxis(side)
    b_adv_l,b_adv_r = behaviour_advance(side)
    b_app_l,b_app_r = behaviour_approach_line(side)
    b_avd_l,b_avd_r = behaviour_avoid_line(side)
    
    #State of cycle can supress Approach_line output by overwriting it with Advance output
    if stro_cycle_change == CYCLE_CHANGE_INACTIVE:
       b_app_l = b_adv_l
       b_app_r = b_adv_r
       
    # Output from avoid_line can inhibit that of approach_line
    if b_avd_l is not None:
        b_app_l = b_avd_l
    if b_avd_r is not None:
        b_app_r = b_avd_r
        
    # Output from approach_line can inhibit that of Advance
    if b_app_l is not None:
        b_adv_l = b_app_l
    if b_app_r is not None:
        b_adv_r = b_app_r
    
    # Output (possibly overwritten) from Advance goes to motors
    m_spd_l = b_adv_l
    m_spd_r = b_adv_r


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
    coordination_subsumption(turning_side)

    # Send actuator commands:
    send_actuator_values()


# End of Main loop
# Exit & cleanup code.
# (none required)