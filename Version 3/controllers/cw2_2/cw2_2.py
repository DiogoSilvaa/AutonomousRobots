"""cw2_2 controller."""
# An implementation of an active decision-making architecture.
# Thymio robot confronts action selection problems and has autonomous solutions, acting accordingly.
# Robot needs all three resources to survive, water, energy and food.
# All resources start at the maximum value (100) and will decrease at different speeds to mimic real-life.
# Hunger decreases at a slower rate (0.002 per tick).
# Energy and water decrease at a faster rate (0.003 per tick).
# When consumming any resource the robot will stop moving and rise up the physiology variable up to 100.
# There are differences for each resource when consumming:
# Food: The hunger will rise at a possibly slower rate than other both resources (up to 0.06 per tick).
# Food: By every tick spent consuming food the robot will slow down (0.05 per tick), until the robot gets to the minimum speed of 1.
# Water: The thirst will rise at a possibly faster rate than hunger (up to 0.09 per tick).
# Energy: The energy will rise at a possibly faster rate than hunger (up to 0.09 per tick).
# Energy: By every tick spent consuming energy the robot will lose thirst (0.02 per tick).
# When any of these variables reach 0, the robot will die.  
# Based on trace_maker controller and task 1 controller.

from controller import Robot 
import random
from datetime import datetime

# Hardware constants
DIST_SENS_COUNT = 7
DIST_SENS_FRONT_COUNT = 5
GRND_SENS_COUNT = 2
TS_NAMES = ('center','left','right','forward','backward')

# Global variable to hold motor speed
MOTOR_SPEED = 3

#Constants to keep a count for the behaviour_curiosity()
CURIOS_COUNT = 0 
THRESH_CURIOS_COUNT = 600
CURIOS_RESET = 1200

#Constants to keep a count for the behaviour_advance()
TURN_COUNT = 0
THRESH_TURN_COUNT = 300
TURN_RESET = 600

#Constants to keep a count for the behaviour_navigate()
NAVI_COUNT = 0
THRESH_NAVI_COUNT = 3000
NAVI_RESET = 6000

#Energy (Green) Resource Colour Constants
THRESH_GREEN_CLUE1_TOP = 713
THRESH_GREEN_CLUE1_BOTTOM = 682
THRESH_GREEN_CLUE2_TOP = 450
THRESH_GREEN_CLUE2_BOTTOM = 420
THRESH_GREEN_CLUE3_TOP = 340
THRESH_GREEN_CLUE3_BOTTOM = 320
THRESH_GREEN_RESOURCE_TOP = 290
THRESH_GREEN_RESOURCE_BOTTOM = 270

#Water (Blue) Resource Colour Constants
THRESH_BLUE_CLUE1_TOP = 750
THRESH_BLUE_CLUE1_BOTTOM = 710
THRESH_BLUE_CLUE2_TOP = 680
THRESH_BLUE_CLUE2_BOTTOM = 645
THRESH_BLUE_CLUE3_TOP = 520
THRESH_BLUE_CLUE3_BOTTOM = 495
THRESH_BLUE_RESOURCE_TOP = 400
THRESH_BLUE_RESOURCE_BOTTOM = 370

#Food (Black) Resource Colour Constants
THRESH_BLACK_CLUE1_TOP = 810
THRESH_BLACK_CLUE1_BOTTOM = 765
THRESH_BLACK_CLUE2_TOP = 610
THRESH_BLACK_CLUE2_BOTTOM = 580
THRESH_BLACK_CLUE3_TOP = 470
THRESH_BLACK_CLUE3_BOTTOM = 455
THRESH_BLACK_RESOURCE_TOP = 150
THRESH_BLACK_RESOURCE_BOTTOM = 130

#Floor (White) Constants
THRESH_FLOOR_TOP = 860
THRESH_FLOOR_BOTTOM = 812

#Variable to hold robot health state
health_state = 100


#Robot Physiology (higher is better)
hunger = 100
thirst = 100 
energy = 100

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
    global ds, ds_val, gs, gs_val, ts, ts_val, rw_button_state

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
        or False otherwise.
        Function is called every tick by behaviour_navigate() function. """
    global ds_val
    
    for i in range(DIST_SENS_FRONT_COUNT):
        if ds_val[i]>0:
            return True
    return False # Nothing detected in loop

#
#MOVEMENT FUNCTIONS
#

def behaviour_advance():
    """ Behavior to move forward avoiding obstacles.
        Returns left and right motor values. 
        Function is called everytime an object is detected ahead."""
    global ds_val

    ret_l = MOTOR_SPEED
    ret_r = MOTOR_SPEED
    
    """Depending on turn_count value, different weights are given to the robot distance sensors.
       This was implemented to avoid the robot being stuck in the corners of the arena. This way,
       the robot can only get stuck for the seconds equivalent to the integer given to THRESH_TURN_COUNT (300).
       This means the robot will change weights every 300 ticks or 30 seconds.
       First set of weights favour the turning towards the right and second set to the left."""
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
        ret_l = MOTOR_SPEED
        ret_r = MOTOR_SPEED
    # If count==0 use default values above
    
    return ret_l,ret_r
    
    
def behaviour_curiosity():
    """Makes the robot navigate in "zig-zag pattern", turning left and right every certain seconds (given by THRESH_CURIOS_COUNT).
    This behaviour was implemented to make the robot try to get closer to the resource when he is getting clues.
    Returns left and right motor values for speed.
    Function is called everytime the robot's ground sensors detect the most desired resource's values."""
    
    
    #Constant given to THRESH_CURIOS_COUNT determines how long the robot turns for each side. In this case, 6 seconds each side.
    if CURIOS_COUNT < THRESH_CURIOS_COUNT:
        #turns left
        ret_l = MOTOR_SPEED
        ret_r = MOTOR_SPEED*0.25 
    else:
        #turns right
        ret_l = MOTOR_SPEED*0.25
        ret_r = MOTOR_SPEED
    return ret_l, ret_r


def behaviour_navigate(winner):
    """ Function takes winner as a parameter. This parameter is related to the Winner-takes-all architecture. Winner is output
    of the function that determines which behaviours' motivation is bigger. After taking the parameter, behaviour_navigate() executes
    the behaviour respective to the winner parameter. If no motivation is bigger, the robot will move randomly.
    Function returns left and right motor values.
    Function is called every tick.""" 
    
    #Checks if there is an object ahead, if so function does nothing.
    if object_ahead():
        return None, None    
        
    #Behaviour selection based on parameter.
    if winner == "hng":
        #Appetitive Hunger behaviour is ran.
        ret_l, ret_r = behaviour_appetitive_hunger()
    elif winner == "tht":
        #Appetitive Thirst behaviour is ran.
        ret_l, ret_r = behaviour_appetitive_thirst()
    elif winner == "nrg":
        #Appetitive Energy behaviour is ran.
        ret_l, ret_r = behaviour_appetitive_energy()
    else:        
    #No motivation is bigger than the others
    #Move randomly, changing turning sides every certain seconds (equivalent to the constant given to THRESH_NAVI_COUNT), in this
    #case, 30 seconds.
        if NAVI_COUNT < THRESH_NAVI_COUNT:
            #Turn right
            ret_l = MOTOR_SPEED*0.85
            ret_r = MOTOR_SPEED
        else:
            #Turn left
            ret_l = MOTOR_SPEED
            ret_r = MOTOR_SPEED*0.85
    
    """If a motivation is indeed bigger than the others, but no ground sensor detects the desired values
    Appetitive behaviour's function will return None. Thus, this code sets the same parameters as above would for no motivation
    being bigger than the others.
    """ 
    if ret_l is None:
        if NAVI_COUNT < THRESH_NAVI_COUNT:
            ret_l = MOTOR_SPEED*0.85
        else:
            ret_l = MOTOR_SPEED
    if ret_r is None:
        if NAVI_COUNT < THRESH_NAVI_COUNT:
            ret_r = MOTOR_SPEED
        else:
            ret_r = MOTOR_SPEED*0.85

    return ret_l, ret_r
    
    
def health_check():   
    """Function that checks robot health state.
    If any physiologic variable is equal or below zero, the robot dies.
    When robot dies, sensors and actuators stop working
    Function is called at every tick.
    Function returns left and right motor values."""
    
    global health_state
    
    #Determines the lowest physiological variable and holds it in health state
    if energy < health_state:
        health_state = energy
    if thirst < health_state:
        health_state = thirst
    if hunger < health_state:
        health_state = hunger
    
    #If health_state is equal or below 0:        
    if health_state <= 0:
    #Robot stops working
        ret_l = 0
        ret_r = 0
    else:
    #If health is fine, function does nothing
        ret_l = None
        ret_r = None
        
    return ret_r, ret_l

#    
#MOTIVATION PROCESSING FUNCTIONS
#

def motivation_hunger():
    """Function determines the stimulus being given to the robot for the robot to look for food. Secondly,
    the deficit is calculated. Finally, it calculates the overall motivation to execute the behaviour_appetitive_hunger()
    Function returns motivation value.
    Function is called every tick by wta_behaviour_selection()."""    
    
    
    #Initial stimulus reset.
    stimulus = 0
    
        #Clue 1: if the one of the sensors of the robot is detecting the values that belong to the black resource's first clue
        #(weakest stimulus), it will be given a stimulus of 0.025. If both sensors detect, a stimulus of 0.05 will be given.
    if THRESH_BLACK_CLUE1_BOTTOM <= gs_val[0] <= THRESH_BLACK_CLUE1_TOP:
        stimulus = stimulus + 0.025
    if THRESH_BLACK_CLUE1_BOTTOM <= gs_val[1] <= THRESH_BLACK_CLUE1_TOP:
        stimulus = stimulus + 0.025
        
        """Clue 2:
        If the one of the sensors of the robot is detecting the values that belong to the black resource's second clue
        (second weakest stimulus), it will be given a stimulus of 0.075. If both sensors detect, a stimulus of 0.075 will be given."""
    if THRESH_BLACK_CLUE2_BOTTOM <= gs_val[0] <= THRESH_BLACK_CLUE2_TOP:
        stimulus = stimulus + 0.075
    if THRESH_BLACK_CLUE2_BOTTOM <= gs_val[1] <= THRESH_BLACK_CLUE2_TOP:
        stimulus = stimulus + 0.075 
        
        """Clue 3:
        If the one of the sensors of the robot is detecting the values that belong to the black resource's final clue
        (second strongest), it will be given a stimulus of 0.25. If both sensors detect, a stimulus of 0.5 will be given."""
    if THRESH_BLACK_CLUE3_BOTTOM <= gs_val[0] <= THRESH_BLACK_CLUE3_TOP:
        stimulus = stimulus + 0.25
    if THRESH_BLACK_CLUE3_BOTTOM <= gs_val[1] <= THRESH_BLACK_CLUE3_TOP:
        stimulus = stimulus + 0.25
        
        """Resource:
        If the one of the sensors of the robot is detecting the values that belong to the black resource itself
        (strongest),it will be given a stimulus of 0.33. If both sensors detect, a stimulus of 0.66 will be given."""   
    if THRESH_BLACK_RESOURCE_BOTTOM <= gs_val[0] <= THRESH_BLACK_RESOURCE_TOP:
        stimulus = stimulus + 0.33
    if THRESH_BLACK_RESOURCE_BOTTOM <= gs_val[1] <= THRESH_BLACK_RESOURCE_TOP:
        stimulus = stimulus + 0.33 
        
    #Deficit is calculated, which is the missing part of the variable hunger (which can go up to 100)   
    deficit = 100 - hunger
    
    #Motivation is calculated.
    motivation = deficit + (deficit * stimulus)
    
    return motivation



def motivation_thirst():
    """Function determines the stimulus being given to the robot for the robot to look for water. Secondly,
    the deficit is calculated. Finally, it calculates the overall motivation to execute the behaviour_appetitive_thirst()
    Function returns motivation value.
    Function is called every tick by wta_behaviour_selection().""" 
    
    
    #Initial stimulus reset.
    stimulus = 0
    
        #Clue 1:
        #If the one of the sensors of the robot is detecting the values that belong to the blue resource's first clue
        #(weakest stimulus), it will be given a stimulus of 0.025. If both sensors detect, a stimulus of 0.05 will be given.
    if THRESH_BLUE_CLUE1_BOTTOM <= gs_val[0] <= THRESH_BLUE_CLUE1_TOP:
        stimulus = stimulus + 0.025
    if THRESH_BLUE_CLUE1_BOTTOM <= gs_val[1] <= THRESH_BLUE_CLUE1_TOP:
        stimulus = stimulus + 0.025
        
        """Clue 2:
        If the one of the sensors of the robot is detecting the values that belong to the blue resource's second clue
        (second weakest stimulus), it will be given a stimulus of 0.075. If both sensors detect, a stimulus of 0.075 will be given."""
    if THRESH_BLUE_CLUE2_BOTTOM <= gs_val[0] <= THRESH_BLUE_CLUE2_TOP:
        stimulus = stimulus + 0.075
    if THRESH_BLUE_CLUE2_BOTTOM <= gs_val[1] <= THRESH_BLUE_CLUE2_TOP:
        stimulus = stimulus + 0.075 
        
        """Clue 3:
        If the one of the sensors of the robot is detecting the values that belong to the blue resource's final clue
        (second strongest), it will be given a stimulus of 0.25. If both sensors detect, a stimulus of 0.5 will be given."""
    if THRESH_BLUE_CLUE3_BOTTOM <= gs_val[0] <= THRESH_BLUE_CLUE3_TOP:
        stimulus = stimulus + 0.25
    if THRESH_BLUE_CLUE3_BOTTOM <= gs_val[1] <= THRESH_BLUE_CLUE3_TOP:
        stimulus = stimulus + 0.25
        
        """Resource:
        If the one of the sensors of the robot is detecting the values that belong to the blue resource itself
        (strongest),it will be given a stimulus of 0.33. If both sensors detect, a stimulus of 0.66 will be given."""    
    if THRESH_BLUE_RESOURCE_BOTTOM <= gs_val[0] <= THRESH_BLUE_RESOURCE_TOP:
        stimulus = stimulus + 0.33
    if THRESH_BLUE_RESOURCE_BOTTOM <= gs_val[1] <= THRESH_BLUE_RESOURCE_TOP:
        stimulus = stimulus + 0.33
        
    #Deficit is calculated, which is the missing part of the variable thirst (which can go up to 100)           
    deficit = 100 - thirst
    
    #Motivation is calculated.
    motivation = deficit + (deficit * stimulus)
    
    return motivation



def motivation_energy():
    """Function determines the stimulus being given to the robot for the robot to look for water. Secondly,
    the deficit is calculated. Finally, it calculates the overall motivation to execute the behaviour_appetitive_thirst()
    Function returns motivation value.
    Function is called every tick by wta_behaviour_selection().""" 
    
    
    #Initial stimulus reset.
    stimulus = 0
    
        #Clue 1:
        #If the one of the sensors of the robot is detecting the values that belong to the blue resource's first clue
        #(weakest stimulus), it will be given a stimulus of 0.025. If both sensors detect, a stimulus of 0.05 will be given.
    if THRESH_GREEN_CLUE1_BOTTOM <= gs_val[0] <= THRESH_GREEN_CLUE1_TOP:
        stimulus = stimulus + 0.025
    if THRESH_GREEN_CLUE1_BOTTOM <= gs_val[1] <= THRESH_GREEN_CLUE1_TOP:
        stimulus = stimulus + 0.025
        
        """Clue 2:
        If the one of the sensors of the robot is detecting the values that belong to the blue resource's second clue
        (second weakest stimulus), it will be given a stimulus of 0.075. If both sensors detect, a stimulus of 0.075 will be given."""
    if THRESH_GREEN_CLUE2_BOTTOM <= gs_val[0] <= THRESH_GREEN_CLUE2_TOP:
        stimulus = stimulus + 0.075
    if THRESH_GREEN_CLUE2_BOTTOM <= gs_val[1] <= THRESH_GREEN_CLUE2_TOP:
        stimulus = stimulus + 0.075
        
        """Clue 3:
        If the one of the sensors of the robot is detecting the values that belong to the blue resource's final clue
        (second strongest), it will be given a stimulus of 0.25. If both sensors detect, a stimulus of 0.5 will be given."""
    if THRESH_GREEN_CLUE3_BOTTOM <= gs_val[0] <= THRESH_GREEN_CLUE3_TOP:
        stimulus = stimulus + 0.25
    if THRESH_GREEN_CLUE3_BOTTOM <= gs_val[1] <= THRESH_GREEN_CLUE3_TOP:
        stimulus = stimulus + 0.25
        
        """Resource:
        If the one of the sensors of the robot is detecting the values that belong to the blue resource itself
        (strongest),it will be given a stimulus of 0.33. If both sensors detect, a stimulus of 0.66 will be given."""       
    if THRESH_GREEN_RESOURCE_BOTTOM <= gs_val[0] <= THRESH_GREEN_RESOURCE_TOP:
        stimulus = stimulus + 0.33
    if THRESH_GREEN_RESOURCE_BOTTOM <= gs_val[1] <= THRESH_GREEN_RESOURCE_TOP:
        stimulus = stimulus + 0.33
    
    #Deficit is calculated, which is the missing part of the variable thirst (which can go up to 100)               
    deficit = 100 - energy
    
    #Motivation is calculated.
    motivation = deficit + (deficit * stimulus)
    return motivation

#
#CONSUMMATORY BEHAVIOUR FUNCTIONS
#

def behaviour_consummatory_hunger():
    """Function emulates the consumming behaviour by rising the hunger level in this case.
    As a consequence, the robot slows down.
    Function returns left and right motor speed values.
    This function is called everytime both sensors detect the resource and behaviour_appetitive_hunger()
    has the biggest motivation to be executed."""
     
    global hunger, MOTOR_SPEED
    
    #Motor speeds set to 0, stopping the robot
    ret_l = 0
    ret_r = 0
    
    #Decrease the robot speed value by 0.05 per tick, up to a minimum of 1.
    if MOTOR_SPEED > 1:
        MOTOR_SPEED = MOTOR_SPEED - 0.05
        
    """Increase hunger by up to 0.06 per tick (maximum hunger amount is 100). Hunger increase value is multiplied by
    a random value that ranges from 0 to 1, adding a random factor to the hunger increase."""      
    if hunger < 99.9999:
        #print("Food:", 0.06*random.random())
        hunger = hunger + 0.06*random.random()
        
    return ret_l, ret_r


def behaviour_consummatory_thirst():
    """Function emulates the consumming behaviour by rising the thirst level in this case.
    Function returns left and right motor speed values.
    This function is called everytime both sensors detect the resource and behaviour_appetitive_thirst()
    has the biggest motivation to be executed."""
    
    global thirst
    
    #Motor speeds set to 0, stopping the robot   
    ret_l = 0
    ret_r = 0
    
    """Increase thirst by up to 0.09 per tick (maximum thirst amount is 100). Thirst increase value is multiplied by
    a random value that ranges from 0 to 1, adding a random factor to the thirst increase."""
    if thirst < 99.9999:
        #print("Water:", 0.09*random.random())
        thirst = thirst + 0.09*random.random()
        
    return ret_l, ret_r
   
    
def behaviour_consummatory_energy():
    """Function emulates the consumming behaviour by rising the energy level in this case.
    As a consequence, the robot loses thirst.
    Function returns left and right motor speed values.
    This function is called everytime both sensors detect the resource and behaviour_appetitive_energy()
    has the biggest motivation to be executed."""
    
    global energy, thirst
    
    #Motor speeds set to 0, stopping the robot
    ret_l = 0
    ret_r = 0
    
    #Decrease the robot thirst value by 0.02 per tick.
    thirst = thirst - 0.02
    
    """Increase energy by up to 0.09 per tick (maximum energy amount is 100). Energy increase value is multiplied by
    a random value that ranges from 0 to 1, adding a random factor to the energy increase."""
    if energy < 99.9999:
        #print("Energy:", 0.09*random.random())
        energy = energy + 0.09*random.random()
    
    return ret_l, ret_r

#    
#APPETITIVE BEHAVIOUR SELECTION FUNCTIOn
#
    
def wta_behaviour_selection():
    """Function processes the different motivations in the winner takes it all type of architecture.
       Function is called every tick by the coordination_subsumption() function.
       Function returns the variable 'winner' that holds the string respective to the biggest motivation."""
     
     #Resets max variable that will hold the winner string.
    max = 0
     
    #Executes all motivations and holds their output in a separate variable.
    hng = motivation_hunger()
    tht = motivation_thirst()
    nrg = motivation_energy()
     
    """Compares all variables that hold the motivation outputted by its respective motivation function.
    Stores the string respective to the biggest motivation in the variable 'winner'.""" 
    if hng > max:
        max = hng
        winner = "hng"
     
    if tht > max:
        max = tht
        winner = "tht"
     
    if nrg > max:
        max = nrg
        winner = "nrg"
     
    return winner

#          
#APPETITIVE FUNCTIONS    
#
    
def behaviour_appetitive_hunger():
    """ Function that orientates the robot towards the food resource because the biggest motivation is to fulfill the hunger.
        Function reads the ground sensor values and processes them to determine the robot's position.
        Returns left and right motor speed values.
        Function is called by the behaviour_navigate() function every time the motivation to fulfill the hunger is the biggest."""    
    
    
    #Processing of the ground sensor readings:
    
    
    #When both sensors read values that belong to the resources value's range:
    if THRESH_BLACK_RESOURCE_BOTTOM <= gs_val[0] <= THRESH_BLACK_RESOURCE_TOP and THRESH_BLACK_RESOURCE_BOTTOM <= gs_val[1] <= THRESH_BLACK_RESOURCE_TOP:
        #Execute behaviour_consummatory_hunger() function, making the robot consumme the resource.
        ret_l, ret_r = behaviour_consummatory_hunger()       
    #When left sensor read a value that belongs to the resources value's range:
    elif THRESH_BLACK_RESOURCE_BOTTOM <= gs_val[0] <= THRESH_BLACK_RESOURCE_TOP:
        #Spin left
        ret_l = 0
        ret_r = MOTOR_SPEED        
    #When right sensor read a value that belongs to the resources value's range:
    elif THRESH_BLACK_RESOURCE_BOTTOM <= gs_val[1] <= THRESH_BLACK_RESOURCE_TOP:
        #Spin right
        ret_l = MOTOR_SPEED
        ret_r = 0      
    
                    
    #When both sensors read values that belong to the resource's clue 3 value's range:
    elif THRESH_BLACK_CLUE3_BOTTOM <= gs_val[0] <= THRESH_BLACK_CLUE3_TOP and THRESH_BLACK_CLUE3_BOTTOM <= gs_val[1] <= THRESH_BLACK_CLUE3_TOP:
        #Execute behaviour_curiosity() function, making the robot move making wider turns to detect bigger stimulus.
        ret_l, ret_r = behaviour_curiosity()           
    #When left sensor read a value that belongs to the resource's clue 3 value's range:
    elif THRESH_BLACK_CLUE3_BOTTOM <= gs_val[0] <= THRESH_BLACK_CLUE3_TOP:
        #Spin left
        ret_l = 0
        ret_r = MOTOR_SPEED 
    #When right sensor read a value that belongs to the resource's clue 3 value's range:
    elif THRESH_BLACK_CLUE3_BOTTOM <= gs_val[1] <= THRESH_BLACK_CLUE3_TOP:
        #Spin right
        ret_l = MOTOR_SPEED
        ret_r = 0 
        
        
         
    #When both sensors read values that belong to the resource's clue 2 value's range:
    elif THRESH_BLACK_CLUE2_BOTTOM <= gs_val[0] <= THRESH_BLACK_CLUE2_TOP and THRESH_BLACK_CLUE2_BOTTOM <= gs_val[1] <= THRESH_BLACK_CLUE2_TOP:
        #Execute behaviour_curiosity() function, making the robot move making wider turns to detect bigger stimulus.        
        ret_l, ret_r = behaviour_curiosity()   
    #When left sensor read a value that belongs to the resource's clue 2 value's range:
    elif THRESH_BLACK_CLUE2_BOTTOM <= gs_val[0] <= THRESH_BLACK_CLUE2_TOP:
        #Spin left
        ret_l = 0
        ret_r = MOTOR_SPEED       
    #When right sensor read a value that belongs to the resource's clue 2 value's range:
    elif THRESH_BLACK_CLUE2_BOTTOM <= gs_val[1] <= THRESH_BLACK_CLUE2_TOP:
        #Spin right
        ret_l = MOTOR_SPEED
        ret_r = 0     
    
    
    
    #When both sensors read values that belong to the resource's clue 1 value's range:
    elif THRESH_BLACK_CLUE1_BOTTOM <= gs_val[0] <= THRESH_BLACK_CLUE1_TOP and THRESH_BLACK_CLUE1_BOTTOM <= gs_val[1] <= THRESH_BLACK_CLUE1_TOP:
        #Execute behaviour_curiosity() function, making the robot move making wider turns to detect bigger stimulus.                
        ret_l, ret_r = behaviour_curiosity()
    #When left sensor read a value that belongs to the resource's clue 1 value's range:
    elif THRESH_BLACK_CLUE1_BOTTOM <= gs_val[0] <= THRESH_BLACK_CLUE1_TOP:
        #Spin left
        ret_l = 0
        ret_r = MOTOR_SPEED
    #When right sensor read a value that belongs to the resource's clue 1 value's range:
    elif THRESH_BLACK_CLUE1_BOTTOM <= gs_val[1] <= THRESH_BLACK_CLUE1_TOP:
        #Spin right
        ret_l = MOTOR_SPEED
        ret_r = 0 
        
        
    #None of the sensors are on the black path, thus, no orientation can be given.                   
    else:
        ret_l = None
        ret_r = None
            
    return ret_l, ret_r




def behaviour_appetitive_thirst():
    """ Function that orientates the robot towards the water resource because the biggest motivation is to fulfill the thirst.
        Function reads the ground sensor values and processes them to determine the robot's position.
        Returns left and right motor speed values.
        Function is called by the behaviour_navigate() function every time the motivation to fulfill the thirst is the biggest."""  
        
    #Processing of the ground sensor readings:
      
      
    #When both sensors read values that belong to the resources value's range:
    if THRESH_BLUE_RESOURCE_BOTTOM <= gs_val[0] <= THRESH_BLUE_RESOURCE_TOP and THRESH_BLUE_RESOURCE_BOTTOM <= gs_val[1] <= THRESH_BLUE_RESOURCE_TOP:
        #Execute behaviour_consummatory_hunger() function, making the robot consumme the resource.
        ret_l, ret_r = behaviour_consummatory_thirst()   
    #When left sensor read a value that belongs to the resources value's range:
    elif THRESH_BLUE_RESOURCE_BOTTOM <= gs_val[0] <= THRESH_BLUE_RESOURCE_TOP:
        #Spin left
        ret_l = 0
        ret_r = MOTOR_SPEED   
    #When right sensor read a value that belongs to the resources value's range:
    elif THRESH_BLUE_RESOURCE_BOTTOM <= gs_val[1] <= THRESH_BLUE_RESOURCE_TOP:
        #Spin right
        ret_l = MOTOR_SPEED
        ret_r = 0
        
        
                    
    #When both sensors read values that belong to the resource's clue 3 value's range:
    elif THRESH_BLUE_CLUE3_BOTTOM <= gs_val[0] <= THRESH_BLUE_CLUE3_TOP and THRESH_BLUE_CLUE3_BOTTOM <= gs_val[1] <= THRESH_BLUE_CLUE3_TOP:
        #Execute behaviour_curiosity() function, making the robot move making wider turns to detect bigger stimulus.
        ret_l, ret_r = behaviour_curiosity()   
    #When left sensor read a value that belongs to the resource's clue 3 value's range:
    elif THRESH_BLUE_CLUE3_BOTTOM <= gs_val[0] <= THRESH_BLUE_CLUE3_TOP:
        #Spin left
        ret_l = 0
        ret_r = MOTOR_SPEED    
    #When right sensor read a value that belongs to the resource's clue 3 value's range:
    elif THRESH_BLUE_CLUE3_BOTTOM <= gs_val[1] <= THRESH_BLUE_CLUE3_TOP:
        #Spin right
        ret_l = MOTOR_SPEED
        ret_r = 0 
        
        
         
    #When both sensors read values that belong to the resource's clue 2 value's range:
    elif THRESH_BLUE_CLUE2_BOTTOM <= gs_val[0] <= THRESH_BLUE_CLUE2_TOP and THRESH_BLUE_CLUE2_BOTTOM <= gs_val[1] <= THRESH_BLUE_CLUE2_TOP:
        #Execute behaviour_curiosity() function, making the robot move making wider turns to detect bigger stimulus.        
        ret_l, ret_r = behaviour_curiosity()   
    #When left sensor read a value that belongs to the resource's clue 2 value's range:
    elif THRESH_BLUE_CLUE2_BOTTOM <= gs_val[0] <= THRESH_BLUE_CLUE2_TOP:
        #Spin left
        ret_l = 0
        ret_r = MOTOR_SPEED       
    #When right sensor read a value that belongs to the resource's clue 2 value's range:
    elif THRESH_BLUE_CLUE2_BOTTOM <= gs_val[1] <= THRESH_BLUE_CLUE2_TOP:
        #Spin right
        ret_l = MOTOR_SPEED
        ret_r = 0     
    
    
    
    #When both sensors read values that belong to the resource's clue 1 value's range:
    elif THRESH_BLUE_CLUE1_BOTTOM <= gs_val[0] <= THRESH_BLUE_CLUE1_TOP and THRESH_BLUE_CLUE1_BOTTOM <= gs_val[1] <= THRESH_BLUE_CLUE1_TOP:
        #Execute behaviour_curiosity() function, making the robot move making wider turns to detect bigger stimulus.                
        ret_l, ret_r = behaviour_curiosity()
    #When left sensor read a value that belongs to the resource's clue 1 value's range:
    elif THRESH_BLUE_CLUE1_BOTTOM <= gs_val[0] <= THRESH_BLUE_CLUE1_TOP:
        #Spin left
        ret_l = 0
        ret_r = MOTOR_SPEED
    #When right sensor read a value that belongs to the resource's clue 1 value's range:
    elif THRESH_BLUE_CLUE1_BOTTOM <= gs_val[1] <= THRESH_BLUE_CLUE1_TOP:
        #Spin right
        ret_l = MOTOR_SPEED
        ret_r = 0 
        
        
    #None of the sensors are on the blue path, thus, no orientation can be given.                   
    else:
        ret_l = None
        ret_r = None
            
    return ret_l, ret_r
    
    
    
        
def behaviour_appetitive_energy():
    """ Function that orientates the robot towards the energy resource because the biggest motivation is to fulfill the energy.
        Function reads the ground sensor values and processes them to determine the robot's position.
        Returns left and right motor speed values.
        Function is called by the behaviour_navigate() function every time the motivation to fulfill the energy is the biggest."""  
        
    #Processing of the ground sensor readings:
      
      
    #When both sensors read values that belong to the resources value's range:
    if THRESH_GREEN_RESOURCE_BOTTOM <= gs_val[0] <= THRESH_GREEN_RESOURCE_TOP and THRESH_GREEN_RESOURCE_BOTTOM <= gs_val[1] <= THRESH_GREEN_RESOURCE_TOP:
        #Execute behaviour_consummatory_hunger() function, making the robot consumme the resource.
       ret_l, ret_r = behaviour_consummatory_energy()     
    #When left sensor read a value that belongs to the resources value's range:
    elif THRESH_GREEN_RESOURCE_BOTTOM <= gs_val[0] <= THRESH_GREEN_RESOURCE_TOP:
        #Spin left
        ret_l = 0
        ret_r = MOTOR_SPEED    
    #When right sensor read a value that belongs to the resources value's range:
    elif THRESH_GREEN_RESOURCE_BOTTOM <= gs_val[1] <= THRESH_GREEN_RESOURCE_TOP:
        #Spin right
        ret_l = MOTOR_SPEED
        ret_r = 0 
        
        
                    
    #When both sensors read values that belong to the resource's clue 3 value's range:
    elif THRESH_GREEN_CLUE3_BOTTOM <= gs_val[0] <= THRESH_GREEN_CLUE3_TOP and THRESH_GREEN_CLUE3_BOTTOM <= gs_val[1] <= THRESH_GREEN_CLUE3_TOP:
        #Execute behaviour_curiosity() function, making the robot move making wider turns to detect bigger stimulus.
        ret_l, ret_r = behaviour_curiosity()
    #When left sensor read a value that belongs to the resource's clue 3 value's range:
    elif THRESH_GREEN_CLUE3_BOTTOM <= gs_val[0] <= THRESH_GREEN_CLUE3_TOP:
        #Spin left
        ret_l = 0
        ret_r = MOTOR_SPEED    
    #When right sensor read a value that belongs to the resource's clue 3 value's range:
    elif THRESH_GREEN_CLUE3_BOTTOM <= gs_val[1] <= THRESH_GREEN_CLUE3_TOP:
        #Spin right
        ret_l = MOTOR_SPEED
        ret_r = 0 
        
        
         
    #When both sensors read values that belong to the resource's clue 2 value's range:
    elif THRESH_GREEN_CLUE2_BOTTOM <= gs_val[0] <= THRESH_GREEN_CLUE2_TOP and THRESH_GREEN_CLUE2_BOTTOM <= gs_val[1] <= THRESH_GREEN_CLUE2_TOP:
        #Execute behaviour_curiosity() function, making the robot move making wider turns to detect bigger stimulus.        
        ret_l, ret_r = behaviour_curiosity()   
    #When left sensor read a value that belongs to the resource's clue 2 value's range:
    elif THRESH_GREEN_CLUE2_BOTTOM <= gs_val[0] <= THRESH_GREEN_CLUE2_TOP:
        #Spin left
        ret_l = 0
        ret_r = MOTOR_SPEED       
    #When right sensor read a value that belongs to the resource's clue 2 value's range:
    elif THRESH_GREEN_CLUE2_BOTTOM <= gs_val[1] <= THRESH_GREEN_CLUE2_TOP:
        #Spin right
        ret_l = MOTOR_SPEED
        ret_r = 0     
    
    
    
    #When both sensors read values that belong to the resource's clue 1 value's range:
    elif THRESH_GREEN_CLUE1_BOTTOM <= gs_val[0] <= THRESH_GREEN_CLUE1_TOP and THRESH_GREEN_CLUE1_BOTTOM <= gs_val[1] <= THRESH_GREEN_CLUE1_TOP:
        #Execute behaviour_curiosity() function, making the robot move making wider turns to detect bigger stimulus.                
        ret_l, ret_r = behaviour_curiosity()
    #When left sensor read a value that belongs to the resource's clue 1 value's range:
    elif THRESH_GREEN_CLUE1_BOTTOM <= gs_val[0] <= THRESH_GREEN_CLUE1_TOP:
        #Spin left
        ret_l = 0
        ret_r = MOTOR_SPEED
    #When right sensor read a value that belongs to the resource's clue 1 value's range:
    elif THRESH_GREEN_CLUE1_BOTTOM <= gs_val[1] <= THRESH_GREEN_CLUE1_TOP:
        #Spin right
        ret_l = MOTOR_SPEED
        ret_r = 0 
        
        
    #None of the sensors are on the green path, thus, no orientation can be given.                   
    else:
        ret_l = None
        ret_r = None
              
    return ret_l, ret_r

#    
#COORDINATION FUNCTION
#

def coordination_subsumption():
    """ Coordinate behaviours using a subsumption architecture. """
    global m_spd_l, m_spd_r, MOTOR_SPEED
    global hunger, thirst, energy
    global CURIOS_COUNT, CURIOS_COUNT, NAVI_COUNT, NAVI_RESET, TURN_COUNT, TURN_RESET
            
    #Increases motor speed up to a maximum of 4 at every tick
    if MOTOR_SPEED < 3.999:
        MOTOR_SPEED = MOTOR_SPEED + 0.001
        
    """Ticker count used to switch turning sides in behaviour_curiosity()
       Count (CURIOS_COUNT) is incremented by 1 at every tick.
       Count is resetted if it hits the reset value (CURIOS_RESET)"""
    CURIOS_COUNT = CURIOS_COUNT + 1
    if CURIOS_COUNT == CURIOS_RESET:
        CURIOS_COUNT = 0
        
    """Ticker count used to switch turning sides in behaviour_navigate()
       Count (NAVI_COUNT) is incremented by 1 at every tick.
       Count is resetted if it hits the reset value (NAVI_RESET)"""    
    NAVI_COUNT = NAVI_COUNT + 1
    if NAVI_COUNT == NAVI_RESET:
        NAVI_COUNT = 0  
              
    """Ticker count used to mitigate the case where the robot gets stuck in corners by switching distance sensor weights.
       Count (TURN_COUNT) is incremented by 1 at every tick.
       Count is resetted if it hits the reset value (TURN_RESET)"""     
    TURN_COUNT = TURN_COUNT + 1 
    if TURN_COUNT == TURN_RESET:
        TURN_COUNT = 0
    
    #Decrease hunger, thirst and energy levels by a certain amount at every tick
    hunger = hunger - 0.002
    thirst = thirst - 0.003
    energy = energy - 0.003
    
    
    #Run Winner-Takes-It-All behaviour Selection and holds its ouput in the variable 'winner'
    winner = wta_behaviour_selection()
    
    # Run health check and hold its output in variables.
    b_health_l, b_health_r = health_check()
    
    # Run all the movement behaviours, taking into consideration the winner in the behaviour_navigate(winner), and store the results in different variables 
    b_adv_l, b_adv_r = behaviour_advance()
    b_nav_l, b_nav_r = behaviour_navigate(winner)
   
    
    #Output from behaviour_navigate() can inhibit that of behaviour_advance()
    if b_nav_l is not None:
        b_adv_l = b_nav_l
    if b_nav_r is not None:    
        b_adv_r = b_nav_r
    
    #Output from health_check() can inhibit that of behaviour_advance()
    if b_health_l is not None:
        b_adv_l = b_health_l
        b_adv_r = b_health_r
        
    #Output (possibly overwritten) from Advance goes to motors    
    m_spd_l = b_adv_l
    m_spd_r = b_adv_r 
    
    # Test commands
    #print("Right ground sensor:", gs_val[1])
    #print("Left ground sensor:", gs_val[0])
    #print("Motor Speed", MOTOR_SPEED) 
    print("Hunger:", hunger)
    print("Thirst:", thirst)
    print("Energy:", energy)
    #print("Winner:", winner) 
#
# Variables, constants, functions for logging data
#
 
LOG_FILE_TEMPLATE = '3rp_log_{:s}.csv'

##
## This will be called in the initialisation, before the main loop starts
## 
def init_logging():
    # Starts logging by creating/opening log file
    # Called on initialisation
    global log_file
    now = datetime.now() # filename includes time to be unique
    filename = LOG_FILE_TEMPLATE.format(now.strftime('%Y%m%d%H%M%S'))
    # Print filename to buffer so we can find the file
    print('Logfile: {}'.format(filename))
    log_file = open(filename, 'w', buffering=1) # 1 = line buffering mode


##
## This will be called after the main loop exits.
## May also be called when the robot dies.
##
def stop_logging():
    # Closes log file
    # Called when main loop terminates or robot dies
    global log_file
    if log_file is not None:
        log_file.close()
        log_file = None


##
## This will be called in the main loop, so loggin happens every tick.
## You may want to check the value of basicTImeStep here.
## You could call this less often than every tick, if you wanted to
## reduce the amount of data.
##
def log_data():
    # Called every tick, writes values of physiological variables
    global log_file, hunger, thirst, energy
    if log_file is not None:
        log_file.write('{},{},{}\n'.format(hunger,thirst,energy))
          
#
# Main entry point for code
# 

# Initialisation
init_actuators()
init_sensors()
init_logging()

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    #print(health_state)
    #if health_state is above 0, robot sensors, actuator and functions work, otherwise, robot stops and nothing functions.
    if health_state > 0:
        # Read the sensors:
        read_sensors()
    
        # Process sensor data here.
        reset_actuator_values()
        # Execute behaviours...
        coordination_subsumption()
        log_data()
        # Send actuator commands:
        send_actuator_values()
    

# End of Main loop
# Exit & cleanup code.
stop_logging()
