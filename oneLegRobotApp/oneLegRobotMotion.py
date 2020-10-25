import agx
import agxSDK
import agxCollide
import agxRender
import oneLegRobotApp
import math

# Import shapes as .obj files
first_joint_shape = oneLegRobotApp.load_shape('assets/bein - ledd 1 v2.obj')

# Debugging variable
debugging = bool(True)

# Define the initial parameters of the robot legs lengths
L_1 = int(120)              # Section length of first aft section
L_2 = int(240)              # Section length of second aft section
L_3 = L_1                   # Section length of first fwd section
L_4 = L_2                   # Section length of second fwd section
L_5 = int(180)              # Spacing between BLDC motor A1 (aft) and A2 (fwd)
restX = L_5 / 2             # Initial resting x position of foot
restY = 150                 # Initial resting y position of foot

# Variables for kinematic model
x_A1 = 0
y_A1 = 0
x_A2 = 0
y_A2 = 0
theta_1 = 0
theta_2 = 0
theta_3 = 0
theta_4 = 0
theta_5 = 0
theta_6 = 0
theta_7 = 0
theta_8 = 0
theta_9 = 0
theta_10 = 0

# Variables
angleMotorAft = math.degrees(0)
angleMotorFwd = math.degrees(0)

# Local coordinate system from center equally spaced from BLDC motor A1 and A2
x_local = 0
y_local = 0
z_local = 0

class OneLegRobotMotion(agxSDK.StepEventListener):
    Atest = 0
    # def __init__(self, oneLegRobot):
    #     super().__init__(agxSDK.StepEventListener.PRE_STEP)

class AftFirstJoint(agx.RigidBody):
    def __init__(self):
        super().__init__()

        first_joint_geometry = agxCollide.Geometry(first_joint_shape.deepCopy())
        first_joint_geometry.setEnableCollisions(True)
        oneLegRobotApp.create_visual(first_joint_geometry, agxRender.Color.Black())

        self.add(first_joint_geometry)

def calculate_new_angle_aft_joint():
    aftMotorAngle = oneLegRobotApp.aftMotorAngle

def calculate_motor_angle(x, y):


    aft_motor_angle = calculate_motor_angle_aft(x, y)
    fwd_motor_angle = calculate_motor_angle_fwd(x, y)

    print_debug_list()

    return aft_motor_angle



def calculate_motor_angle_aft(x, y):
    # Calculate for aft motor
    global x_A1, y_A1, theta_1, theta_2, theta_3, theta_4, theta_9
    x_A1 = restX + x
    y_A1 = restY - y
    theta_1 = 180 - math.degrees(math.atan2(y_A1, x_A1))
    theta_4 = math.degrees(
        math.acos((math.pow(x_A1, 2) + math.pow(y_A1, 2) - math.pow(L_1, 2) - math.pow(L_2, 2)) / (2 * L_1 * L_2)))
    theta_9 = math.degrees(
        math.asin((L_1 / math.sqrt(math.pow(x_A1,2) + math.pow(y_A1, 2))) * math.sin(math.radians(theta_4)) ))
    theta_3 = 180 - (180-theta_4) - theta_9
    if (theta_1 - theta_3) <= 0:
        theta_2 = 180 + (theta_1 - theta_3)
    else:
        theta_2 = 180 + (theta_1 - theta_3)

    def aft_first_section_outer_position(x, y):
        r_A1 = L_1
        x_1 = r_A1 * math.cos(math.degrees(theta_2))
        y_1 = r_A1 * math.sin(math.degrees(theta_2))

        return x_1, y_1

    return theta_2

def calculate_motor_angle_fwd(x, y):
    # Calculate for fwd motor
    global x_A2, y_A2, theta_5, theta_6, theta_7, theta_8, theta_10
    x_A2 = restX - x
    y_A2 = restY - y
    theta_5 = 180 - math.degrees(math.atan2(y_A2, x_A2))
    theta_8 = math.degrees(
        math.acos((math.pow(x_A2, 2) + math.pow(y_A2, 2) - math.pow(L_3, 2) - math.pow(L_4, 2)) / (2 * L_3 * L_4)))
    theta_10 = math.degrees(
        math.asin((L_3 / math.sqrt(math.pow(x_A2, 2) + math.pow(y_A2, 2))) * math.sin(math.radians(theta_8))))
    theta_7 = 180 - (180 - theta_8) - theta_10
    if (theta_5 - theta_7) <= 0:
        theta_6 = abs((theta_5 - theta_7))
    else:
        theta_6 = 360 - (theta_5 - theta_7)

    return theta_6

def get_debug():
    return debugging

def print_debug_list():
    if get_debug():
        print("---- DATA FOR AFT SECTION ----")
        print("x_A1 pos: ", x_A1)
        print("y_A1 pos: ", y_A1)
        print("Angle theta_1 is: ", theta_1)
        print("Angle theta_4 is: ", theta_4)
        print("Angle theta_9 is: ", theta_9)
        print("Angle theta_3 is: ", theta_3)
        print("Angle theta_2 is: ", theta_2)
        print("")

    if get_debug():
        print("---- DATA FOR FWD SECTION ----")
        print("x_A2 pos: ", x_A2)
        print("y_A2 pos: ", y_A2)
        print("Angle theta_5 is: ", theta_5)
        print("Angle theta_8 is: ", theta_8)
        print("Angle theta_10 is: ", theta_10)
        print("Angle theta_7 is: ", theta_7)
        print("Angle theta_6 is: ", theta_6)
        print("")