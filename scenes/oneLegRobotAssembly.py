import agx
import agxCollide
import agxSDK
import agxRender

import math
import oneLegRobotApp

# Import shapes as .obj files
first_joint_shape_aft = oneLegRobotApp.load_shape('assets/bein - ledd 1 v2.obj')
second_joint_shape_aft = oneLegRobotApp.load_shape('assets/bein - ledd 2 v2.obj')
first_joint_shape_fwd = oneLegRobotApp.load_shape('assets/Bein - Ledd 1 v2.obj')
second_joint_shape_fwd = oneLegRobotApp.load_shape('assets/Bein - Ledd 2 v2.obj')

# Initial variables
MODEL_SCALE = 1/100

# Debugging variable
debugging = bool(False)

# Define the initial parameters of the robot legs lengths
L_1 = int(120)              # Section length of first aft section
L_2 = int(240)              # Section length of second aft section
L_3 = L_1                   # Section length of first fwd section
L_4 = L_2                   # Section length of second fwd section
L_5 = int(180)              # Spacing between BLDC motor A1 (aft) and A2 (fwd)
restX = L_5 / 2             # Initial resting x position of foot
restY = 150                 # Initial resting y position of foot

# Robot leg dimensions for width and breadth.
UPPER_LEG_SECTION_WIDTH = 20
LOWER_LEG_SECTION_WIDTH = 20
UPPER_LEG_SECTION_BREADTH = 20
LOWER_LEG_SECTION_BREADTH = 20
UPPER_LEG_SECTION_SIZE = [UPPER_LEG_SECTION_WIDTH*MODEL_SCALE, UPPER_LEG_SECTION_BREADTH*MODEL_SCALE, L_1*MODEL_SCALE]
LOWER_LEG_SECTION_SIZE = [LOWER_LEG_SECTION_WIDTH*MODEL_SCALE, LOWER_LEG_SECTION_BREADTH*MODEL_SCALE, L_2*MODEL_SCALE]

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
angleMotorAft = None
angleMotorFwd = None

# Local coordinate system from center equally spaced from BLDC motor A1 and A2
x_local = 0
y_local = 0
z_local = 0


def create_bodies(position1, position2, sizeUpper, sizeLower, scale, reducedLength):
    # Create upper section.
    upper = create_box_mod(sizeUpper, reducedLength)
    upper.setPosition(agx.Vec3(position1[0], position1[1], position1[2]))

    # Create lower section.
    lower = create_box_mod(sizeLower, reducedLength)
    lower.setPosition(agx.Vec3(position2[0], position2[1], position2[2]))

    return upper, lower

def create_box_mod(size, reducedLength):
    body = agx.RigidBody()
    shape = agxCollide.Box(size[0], size[1], size[2]-reducedLength)
    geometry = agxCollide.Geometry(shape)
    body.add(geometry)

    return body

def create_sphere(position, diam):
    body = agx.RigidBody()
    shape = agxCollide.Sphere(diam)
    geometry = agxCollide.Geometry(shape)
    body.add(geometry)
    body.setPosition(position[0], position[1], position[2])

    return body

def create_joints():
    # Initial setup
    reducedLength = 0.5
    increaseHeight = 2.8
    sizeUpper = UPPER_LEG_SECTION_SIZE
    sizeLower = LOWER_LEG_SECTION_SIZE

    # --------- Create aft section -----------
    posUpperAft = [L_5*MODEL_SCALE, 0, 3.1 + increaseHeight]
    posLowerAft = [L_5*MODEL_SCALE, 0, 1.3 + increaseHeight-1]
    aftUpper, aftLower = create_bodies(position1=posUpperAft, position2=posLowerAft,
                                           sizeUpper=sizeUpper, sizeLower=sizeLower, scale=MODEL_SCALE, reducedLength=reducedLength)

    # Add upper and lower sections of aft part of robot to simulation
    oneLegRobotApp.sim().add(aftUpper)
    oneLegRobotApp.sim().add(aftLower)

    # Create frame for aft motor
    f1 = agx.Frame()
    f1.setLocalTranslate(0, 0, sizeUpper[2]-reducedLength/2)
    f1.setLocalRotate(agx.EulerAngles(math.radians(90), 0, 0))

    # Create hinge to aft motor
    hinge1Range = [-math.pi / 4, math.pi / 4]
    hinge1 = create_hinge_1RB(aftUpper, f1, hinge1Range)
    oneLegRobotApp.sim().add(hinge1)

    # Create frames for upper and lower aft section
    f2 = agx.Frame()
    f2.setLocalTranslate(0, 0, -sizeUpper[2]-reducedLength/2)
    f2.setLocalRotate(agx.EulerAngles(math.radians(90), 0, 0))

    f3 = agx.Frame()
    f3.setLocalTranslate(0, 0, sizeLower[2])
    f3.setLocalRotate(agx.EulerAngles(math.radians(90), 0, 0))

    # Create hinge between upper and lower fwd section
    hinge2Range = [-math.pi/4, math.pi/4]
    hinge2 = create_hinge_2RB(aftUpper, f2, aftLower, f3, hinge2Range)
    oneLegRobotApp.sim().add(hinge2)

    # --------- Create fwd section -----------
    posUpperFwd = [-L_5*MODEL_SCALE, 0, 3.1 + increaseHeight]
    posLowerFwd = [-L_5*MODEL_SCALE, 0, 1.3 + increaseHeight-1]
    fwdUpper, fwdLower = create_bodies(position1=posUpperFwd, position2=posLowerFwd,
                                           sizeUpper=sizeUpper, sizeLower=sizeLower, scale=MODEL_SCALE, reducedLength=reducedLength)

    # Add upper and lower sections of aft part of robot to simulation
    oneLegRobotApp.sim().add(fwdUpper)
    oneLegRobotApp.sim().add(fwdLower)

    # Create frame for forward motor
    f4 = agx.Frame()
    f4.setLocalTranslate(0, 0, sizeUpper[2] - reducedLength / 2)
    f4.setLocalRotate(agx.EulerAngles(math.radians(90), 0, 0))

    # Create hinge to forward motor
    hinge3Range = [-math.pi/4, math.pi/4]
    hinge3 = create_hinge_1RB(fwdUpper, f4, hinge3Range)
    hinge3.getLock1D().setEnable(False)
    oneLegRobotApp.sim().add(hinge3)

    # Create frames for upper and lower fwd section
    f5 = agx.Frame()
    f5.setLocalTranslate(0, 0, -sizeUpper[2]-reducedLength/2)
    f5.setLocalRotate(agx.EulerAngles(math.radians(90), 0, 0))

    f6 = agx.Frame()
    f6.setLocalTranslate(0, 0, sizeLower[2])
    f6.setLocalRotate(agx.EulerAngles(math.radians(90), 0, 0))

    # Create hinge between upper and lower fwd section
    hinge4Range = [-math.pi/4, math.pi/4]
    hinge4 = create_hinge_2RB(fwdUpper, f5, fwdLower, f6, hinge4Range)
    oneLegRobotApp.sim().add(hinge4)

    # Create end effector frame for lower aft and forward section.
    f7 = agx.Frame()
    f7.setLocalTranslate(0, 0, -sizeUpper[2]*2-reducedLength/2)
    f7.setLocalRotate(agx.EulerAngles(math.radians(90), 0, 0))

    f8 = agx.Frame()
    f8.setLocalTranslate(0, 0, -sizeUpper[2]*2-reducedLength/2)
    f8.setLocalRotate(agx.EulerAngles(math.radians(90), 0, 0))

    # --------- Create end effector -----------
    # Create hinge for end effector between lower aft and forward section.
    hinge5Range = [-math.pi/4, math.pi/4]
    hinge5 = create_hinge_2RB(aftLower, f7, fwdLower, f8, hinge5Range)
    oneLegRobotApp.sim().add(hinge5)

    # Create end-effector point as ball.
    f9 = agx.Frame()
    f9.setLocalTranslate(0, 0, 0)
    f9.setLocalRotate(agx.EulerAngles(math.radians(90), 0, 0))

    #foot = create_sphere([0, 0, 1], 0.1)
    #hinge6 = create_hinge_2RB(foot, f7, foot, f8, [-math.pi/4, math.pi/4])
    #oneLegRobotApp.sim().add(foot)
    #oneLegRobotApp.sim().add(hinge6)
    hinge6Range = [-math.pi/4, math.pi/4]
    foot = create_sphere([0, 0, 1], 0.1)
    oneLegRobotApp.sim().add(foot)
    hinge6 = create_hinge_2RB(aftLower, f7, foot, f9, hinge6Range)
    oneLegRobotApp.sim().add(hinge6)

    #f1Pos = frameReader(f1)
    #oneLegRobotApp.sim().add(f1Pos)

    # Make the first motor swing back and forth
    speed_controller_aft = MotorSpeedControllerAft(hinge1, hinge3, 1, 2, aftUpper, fwdUpper, posUpperAft, posUpperFwd)
    oneLegRobotApp.sim().add(speed_controller_aft)

    print("Test angle aft: ", speed_controller_aft.get_angle_aft())


    if (debugging):
        staticAbove1 = agxCollide.Geometry(agxCollide.Box(1.8, 1.8, -1.2))
        staticAbove1.setLocalPosition(0, -2.5, 7 - 1.2)
        oneLegRobotApp.sim().add(staticAbove1)

        staticAbove2 = agxCollide.Geometry(agxCollide.Box(1.8, 2, -2.4))
        staticAbove2.setLocalPosition(0, -2.5, 7 - 1.2 - 2.4 - 2.4 / 2)
        oneLegRobotApp.sim().add(staticAbove2)

def create_floor():
    floor = agxCollide.Geometry(agxCollide.Box(10, 10, 0.1))
    floor.setPosition(0, 0, -0.3)
    oneLegRobotApp.sim().add(floor)

    return floor


def build_scene():


    # Create scenes
    floor = create_floor()
    create_joints()

    # Create moveable floor
    floor_controller = moveFloorController(floor=floor, movement=[0, 0, 1])
    oneLegRobotApp.sim().add(floor_controller)

    # Rendering details
    oneLegRobotApp.app().getSceneDecorator().setEnableShadows(False)
    oneLegRobotApp.app().setEnableDebugRenderer(True)

    #print("Floor pos: ", floor.getPosition())

    # Arrange camera to be centered around center of floor
    oneLegRobotApp.init_camera(eye=agx.Vec3(20, 20, 30), center=floor.getPosition())

class frameReader(agxSDK.StepEventListener):
    def __init__(self, frame):
        super().__init__(agxSDK.StepEventListener.PRE_STEP)

        self.frame = frame

    def pre(self, time):
        #framePos = self.frame.getLocalTranslate()
        print("Frame position: ", self.frame.getTranslate())

class moveFloorController(agxSDK.StepEventListener):
    def __init__(self, floor, movement):
        super().__init__(agxSDK.StepEventListener.PRE_STEP)

        self.floor = floor
        self.interval = 1
        self.speed = 1
        self.last = 0

        self._omega = None
        self._period = None
        self.set_period(2)
        self.amplitude = math.radians(35)
        self._phase = 0
        self.movement = movement

    def get_period(self):
        return self._period

    def set_period(self, period: float):
        f = 1 / period
        self._period = period
        self._omega = 2 * math.pi * f

    def pre(self, time):
        speed = self.amplitude * math.sin(self._omega * time + self._phase)
        print("Height: ", speed)
        self.floor.setPosition(speed * self.movement[0], speed * self.movement[1], speed * self.movement[2])

def create_hinge_2RB(rb1, frame1, rb2, fram2, range):
    hinge = agx.Hinge(rb1, frame1, rb2, fram2)

    # Sets the angular range of the hinge.
    hinge.getRange1D().setRange(range[0], range[1])

    # Sets the compliance of the hinge DOF.
    hinge.setCompliance(1E-12)
    hinge.getMotor1D().setCompliance(1E-10)
    hinge.getMotor1D().setEnable(False)
    hinge.getLock1D().setEnable(False)

    return hinge

def create_hinge_1RB(rb1, frame1, range):
    hinge = agx.Hinge(rb1, frame1)

    # Sets the angular range of the hinge.
    hinge.getRange1D().setRange(range[0], range[1])

    # Sets the compliance of the hinge DOF.
    hinge.setCompliance(1E-12)
    hinge.getMotor1D().setCompliance(1E-10)
    hinge.getMotor1D().setEnable(False)
    hinge.getLock1D().setEnable(False)

    return hinge

class MotorSpeedControllerAft_New(agxSDK.StepEventListener):
    def __init__(self, hinge1, hinge2):
        super().__init__(agxSDK.StepEventListener.PRE_STEP)

        # Assign variables necessary for the listener.
        self.hinge1 = hinge1
        self.hinge2 = hinge2

    def pre(self, time):
        A = 0

class MotorSpeedControllerAft(agxSDK.StepEventListener):
    def __init__(self, motorAft, motorFwd, speed, interval, rb1, rb2, initPosRb1, initPosRb2):
        super().__init__(agxSDK.StepEventListener.PRE_STEP)

        # Enable the motor and set the initial speed
        #motor.getMotor1D().setEnable(True)
        #motor.getMotor1D().setSpeed(speed)

        # Assign some variables that the listener needs
        self.interval = interval
        self.speed = speed
        self.last = 0
        self.motorAft = motorAft
        self.motorFwd = motorFwd
        self.step = 0

        # Aft section
        self.rb1 = rb1
        self.initPosRb1 = initPosRb1
        self.initAftX = self.initPosRb1[0]
        self.initAftY = self.initPosRb1[1]
        self.initAftZ = self.initPosRb1[2]
        self.angleAft = None

        # Forward section
        self.rb2 = rb2
        self.initPosRb2 = initPosRb2
        self.initFwdX = self.initPosRb2[0]
        self.initFwdY = self.initPosRb2[1]
        self.initFwdZ = self.initPosRb2[2]
        self.angleFwd = None

    def get_angle_aft(self):
        return self.angleAft

    def get_angle_fwd(self):
        return self.angleFwd

    def pre(self, time):
        # Count number of steps
        self.step = self.step + 1

        print("")
        print("Loop no.: ", self.step)

        # Aft section
        print("----- Aft section -----")
        self.angleAft = math.degrees(self.motorAft.getAngle())
        print("Angle aft:     ", self.angleAft)
        #aftSectionPos = self.rb1.getPosition()
        #aftX = aftSectionPos[0]
        #aftY = aftSectionPos[1]
        #aftZ = aftSectionPos[2]
        #diffXAft = self.initAftX - aftX
        #diffYAft = self.initAftY - aftY
        #diffZAft = self.initAftZ - aftZ
        #theta_2_fwd_kin = math.degrees(math.atan(diffZAft / diffXAft)) + 90
        #print("X: ", aftX, "     Diff. X: ", diffXAft)
        #print("Y: ", aftY, "     Diff. Y: ", diffYAft)
        #print("Z: ", aftZ, "     Diff. Z: ", diffZAft)
        #print("Theta_2: ", theta_2_fwd_kin)

        # Fwd section
        print("----- Fwd section -----")
        self.angleFwd = math.degrees(self.motorFwd.getAngle())
        print("Angle fwd:     ", self.angleFwd)
        #fwdSectionPos = self.rb2.getPosition()
        #fwdX = fwdSectionPos[0]
        #fwdY = fwdSectionPos[1]
        #fwdZ = fwdSectionPos[2]
        #diffXFwd = self.initFwdX - fwdX
        #diffYFwd = self.initFwdY - fwdY
        #diffZFwd = self.initFwdZ - fwdZ
        #theta_6_fwd_kin = math.degrees(math.atan(diffZFwd / diffXFwd)) + 90
        #print("X: ", fwdX, "     Diff. X: ", diffXFwd)
        #print("Y: ", fwdY, "     Diff. Y: ", diffYFwd)
        #print("Z: ", fwdZ, "     Diff. Z: ", diffZFwd)
        #print("Theta_6: ", theta_6_fwd_kin)

def get_end_effector_pos():
    x = 0

    return x



















# Create two boxes at the given positions, with the specified size
def create_bodies_old(position1, position2, size):



    b1 = agx.RigidBody()
    # geometry = agxCollide.Geometry(first_joint_shape_aft.deepCopy(),
    #                                agx.AffineMatrix4x4.translate(20, 0, 240))
    # geometry.setLocalRotation(agx.EulerAngles(0, 0, 0))
    # geometry.setEnableCollisions(True)
    # b1 = agx.RigidBody(geometry)
    # #rigidBody.setMotionControl(agx.RigidBody.DYNAMICS)
    # #oneLegRobotApp.create_visual(rigidBody, agxRender.Color.Red())




    b2 = agx.RigidBody()
    # geometry = agxCollide.Geometry(second_joint_shape_aft.deepCopy(),
    #                                agx.AffineMatrix4x4.translate(0, 0, 0))
    # geometry.setLocalRotation(agx.EulerAngles(0, 0, 0))
    # #geometry.setEnableCollisions(True)
    # b2 = agx.RigidBody(geometry)

    b1.add(agxCollide.Geometry(agxCollide.Box(size[0], size[1], size[2])))
    b2.add(agxCollide.Geometry(agxCollide.Box(size[0], size[1], size[2])))

    b1.setPosition(agx.Vec3(position1[0], position1[1], position1[2]))
    b2.setPosition(agx.Vec3(position2[0], position2[1], position2[2]))

    return b1, b2


# Create a hinge scene
def create_one_leg_robot_scene():
    # Create floor
    floor = agxCollide.Geometry(agxCollide.Box(250, 250, 10))
    floor.setPosition(0, 0, -400)
    oneLegRobotApp.sim().add(floor)

    size = [0.5, 0.25, 1]
    #b1, b2 = create_bodies([0, 0, 0], [0, 0, - 2.5], size)


    posAftUpper = [0, 0, 0]
    posAftLower = [0, 0, -2.5]
    b1, b2 = build_aft_part(posAftUpper, posAftLower)

    b2.setParentFrame(floor.getParentFrame())

    oneLegRobotApp.sim().add(b1)
    oneLegRobotApp.sim().add(b2)

    f1 = agx.Frame()
    f1.setLocalTranslate(0*MODEL_SCALE, 0*MODEL_SCALE, posAftUpper[2]*MODEL_SCALE)
    f1.setLocalRotate(agx.EulerAngles(0, math.radians(90), 0))

    hinge1 = agx.Hinge(b1, f1)
    oneLegRobotApp.sim().add(hinge1)

    # Make the first motor swing back and forth
    #speed_controller = AlternatingSpeedController(hinge1.getMotor1D(), 1, 2)
    #oneLegRobotApp.sim().add(speed_controller)

    distance = (b2.getPosition() - b1.getPosition()).length()
    f1 = agx.Frame()
    f1.setLocalTranslate(0*MODEL_SCALE, 0*MODEL_SCALE, -distance / 2)
    f1.setLocalRotate(agx.EulerAngles(0, math.radians(90), 0))

    f2 = agx.Frame()
    f2.setLocalTranslate(0*MODEL_SCALE, 0*MODEL_SCALE, distance / 2)
    f2.setLocalRotate(agx.EulerAngles(0, math.radians(90), 0))

    hinge2 = agx.Hinge(b1, f1, b2, f2)
    oneLegRobotApp.sim().add(hinge2)

    return floor

def build_aft_part(pos1, pos2):
    size1 = UPPER_LEG_SECTION_SIZE
    a1 = create_box(pos=pos1, size=size1)
    #a1 = agx.RigidBody()
    #a1_geometry = agxCollide.Geometry(agxCollide.Box(size[0]*MODEL_SCALE, size[1]*MODEL_SCALE, size[2]*MODEL_SCALE))
    #a1.add(a1_geometry)
    #a1_geometry.setPosition(agx.Vec3(pos1[0], pos1[1], pos1[2]))

    size2 = LOWER_LEG_SECTION_SIZE
    a2 = create_box(pos=pos2, size=size2)

    return a1, a2


    #a2 = agx.RigidBody()

    #size = [0.5, 0.25, 1]
    #pos1 = [0, 0, 0]
    #pos2 = [0, 0, -2.5]
    #a1, a2 = create_bodies(position1=pos1, position2=pos2, size=size)

    #return a1_geometry

# def create_box(pos, size):
#     #size = [UPPER_LEG_SECTION_WIDTH, UPPER_LEG_SECTION_BREADTH, L_1]
#     #pos = [0, 0, 0]
#     rigidBody = agx.RigidBody()
#     geometry = agxCollide.Geometry(
#         agxCollide.Box(size[0] * MODEL_SCALE, size[1] * MODEL_SCALE, size[2] * MODEL_SCALE))
#     rigidBody.add(geometry)
#     rigidBody.setPosition(agx.Vec3(pos[0], pos[1], pos[2]))
#
#     return rigidBody

########################################
# Our function which creates the scene
########################################
def build_scene_old():
    #floor = agxCollide.Geometry(agxCollide.Box(10*scale, 10*scale, 0.1*scale))
    #floor.setPosition(0*scale, 0*scale, -4*scale)
    #oneLegRobotApp.sim().add(floor)

    # Create each and every one of the scenes
    floor = create_one_leg_robot_scene()

    oneLegRobotApp.app().getSceneDecorator().setEnableShadows(False)
    oneLegRobotApp.app().setEnableDebugRenderer(True)

    # Arrange camera to be centered on floor
    oneLegRobotApp.init_camera(eye=agx.Vec3(250*MODEL_SCALE, 250*MODEL_SCALE, 250*MODEL_SCALE), center=floor.getPosition())




















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
