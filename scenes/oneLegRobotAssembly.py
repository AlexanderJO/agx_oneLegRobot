import agx
import agxCollide
import agxSDK
import agxRender
from agxRender import Color

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
debugging = bool(True)

# Define the initial parameters of the robot legs lengths
L_1 = int(120)              # Section length of first aft section
L_2 = int(240)              # Section length of second aft section
L_3 = L_1                   # Section length of first fwd section
L_4 = L_2                   # Section length of second fwd section
L_5 = int(180)              # Spacing between BLDC motor A1 (aft) and A2 (fwd)
restX = L_5 / 2             # Initial resting x position of foot
restZ = 150                 # Initial resting y position of foot

# Robot leg dimensions for width and breadth.
UPPER_LEG_SECTION_WIDTH = 20
LOWER_LEG_SECTION_WIDTH = 20
UPPER_LEG_SECTION_BREADTH = 20
LOWER_LEG_SECTION_BREADTH = 20
UPPER_LEG_SECTION_SIZE = [UPPER_LEG_SECTION_WIDTH*MODEL_SCALE, UPPER_LEG_SECTION_BREADTH*MODEL_SCALE, L_1*MODEL_SCALE]
LOWER_LEG_SECTION_SIZE = [LOWER_LEG_SECTION_WIDTH*MODEL_SCALE, LOWER_LEG_SECTION_BREADTH*MODEL_SCALE, L_2*MODEL_SCALE]

# Variables for kinematic model
x_A1 = 0
z_A1 = 0
x_A2 = 0
z_A2 = 0
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
angle_motor_aft = None
angle_motor_fwd = None

# Local coordinate system from center equally spaced from BLDC motor A1 and A2
x_local = 0
y_local = 0
z_local = 0


def create_bodies(position1, position2, size_upper, size_lower, scale, reduced_length):
    # Create upper section.
    upper = create_box_mod(size_upper, reduced_length)
    upper.setPosition(agx.Vec3(position1[0], position1[1], position1[2]))

    # Create lower section.
    lower = create_box_mod(size_lower, reduced_length)
    lower.setPosition(agx.Vec3(position2[0], position2[1], position2[2]))

    return upper, lower


def create_box_mod(size, reduced_length):
    body = agx.RigidBody()
    shape = agxCollide.Box(size[0], size[1], size[2] - reduced_length)
    geometry = agxCollide.Geometry(shape)
    body.add(geometry)

    return body


def create_sphere(position: agx.Vec3, diam):
    body = agx.RigidBody()
    shape = agxCollide.Sphere(diam)
    geometry = agxCollide.Geometry(shape)
    body.add(geometry)
    body.setPosition(position)

    return body


def create_capsule(position: agx.Vec3, scale):
    body = agx.RigidBody()
    shape = agxCollide.Capsule(0.5 * scale, 1 * scale)
    geometry = agxCollide.Geometry(shape)
    body.add(geometry)
    body.setPosition(position)

    return body


def create_cylinder(position, scale):
    body = agx.RigidBody()
    shape = agxCollide.Cylinder(0.5 * scale, 1 * scale)
    geometry = agxCollide.Geometry(shape)
    body.add(geometry)
    body.setPosition(position[0], position[1], position[2])

    return body


class CreateRobot():
    # Initial setup
    hinge_list = list()
    frame_list = list()

    def __init__(self):
        # Initial setup
        self.hinge_counter = 0
        self.reduced_length = 0.5
        self.increase_height = 2.95
        self.size_upper = UPPER_LEG_SECTION_SIZE
        self.size_lower = LOWER_LEG_SECTION_SIZE

    def create_joints(self):
        # --------- Create aft section -----------
        pos_upper_aft = [L_5*MODEL_SCALE, 0, 3.1 + self.increase_height]
        pos_lower_aft = [L_5*MODEL_SCALE, 0, 1.3 + self.increase_height-1]
        aft_upper, aft_lower = create_bodies(position1=pos_upper_aft, position2=pos_lower_aft,
                                             size_upper=self.size_upper, size_lower=self.size_lower, scale=MODEL_SCALE,
                                             reduced_length=self.reduced_length)

        # Add upper and lower sections of aft part of robot to simulation
        oneLegRobotApp.sim().add(aft_upper)
        oneLegRobotApp.sim().add(aft_lower)

        # Create frame for aft motor
        f1 = agx.Frame()
        f1.setLocalTranslate(0, 0, self.size_upper[2]-self.reduced_length/2)
        f1.setLocalRotate(agx.EulerAngles(math.radians(90), 0, 0))
        self.frame_list.append(f1)

        # Create hinge to aft motor
        hinge1_range = [-math.pi / 4, math.pi / 4]
        hinge1 = self.create_hinge_1RB(aft_upper, f1, hinge1_range)
        hinge1.getLock1D().setEnable(True)
        self.hinge_list.append(hinge1)
        #oneLegRobotApp.sim().add(hinge1)

        # Create frames for upper and lower aft section
        f2 = agx.Frame()
        f2.setLocalTranslate(0, 0, -self.size_upper[2]-self.reduced_length/2)
        f2.setLocalRotate(agx.EulerAngles(math.radians(90), 0, 0))
        self.frame_list.append(f2)

        f3 = agx.Frame()
        f3.setLocalTranslate(0, 0, self.size_lower[2]-self.reduced_length/2)
        f3.setLocalRotate(agx.EulerAngles(math.radians(90), 0, 0))
        self.frame_list.append(f3)

        # Create hinge between upper and lower fwd section
        hinge2_range = [-math.pi/4, math.pi/4]
        hinge2 = self.create_hinge_2RB(aft_upper, f2, aft_lower, f3, hinge2_range)
        self.hinge_list.append(hinge2)
        #oneLegRobotApp.sim().add(hinge2)

        # --------- Create fwd section -----------
        pos_upper_fwd = [-L_5*MODEL_SCALE, 0, 3.1 + self.increase_height]
        pos_lower_fwd = [-L_5*MODEL_SCALE, 0, 1.3 + self.increase_height-1]
        fwd_upper, fwd_lower = create_bodies(position1=pos_upper_fwd, position2=pos_lower_fwd,
                                             size_upper=self.size_upper, size_lower=self.size_lower, scale=MODEL_SCALE,
                                             reduced_length=self.reduced_length)

        # Add upper and lower sections of aft part of robot to simulation
        oneLegRobotApp.sim().add(fwd_upper)
        oneLegRobotApp.sim().add(fwd_lower)

        # Create frame for forward motor
        f4 = agx.Frame()
        f4.setLocalTranslate(0, 0, self.size_upper[2] - self.reduced_length / 2)
        f4.setLocalRotate(agx.EulerAngles(math.radians(90), 0, 0))
        self.frame_list.append(f4)

        # Create hinge to forward motor
        hinge3_range = [-math.pi/4, math.pi/4]
        hinge3 = self.create_hinge_1RB(fwd_upper, f4, hinge3_range)
        hinge3.getLock1D().setEnable(True)
        self.hinge_list.append(hinge3)
        #oneLegRobotApp.sim().add(hinge3)

        # Create frames for upper and lower fwd section
        f5 = agx.Frame()
        f5.setLocalTranslate(0, 0, -self.size_upper[2]-self.reduced_length/2)
        f5.setLocalRotate(agx.EulerAngles(math.radians(90), 0, 0))
        self.frame_list.append(f5)

        f6 = agx.Frame()
        f6.setLocalTranslate(0, 0, self.size_lower[2]-self.reduced_length/2)
        f6.setLocalRotate(agx.EulerAngles(math.radians(90), 0, 0))
        self.frame_list.append(f6)

        # Create hinge between upper and lower fwd section
        hinge4_range = [-math.pi/4, math.pi/4]
        hinge4 = self.create_hinge_2RB(fwd_upper, f5, fwd_lower, f6, hinge4_range)
        self.hinge_list.append(hinge4)
        #oneLegRobotApp.sim().add(hinge4)

        # Create end effector frame for lower aft and forward section.
        f7 = agx.Frame()
        f7.setLocalTranslate(0, 0, -self.size_upper[2]*2-self.reduced_length/2+0.1)
        f7.setLocalRotate(agx.EulerAngles(math.radians(90), 0, 0))
        self.frame_list.append(f7)

        f8 = agx.Frame()
        f8.setLocalTranslate(0, 0, -self.size_upper[2]*2-self.reduced_length/2+0.1)
        f8.setLocalRotate(agx.EulerAngles(math.radians(90), 0, 0))
        self.frame_list.append(f8)

        # --------- Create end effector -----------
        # Create hinge for end effector between lower aft and forward section.
        hinge5_range = [-math.pi/4, math.pi/4]
        hinge5 = self.create_hinge_2RB(aft_lower, f7, fwd_lower, f8, hinge5_range)
        self.hinge_list.append(hinge5)
        #oneLegRobotApp.sim().add(hinge5)

        # Create frame for end-effector point as ball.
        f_end_effector = agx.Frame()
        f_end_effector.setLocalTranslate(agx.Vec3(0,0,0))
        f_end_effector.setLocalRotate(agx.EulerAngles(math.radians(90), 0, 0))
        self.frame_list.append(f_end_effector)



        #f10 = agx.Frame()
        #f10.setLocalTranslate(agx.Vec3(0,0,0))
        #ball = create_sphere(agx.Vec3(2,2,0), 0.1)
        #ball.addAttachment(f10, "f10")
        #oneLegRobotApp.sim().add(ball)

        # Create hinge for ball end-effector point
        hinge6_range = [-math.pi/4, math.pi/4]
        foot = create_sphere(agx.Vec3(0, 0, 0), 0.1)
        foot.addAttachment(f_end_effector, "position_end_effector")
        oneLegRobotApp.sim().add(foot)
        hinge6 = self.create_hinge_2RB(aft_lower, f7, foot, f_end_effector, hinge6_range)
        hinge6.getLock1D().setEnable(True)
        self.hinge_list.append(hinge6)
        #print("Number of DOF: ", hinge6.getNumDOF())
        #oneLegRobotApp.sim().add(hinge6)

        # --------- Create frame at aft motor -----------
        # Create frame for motor aft.
        f_motor_aft = agx.Frame()
        f_motor_aft.setLocalTranslate(agx.Vec3(0, 0, 0))
        f_motor_aft.setLocalRotate(agx.EulerAngles(math.radians(90), 0, 0))
        self.frame_list.append(f_motor_aft)

        # Create hinge for motor aft.
        hinge7_range = [-math.pi / 4, math.pi / 4]
        motor_aft = create_sphere(agx.Vec3(0, 0, -self.size_upper[2]-self.reduced_length), 0.1)
        motor_aft.addAttachment(f_motor_aft, "motor_aft")
        oneLegRobotApp.sim().add(motor_aft)
        hinge7 = self.create_hinge_2RB(aft_upper, f1, motor_aft, f_motor_aft, hinge7_range)
        hinge7.getLock1D().setEnable(True)
        self.hinge_list.append(hinge7)

        # --------- Create frame at fwd motor -----------
        # Create frame for motor aft.
        f_motor_fwd = agx.Frame()
        f_motor_fwd.setLocalTranslate(agx.Vec3(0, 0, 0))
        f_motor_fwd.setLocalRotate(agx.EulerAngles(math.radians(90), 0, 0))
        self.frame_list.append(f_motor_fwd)

        # Create hinge for motor aft.
        hinge8_range = [-math.pi / 4, math.pi / 4]
        motor_fwd = create_sphere(agx.Vec3(0, 0, -self.size_upper[2] - self.reduced_length), 0.1)
        motor_fwd.addAttachment(f_motor_fwd, "motor_fwd")
        oneLegRobotApp.sim().add(motor_fwd)
        hinge8 = self.create_hinge_2RB(fwd_upper, f4, motor_fwd, f_motor_fwd, hinge8_range)
        hinge8.getLock1D().setEnable(True)
        self.hinge_list.append(hinge8)

        # --------- Reading data -----------
        # Read current frame position
        f_end_effector_pos = FrameReader(f_end_effector)
        oneLegRobotApp.sim().add(f_end_effector_pos)

        # Read current frame position
        f_motor_aft_pos = FrameReader(f_motor_aft)
        oneLegRobotApp.sim().add(f_motor_aft_pos)

        # Read current frame position
        f_motor_fwd_pos = FrameReader(f_motor_fwd)
        oneLegRobotApp.sim().add(f_motor_fwd_pos)

        # --------- Reading data - Aft motor -----------
        # Read current hinge position
        hinge1_pos = HingePosition(hinge1, 1, 1)
        oneLegRobotApp.sim().add(hinge1_pos)

        # Read current hinge position
        hinge7_pos = HingePosition(hinge7, 7, 1)
        oneLegRobotApp.sim().add(hinge7_pos)

        # --------- Reading data - Aft motor -----------
        # Read current hinge position
        hinge3_pos = HingePosition(hinge3, 3, 1)
        oneLegRobotApp.sim().add(hinge3_pos)

        # Read current hinge position
        hinge8_pos = HingePosition(hinge8, 8, 1)
        oneLegRobotApp.sim().add(hinge8_pos)

        # --------- Reading data - Ball motor -----------

        # Read current hinge position
        hinge5_pos = HingePosition(hinge5, 5, 1)
        oneLegRobotApp.sim().add(hinge5_pos)

        # --------- Add to simulation -----------
        # Make the first motor swing back and forth
        #speed_controller_aft = MotorSpeedController(hinge1, hinge3, 1, 2, aft_upper, fwd_upper, pos_upper_aft, pos_upper_fwd)
        #oneLegRobotApp.sim().add(speed_controller_aft)

        # --------- Add to simulation -----------
        # Add hinges to simulation
        for i in range(len(self.hinge_list)):
            oneLegRobotApp.sim().add(self.hinge_list[i])

        if (debugging):
            static_above1 = agxCollide.Geometry(agxCollide.Box(1.8, 1.8, -1.2))
            static_above1.setLocalPosition(0, 2.5, 7 - 1.2)
            oneLegRobotApp.sim().add(static_above1)

            static_above2 = agxCollide.Geometry(agxCollide.Box(1.8, 1.8, -2.4))
            static_above2.setLocalPosition(0, 2.5, 7 - 1.2 - 2.4 - 2.4 / 2)
            oneLegRobotApp.sim().add(static_above2)

    def get_hinge_list(self):
        return self.hinge_list

    def get_frame_list(self):
        return self.frame_list

    def create_hinge_1RB(self, rb1, frame1, range):
        hinge = agx.Hinge(rb1, frame1)

        # Sets the angular range of the hinge.
        hinge.getRange1D().setRange(range[0], range[1])

        # Sets the compliance of the hinge DOF.
        hinge.setCompliance(1E-12)
        hinge.getMotor1D().setCompliance(1E-10)
        hinge.getMotor1D().setEnable(False)
        hinge.getLock1D().setEnable(False)

        return hinge

    def create_hinge_2RB(self, rb1, frame1, rb2, fram2, range):
        hinge = agx.Hinge(rb1, frame1, rb2, fram2)

        # Sets the angular range of the hinge.
        hinge.getRange1D().setRange(range[0], range[1])

        # Sets the compliance of the hinge DOF.
        hinge.setCompliance(1E-12)
        hinge.getMotor1D().setCompliance(1E-10)
        hinge.getMotor1D().setEnable(False)
        hinge.getLock1D().setEnable(False)

        return hinge


def create_floor():
    """
    Builds the flooring of the simulation.
    """
    shape = agxCollide.Box(10, 10, 0.1)
    floor = agxCollide.Geometry(shape)
    #oneLegRobotApp.create_visual(floor, diffuse_color=agxRender.Color.Green())
    floor.setPosition(0, 0, -0.3)
    oneLegRobotApp.sim().add(floor)

    return floor


def build_scene():
    """
    Builds the scene and starts simulation and controllers.
    """
    # Create scenes
    floor = create_floor()
    CreateRobot().create_joints()
    hinge_list = CreateRobot().get_hinge_list()

    end_effector_controller = EndEffectorController(hinge_list[0], hinge_list[2], 0.5, 1, 2)
    oneLegRobotApp.sim().add(end_effector_controller)

    # Create moveable floor
    floor_controller = MoveFloorController(floor=floor, movement=[0, 0, 0.9])
    oneLegRobotApp.sim().add(floor_controller)

    # Rendering details
    oneLegRobotApp.app().getSceneDecorator().setEnableShadows(False)
    oneLegRobotApp.app().setEnableDebugRenderer(True)

    # Set grid
    # oneLegRobotApp.app().setEnableGrid(True)
    # oneLegRobotApp.app().setGridSize(agx.Vec2(10,10))

    # Arrange camera to be centered around center of floor
    oneLegRobotApp.init_camera(eye=agx.Vec3(20, 20, 30), center=floor.getPosition())


class HingePosition(agxSDK.StepEventListener):
    """
    Class used to display current hinge position.

    hinge : agx.Hinge()
        Hinge object
    hinge_num : int
        Hinge number
    index : int
        Index number stating whether Frame() is child to RigidBody1 (1) or RigidBody2 (2)
    """
    def __init__(self, hinge, hinge_num, index):
        super().__init__(agxSDK.StepEventListener.PRE_STEP)

        self.hinge = hinge
        self.index = index
        self.hinge_num = hinge_num
        self.hinge_pos = None

    def pre(self, time):
        self.hinge_pos = self.hinge.getAttachment(self.index).getFrame().getTranslate()
        #print("Hinge ", self.hinge_num, " position: ", self.hinge_pos)


class FrameReader(agxSDK.StepEventListener):
    """
    Class used to display current frane position.

    frame : agx.Frame()
        Frame object
    """
    def __init__(self, frame):
        super().__init__(agxSDK.StepEventListener.PRE_STEP)

        self.frame = frame
        self.framePos = None

    def pre(self, time):
        self.framePos = self.frame.getTranslate()
        #print("Frame position: ", self.framePos)


class MoveFloorController(agxSDK.StepEventListener):
    """
    Moveable floor controller to move floor in either x-, y- or z-direction or a combination of them.

    floor : agx.Geometry()
        Floor object
    movement : List (x, y, z)
        List of movements in x-, y- and z-direction
    """
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

class MotorSpeedController(agxSDK.StepEventListener):
    def __init__(self, motor_aft, motor_fwd, speed, interval, rb1, rb2, init_pos_rb1, init_pos_rb2):
        super().__init__(agxSDK.StepEventListener.PRE_STEP)

        # Desired end-effector position
        self.x = 0
        self.y = 0
        self.z = 0

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
        self.angle_aft = math.degrees(self.motor_aft.getAngle())
        print("Angle aft:     ", self.angle_aft)
        # aft_section_pos = self.rb1.getPosition()
        # aft_x = aft_section_pos[0]
        # aft_y = aft_section_pos[1]
        # aft_z = aft_section_pos[2]
        # diff_x_aft = self.init_aft_X - aft_x
        # diff_y_aft = self.init_aft_Y - aft_y
        # diff_z_aft = self.init_aft_Z - aft_z
        # theta_2_fwd_kin = math.degrees(math.atan(diff_z_aft / diff_x_aft)) + 90
        # print("X: ", aft_x, "     Diff. X: ", diff_x_aft)
        # print("Y: ", aft_y, "     Diff. Y: ", diff_y_aft)
        # print("Z: ", aft_z, "     Diff. Z: ", diff_z_aft)
        # print("Theta_2: ", theta_2_fwd_kin)

        # Fwd section
        print("----- Fwd section -----")
        self.angle_fwd = math.degrees(self.motor_fwd.getAngle())
        print("Angle fwd:     ", self.angle_fwd)
        # fwd_section_pos = self.rb2.getPosition()
        # fwd_x = fwd_section_pos[0]
        # fwd_y = fwd_section_pos[1]
        # fwd_z = fwd_section_pos[2]
        # diff_x_fwd = self.init_fwd_X - fwd_x
        # diff_y_fwd = self.init_fwd_Y - fwd_y
        # diff_z_fwd = self.init_fwd_Z - fwd_z
        # theta_6_fwd_kin = math.degrees(math.atan(diff_z_fwd / diff_x_fwd)) + 90
        # print("X: ", fwd_x, "     Diff. X: ", diff_x_fwd)
        # print("Y: ", fwd_y, "     Diff. Y: ", diff_y_fwd)
        # print("Z: ", fwd_z, "     Diff. Z: ", diff_z_fwd)
        # print("Theta_6: ", theta_6_fwd_kin)


def calculate_motor_angle_fwd(x, y, z, scale):
    # Calculate for fwd motor
    global x_A1, z_A1, theta_1, theta_2, theta_3, theta_4, theta_9
    x_A1 = (restX + x) / scale
    z_A1 = (restZ - z) / scale
    theta_1 = 180 - math.degrees(math.atan2(z_A1, x_A1))
    theta_4 = math.degrees(
        math.acos((math.pow(x_A1, 2) + math.pow(z_A1, 2) - math.pow((L_1 / scale), 2) - math.pow((L_2 / scale), 2)) / (2 * (L_1 / scale) * (L_2 / scale))))
    theta_9 = math.degrees(
        math.asin(((L_1 / scale) / math.sqrt(math.pow(x_A1,2) + math.pow(z_A1, 2))) * math.sin(math.radians(theta_4))))
    theta_3 = 180 - (180-theta_4) - theta_9
    if (theta_1 - theta_3) <= 0:
        theta_2 = 180 + (theta_1 - theta_3)
    else:
        theta_2 = 180 + (theta_1 - theta_3)

    def aft_first_section_outer_position(x, y, z):
        r_A1 = L_1
        x_1 = r_A1 * math.cos(math.degrees(theta_2))
        z_1 = r_A1 * math.sin(math.degrees(theta_2))

        return x_1, z_1

    return theta_2


def calculate_motor_angle_aft(x, y, z, scale):
    # Calculate for aft motor
    global x_A2, z_A2, theta_5, theta_6, theta_7, theta_8, theta_10
    x_A2 = (restX - x) / scale
    z_A2 = (restZ - z) / scale
    theta_5 = 180 - math.degrees(math.atan2(z_A2, x_A2))
    theta_8 = math.degrees(
        math.acos((math.pow(x_A2, 2) + math.pow(z_A2, 2) - math.pow((L_3 / scale), 2) - math.pow((L_4 / scale), 2)) / (2 * (L_3 / scale) * (L_4 / scale))))
    theta_10 = math.degrees(
        math.asin(((L_3 / scale) / math.sqrt(math.pow(x_A2, 2) + math.pow(z_A2, 2))) * math.sin(math.radians(theta_8))))
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
        print("y_A1 pos: ", z_A1)
        print("Angle theta_1 is: ", theta_1)
        print("Angle theta_4 is: ", theta_4)
        print("Angle theta_9 is: ", theta_9)
        print("Angle theta_3 is: ", theta_3)
        print("Angle theta_2 is: ", theta_2)
        print("")

    if get_debug():
        print("---- DATA FOR FWD SECTION ----")
        print("x_A2 pos: ", x_A2)
        print("y_A2 pos: ", z_A2)
        print("Angle theta_5 is: ", theta_5)
        print("Angle theta_8 is: ", theta_8)
        print("Angle theta_10 is: ", theta_10)
        print("Angle theta_7 is: ", theta_7)
        print("Angle theta_6 is: ", theta_6)
        print("")

















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