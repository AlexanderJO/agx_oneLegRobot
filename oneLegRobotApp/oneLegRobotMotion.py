import agx
import agxSDK
import agxCollide
import agxRender
import oneLegRobotApp
import math

# Import shapes as .obj files
first_joint_shape = oneLegRobotApp.load_shape('assets/bein - ledd 1 v2.obj')


# Import shapes as .obj files
first_joint_shape_aft = oneLegRobotApp.load_shape('assets/bein - ledd 1 v2.obj')
second_joint_shape_aft = oneLegRobotApp.load_shape('assets/bein - ledd 2 v2.obj')
first_joint_shape_fwd = oneLegRobotApp.load_shape('assets/Bein - Ledd 1 v2.obj')
second_joint_shape_fwd = oneLegRobotApp.load_shape('assets/Bein - Ledd 2 v2.obj')

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


class OneLegRobot(agxSDK.Assembly):

    def __init__(self, num_robots: int = 1):
        super().__init__()

        self.jointCon = []  # type: list[agx.Hinge]
        self.forceFeedback = []  # type: list[agx.RigidBody]
        self.len = 0

        self.material = agx.Material("robot_material{}".format(self.getUuid().__str__))

        def connect(rb1: agx.RigidBody, rb2: agx.RigidBody):
            axis = agx.Vec3(0, 0, 0)
            pos = agx.Vec3(0, 0, 0,)

            hinge = oneLegRobotApp.create_constraint(pos=pos, axis=axis, c=agx.Hinge, rb1=rb1, rb2=rb2)  # type: agx.Hinge
            hinge.setCompliance(1E-12)
            hinge.getMotor1D().setEnable(True)
            hinge.getMotor1D().setCompliance(1E-12)
            hinge.getLock1D().setEnable(False)
            hinge.getRange1D().setEnable(True)
            hinge.getRange1D().setRange(-math.pi / 2, math.pi)
            self.add(hinge)
            self.jointCon.append(hinge)

        # def add(part):
        #     self.len += part.len
        #     self.add(part)
        #
        # last_part = None

        #testRobotLeg = OneLegRobotAssembly(self.material)
        self.floor = OneLegRobotAssembly(self.material).create_floor()

        self.first_joint_geometry_aft = OneLegRobotAssembly(self.material).create_section(20,-180/2,440,0,0,math.pi / 2, first_joint_shape_aft)
        self.second_joint_geometry_aft = OneLegRobotAssembly(self.material).create_section(0,-180/2,320,0,0,math.pi / 2, second_joint_shape_aft)

        self.first_joint_geometry_fwd = OneLegRobotAssembly(self.material).create_section(0,180/2,440,0,0,math.pi / 2, first_joint_shape_fwd)
        self.second_joint_geometry_fwd = OneLegRobotAssembly(self.material).create_section(-20,180/2,320,0,0,math.pi / 2, second_joint_shape_fwd)

        # Connect joints
        connect(self.first_joint_geometry_aft, self.second_joint_geometry_aft)
        connect(self.first_joint_geometry_fwd, self.second_joint_geometry_fwd)

        # Build the robot
        oneLegRobotApp.add(self.floor)
        self.second_joint_geometry_aft.setParentFrame(self.first_joint_geometry_aft.getFrame())
        self.second_joint_geometry_fwd.setParentFrame(self.first_joint_geometry_fwd.getFrame())
        oneLegRobotApp.add(self.first_joint_geometry_aft)
        oneLegRobotApp.add(self.second_joint_geometry_aft)
        oneLegRobotApp.add(self.first_joint_geometry_fwd)
        oneLegRobotApp.add(self.second_joint_geometry_fwd)


        # Arrange camera to be centered on floor
        oneLegRobotApp.init_camera(eye=agx.Vec3(800, 0, 800), center=self.floor.getPosition())


        #self.first_joint_geometry = agx.RigidBody(agxCollide.Geometry(first_joint_shape.deepCopy()))
        #self.first_joint_geometry.setEnableCollisions(True)
        #oneLegRobotApp.create_visual(self.first_joint_geometry, agxRender.Color.Black())
        #self.bottom = agx.RigidBody

    def get_contacts(self, contacts=None) -> list:
        if contacts is None:
            contacts = []
        contacts.clear()
        for force in self.forceFeedback:  # type: agx.RigidBody
            c = oneLegRobotApp.get_contacts(force)
            contacts.append(c)
        return contacts

    def get_force_magnitude_at(self, intermediate_index):
        return oneLegRobotApp.get_sum_force_magnitude(self.forceFeedback[intermediate_index])

    def set_hinge_compliance(self, compliance):
        for joint in self.jointCon:
            joint.getMotor1D().setCompliance(compliance)

class OneLegRobotAssembly(agxSDK.Assembly):

    def __init__(self, material: agx.Material = None):
        super().__init__()

        # Create the rigid body of the robot
        self.floor = self.create_floor()
        self.first_joint_geometry_aft = self.create_section(20,0,340,0,0,math.pi / 2, first_joint_shape_aft)
        self.second_joint_geometry_aft = self.create_section(0,0,220,0,0,math.pi / 2, second_joint_shape_aft)

        # Build the robot
        # oneLegRobotApp.add(self.floor)
        # oneLegRobotApp.add(self.first_joint_geometry_aft)
        # oneLegRobotApp.add(self.second_joint_geometry_aft)

    def create_floor(self):
        w = 200
        b = 200
        h = 10
        # floor = agxCollide.Geometry(agxCollide.Box(2.5, 0.5, h), agx.AffineMatrix4x4.translate(0, 0, -h))

        floor = agxCollide.Geometry(agxCollide.Box(w, b, h))
        floor.setPosition(0, 0, 0)
        floor.setRotation(agx.EulerAngles(0, 0, 0))
        floor.setEnableCollisions(True)
        rigidBody = agx.RigidBody(floor)
        rigidBody.setMotionControl(agx.RigidBody.STATIC)
        oneLegRobotApp.create_visual(rigidBody, diffuse_color=agxRender.Color.Green())
        return floor

    def create_section(self, x, y, z, rotx, roty, rotz, path):
        geometry = agxCollide.Geometry(path.deepCopy(),
                                       agx.AffineMatrix4x4.translate(x, y, z))
        geometry.setLocalRotation(agx.EulerAngles(rotx, roty, rotz))
        geometry.setEnableCollisions(True)
        rigidBody = agx.RigidBody(geometry)
        rigidBody.setMotionControl(agx.RigidBody.DYNAMICS)
        oneLegRobotApp.create_visual(rigidBody, agxRender.Color.Red())
        return rigidBody



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