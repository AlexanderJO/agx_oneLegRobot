import agx
import agxCollide
import agxRender
import agxSDK
import math
import time

import oneLegRobotApp
from oneLegRobotApp.oneLegRobotMotion import OneLegRobot
#from oneLegRobotApp.oneLegRobotMotion import AftFirstJoint

# Import shapes as .obj files
first_joint_shape_aft = oneLegRobotApp.load_shape('assets/bein - ledd 1 v2.obj')
second_joint_shape_aft = oneLegRobotApp.load_shape('assets/bein - ledd 2 v2.obj')
first_joint_shape_fwd = oneLegRobotApp.load_shape('assets/Bein - Ledd 1 v2.obj')
second_joint_shape_fwd = oneLegRobotApp.load_shape('assets/Bein - Ledd 2 v2.obj')

# Local coordinate system from center
x_local = 0
y_local = 0
z_local = 0

# class RobotMovement(agxSDK.Assembly):
#     def __init__(self):
#         super.__init__()



# class OneLegRobot(agxSDK.Assembly):
#     def __init__(self):
#         super.__init__()



def create_bodies(position1, position2, size):
    # Create first leg section and set position
    leg_section1 = agx.RigidBody()
    leg_section1.add(agxCollide.Geometry(agxCollide.Box(size[0],size[1],size[2])))
    leg_section1.setPosition(agx.Vec3(position1[0], position1[1], position1[2]))

    # Create second leg section and set position
    leg_section2 = agx.RigidBody()
    leg_section2.add(agxCollide.Geometry(agxCollide.Box(size[0],size[1],size[2])))
    leg_section2.setPosition(agx.Vec3(position2[0], position2[1], position2[2]))

    return leg_section1, leg_section2


def build_scene():
    # Registers a new scene
    oneLegRobotApp.register_additional_scenes('robot_leg_scene_test')


    #create_floor()
    oneLegRobot = OneLegRobot(1)
    oneLegRobotApp.add(oneLegRobot)

    #oneLegRobot.oneLegRobotMotion.OneLegRobotAssembly.create_floor()
    # add_first_joint_aft()
    # add_second_joint_aft()
    #first_joint_geometry_aft = add_section(20,0,240,0,0,math.pi / 2, first_joint_shape_aft)
    #second_joint_geometry_aft = add_section(0,0,120,0,0,math.pi / 2, second_joint_shape_aft)

    #oneLegRobotApp.add(first_joint_geometry_aft)
    #oneLegRobotApp.add(second_joint_geometry_aft)

    # x = -120
    # y = -60
    # x = x - 5
    # for i in range(10):
    #     print("------------------ LOOP NO. ", i, " ------------------")
    #     x += 5
    #     y = y
    #
    #     a1_rot_z = oneLegRobotApp.oneLegRobotMotion.calculate_motor_angle(x, y)
    #     #first_joint_geometry_aft.setLocalRotation(agx.EulerAngles(0,0,math.radians(a1_rot_z)))
    #     #oneLegRobotApp.add(first_joint_geometry_aft)
    #
    #     # Add a listener to the new values
    #     # oneLegRobotApp.add_event_listener()
    #
    # print("x = ", x, " and y = ", y)

    # simulatorTest = oneLegRobotApp.sim()
    # scale = 50
    # simulatorTest.add(create_floor())
    # simulatorTest.add(create_box(agx.Vec3(-2, 3, 5), scale))
    # simulatorTest.add(create_sphere(agx.Vec3(0, 1, 5), scale))
    # simulatorTest.add(create_capsule(agx.Vec3(3, 5, 5), scale))
    # simulatorTest.add(create_cylinder(agx.Vec3(-3, -2, 5), scale))

    # f1 = agx.Frame()
    # f1.setLocalTranslate(0, 0, 100)
    # f1.setLocalRotate(agx.EulerAngles(0, math.radians(90), 0))

    #geometry = agxCollide.Geometry(first_joint_shape_aft)
    #b1 = agx.RigidBody()
    #b1 = oneLegRobotApp.create_visual(geometry, agxRender.Color.Red())
    #oneLegRobotApp.sim().add(agx.Hinge(b1, f1))
    #oneLegRobotApp.add(geometry)
    #geometry.setLocalRotation(agx.EulerAngles(0, 0, 0))
    #b1.setEnableCollisions(True)
    #geometry.setPosition(agx.Vec3(200,200,0))
    #oneLegRobotApp.sim().add(geometry)
    #oneLegRobotApp.create_visual(b1, agxRender.Color.Yellow())

    # size = [100, 50, 100]
    # b1, b2 = create_bodies([0, 0, 0], [0, 0, - 2.5], size)
    #
    # b1v = agxCollide.Geometry(b1,
    #                                agx.AffineMatrix4x4.translate(0, 0, 100))
    # #b1v = oneLegRobotApp.create_visual(b1,agxRender.Color.Red())
    # #oneLegRobotApp.add(b1v)
    #
    # oneLegRobotApp.sim().add(b1)
    # oneLegRobotApp.sim().add(b2)
    #
    # frameTest1 = agx.Frame()
    # frameTest1.setLocalTranslate(100, 100, 0)
    # frameTest1.setLocalRotate(agx.EulerAngles(0, 0, 0))
    #
    # hingeTest1 = agx.Hinge(b1, frameTest1)
    # oneLegRobotApp.sim().add(hingeTest1)
    #
    # frameTest2 = agx.Frame()
    # frameTest2.setLocalTranslate(100, 100, 0)
    # frameTest2.setLocalRotate(agx.EulerAngles(0, 0, 0))
    #
    # hingeTest2 = agx.Hinge(b1, frameTest1, b2, frameTest2)
    # oneLegRobotApp.sim().add(hingeTest2)


    #size = [0.5, 0.5, 0.5]

def create_box(position: agx.Vec3, scale):
    shape = agxCollide.Box(agx.Vec3(0.5 * scale, 0.5 * scale, 0.5 * scale))
    geometry = agxCollide.Geometry(shape)
    body = agx.RigidBody(geometry)
    body.setPosition(position)
    oneLegRobotApp.create_visual(body)
    return body


def create_sphere(position: agx.Vec3, scale):
    shape = agxCollide.Sphere(0.5 * scale)
    geometry = agxCollide.Geometry(shape)
    body = agx.RigidBody(geometry)
    body.setPosition(position)
    oneLegRobotApp.create_visual(body)
    return body


def create_capsule(position: agx.Vec3, scale):
    shape = agxCollide.Capsule(0.5 * scale, 1 * scale)
    geometry = agxCollide.Geometry(shape)
    body = agx.RigidBody(geometry)
    body.setPosition(position)
    oneLegRobotApp.create_visual(body)
    body.setRotation(agx.EulerAngles(agx.PI_2, 0, 0))
    return body


def create_cylinder(position: agx.Vec3, scale):
    shape = agxCollide.Cylinder(0.5 * scale, 1 * scale)
    geometry = agxCollide.Geometry(shape)
    body = agx.RigidBody(geometry)
    body.setPosition(position)
    oneLegRobotApp.create_visual(body)
    body.setRotation(agx.EulerAngles(agx.PI_2, 0, 0))
    return body

def create_floor():
    w = 200
    b = 200
    h = 10
    #floor = agxCollide.Geometry(agxCollide.Box(2.5, 0.5, h), agx.AffineMatrix4x4.translate(0, 0, -h))

    floor = agxCollide.Geometry(agxCollide.Box(w, b, h))
    floor.setPosition(0, 0, 0)
    floor.setRotation(agx.EulerAngles(0, 0, 0))
    oneLegRobotApp.create_visual(floor, diffuse_color=agxRender.Color.Green())
    oneLegRobotApp.add(floor)
    # return floor

def add_first_joint_aft():
    geometry = agxCollide.Geometry(first_joint_shape_aft.deepCopy(),
                                   agx.AffineMatrix4x4.translate(0,0,200))
    geometry.setLocalRotation(agx.EulerAngles(0,0,math.pi/2))
    print(geometry.getRotation())
    geometry.setEnableCollisions(True)
    rigidBody = agx.RigidBody(geometry)
    oneLegRobotApp.create_visual(rigidBody, agxRender.Color.Red())
    oneLegRobotApp.add(rigidBody)
    rigidBody.setMotionControl(agx.RigidBody.STATIC)

def add_second_joint_aft():
    geometry = agxCollide.Geometry(second_joint_shape_aft.deepCopy(),
                                   agx.AffineMatrix4x4.translate(0,0,100))
    geometry.setLocalRotation(agx.EulerAngles(0,0,math.pi/2))
    print(geometry.getRotation())
    geometry.setEnableCollisions(True)
    oneLegRobotApp.create_visual(geometry, agxRender.Color.Red())
    oneLegRobotApp.add(geometry)

def add_section_old(x,y,z,rotx,roty,rotz,path):
    geometry = agxCollide.Geometry(path.deepCopy(),
                                   agx.AffineMatrix4x4.translate(x,y,z))
    geometry.setLocalRotation(agx.EulerAngles(rotx,roty,rotz))
    #print(path + " were added with rotation " + geometry.getRotation())
    geometry.setEnableCollisions(True)
    oneLegRobotApp.create_visual(geometry, agxRender.Color.Red())
    #oneLegRobotApp.add(geometry)
    return geometry

def add_section(x,y,z,rotx,roty,rotz,path):
    geometry = agxCollide.Geometry(path.deepCopy(),
                                   agx.AffineMatrix4x4.translate(x,y,z))
    geometry.setLocalRotation(agx.EulerAngles(rotx,roty,rotz))
    #print(path + " were added with rotation " + geometry.getRotation())
    geometry.setEnableCollisions(True)
    rigidBody = agx.RigidBody(geometry)
    rigidBody.setMotionControl(agx.RigidBody.DYNAMICS)

    oneLegRobotApp.create_visual(rigidBody, agxRender.Color.Red())
    #oneLegRobotApp.add(rigidBody)
    return rigidBody

def testDef():
    A1 = 0
    def testDef2():
        A2 = 1


def micro_step_leg(oldAngle, newAngle):
    oldAngle = math.degrees(oldAngle)
    newAngle = math.degrees(newAngle)