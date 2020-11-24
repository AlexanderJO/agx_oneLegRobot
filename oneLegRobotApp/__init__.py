# Initially created by: Lars Ivar Hatledal
# Modified by: One Legged Robot group.

# Import libraries
import osg
import agxOSG
import agxIO
import agx
import agxSDK
import agxPython
import agxCollide
import math
from agxRender import Color
from agxUtil import createTrimeshFromFile

# Define variables
aftMotorAngle = math.degrees(0)
fwdMotorAngle = math.degrees(0)
xLegPos = float(0)
yLegPos = float(0)
zLegPos = float(1)

# Define simulation environment
def sim() -> agxSDK.Simulation:
    return agxPython.getContext().environment.getSimulation()

# Defines the initial agx application
def app() -> agxOSG.ExampleApplication:
    return agxPython.getContext().environment.getApplication()

# Get the root of the scene
def root() -> osg.Group:
    return agxPython.getContext().environment.getSceneRoot()

# Add object
def add(obj):
    if isinstance(obj, agxSDK.EventListener):
        sim().addEventListener(obj)
    else:
        sim().add(obj)
    return obj

# Add event listener
def add_event_listener(listener):
    sim().addEventListener(listener)

# Set the time step for the simulation
def set_time_step(dt: float):
    sim().setTimeStep(dt)

# Return the time step of the simulation
def get_time_step() -> float:
    return sim().getTimeStep()

# Get the current time of the simulation
def get_time():
    return sim().getTimeStamp()

# Register additional scenes and alters the script name to reflect
def register_additional_scenes(*additional_scenes):
    if len(additional_scenes) > 0:

        def add_scene(name):
            scene_key = app().getNumScenes() + 1            # Count number of active scenes
            app().addScene(script_file_name, name, ord(ascii(scene_key)), True)     # Add scene to simulation

        script_file_name = app().getArguments().getArgumentName(1)
        script_file_name = script_file_name.replace('agxscene:', '')
        print(script_file_name)

        for scene in additional_scenes:
            add_scene(scene)

# Load model
def load_model(path) -> agxCollide.Geometry:
    return agxCollide.Geometry(load_shape(path))

# Load shape
def load_shape(path) -> agxCollide.Trimesh:
    r"""Load shape from path and create a trimesh
    :return: Trimesh of shape"""
    return createTrimeshFromFile(
        path, agxCollide.Trimesh.REMOVE_DUPLICATE_VERTICES, agx.Matrix3x3())

# Set render quality
def create_visual(obj, diffuse_color: Color = None, ambient_color: Color = None,
                  shininess=None, alpha: float = None):
    node = agxOSG.createVisual(obj, root())

    if diffuse_color is not None:
        agxOSG.setDiffuseColor(node, diffuse_color)

    if ambient_color is not None:
        agxOSG.setAmbientColor(node, ambient_color)

    if shininess is not None:
        agxOSG.setShininess(node, shininess)

    if alpha is not None:
        agxOSG.setAlpha(node, alpha)

    return node

# Create sky
def create_sky():
    c1 = Color.SkyBlue()
    c2 = Color.DodgerBlue()

    def to_vec3(o: Color):
        return agx.Vec3(o.x(), o.y(), o.z())

    app().getSceneDecorator().setBackgroundColor(to_vec3(c1), to_vec3(c2))

# Sets the initial camera position
def init_camera(eye=agx.Vec3(-25, -25, 25), center=agx.Vec3(), up=agx.Vec3.Z_AXIS()):
    app().setCameraHome(eye, center, up)

# Change the camera position
def change_camera(eye=agx.Vec3(-25, -25, 25), center=agx.Vec3(), up=agx.Vec3.Z_AXIS()):
    app().setCameraHome(eye, center, up)

def get_contacts(body: agx.RigidBody) -> list:
    contacts = []
    contacts_ = agxCollide.GeometryContactPtrVector()
    space = sim().getSpace()  # type: agxCollide.Space
    space.getGeometryContacts(contacts_, body)
    for gc in contacts_:  # type: agxCollide.GeometryContact
        if not gc.isEnabled():
            continue
        points = gc.points()  # type: agxCollide.ContactPointVector
        for point in points:  # type: agxCollide.ContactPoint
            if point.getEnabled():
                contacts.append(point)

    return contacts

def get_sum_force_magnitude(body: agx.RigidBody) -> float:
    sum_force_mag = 0
    contacts = agxCollide.GeometryContactPtrVector()
    space = sim().getSpace()  # type: agxCollide.Space
    space.getGeometryContacts(contacts, body)
    for gc in contacts:  # type: agxCollide.GeometryContact
        if not gc.isEnabled():
            continue
        points = gc.points()  # type: agxCollide.ContactPointVector
        for point in points:  # type: agxCollide.ContactPoint
            if point.getEnabled():
                sum_force_mag += point.getNormalForceMagnitude()

    return sum_force_mag

def create_constraint(**kwds) -> agx.Constraint:
    if 'pos' in kwds:
        pos = kwds['pos']
    else:
        pos = agx.Vec3()

    if 'axis' in kwds:
        axis = kwds['axis']
    else:
        axis = agx.Vec3.Z_AXIS()

    c = kwds['c']
    rb1 = kwds['rb1']
    rb2 = kwds['rb2']

    f1 = agx.Frame()
    f2 = agx.Frame()

    agx.Constraint.calculateFramesFromBody(pos, axis, rb1, f1, rb2, f2)
    return c(rb1, f1, rb2, f2)

if agxPython.getContext() is None:

    import os
    if os.name == "posix":
        agx_dir = os.environ['AGX_DIR']
        agxIO.Environment_instance().getFilePath(agxIO.Environment.RESOURCE_PATH).addFilePath(agx_dir)
        agxIO.Environment_instance().getFilePath(agxIO.Environment.RESOURCE_PATH).addFilePath(agx_dir + "/data")

    init = agx.AutoInit()

    import sys

    argParser = agxIO.ArgumentParser([sys.executable] + sys.argv)
    example_app = agxOSG.ExampleApplication()
    example_app.addScene(argParser.getArgumentName(1), "build_scene", ord('1'), True)

    if example_app.init(argParser):
        example_app.run()
    else:
        print("An error occurred while initializing ExampleApplication.")

# ---------------- SETTERS OR GETTERS ------------------

def set_aft_motor_angle(angle):
    aftMotorAngle = math.degrees(angle)

def set_fwd_motor_angle(angle):
    fwdMotorAngle = math.degrees(angle)

def get_aft_motor_angle():
    return aftMotorAngle

def get_fwd_motor_angle():
    return fwdMotorAngle

def set_x_leg_pos(x):
    xLegPos = float(x)

def set_y_leg_pos(y):
    yLegPos = float(y)

def set_z_leg_pos(z):
    zLegPos = float(z)

def get_x_leg_pos():
    return xLegPos

def get_y_leg_pos():
    return yLegPos

def get_z_leg_pos():
    return zLegPos