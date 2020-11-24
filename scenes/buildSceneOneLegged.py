import oneLegRobotApp

def build_scene():
    """
    Builds the scene and starts simulation and controllers.
    """
    # --------- Create scene -----------
    # Build robot
    robot = CreateRobot()
    init_upper_aft_pos_mod = agx.Vec3(0, 0, 0)
    init_upper_fwd_pos_mod = agx.Vec3(0, 0, 0)
    robot.set_upper_part_pos(init_upper_aft_pos_mod, init_upper_fwd_pos_mod)
    robot.create_joints()
    hinge_list = robot.get_hinge_list()

    # Build flooring scene
    floor = create_floor()

    # --------- Start controllers -----------
    # Start end-effector controller for moving the aft and fwd motors with relation to the tip.
    aft_motor = hinge_list[0]
    fwd_motor = hinge_list[2]
    end_effector = hinge_list[5]
    end_effector_controller = EndEffectorController(aft_motor, fwd_motor, end_effector, floor, 0.1, 0.5, 0.8)
    oneLegRobotApp.sim().add(end_effector_controller)

    # Robot force controller
    # movement = agx.Vec3(0, 0, 0.9)
    # robot_force_controller = ForceTopRobot(robot, movement, movement)
    # oneLegRobotApp.sim().add(robot_force_controller)

    # Moveable floor controller
    movement = agx.Vec3(0, 0, 0.9)
    floor_controller = MoveFloorController(robot=robot, floor=floor, movement=movement)
    oneLegRobotApp.sim().add(floor_controller)

    # Rendering details
    oneLegRobotApp.app().getSceneDecorator().setEnableShadows(True)
    oneLegRobotApp.app().setEnableDebugRenderer(True)

    # Set grid
    # oneLegRobotApp.app().setEnableGrid(True)
    # oneLegRobotApp.app().setGridSize(agx.Vec2(10,10))

    # Arrange camera to be centered around center of floor
    oneLegRobotApp.init_camera(eye=agx.Vec3(0, -50, 0), center=floor.getPosition())