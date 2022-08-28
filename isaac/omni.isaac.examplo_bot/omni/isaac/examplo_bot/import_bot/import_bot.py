from omni.isaac.examplo_bot.base_sample import BaseSample
from omni.isaac.urdf import _urdf
from omni.isaac.core.robots import Robot
import omni.graph.core as og
from omni.isaac.core_nodes.scripts.utils import set_target_prims
import omni.kit.commands
import omni.usd
from pxr import UsdPhysics
import numpy as np
import math

def set_drive_params(drive, stiffness, damping, max_force):
    drive.GetStiffnessAttr().Set(stiffness)
    drive.GetDampingAttr().Set(damping)
    drive.GetMaxForceAttr().Set(max_force)
    return

class ImportBot(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()
        self.setup_world_action_graph()
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        self.robot_name = "examplo"
        root_path = "/home/helios/examplo-ros2/src/robot_description/"
        file_name = "examplo.urdf"
        self._robot_prim_path = self.import_robot("{}/{}".format(root_path, file_name))
        
        if self._robot_prim_path is None:
            print("Error: failed to import robot")
            return
        
        self._robot_prim = self._world.scene.add(
            Robot(prim_path=self._robot_prim_path, name=self.robot_name, position=np.array([0.0, 0.0, 0.3]))
        )
        self.configure_robot(self._robot_prim_path)
        return
    
    def import_robot(self, urdf_path):
        import_config = _urdf.ImportConfig()
        import_config.merge_fixed_joints = False
        import_config.fix_base = False
        import_config.make_default_prim = True
        import_config.self_collision = False
        import_config.create_physics_scene = False
        import_config.import_inertia_tensor = True
        import_config.default_drive_strength = 1047.19751
        import_config.default_position_drive_damping = 52.35988
        import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY
        import_config.distance_scale = 1.0
        import_config.density = 0.0
        result, prim_path = omni.kit.commands.execute( "URDFParseAndImportFile", 
            urdf_path=urdf_path,
            import_config=import_config)

        if result:
            return prim_path
        return None

    
    def configure_robot(self, robot_prim_path):
        w_sides = ['left', 'right']
        l_sides = ['front', 'rear']
        stage = self._world.stage

        for w_side in w_sides:
            for l_side in l_sides:
                for i in range(9):
                    joint_name = "{}_{}_roller_{}_joint".format(l_side, w_side, i)
                    joint_path = "{}/{}_{}_mecanum_link/{}".format(robot_prim_path, l_side, w_side, joint_name)
                    prim = stage.GetPrimAtPath(joint_path)
                    omni.kit.commands.execute(
                        "UnapplyAPISchemaCommand",
                        api=UsdPhysics.DriveAPI,
                        prim=prim,
                        api_prefix="drive",
                        multiple_api_token="angular")
                    # drive = UsdPhysics.DriveAPI.Get(prim, "angular")
                    # set_drive_params(drive, 0.0, 2.0, 0.0)

        front_left = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/chassis_link/front_left_mecanum_joint"), "angular")
        front_right = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/chassis_link/front_right_mecanum_joint"), "angular")
        rear_left = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/chassis_link/rear_left_mecanum_joint"), "angular")
        rear_right = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/chassis_link/rear_right_mecanum_joint"), "angular")

        set_drive_params(front_left, 0, math.radians(1e5), 98.0)
        set_drive_params(front_right, 0, math.radians(1e5), 98.0)
        set_drive_params(rear_left, 0, math.radians(1e5), 98.0)
        set_drive_params(rear_right, 0, math.radians(1e5), 98.0)
        self.create_lidar(robot_prim_path)
        self.setup_robot_action_graph(robot_prim_path)
        return

    def create_lidar(self, robot_prim_path):
        lidar_parent = "/examplo/lidar_link".format(robot_prim_path)
        lidar_path = "/lidar"
        result, prim = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path=lidar_path,
            parent=lidar_parent,
            min_range=0.4,
            max_range=25.0,
            draw_points=False,
            draw_lines=True,
            horizontal_fov=360.0,
            vertical_fov=30.0,
            horizontal_resolution=0.4,
            vertical_resolution=4.0,
            rotation_rate=0.0,
            high_lod=False,
            yaw_offset=0.0,
            enable_semantics=False
        )
        self.lidar_prim_path = prim
        return

    def setup_world_action_graph(self):
        og.Controller.edit(
            {"graph_path": "/globalclock", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                    ("Context.outputs:context", "PublishClock.inputs:context"),
                    ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ],
            }
        )
        return

    def setup_robot_action_graph(self, robot_prim_path):
        robot_controller_path = f"{robot_prim_path}/ros_interface_controller"
        og.Controller.edit(
            {"graph_path": robot_controller_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                    ("SubscribeJointState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
                    ("isaac_read_lidar_beams_node", "omni.isaac.range_sensor.IsaacReadLidarBeams"),
                    ("ros2_publish_laser_scan", "omni.isaac.ros2_bridge.ROS2PublishLaserScan"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("PublishJointState.inputs:topicName", "isaac_joint_states"),
                    ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "isaac_read_lidar_beams_node.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                    ("ReadSimTime.outputs:simulationTime", "ros2_publish_laser_scan.inputs:timeStamp"),
                    ("Context.outputs:context", "PublishJointState.inputs:context"),
                    ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                    ("Context.outputs:context", "ros2_publish_laser_scan.inputs:context"),
                    ("isaac_read_lidar_beams_node.outputs:execOut", "ros2_publish_laser_scan.inputs:execIn"),
                ],
            }
        )

        set_target_prims(primPath=f"{robot_controller_path}/SubscribeJointState", targetPrimPaths=[robot_prim_path])
        set_target_prims(primPath=f"{robot_controller_path}/PublishJointState", targetPrimPaths=[robot_prim_path])

        return

    async def setup_pre_reset(self):
        print("Here")
        return

    async def setup_post_reset(self):
        print("Here")
        return
    
    async def setup_post_clear(self):
        print("Here")
        return
    
    def world_cleanup(self):
        self._world.scene.remove_object(self.robot_name)
        return
