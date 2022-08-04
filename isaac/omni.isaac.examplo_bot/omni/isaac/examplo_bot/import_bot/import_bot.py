from omni.isaac.examplo_bot.base_sample import BaseSample
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.urdf import _urdf
from omni.isaac.core.robots import Robot
import omni.kit.commands
import omni.usd
import numpy as np

class ImportBot(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        # Acquire the URDF extension interface
        # Set the settings in the import config
        import_config = _urdf.ImportConfig()
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = False
        import_config.fix_base = False
        import_config.make_default_prim = True
        import_config.self_collision = False
        import_config.create_physics_scene = True
        import_config.import_inertia_tensor = False
        import_config.default_drive_strength = 1047.19751
        import_config.default_position_drive_damping = 52.35988
        import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY
        import_config.distance_scale = 1.0
        import_config.density = 0.0
        # Get the urdf file path
        root_path = "/home/helios/examplo-ros2/src/robot_description/"
        file_name = "examplo.urdf"
        # Finally import the robot
        result, prim_path = omni.kit.commands.execute( "URDFParseAndImportFile", 
            urdf_path="{}/{}".format(root_path, file_name),
            import_config=import_config)
        self.robot_name = "examplo"
        self.examplo_robot_prim = self._world.scene.add(Robot(
            prim_path=prim_path, name=self.robot_name, position=np.array([0.0, 0.0, 0.3])
        ))
        
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
