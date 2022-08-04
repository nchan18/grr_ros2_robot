from omni.isaac.core import World
from omni.isaac.core.robots import Robot
world = World.instance()
examplo_robot = world.scene.add(Robot(prim_path="/examplo", name="examplo"))
world.scene.remove_object("examplo")