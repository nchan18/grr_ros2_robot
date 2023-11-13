# examplo-ros2
ROS2 pkgs and resources for examplo bot

## Setup steps
-------
- Install ROS2
- Build+Install repo
- Install Isaac Sim
- Make ROS2 Default Ext

1. Open `~/.local/share/ov/pkg/isaac_sim-2022.1.1/apps/omni.isaac.sim.base.kit` \
2. search for `omni.isaac.ros_bridge` and change it to `omni.isaac.ros2_bridge`

- Setup Examplobot Extension

1. Open the extension manager by doing `Window -> Extensions`
2. Hit the gear icon and add the path `<path-to-repo>/isaac`
3. Enable the Examplo Bot extension by searching for it and hitting the toggle button.

https://docs.omniverse.nvidia.com/prod_launcher/prod_kit/linux-troubleshooting.html