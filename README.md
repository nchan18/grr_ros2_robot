# examplo-ros2
ROS2 pkgs and resources for examplo bot

## Setup steps
-------
-Install ROS2
-Build+Install repo
-Install Isaac Sim
-Make ROS2 Default Ext
Open `~/.local/share/ov/pkg/isaac_sim-2022.1.1/apps/omni.isaac.sim.base.kit`
search for `omni.isaac.ros_bridge` and change it to `omni.isaac.ros2_bridge`

-Setup Examplobot Extension
Open the extension manager by doing `Window -> Extensions`
Hit the gear icon and add the path `<path-to-repo>/isaac`
Enable the Examplo Bot extension by searching for it and hitting the toggle button.

https://docs.omniverse.nvidia.com/prod_launcher/prod_kit/linux-troubleshooting.html