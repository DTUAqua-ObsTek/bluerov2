# UUV Simulator Tutorial #

## Spawning a Basic World ##

In a new terminal (ctrl + alt + t), start up a ROS server.

`roscore`

In a new terminal (ctrl + shift + t), launch an underwater world.

`roslaunch uuv_gazebo_worlds herkules_ship_wreck.launch gui:=false paused:=true`

Hint: you can add `--ros-args` to the command to get a list of arguments available.

Note: if you omit the gui argument, or set it to true, then the gazebo frontend GUI will launch and display the world.

Note: the paused argument sets the physics simulator to freeze (nothing is published), it can then be unfrozen when you're ready.

In a new terminal, explore the topics currently available.

`rostopic list`

**/hydrodynamics/current_velocity**: the world referenced current.

**/gazebo/model_states**: pose and velocity of any models in the simulation.

**/gazebo/link_states**: pose and velocity of any links in the simulation.

## Spawning a BlueROV2 ##

With a world active, you can now simulate a vehicle in it. We will upload the BlueROV2 into the world at 40 m depth.

`roslaunch bluerov2_description upload_bluerov2.launch z:=-50 x:=-15`

Hint: If you are viewing with Gazebo, you can right click on Models->bluerov2 in the left panel and click "move to", or "follow" to find the BlueROV2.

## Unpausing the simulation ##

First, pull up an RVIZ window:

`rosrun rviz rviz -d $(rospack find bluerov2_gazebo)/rviz/bluerov2.rviz`

Once rviz starts, check the box marked Image on the left panel.

Next, unpause the simulation:

`rosservice call /gazebo/unpause_physics`

You should see the camera view of the ROV in RVIZ, and the ROV starting to rise in the gazebo view. This makes sense as the BlueROV2 is positively buoyant!

### Activity ###

1. Explore the topics provided in /bluerov2/...
2. Run the rqt_publisher GUI: `rosrun rqt_publisher rqt_publisher`
3. Play around with publishing thrust commands to /blurov2/thrusters/[0-5]/input topics
4. See if you can get the ROV to turn on the spot, or hold depth.
5. Apply 0.0 to the thrusters to stop them.


## Configure BlueROV2 with Thruster Allocation Matrix Support ##

At this point, you are able to publish thrust messages to each of the thrusters manually and cause the model to move. UUV simulator comes with support for a Thruster Allocation Matrix (TAM) manager that converts a commanded body-frame wrench (axial force + rotational moment) into individual thrusts, whose resultant should match the command (within limits).

First, pause the simulation and delete the model.

`rosservice call /gazebo/pause_physics`

`rosservice call /gazebo/delete_model "model_name: 'bluerov2'"`

If your gazebo GUI crashes at this point, you can restart it by running `gzclient` in a terminal.

Next, upload another bluerov2 as you did before. Then run the following to launch the TAM manager:

`roslaunch bluerov2_control start_thruster_manager.launch`

Publish wrench topics to the /bluerov2/thruster_manager/input topic.

### Activity ###

Try to get the ROV to hold depth (view the /bluerov2/pose_gt/twist topic to get a velocity reference.)

## Reconfiguring the BlueROV2 Payload ##

The upload_bluerov2.launch script uses the information provided in `$(rospack find bluerov2_description)/robots/bluerov2_default.xacro` to spawn the vehicle with the correct payload and properties. This can be changed!

The xacro layout allows for layers of objects to be loaded in, the robots/bluerov2_default.xacro simply specifies all of the files .xacro files to be used. 

1. Pause and delete the BlueROV2.
2. Open the `bluerov2_description/robots/bluerov2_default.xacro` and save as a new file `bluerov2_description/robots/bluerov2_down_facing_camera.xacro`.
3. After the line `</xacro:bluerov2_base>`, paste in the following:
	`<xacro:bluerov_camera namespace="" parent_link="$(arg namespace)/base_link" suffix="_down">
    <origin xyz="0 0 -0.3" rpy="0 1.57 0"/>
  </xacro:bluerov_camera>`
4. Spawn the BlueROV2 with the new configuration:
`roslaunch bluerov2_description upload_bluerov2.launch mode:=down_facing_camera`

### For your own work ###

Extra sensors are defined in `$(rospack find bluerov2_description)/urdf/sensors.xacro`, `$(rospack find bluerov2_description)/urdf/snippets.xacro`, which inherit from (among other places) `$(rospack find uuv_sensor_ros_plugins)/urdf`. Try modifying and using the bluerov_altimeter snippet defined in `$(rospack find bluerov2_description)/urdf/snippets.xacro` to spawn a new robot `bluerov2_altimeter.xacro` that has a downward facing rangefinder like in the real model.

