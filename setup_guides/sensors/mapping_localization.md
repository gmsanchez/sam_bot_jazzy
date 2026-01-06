# Mapping and Localization

Now that we have a robot with its sensors set up, we can use the
obtained sensor information to build a map of the environment and to
localize the robot on the map. The `slam_toolbox` package is a set of
tools and capabilities for 2D Simultaneous Localization and Mapping
(SLAM) in potentially massive maps with ROS2. It is also one of the
officially supported SLAM libraries in Nav2, and we recommend to use
this package in situations you need to use SLAM on your robot setup.
Aside from the `slam_toolbox`, localization can also be implemented
through the `nav2_amcl` package. This package implements Adaptive Monte
Carlo Localization (AMCL) which estimates the position and orientation
of the robot in a map. Other techniques may also be available, please
check Nav2 documentation for more information.

Both the `slam_toolbox` and `nav2_amcl` use information from the laser
scan sensor to be able to perceive the robot's environment. Hence, to
verify that they can access the laser scan sensor readings, we must make
sure that they are subscribed to the correct topic that publishes the
`sensor_msgs/LaserScan` message. This can be configured by setting their
`scan_topic` parameters to the topic that publishes that message. It is
a convention to publish the `sensor_msgs/LaserScan` messages to `/scan`
topic. Thus, by default, the `scan_topic` parameter is set to `/scan`.
Recall that when we added the lidar sensor to `sam_bot` in the previous
section, we set the topic to which the lidar sensor will publish the
`sensor_msgs/LaserScan` messages as `/scan`.

In-depth discussions on the complete configuration parameters will not
be a scope of our tutorials since they can be pretty complex. Instead,
we recommend you to have a look at their official documentation in the
links below.

> **_SEE ALSO:_**
\| For the complete list of configuration parameters of `slam_toolbox`,
see the [Github repository of
slam_toolbox](https://github.com/SteveMacenski/slam_toolbox#readme). \|
For the complete list of configuration parameters and example
configuration of `nav2_amcl`, see the [AMCL Configuration
Guide](https://docs.nav2.org/configuration/packages/configuring-amcl.html).


You can also refer to the [(SLAM) Navigating While Mapping
guide](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)
for the tutorial on how to use Nav2 with SLAM. You can verify that
`slam_toolbox` and `nav2_amcl` have been correctly setup by visualizing
the map and the robot's pose in RViz, similar to what was shown in the
previous section.

## Costmap 2D

The costmap 2D package makes use of the sensor information to provide a
representation of the robot's environment in the form of an occupancy
grid. The cells in the occupancy grid store cost values between 0-254
which denote a cost to travel through these zones. A cost of 0 means the
cell is free while a cost of 254 means that the cell is lethally
occupied. Values in between these extremes are used by navigation
algorithms to steer your robot away from obstacles as a potential field.
Costmaps in Nav2 are implemented through the `nav2_costmap_2d` package.

The costmap implementation consists of multiple layers, each of which
has a certain function that contributes to a cell's overall cost. The
package consists of the following layers, but are plugin-based to allow
customization and new layers to be used as well: static layer, inflation
layer, range layer, obstacle layer, and voxel layer. The static layer
represents the map section of the costmap, obtained from the messages
published to the `/map` topic like those produced by SLAM. The obstacle
layer includes the objects detected by sensors that publish either or
both the `LaserScan` and `PointCloud2` messages. The voxel layer is
similar to the obstacle layer such that it can use either or both the
`LaserScan` and `PointCloud2` sensor information but handles 3D data
instead. The range layer allows for the inclusion of information
provided by sonar and infrared sensors. Lastly, the inflation layer
represents the added cost values around lethal obstacles such that our
robot avoids navigating into obstacles due to the robot's geometry. In
the next subsection of this tutorial, we will have some discussion about
the basic configuration of the different layers in `nav2_costmap_2d`.

The layers are integrated into the costmap through a plugin interface
and then inflated using a user-specified [inflation
radius](http://wiki.ros.org/costmap_2d/hydro/inflation), if the
inflation layer is enabled. For a deeper discussion on costmap concepts,
you can have a look at the [ROS1 costmap_2D
documentation](http://wiki.ros.org/costmap_2d). Note that the
`nav2_costmap_2d` package is mostly a straightforward ROS2 port of the
ROS1 navigation stack version with minor changes required for ROS2
support and some new layer plugins.

### Configuring nav2_costmap_2d

In this subsection, we will show an example configuration of
`nav2_costmap_2d` such that it uses the information provided by the
lidar sensor of `sam_bot`. We will show an example configuration that
uses static layer, obstacle layer, voxel layer, and inflation layer. We
set both the obstacle and voxel layer to use the `LaserScan` messages
published to the `/scan` topic by the lidar sensor. We also set some of
the basic parameters to define how the detected obstacles are reflected
in the costmap. Note that this configuration is to be included in the
configuration file of Nav2.

``` {.yaml lineno-start="1"}
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: false
      rolling_window: false
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True
```

In the configuration above, notice that we set the parameters for two
different costmaps: `global_costmap` and `local_costmap`. We set up two
costmaps since the `global_costmap` is mainly used for long-term
planning over the whole map while `local_costmap` is for short-term
planning and collision avoidance.

The layers that we use for our configuration are defined in the
`plugins` parameter, as shown in line 13 for the `global_costmap` and
line 50 for the `local_costmap`. These values are set as a list of
mapped layer names that also serve as namespaces for the layer
parameters we set up starting at lines 14 and line 51. Note that each
layer/namespace in this list must have a `plugin` parameter (as
indicated in lines 15, 18, 32, 52, and 68) defining the type of plugin
to be loaded for that specific layer.

For the static layer (lines 14-16), we set the
`map_subscribe_transient_local` parameter to `True`. This sets the QoS
settings for the map topic. Another important parameter for the static
layer is the `map_topic` which defines the map topic to subscribe to.
This defaults to `/map` topic when not defined.

For the obstacle layer (lines 17-30), we define its sensor source under
the `observation_sources` parameter (line 20) as `scan` whose parameters
are set up in lines 22-30. We set its `topic` parameter as the topic
that publishes the defined sensor source and we set the `data_type`
according to the sensor source it will use. In our configuration, the
obstacle layer will use the `LaserScan` published by the lidar sensor to
`/scan`.

Note that the obstacle layer and voxel layer can use either or both
`LaserScan` and `PointCloud2` as their `data_type` but it is set to
`LaserScan` by default. The code snippet below shows an example of using
both the `LaserScan` and `PointCloud2` as the sensor sources. This may
be particularly useful when setting up your own physical robot.

``` shell
obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  enabled: True
  observation_sources: scan pointcloud
  scan:
    topic: /scan
    data_type: "LaserScan"
  pointcloud:
    topic: /depth_camera/points
    data_type: "PointCloud2"
```

For the other parameters of the obstacle layer, the
`max_obstacle_height` parameter sets the maximum height of the sensor
reading to return to the occupancy grid. The minimum height of the
sensor reading can also be set using the `min_obstacle_height`
parameter, which defaults to 0 since we did not set it in the
configuration. The `clearing` parameter is used to set whether the
obstacle is to be removed from the costmap or not. The clearing
operation is done by raytracing through the grid. The maximum and
minimum range to raytrace clear objects from the costmap is set using
the `raytrace_max_range` and `raytrace_min_range` respectively. The
`marking` parameter is used to set whether the inserted obstacle is
marked into the costmap or not. We also set the maximum and minimum
range to mark obstacles in the costmap through the `obstacle_max_range`
and `obstacle_min_range` respectively.

For the inflation layer (lines 31-34 and 67-70), we set the exponential
decay factor across the inflation radius using the `cost_scaling_factor`
parameter. The value of the radius to inflate around lethal obstacles is
defined using the `inflation_radius`.

For the voxel layer (lines 51-66), we set the `publish_voxel_map`
parameter to `True` to enable the publishing of the 3D voxel grid. The
resolution of the voxels in height is defined using the `z_resolution`
parameter, while the number of voxels in each column is defined using
the `z_voxels` parameter. The `mark_threshold` parameter sets the
minimum number of voxels in a column to mark as occupied in the
occupancy grid. We set the `observation_sources` parameter of the voxel
layer to `scan`, and we set the scan parameters (in lines 61-66) similar
to the parameters that we have discussed for the obstacle layer. As
defined in its `topic` and `data_type` parameters, the voxel layer will
use the `LaserScan` published on the `/scan` topic by the lidar scanner.

Note that the we are not using a range layer for our configuration but
it may be useful for your own robot setup. For the range layer, its
basic parameters are the `topics`, `input_sensor_type`, and
`clear_on_max_reading` parameters. The range topics to subscribe to are
defined in the `topics` parameter. The `input_sensor_type` is set to
either `ALL`, `VARIABLE`, or `FIXED`. The `clear_on_max_reading` is a
boolean parameter that sets whether to clear the sensor readings on max
range. Have a look at the configuration guide in the link below in case
you need to set it up.

> **_SEE ALSO:_**
For more information on `nav2_costmap_2d` and the complete list of layer
plugin parameters, see the [Costmap 2D Configuration
Guide](https://docs.nav2.org/configuration/packages/configuring-costmaps.html).


### Build, Run and Verification

We will first launch `display.launch.py` which launches the robot state
publisher that provides the `base_link` =\> `sensors` transformations in
our URDF, launches Gazebo that acts as our physics simulator, and
provides the `odom` =\> `base_link` from the differential drive plugin
or the ekf_node. It also launches RViz which we can use to visualize the
robot and sensor information.

Then we will launch `slam_toolbox` to publish to `/map` topic and
provide the `map` =\> `odom` transform. Recall that the `map` =\> `odom`
transform is one of the primary requirements of the Nav2 system. The
messages published on the `/map` topic will then be used by the static
layer of the `global_costmap`.

After we have properly setup our robot description, odometry sensors,
and necessary transforms, we will finally launch the Nav2 system itself.
For now, we will only be exploring the costmap generation system of
Nav2. After launching Nav2, we will visualize the costmaps in RViz to
confirm our output.

#### Launching Description Nodes, RViz and Gazebo

Let us now launch our Robot Description Nodes, RViz and Gazebo through
the launch file `display.launch.py`. Open a new terminal and execute the
lines below.

``` shell
colcon build
. install/setup.bash
ros2 launch sam_bot_description display.launch.py
```

RViz and the Gazebo should now be launched with `sam_bot` present in
both. Recall that the `base_link` =\> `sensors` transform is now being
published by `robot_state_publisher` and the `odom` =\> `base_link`
transform by our Gazebo plugins. Both transforms should now be displayed
show without errors in RViz.

#### Launching slam_toolbox

To be able to launch `slam_toolbox`, make sure that you have installed
the `slam_toolbox` package by executing the following command:

``` shell
sudo apt install ros-<ros2-distro>-slam-toolbox
```

We will launch the `async_slam_toolbox_node` of `slam_toolbox` using the
package's built-in launch files, but first we **must** make sure we have our own configuration file. Create a file named `mapper_params_online_async.yaml` in the `config` directory with the following contents

``` yaml
slam_toolbox:
  ros__parameters:

    # Plugin params
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # ROS Parameters
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    use_map_saver: true
    mode: mapping #localization

    # if you'd like to immediately start continuing a map at a given pose
    # or at the dock, but they are mutually exclusive, if pose is given
    # will use pose
    #map_file_name: test_steve
    # map_start_pose: [0.0, 0.0, 0.0]
    #map_start_at_dock: true

    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02 #if 0 never publishes odometry
    map_update_interval: 5.0
    resolution: 0.05
    max_laser_range: 20.0 #for rastering images
    minimum_time_interval: 0.5
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000 #// program needs a larger stack size to serialize large maps
    enable_interactive_mode: true

    # General Parameters
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.5
    minimum_travel_heading: 0.5
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1  
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    do_loop_closing: true 
    loop_match_minimum_chain_size: 10           
    loop_match_maximum_variance_coarse: 3.0  
    loop_match_minimum_response_coarse: 0.35    
    loop_match_minimum_response_fine: 0.45

    # Correlation Parameters - Correlation Parameters
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1 

    # Correlation Parameters - Loop Closure Parameters
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    # Scan Matcher Parameters
    distance_variance_penalty: 0.5      
    angle_variance_penalty: 1.0    

    fine_search_angle_offset: 0.00349     
    coarse_search_angle_offset: 0.349   
    coarse_angle_resolution: 0.0349        
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
    min_pass_through: 2
    occupancy_threshold: 0.1
```

> _*NOTE:*_ The same exact file can be found on the `slam_toolbox` package and is generally located at `/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml`.

 Open a new terminal and then execute
the following lines:

``` shell
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true slam_params_file:=./sam_bot_description/config/mapper_params_online_async.yaml
```

The `slam_toolbox` should now be publishing to the `/map` topic and
providing the `map` =\> `odom` transform.

We can verify in RViz that the `/map` topic is being published. In the
RViz window, click the add button at the bottom-left part then go to
`By topic` tab then select the `Map` under the `/map` topic. You should
be able to visualize the message received in the `/map` as shown in the
image below.

![image](images/map.png)

We can also check that the transforms are correct by executing the
following lines in a new terminal:

``` shell
ros2 run tf2_tools view_frames
```

The line above will create a `frames.pdf` file that shows the current
transform tree. Your transform tree should be similar to the one shown
below:

![image](images/view_frames.png)

#### Launching Nav2

First, Make sure you have installed the Nav2 packages by executing the
following:

``` shell
sudo apt install ros-<ros2-distro>-navigation2
sudo apt install ros-<ros2-distro>-nav2-bringup
```

We will now launch Nav2 using the `nav2_bringup`'s built-in launch
file, `navigation_launch.py`, but first we must make sure we have our own configuration file. Create a file named `nav2_params.yaml` in the config directory with the following contents

``` yaml
amcl:
  ros__parameters:
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan

bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    wait_for_service_timeout: 1000
    action_server_result_timeout: 900.0
    navigators: ["navigate_to_pose", "navigate_through_poses"]
    navigate_to_pose:
      plugin: "nav2_bt_navigator::NavigateToPoseNavigator"
    navigate_through_poses:
      plugin: "nav2_bt_navigator::NavigateThroughPosesNavigator"
    # 'default_nav_through_poses_bt_xml' and 'default_nav_to_pose_bt_xml' are use defaults:
    # nav2_bt_navigator/navigate_to_pose_w_replanning_and_recovery.xml
    # nav2_bt_navigator/navigate_through_poses_w_replanning_and_recovery.xml
    # They can be set here or via a RewrittenYaml remap from a parent launch file to Nav2.

    # plugin_lib_names is used to add custom BT plugins to the executor (vector of strings).
    # Built-in plugins are added automatically
    # plugin_lib_names: []

    error_code_names:
      - compute_path_error_code
      - follow_path_error_code

controller_server:
  ros__parameters:
    enable_stamped_cmd_vel: True
    controller_frequency: 20.0
    costmap_update_timeout: 0.30
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"] # "precise_goal_checker"
    controller_plugins: ["FollowPath"]
    use_realtime_priority: false

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    # Goal checker parameters
    #precise_goal_checker:
    #  plugin: "nav2_controller::SimpleGoalChecker"
    #  xy_goal_tolerance: 0.25
    #  yaw_goal_tolerance: 0.25
    #  stateful: True
    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 56
      model_dt: 0.05
      batch_size: 2000
      ax_max: 3.0
      ax_min: -3.0
      ay_max: 3.0
      ay_min: -3.0
      az_max: 3.5
      vx_std: 0.2
      vy_std: 0.2
      wz_std: 0.4
      vx_max: 0.5
      vx_min: -0.35
      vy_max: 0.5
      wz_max: 1.9
      iteration_count: 1
      prune_distance: 1.7
      transform_tolerance: 0.1
      temperature: 0.3
      gamma: 0.015
      motion_model: "DiffDrive"
      visualize: true
      regenerate_noises: true
      TrajectoryVisualizer:
        trajectory_step: 5
        time_step: 3
      AckermannConstraints:
        min_turning_r: 0.2
      critics: [
        "ConstraintCritic", "CostCritic", "GoalCritic",
        "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic",
        "PathAngleCritic", "PreferForwardCritic"]
      ConstraintCritic:
        enabled: true
        cost_power: 1
        cost_weight: 4.0
      GoalCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 1.4
      GoalAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.0
        threshold_to_consider: 0.5
      PreferForwardCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 0.5
      CostCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.81
        near_collision_cost: 253
        critical_cost: 300.0
        consider_footprint: false
        collision_cost: 1000000.0
        near_goal_distance: 1.0
        trajectory_point_step: 2
      PathAlignCritic:
        enabled: true
        cost_power: 1
        cost_weight: 14.0
        max_path_occupancy_ratio: 0.05
        trajectory_point_step: 4
        threshold_to_consider: 0.5
        offset_from_furthest: 20
        use_path_orientations: false
      PathFollowCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        offset_from_furthest: 5
        threshold_to_consider: 1.4
      PathAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 2.0
        offset_from_furthest: 4
        threshold_to_consider: 0.5
        max_angle_to_furthest: 1.0
        mode: 0
      # TwirlingCritic:
      #   enabled: true
      #   twirling_cost_power: 1
      #   twirling_cost_weight: 10.0

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.70
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.7
      always_send_full_costmap: True

# The yaml_filename does not need to be specified since it going to be set by defaults in launch.
# If you'd rather set it in the yaml, remove the default "map" value in the tb3_simulation_launch.py
# file & provide full path to map below. If CLI map configuration or launch default is provided, that will be used.
# map_server:
#   ros__parameters:
#     yaml_filename: ""

map_saver:
  ros__parameters:
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65
    map_subscribe_transient_local: True

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    costmap_update_timeout: 1.0
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

smoother_server:
  ros__parameters:
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: True

behavior_server:
  ros__parameters:
    enable_stamped_cmd_vel: True
    local_costmap_topic: local_costmap/costmap_raw
    global_costmap_topic: global_costmap/costmap_raw
    local_footprint_topic: local_costmap/published_footprint
    global_footprint_topic: global_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "assisted_teleop", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
    backup:
      plugin: "nav2_behaviors::BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors::DriveOnHeading"
    wait:
      plugin: "nav2_behaviors::Wait"
    assisted_teleop:
      plugin: "nav2_behaviors::AssistedTeleop"
    local_frame: odom
    global_frame: map
    robot_base_frame: base_link
    transform_tolerance: 0.1
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    action_server_result_timeout: 900.0
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: True
      waypoint_pause_duration: 200

route_server:
  ros__parameters:
    # The graph_filepath does not need to be specified since it going to be set by defaults in launch.
    # If you'd rather set it in the yaml, remove the default "graph" value in the launch file(s).
    # file & provide full path to map below. If graph config or launch default is provided, it is used
    # graph_filepath: $(find-pkg-share nav2_route)/graphs/aws_graph.geojson
    boundary_radius_to_achieve_node: 1.0
    radius_to_achieve_node: 2.0
    smooth_corners: true
    operations: ["AdjustSpeedLimit", "ReroutingService", "CollisionMonitor"]
    ReroutingService:
      plugin: "nav2_route::ReroutingService"
    AdjustSpeedLimit:
      plugin: "nav2_route::AdjustSpeedLimit"
    CollisionMonitor:
      plugin: "nav2_route::CollisionMonitor"
      max_collision_dist: 3.0
    edge_cost_functions: ["DistanceScorer", "CostmapScorer"]
    DistanceScorer:
      plugin: "nav2_route::DistanceScorer"
    CostmapScorer:
      plugin: "nav2_route::CostmapScorer"

velocity_smoother:
  ros__parameters:
    enable_stamped_cmd_vel: True
    smoothing_frequency: 20.0
    scale_velocities: False
    feedback: "OPEN_LOOP"
    max_velocity: [0.5, 0.0, 2.0]
    min_velocity: [-0.5, 0.0, -2.0]
    max_accel: [2.5, 0.0, 3.2]
    max_decel: [-2.5, 0.0, -3.2]
    odom_topic: "odom"
    odom_duration: 0.1
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0

collision_monitor:
  ros__parameters:
    enable_stamped_cmd_vel: True
    base_frame_id: "base_footprint"
    odom_frame_id: "odom"
    cmd_vel_in_topic: "cmd_vel_smoothed"
    cmd_vel_out_topic: "cmd_vel"
    state_topic: "collision_monitor_state"
    transform_tolerance: 0.2
    source_timeout: 1.0
    base_shift_correction: True
    stop_pub_timeout: 2.0
    # Polygons represent zone around the robot for "stop", "slowdown" and "limit" action types,
    # and robot footprint for "approach" action type.
    polygons: ["FootprintApproach"]
    FootprintApproach:
      type: "polygon"
      action_type: "approach"
      footprint_topic: "/local_costmap/published_footprint"
      time_before_collision: 1.2
      simulation_time_step: 0.1
      min_points: 6
      visualize: False
      enabled: True
    observation_sources: ["scan"]
    scan:
      type: "scan"
      topic: "scan"
      min_height: 0.15
      max_height: 2.0
      enabled: True

docking_server:
  ros__parameters:
    enable_stamped_cmd_vel: True
    controller_frequency: 50.0
    initial_perception_timeout: 5.0
    wait_charge_timeout: 5.0
    dock_approach_timeout: 30.0
    undock_linear_tolerance: 0.05
    undock_angular_tolerance: 0.1
    max_retries: 3
    base_frame: "base_link"
    fixed_frame: "odom"
    dock_backwards: false
    dock_prestaging_tolerance: 0.5

    # Types of docks
    dock_plugins: ['simple_charging_dock']
    simple_charging_dock:
      plugin: 'opennav_docking::SimpleChargingDock'
      docking_threshold: 0.05
      staging_x_offset: -0.7
      use_external_detection_pose: true
      use_battery_status: false # true
      use_stall_detection: false # true

      external_detection_timeout: 1.0
      external_detection_translation_x: -0.18
      external_detection_translation_y: 0.0
      external_detection_rotation_roll: -1.57
      external_detection_rotation_pitch: -1.57
      external_detection_rotation_yaw: 0.0
      filter_coef: 0.1

    # Dock instances
    # The following example illustrates configuring dock instances.
    # docks: ['home_dock']  # Input your docks here
    # home_dock:
    #   type: 'simple_charging_dock'
    #   frame: map
    #   pose: [0.0, 0.0, 0.0]

    controller:
      k_phi: 3.0
      k_delta: 2.0
      v_linear_min: 0.15
      v_linear_max: 0.15
      use_collision_detection: true
      costmap_topic: "local_costmap/costmap_raw"
      footprint_topic: "local_costmap/published_footprint"
      transform_tolerance: 0.1
      projection_time: 5.0
      simulation_step: 0.1
      dock_collision_threshold: 0.3

loopback_simulator:
  ros__parameters:
    base_frame_id: "base_footprint"
    odom_frame_id: "odom"
    map_frame_id: "map"
    scan_frame_id: "base_scan"  # tb4_loopback_simulator.launch.py remaps to 'rplidar_link'
    update_duration: 0.02
    scan_range_min: 0.05
    scan_range_max: 30.0
    scan_angle_min: -3.1415
    scan_angle_max: 3.1415
    scan_angle_increment: 0.02617
    scan_use_inf: true

```
> _*NOTE:*_ The same exact file can be found on the `nav2_bringup` package and is generally located at `/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml`. However, notice the addition of `enable_stamped_cmd_vel: True`, in order for Nav2 to use `TwistStamped` messages.

Open a new terminal and execute the following:

``` shell
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=./sam_bot_description/config/nav2_params.yaml
```

Note that the parameters of the `nav2_costmap_2d` that we discussed in
the previous subsection are included in the default parameters of
`navigation_launch.py`. Aside from the `nav2_costmap_2d` parameters, it
also contains parameters for the other nodes that are included in Nav2
implementation.

After we have properly set up and launched Nav2, the `/global_costmap`
and `/local_costmap` topics should now be active.

> **_NOTE:_**
To make the costmaps show up, run the 3 commands in this order
1.  Launching Description Nodes, RViz and Gazebo - wait a bit for everything to launch
2.  Launching slam_toolbox - in logs wait for "Registering sensor"
3.  Launching Nav2 - in logs wait for "Creating bond timer"

#### Visualizing Costmaps in RViz

The `global_costmap`, `local_costmap` and the voxel representation of
the detected obstacles can be visualized in RViz.

To visualize the `global_costmap` in RViz, click the add button at the
bottom-left part of the RViz window. Go to `By topic` tab then select
the `Map` under the `/global_costmap/costmap` topic. The
`global_costmap` should show in the RViz window, as shown below. The
`global_costmap` shows areas which should be avoided (black) by our
robot when it navigates our simulated world in Gazebo.

![image](images/costmap_global_rviz.png)

To visualize the `local_costmap` in RViz, select the `Map` under the
`/local_costmap/costmap` topic. Set the `color scheme` in RViz to
`costmap` to make it appear similar to the image below.

![image](images/local_costmap_rviz.png)

To visualize the voxel representation of the detected object, open a new
terminal and execute the following lines:

``` shell
ros2 run nav2_costmap_2d nav2_costmap_2d_markers voxel_grid:=/local_costmap/voxel_grid visualization_marker:=/my_marker
```

The line above sets the topic where the the markers will be published to
`/my_marker`. To see the markers in RViz, select `Marker` under the
`/my_marker` topic, as shown below.

![image](images/add_my_marker.png)

Then set the `fixed frame` in RViz to `odom` and you should now see the
voxels in RViz, which represent the cube and the sphere that we have in
the Gazebo world:

![image](images/voxel_layer.png)

## Conclusion

In this section of our robot setup guide, we have discussed the
importance of sensor information for different tasks associated with
Nav2. More specifically, tasks such as mapping (SLAM), localization
(AMCL), and perception (costmap) tasks. Then, we set up a basic
configuration for the `nav2_costmap_2d` package using different layers
to produce a global and local costmap. We then verify our work by
visualizing these costmaps in RViz.
