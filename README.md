gazebo_video_monitors
=====================

gazebo_video_monitors contains packages that lets the user record videos of a [Gazebo](http://gazebosim.org/) simulation. It provides a multicamera sensor that can be used for creating different types of videos with multiple views from inside the gazebo world.

There is a number of plugins available in the package which are explained next. More plugins can be developed, with minimal effort, to fit arbitrary use cases.

GazeboVideoMonitorPlugin
------------------------

The GazeboVideoMonitorPlugin plugin records videos in the following format.

![video-monitor-plugin](https://github.com/nlamprian/gazebo_video_monitors/wiki/assets/video-monitor-plugin.png)

It assumes two cameras: one which can be used to get a view of the world, and another one which can be attached to a robot to get a view from its perspective. Additionally, time metadata are logged in the video to make tracking of world and robot events easier. To configure the plugin, see the [gazebo_video_monitor_plugin.world](test/worlds/gazebo_video_monitor_plugin.world#L77) file. The plugin exposes two ROS services to control the recordings:

* Use the start_recording service to initiate a recording. Setting disable_window to true removes the second view from the recording. world_as_main_view can be used to set the world or robot camera as the main view of the recording.

```bash
rosservice call /gazebo/start_recording "{disable_window: false, world_as_main_view: false}"
```

* Use the stop_recording service to save or discard a recording. The filename must be given without the extension (.mp4). The recording is saved in the directory defined in the plugin configuration.

```bash
rosservice call /gazebo/stop_recording "{discard: false, filename: navigation-test}"
```

GazeboMultiVideoMonitorPlugin
-----------------------------

The GazeboMultiVideoMonitorPlugin plugin records multiple videos from different cameras simultaneously.

![multi-video-monitor-plugin](https://github.com/nlamprian/gazebo_video_monitors/wiki/assets/multi-video-monitor-plugin.gif)

An arbitrary number of cameras can be set up, which the plugin will read and record a video for each one of them. Time metadata can be logged in the videos as well. To configure the plugin, see the [gazebo_multi_video_monitor_plugin.world](test/worlds/gazebo_multi_video_monitor_plugin.world#L122) file. The plugin exposes two ROS services to control the recordings:

* Use the start_recording service to initiate a recording. There are no arguments.

* Use the stop_recording service to save or discard a recording, as explained in [GazeboVideoMonitorPlugin](#gazebovideomonitorplugin). The recordings are saved together in the subdirectory under the directory defined in the plugin configuration.

GazeboMultiCameraMonitorPlugin
------------------------------

The GazeboMultiCameraMonitorPlugin plugin records videos with a multi-camera setup. See the gif below (the video was created automatically; no editing was done).

![multi-camera-monitor-plugin](https://github.com/nlamprian/gazebo_video_monitors/wiki/assets/multi-camera-monitor-plugin.gif)

An arbitrary number of cameras can be set up, from which the plugin can select and configure the video stream. The cameras can be updated dynamically during the recording. Time metadata can be logged in the videos as well. To configure the plugin, see the [gazebo_multi_camera_monitor_plugin.world](test/worlds/gazebo_multi_camera_monitor_plugin.world#L168) file. The plugin exposes two ROS services and one topic to control the recordings:

* Use the start_recording service to initiate a recording. You can optionally specify the cameras with which to initialize the video stream.

```bash
rosservice call /gazebo/start_recording "{cameras: {names: []}}"
```

* Use the stop_recording service to save or discard a recording, as explained in [GazeboVideoMonitorPlugin](#gazebovideomonitorplugin).

* Publish a message to the camera_select topic to update the video stream. You can specify one or two cameras for the main and window view, respectively.

```bash
rostopic pub /gazebo/camera_select gazebo_video_monitor_msgs/Strings "{names: [camera_0, camera_2]}"
```

GazeboMultiViewMonitorPlugin
----------------------------

The GazeboMultiViewMonitorPlugin plugin records videos with up to 4 parallel camera streams.

![multi-view-monitor-plugin](https://github.com/nlamprian/gazebo_video_monitors/wiki/assets/multi-view-monitor-plugin.gif)

An arbitrary number of cameras can be set up, from which the plugin can select and configure the video stream. The cameras can be updated dynamically during the recording. Time metadata can be logged in the videos as well. To configure the plugin, see the [gazebo_multi_view_monitor_plugin.world](test/worlds/gazebo_multi_view_monitor_plugin.world#L168) file. The plugin exposes two ROS services and one topic to control the recordings:

* Use the start_recording service to initiate a recording. You can pass as arguments the cameras with which to initialize the video stream. An empty camera name results in a null camera stream in the respective quadrant.

```bash
rosservice call /gazebo/start_recording "{cameras: {names: ['', camera_1, camera_2, '']}}"
```

* Use the stop_recording service to save or discard a recording, as explained in [GazeboVideoMonitorPlugin](#gazebovideomonitorplugin).

* Publish a message to the camera_select topic to update the video stream. You can specify up to 4 cameras for the top left, top right, bottom left, and bottom right quadrant, respectively. A camera name can be left empty to disable the respective quadrant.

```bash
rostopic pub /gazebo/camera_select gazebo_video_monitor_msgs/Strings "{names: [camera_3, camera_1, '', camera_2]}"
```

Camera Configuration
--------------------

All plugins expose a set_camera ROS service for configuring the pose of a camera and the link to which the camera should be attached. You can call this service to dynamically reconfigure a camera during a simulation, or to fine tune and extract the initial pose of the camera when configuring the plugin (enabling the [camera visualizations](test/worlds/gazebo_video_monitor_plugin.world#L80) makes this process much easier).

```bash
rosservice call /gazebo/set_camera "{camera_name: robot_camera, model_name: robot-0001, link_name: gripper_link, pose: {x: 0.0, y: -0.05, z: -0.1, roll: -0.2, pitch: 0.0, yaw: 0.0}}"
```

Utilities
---------

A set of utility plugins is also available to be used along with the monitor plugins.

* **CameraContainsPlugin**: publishes a list of camera names when one or more of the tracked models enter the space of a box container.

Known Issues
------------

* The camera visualizations (not the cameras used for recording) are created and set correctly, but they don't get attached to the models.
* The cameras are not always moved correctly during initialization. The numberOfInitialAttachRetries parameter was introduced as a temporary fix.
