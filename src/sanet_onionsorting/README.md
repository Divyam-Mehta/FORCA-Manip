# sanet_onionsorting
Author: Prasanth Sengadu Suresh.

Owned By: THINC Lab, Department of Computer Science,
          University of Georgia.

Currently Managed By: Jacob Kruse (jacob-kruse@outlook.com).

This package uses Kinect V2 and YOLO to detect and obtain 3D coordinates of objects in the real world.
     
## The following instructions are written for Ubuntu 24.04, ROS2 Jazzy Jalisco

 ##### Note: For other versions, you might need to make appropriate changes

  ### 1) Install Anaconda 
  
  With the transition to virtual python environments in newer versions of Ubuntu, an Anaconda environment was used in this project. If you need to install Anaconda, check out the link below.

   [Anaconda](https://www.anaconda.com/download)

  After signing up, you should recieve a download link for Anaconda. Download the Linux installer to your machine. Give execute permission to the installation script and execute the script using the following commands.

    cd ~/Downloads
    chmod +x Anaconda3-2024.10-1-Linux-x86_64.sh
    ./Anaconda3-2024.10-1-Linux-x86_64.sh

  ##### Note: The download location and file name might be different, replace the commands accordingly

  Before the installation process completes, Anaconda's installer will ask if you would like Anaconda and the `conda` command to be initialized automatically in your shell by editing your `~/.bashrc`. It is recommended to answer "yes" for convenience. However, every time you open a new terminal, the Anaconda base environment will automatically activate. Use the following command to turn this off. 

    conda config --set auto_activate_base false

  ### 2) Install ROS2 and create a ROS2 workspace

  The instructions at the link below describe how to install ROS2 Jazzy Jalisco on your machine. 

   [ROS2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation.html)

  After installing ROS2, go to the following link to create a ROS2 Workspace. It is a good idea to have a dedicated workspace for this package that you can build independently within the Anaconda environment. 
      
   [ROS2 Workspace](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

   ### 3) Clone the package into your ROS2 Workspace

  Use the following commands to clone this package into your `~/ros2_ws/src` folder and resolve dependencies. 

    cd ~/ros2_ws/src
    git clone -b jazzy https://github.com/thinclab/sanet_onionsorting 
    cd ..
    rosdep install --from-paths src --ignore-src -r -y

  ##### Note: Do not use `colcon build` yet.

  ### 4) Create a Python environment with Anaconda and the `environment.yml`

  After you have cloned the `sanet_onionsorting` package, use the following commands to go to the package's `/anaconda_env` directory and create the required Python environment specified in the `environment.yml`. Before doing so, feel free to change the name of environment at the top of the `environment.yml` file.

    cd ~/ros2_ws/src/sanet_onionsorting/anaconda_env
    conda env create -f environment.yml

  Use the next command to activate the environment after installation. Replace `ros2_env` in the next couple of commands if you changed your environment name. 

    conda activate ros2_env

  Make sure to deactivate your Python environment whenever necessary using the following command.

    conda deactivate

  You can also add alias scripts into your `~/.bashrc` to easily activate and deactivate your environment. Use the commands below to do this. You can now use `py` and `de` in your terminal to activate or deactivate the environment, respectively.

    echo 'alias py="conda activate ros2_env"' >> ~/.bashrc
    echo 'alias de="conda deactivate"' >> ~/.bashrc
    source ~/.bashrc

  After setting up your environment, use the following commands to add the `activate_env_vars.sh` and `deactivate_env_vars.sh` scripts to your Anaconda environment's `/activate.d` and `/deactivate.d` folders. These scripts execute automatically when the environment is activated or deactivated. These scripts set the `$LD_LIBRARY_PATH` so that the Anaconda environment's library versions are prioritized in the terminal session when the Anaconda environment is activated. This helps avoid issues when building and executing the scripts. 

    cd ~/ros2_ws/src/sanet_onionsorting/anaconda_env
    mv activate_env_vars.sh ~/anaconda3/envs/ros2_env/etc/conda/activate.d/
    mv deactivate_env_vars.sh ~/anaconda3/envs/ros2_env/etc/conda/deactivate.d/

  ### 5) Build the `sanet_onionsorting` package

  With the previous steps complete, the package is ready to be built. Use the following commands to go to your workspace, activate your environment, and build the package. It is important to activate your environement before building the first time. 

    cd ~/ros2_ws
    conda activate ros2_env
    colcon build

  Feel free to replace the `colcon build` above with the following to avoid compiling when changes are made to any Python files.

    colcon build --symlink-install

  If you accidentally built the package without first activating the environment, you will get this error when you build inside the environment thereafter:

    /usr/bin/cmake: /home/psuresh/anaconda3/envs/ros2_env/lib/libcurl.so.4: no version information available (required by /usr/bin/cmake)
  
  This will not lead to any errors, but it can be annoying to see this. Remove the `/build` and `/install` folders in your `/ros2_ws` and build again in the Python environment to fix this. 

    cd ~/ros2_ws
    rm -rf build install
    conda activate ros2_env
    colcon build

  After the initial build in the environment, you can rebuild this package inside or outside of your Python environment without issue. 

  Make sure to source your workspace. You can add the following lines to your `~/.bashrc` with the commands below so that this happens automatically. 

    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
    echo "source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc

   ### 6) Install Various Camera Drivers/Wrappers

  Currently, this package supports the "Oak-D S2", "RealSense D435", and "Kinect V2" Cameras. Go to the corresponding section to install these drivers. It is recommended to have a separate workspace for these drivers/wrappers. 

   If you create a new workspace, make sure to source it after building the desired package(s). You can add the following lines to your `~/.bashrc` using the commands below so that this happens automatically. 

    echo "source ~/cam_ws/install/setup.bash" >> ~/.bashrc
    echo "source ~/cam_ws/install/local_setup.bash" >> ~/.bashrc

  #### For Oak-D:

  The DepthAI-ROS GitHub Repository and Documentation can be found at the links below. 

  [DepthAI-ROS GitHub](https://github.com/luxonis/depthai-ros.git)

  [DepthAI-ROS Documentation](https://docs.luxonis.com/software/depthai/manual-install/)

  Use the following command to download dependencies for the Oak-D package.

    sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash

  After this, clone the Jazzy branch from the `depthai-ros` repository into your desired workspace's `/src` folder, resolve dependencies, and build the package in the workspace using the commands below.

    cd ~/cam_ws/src
    git clone -b jazzy https://github.com/luxonis/depthai-ros.git
    cd ..
    rosdep install --from-paths src --ignore-src -r -y
    colcon build

  Then, give the camera read and write permissions for your machine. Go to the `/udev_rules` folder of the `sanet_onionsorting` package, move the `99-oak.rules` file to your `/rules.d` folder, and reload the udev for the changes to take effect.

    cd ~/ros2_ws/src/sanet_onionsorting/udev_rules
    sudo mv 99-oak.rules /etc/udev/rules.d/
    sudo udevadm control -R

  A static transform is required for the nodes in this package to function properly. For our current setup, this is being published in the Oak-D launch file because this is the main camera being utilized. If you would like to add this as well, add the following block of code into lines 183-188 in `/depthai-ros/depthai_ros_driver/launch/camera.launch.py`.

    183   Node(
    184       package="tf2_ros",
    185       executable="static_transform_publisher",
    186       name="root_transform",
    187       arguments=["0", "0", "0", "0", "0", "0", "1", "world", "root"],
    188   ),

  You can also just run a static transform publisher in a terminal using the following command.

    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 world root

  If you want to set the position of the camera, go to `/depthai-ros/depthai_ros_driver/launch/camera.launch.py` and edit the parameters below in lines 288-295 or 294-301 (depends on whether or not you added the block of code to publish the static transform). The current values shown are used for our setup.

    288 or 294   DeclareLaunchArgument("parent_frame", default_value="root"),

    290 or 296   DeclareLaunchArgument("cam_pos_x", default_value="1.0"),
    291 or 297   DeclareLaunchArgument("cam_pos_y", default_value="0.4"),
    292 or 298   DeclareLaunchArgument("cam_pos_z", default_value="1.72"),
    293 or 299   DeclareLaunchArgument("cam_roll", default_value="3.14"),
    294 or 300   DeclareLaunchArgument("cam_pitch", default_value="0.7"),
    295 or 301   DeclareLaunchArgument("cam_yaw", default_value="0.0"),

  #### For RealSense D435:

  The librealsense GitHub Repository, Realsense-ROS GitHub Repository, and Realsense-ROS Ubuntu Installation Instructions can be found at the links below.
  
  [librealsense GitHub](https://github.com/IntelRealSense/librealsense)

  [RealSense GitHub](https://github.com/IntelRealSense/realsense-ros)

  [RealSense Ubuntu Instructions](https://github.com/IntelRealSense/realsense-ros#installation-on-ubuntu)

  First, install the Intel RealSense SDK from source. See the RealSense Ubuntu Instructions above for other installation options.

    sudo apt install ros-jazzy-librealsense2*

  After this, clone the ros2-master branch from the `realsense-ros` repository into your desired workspace's `/src` folder, resolve dependencies, and build the package in the workspace using the commands below.

    cd ~/cam_ws/src
    git clone -b ros2-master https://github.com/IntelRealSense/realsense-ros.git
    cd ..
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
    colcon build

  Then, give the camera read and write permissions for your machine. Go to the `/udev_rules` folder of the `sanet_onionsorting` package, move the `99-realsense-libusb.rules` file to your `/rules.d` folder, and reload the udev for the changes to take effect.

    cd ~/ros2_ws/src/sanet_onionsorting/udev_rules
    sudo mv 99-realsense-libusb.rules /etc/udev/rules.d/
    sudo udevadm control -R

  A static transform is required for the nodes in this package to function properly. For our current setup, this is being published in the Oak-D launch file because this is the main camera being utilized. If you would like to add this to the RealSense launch file, add the following block of code into lines 120-125 in `/realsense-ros/realsense2_camera/launch/rs_launch.py`.

    120   launch_ros.actions.Node(
    121       package='tf2_ros',
    122       executable='static_transform_publisher',
    123       name="root_transform",
    124       arguments=["0", "0", "0", "0", "0", "0", "1", "world", "root"],
    125       ),

  You can also just run a static transform publisher in a terminal using the following command.

    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 world root

  If you want to set the position of the camera, go to `/realsense-ros/realsense2_description/urdf/test_d435_camera.urdf.xacro` and edit the parameters below in lines 9-12. The values shown below are used for our setup.

    9    <link name="root" />
    10   <xacro:sensor_d435 name="camera" parent="root" use_nominal_extrinsics="$(arg use_nominal_extrinsics)" add_plug="$(arg add_plug)" use_mesh="$(arg use_mesh)">
    11      <origin xyz="-0.227 -0.365 1.35355" rpy="0.0 0.523599 1.570796"/>
    12   </xacro:sensor_d435>

  The launch file that publishes the URDF for the RealSense D435 also starts an RViz2 session. If you wish to remove this, go to `/realsense-ros/realsense2_description/launch/view_model.launch.py` and replace line 55 with the following to stop the RViz2 node from executing.

    55   return launch.LaunchDescription([model_node])

  #### For Kinect V2:

  Will add instructions for Kinect V2 later when depth scaling is implemented.

---

  Although this package currently supports the cameras listed above, it is easy to implement your own camera as well. After downloading the driver for your desired camera, obtain the topics where it publishes the rgb images, depth images, and camera info. Then, open  `/sanet_onionsorting/scripts/service.py` and locate the logic for camera handling in lines 45-79. Add another "else if" statement like the one shown below in the "real" or "gazebo" section depending on your application. Make sure to replace the camera name and topic name in the lines of code. The topic should be where the camera is publishing the rgb images and the camera name you use is the parameter you should pass when running the launch file in step 6 (See Parameters below for more details). 
      
    elif (self.camera_name == 'add your camera name here'):
        self.subscription = self.create_subscription(Image, "/add your rgb topic name here", self.grabrgb, 10)

  Now open `/sanet_onionsorting/scripts/rgbd_imgpoint_to_tf.py` and locate the logic in lines 71-108. Add another "else if" statement like the one below in the "real" or "gazebo" section or both. Fill in the camera name and the corresponding rgb image, depth image, and camera info topics.

    elif (camera_name == "add your camera name here"):
        rgb_topic = 'add your rgb image topic name here'
        depth_topic = 'add your depth image topic name here'
        camera_info_topic = 'add your camera info topic name here'

  You can also add your camera in a similar fashion to the individual Yolo services, but it is not necessary as the launch file runs `service.py` and `rgbd_imgpoint_to_tf.py`.

   ### 7) How to run package

  Before you can utilize any of the object detection methods, you need to add the corresponding weights files. Use the command below to move your weights file(s) to the correct directory. Replace `[weights_file.pt]` with your weights file and `[version]` with the object detection method you will be utilizing. 

    mv [weights_file.pt] ~/ros2_ws/src/sanet_onionsorting/thirdparty/[version]/weights/
    
  Here is an example of moving the weights file `yolov8_weights.pt` to the correct directory for YOLO V8. 

    mv yolov8_weights.pt ~/ros2_ws/src/sanet_onionsorting/thirdparty/yolov8/weights/

  After all of the previous steps have been completed, the package is ready to be utilized. First, start whichever camera you will be using. The instructions for the supported cameras are shown below. 

  #### For Oak-D S2:
  
  Start the camera driver with the following launch command. This also publishes the URDF and its transforms for the position you specified in Step 5. 

    ros2 launch depthai_ros_driver camera.launch.py

   #### For RealSense D435:

   Start the camera driver with this command. 

    ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true

  Start publishing the URDF and its transforms using the following command. The URDF will be located where you specified in Step 5.  

    ros2 launch realsense2_description view_model.launch.py model:=test_d435_camera.urdf.xacro

  #### For Kinect V2:

  Will add instructions for Kinect V2 later when depth scaling is implemented.

---

  After you have the camera driver running, you are now ready to start the launch file to get the 3D locations. A configuration file,`/sanet_onionsorting/config/params.yaml`, is provided to easily change the different parameters of the `service.py` and `rgbd_imgpoint_to_tf.py` scripts (See Parameters Below). Make sure to build after changing if you did not use `colcon build --symlink-install`. You can then use the command below to launch the two nodes.

    ros2 launch sanet_onionsorting yolo_srv_launch.py

  You can also pass the parameters on the command line as shown below. 

    ros2 launch sanet_onionsorting yolo_srv_launch.py --ros-args -p debug:=True -p camera_name:='realsense' -p version:='yolov8'

  If everything is runnning correctly, you can echo the `/object_location` topic to print the 3D locations.

    ros2 topic echo /object_location

## Parameters

For the launch file in Step 6, a number of different parameters can be selected for different Yolo Versions, Cameras, and other options. Below are the parameters that can be changed in the `params.yaml` file found in the `/config` directory.

### Yolo Service Parameters ( `service.py` )
* `version`: This determines which object detection method you will be using
  * `frcnn`: Faster RCNN
  * `yolov5`: Yolo V5
  * `yolov7`: Yolo V7 Deepsort
  * `yolov8`: Yolo V8

* `choice`: This determines whether or not you will be working in the real world or in a simulated Gazebo environment.
  * `real`: Working in the real world
  * `gazebo`: Working in a simulated environment

* `camera_name`: This determines which camera you will be using to execute this package. At the moment, it can accept the following cameras. If you are utilizing a different camera, you will have to make proper changes in the `service.py` and `rgbd_imgpoint_to_tf.py` code to subscribe to your new camera's rgb, depth, and information topics.
  * `realsense`: RealSense D435 Camera
  * `kinect`: Kinect V2 Camera
  * `oak`: Oak-D S2 Camera

* `deepsort`: This is a boolean value that determines whether or not you will be using the deepsort option that is avaliable for ONLY Yolo V7. This defaults to `False` and should be set to `False` for all other versions of Yolo

### Yolo Cliet Parameters ( `rgbd_imgpoint.py` )

* `choice`: See "Yolo Service Parameters"

* `camera_name`: See "Yolo Service Parameters"

* `deepsort_sesrvice`: This is a boolean value that handles the service call for Yolo V7. This defaults to `False` and should remain `False` in most situations. If you are using Yolo V7, make sure to set this to `True`. A way to tell if this is set incorrectly is if the launch file is printing "Waiting for Service ..."

* `debug`: This is a boolean value that turns the `rgbd_imgpoint_to_tf.py` into debug mode if `True`. When in debug mode, an image window will open that allows the user to click on different areas. It will print the corresponding depth data to see if the depth function is working correctly. This defaults to `False`
   ##### Note: Although the `debug` parameter is included in the launch file, `rgbd_imgpoint_to_tf.py` can be run in debug mode independently without any of the services. Feel free to run `rgbd_imgpoint_to_tf.py` by itself with one of the camera drivers runnning and pass `True` to the `debug` parameter. You might also have to pass it the camera name depending on what camera you are utilizing. See the command below.
       
      ros2 run sanet_onionsorting rgbd_imgpoint_to_tf.py --ros-args -p debug:=True -p camera_name:='oak'

## Testing
A test launch file, `test_launch.py`, is also included with this package to test the functionality of the object detection methods. The  `test_params.yaml` file found in the `config` directory contains the parameters, which are the same as described in "Yolo Service Parameters". The command is shown below. 

    ros2 launch sanet_onionsorting test_launch.py

To see the output of each detection method, navigate to `~/ros2_ws/install/sanet_onionsorting/share/sanet_onionsorting/thirdparty/[version]/inference/output/`

Replace "[version]" with the object detection method you are utiizing, 

##### Note: At this point the FRCNN Service and Client has not been tested for the Jazzy Branch