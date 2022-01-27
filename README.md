# IEEE UAV Competition 2022 - [link1](https://ri4rover.org/index.html), [link2](https://www.computer.org/publications/tech-news/events/uav-2022)
IEEE UAV Competition 2022 - Low Power Computer Vision Challenges (LPCVC): Chase

<br>

### TO DO
+ Current MPC: position only -> velocities are needed
+ Better heading control (and planning)
+ Better centroid (Shape Lim)
+ Better target prediction
+ Obstacle avoidance

<br>



### Requirements
<details><summary>[click to see]</summary>
	
+ `OpenCV` version >= 4.4.0
+ `cv_bridge` with the corresponding `OpenCV`
+ `ROS` and `Gazebo`
    + refer [here](http://wiki.ros.org/ROS/Installation)
    + `$ sudo apt install ros-<distro>-desktop-full`
+ ROS dependencies
~~~shell
$ sudo apt install ros-melodic-gazebo-plugins

$ wget -O ubuntu.sh https://raw.githubusercontent.com/PX4/PX4-Autopilot/master/Tools/setup/ubuntu.sh
$ source ubuntu.sh
$ sudo apt upgrade libignition-math4
~~~

+ `UAVCC-simulator`
	+ Please follow instructions [here](https://github.com/Hunter314/uavcc-simulator)
	+ DO NOT FORGET `GAZEBO_PLUGIN_PATH`, `GAZEBO_MODEL_PATH`, and `animated_box` plugin build steps there.
	+ Check if successfully installed `UAVCC-simulator`, or not by
	~~~shell
	$ cd ~/path_to_uavcc_simulator/trial_1_setup
	$ roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/trial_1.world
	~~~

+ `PX4-SITL`
	+ Follow the instructions [here](https://github.com/engcang/mavros-gazebo-application#installation)

+ This Repo
~~~shell
$ cd ~/your_workspace/src

$ git clone --recursive https://github.com/engcang/ieee_uav_2022

$ cd ieee_uav_2022/

$ echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/drone_models" >> ~/.bashrc
$ . ~/.bashrc

$ cd ..
$ catkin build -DCMAKE_BUILD_TYPE=Release
~~~

+ OOQP
	+ Dependencies.
	```
	$ sudo apt-get install gfortran
	$ sudo apt-get install doxygen
        $ sudo apt install texlive-latex-base
	```
	+ Install `ma27` and type below commands in MA27's folder.
	```
	$ cd ma27-1.0.0
        $ mkdir build
        $ cd build
        $ ../configure
	$ make
	$ sudo make install
	```
	+ Install `OOQP` and type below commands in OOQP's folder.
	```
        $ cd OOQP
	$ ./configure
	$ make 
	$ sudo make install
	```

---

</details>

<br>


### How to run
+ Launch Gazebo world
	+ Change `world` in the launch file, if you want.
~~~shell
$ roslaunch ieee_uav gazebo.launch
~~~

<br>

+ Manually control drone with `joystick`
~~~shell
$ rosrun joy joy_node

or

$ rosrun joy joy_node _/dev:=/dev/input/js1 # check your joystick number

$ python mavros_joy_control.py
~~~
+ Then, press triangle to arm the drone
+ Then, press rectangle to change the mode into OFFBOARD
+ Then, press circle to change the control mode into POSITION control mode
+ Then, move the joystick axes to control

<br>

+ Run `ieee_uav`
~~~shell
$ roslaunch ieee_uav main.launch
~~~


<br>


### Components
+ ROS-YOLO using `OpenCV` code: from [here (myself)](https://github.com/engcang/ros-yolo-sort/blob/master/YOLO_and_ROS_ver/ros_opencv_dnn.py)