# IEEE UAV Competition 2022 - [link1](https://ri4rover.org/index.html), [link2](https://www.computer.org/publications/tech-news/events/uav-2022)
IEEE UAV Competition 2022 - Low Power Computer Vision Challenges (LPCVC): Chase


<br>

## Team: Hercules
### Members: EungChang Lee, Dongkyu Lee, HyungTae Lim, Seungwon Song

<br>
<br>


### Testing setup
+ CPU: i9-10900K, **GPU: RTX3080**
+ `CUDA` version 11.5.119
+ `cuDNN` 8.3.1 for `CUDA` 11.5
+ `OpenCV` version 4.5.2 with `CUDA` enabled (not ROS default version) - [reference](https://github.com/engcang/vins-application#-optional-if-also-contrib-for-opencv-should-be-built)
  + with the options below
~~~makefile
$ cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_C_COMPILER=gcc-6 \
      -D CMAKE_CXX_COMPILER=g++-6 \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_GENERATE_PKGCONFIG=YES \
      -D WITH_CUDA=ON \
      -D OPENCV_DNN_CUDA=ON \
      -D WITH_CUDNN=ON \
      -D CUDA_ARCH_BIN=8.6 \
      -D CUDA_ARCH_PTX="" \
      -D ENABLE_FAST_MATH=ON \
      -D CUDA_FAST_MATH=ON \
      -D WITH_CUBLAS=ON \
      -D WITH_LIBV4L=ON \
      -D WITH_GSTREAMER=ON \
      -D WITH_GSTREAMER_0_10=OFF \
      -D WITH_QT=ON \
      -D WITH_OPENGL=ON \
      -D BUILD_opencv_cudacodec=OFF \
      -D CUDA_NVCC_FLAGS="--expt-relaxed-constexpr" \
      -D WITH_TBB=ON \
      -D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.5.2/modules \
      ../
~~~
+ `cv_bridge` manually built with new `OpenCV` version 4.5.2 (not ROS default version) - [reference](https://github.com/engcang/vins-application#-cv_bridge-with-opencv-4x-version)
+ ROS `melodic` with Ubuntu 18.04 LTS

<br>


### How to install and setup
+ Assuming `PX4-SITL` simulator is already installed through following instructions to install `PX4-Avoidance` package
+ Clone the repository
~~~shell
$ git clone --recursive https://github.com/engcang/ieee_uav_2022
~~~

+ Setup `UAVCC-simulator`
~~~shell
$ cd ieee_uav_2022/uavcc-simulator/trial_1_setup
$ mkdir build && cd build
$ cmake ..
$ make

$ cd ieee_uav_2022/uavcc-simulator/trial_2_setup
$ mkdir build && cd build
$ cmake ..
$ make

$ cd ieee_uav_2022/uavcc-simulator
$ echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/trial_1_setup:$(pwd)/trial_2_setup" >> ~/.bashrc
$ source ~/.bashrc

$ cd ieee_uav_2022/uavcc-simulator
$ echo "export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$(pwd)/trial_1_setup/build:$(pwd)/trial_2_setup/build" >> ~/.bashrc
$ source ~/.bashrc
~~~

+ Setup `OOQP`
	+ Dependencies.
	~~~shell
	$ sudo apt-get install gfortran
	$ sudo apt-get install doxygen
	$ sudo apt-get install texlive-latex-base
	~~~
	+ Install `ma27` and type below commands in MA27's folder.
	~~~shell
	$ cd ieee_uav_2022/ma27-1.0.0
	$ ./configure
	$ make
	$ sudo make install
	~~~
	+ Install `OOQP` and type below commands in OOQP's folder.
	~~~shell
	$ cd ieee_uav_2022/OOQP
	$ ./configure
	$ make 
	$ sudo make install
	~~~

+ Build this Repo
~~~shell
$ cd ieee_uav_2022/
$ echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/drone_models" >> ~/.bashrc
$ source ~/.bashrc

$ cd ..
$ catkin build -DCMAKE_BUILD_TYPE=Release
~~~


---


<br>


### How to run
+ Run `roscore`
~~~shell
$ roscore
~~~
+ Run `rviz` for debugging
~~~shell
$ roscd ieee_uav
$ rviz -d rviz.rviz
~~~
+ Run `rqt_plot` to see distance between rover and the UAV
~~~shell
$ rqt_plot

add /distance/data
~~~

+ Launch Gazebo world
~~~shell
$ roslaunch ieee_uav trial1.launch

or

$ roslaunch ieee_uav trial2.launch
~~~

+ Run `ieee_uav`
~~~shell
$ roslaunch ieee_uav main.launch
~~~

## Important: launching gazebo and running the main code should be done in a short second (ASAP).

<br>

### Expected result
+ UAV starts to track the rover
+ Paths and debugging images should be visualized in `rviz`
+ `YOLO` detection should work at least at 100Hz, if you setup `CUDA`, `cuDNN`, `OpenCV`, and `cv_bridge` properly
  + Note, the inference can work upto at 100Hz purely, but it works at fixed 15Hz actually, considering limited computational resource of the UAV.