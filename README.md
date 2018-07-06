# aruco_positioning_system

[ROS](http://ros.org) package

Aruco mapping package, see [Wiki](http://wiki.ros.org/aruco_mapping) page for more details:

* Authors: [Jan Bacik] (http://www.smartroboticsys.eu/?page_id=895&lang=en), [Smart Robotic Systems] (http://www.smartroboticsys.eu)

<a href="http://www.youtube.com/watch?feature=player_embedded&v=MlOy9qt_K4Y
" target="_blank"><img src="http://img.youtube.com/vi/MlOy9qt_K4Y/0.jpg" 
alt="Aruco mapping" width="480" height="360" border="10" /></a>

# Install

1. Download aruco-2.0.19.zip from https://sourceforge.net/projects/aruco/files/2.0.19/aruco-2.0.19.zip/download
2. Build ArUco and install it:
```bash
$ unzip aruco-2.0.19.zip
$ cd aruco-2.0.19
$ mkdir build
$ cd build
$ cmake ..
$ make -j4
$ sudo make install
$ sudo ldconfig
```
3. Clone this repository into the src folder of your catkin workspace and run catkin_make
