# GTOP:Gradient-Based Trajectory Optimizer
(This repo is mainly developed and maintained by [Boyu Zhou](https://boyuzhou.net), please contace him if necessary)
## 1.Introduction

 Gradient-Based Online Safe Trajectory Generation is trajectory optimization framework, for generating a
safe, smooth and dynamically feasible trajectory based on the piecewise line segment initial path. The planning 
problem is formulating as minimizing the penalty of collision cost,
smoothness and dynamical feasibility.

**Authors:**[Fei Gao](https://ustfei.com/),[Boyu Zhou](https://boyuzhou.net),and [Shaojie Shen](http://www.ece.ust.hk/ece.php/profile/facultydetail/eeshaojie) from the [HUKST Aerial Robotics Group](uav.ust.hk).

**Disclaimer**

This is research code, any fitness for a particular purpose is disclaimed.

**Related Paper**
* **Gradient-Based Online Safe Trajectory Generation
for Quadrotor Flight in Complex Environments,** Fei Gao, Yi Lin and Shaojie Shen

Video of this paper can be found [here](http://www.bilibili.com/video/av16979476/).

&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;[![](https://github.com/HKUST-Aerial-Robotics/grad_traj_optimization/blob/master/pic/video.png)](http://www.bilibili.com/video/av16979476/)


If you use this generator for your academic research, please cite our related paper.
```
@inproceedings{Fei2017IROS,
	Address = {Vancouver, Canada},
	Author = {F. Gao and W.Wu and Y. Lin and S. Shen},
	Booktitle = {Gradient-Based Online Safe Trajectory Generation
for Quadrotor Flight in Complex Environments},
	Title = {Proc. of the {IEEE/RSJ} Intl. Conf. on Intell. Robots and Syst.},
	Month = Sept.,
	Year = {2017}}
}
```
## 2.Prerequisities
  Our testing environment: **Ubuntu** 14.04, **ROS** Indigo.

  We use **NLopt** as optimization solver. Installation is straight forward. Just download, extract and compile:
  ```
  mkdir build
  cd build
  cmake ..
  make
  ```

  Finally you should install it.
  ```
  sudo make install
  ```

  Detailed information can be found [here](https://nlopt.readthedocs.io/en/latest/).

  *Note:The default installation prefix of **NLopt** is /usr/local*


## 3.Build on ROS
  Clone the repository to your catkin workspace and catkin_make. For example:
```
  cd ~/catkin_ws/src
  git clone https://github.com/HKUST-Aerial-Robotics/grad_traj_optimization.git
  cd ../
  catkin_make
  source ~/catkin_ws/devel/setup.bash
```


## 4.Random Map and Waypoints Example
Open two terminals and run:
```
  roslaunch grad_traj_optimization traj_rviz.launch
  roslaunch grad_traj_optimization random.launch
```

  After running and open *rviz* with traj.rviz file, you should find a randomly built collision map with some waypoints going through it.
  Then a smooth and collision free trajectory is generated. 
  
  <div align=center>
  <img src="https://github.com/HKUST-Aerial-Robotics/grad_traj_optimization/blob/master/pic/random.gif" width = "360" height = "360">
  </div>
  
## 5.Random Map and Clicked Waypoints Example
Similarly, run:
```
  roslaunch grad_traj_optimization traj_rviz.launch
  roslaunch grad_traj_optimization click.launch
```

  Likewise, a random collision map is built but with fewer obstacles. Then you can click in *rviz* using *2D Nav Goal* to add 
  some waypoints. The *Z* coordinate of each waypoint is set to *2.0*.  The default waypoint number is 7 and you can change it in *click.launch*. Trajectory is 
  generated as long as enough waypoints are added. 

  <div align=center>
  <img src="https://github.com/HKUST-Aerial-Robotics/grad_traj_optimization/blob/master/pic/click.gif" width = "360" height = "360">
  </div>

  *Note:Trajectory with too many segments or with sharp corner is difficult to optimized and is tend to fail.*

## 6.Text Input Example
If you want to set the collision map and waypoints as much as you like, run
```
  roslaunch grad_traj_optimization traj_rviz.launch
  roslaunch grad_traj_optimization text_input.launch
```

Instead of randomly generated ,the collision map and waypoints in this example is specified in *text_input.launch*.Just change it to what you want.

  <div align=center>
  <img src="https://github.com/HKUST-Aerial-Robotics/grad_traj_optimization/blob/master/pic/text1.png" width = "360" height = "360">
  </div>

  <div align=center>
  <img src="https://github.com/HKUST-Aerial-Robotics/grad_traj_optimization/blob/master/pic/text2.png" width = "360" height = "360">
  </div>


## 7.Acknowledgements
  We use **NLopt** for non-linear optimization and [sdf_tools](https://github.com/UM-ARM-Lab/sdf_tools) for building signed distance field.

## 8.Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.


