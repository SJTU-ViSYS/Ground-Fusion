# Ground-Fusion: A Low-cost Ground SLAM System Robust to Corner Cases (ICRA2024)
## Project Author: [Jie Yin](https://github.com/sjtuyinjie?tab=repositories) at (1195391308@qq.com)

## news
1. 2024.1.29 Accepted by ICRA2024. I have already released the datasets, and will release the code when I attend the ICRA in May,2024.
2. ## Notice! I am currently looking for an experienced developer to collaborate on the next version of Ground-Fusion. My goal is to create an affordable yet highly robust and precise SLAM system suitable for deployment on practical ground-based robots and vehicles. This effort involves addressing several engineering issues, including transitioning to ROS2, improving map fidelity, optimizing computational performance, and more. Engaging in this project not only promises to refine one's coding abilities but also guarantees a very substantial and practical project engagement. Should you be an experienced and enthusiastic SLAM developer, feel free to contact 195391308@qq.com. I will proactively share the source code with you and provide mentorship throughout the collaboration journey.##
   
注意！本人正在寻找有经验的开发者合作开发Ground-Fusion的下一个版本。本人的愿景是开发一个低成本的、尽可能鲁棒和精确的开源SLAM系统以便部署到现实的ground robots/vehicles上。经过本人的丰富测试，发现Ground-Fusion的定位性能已经非常不错，但在实现上还有许多值得进一步优化和改进的地方。并且这需要解决很多工程上的问题，例如迁移到ROS2、提高地图的质量、优化计算效率等等。相信在这个过程中不仅可以锻炼自己的coding能力，也会让你有一个非常扎实的项目经历。如果你也是个经验丰富、充满热情的开源开发者，欢迎联系195391308@qq.com。我会提前给你源码并提供指导~ 

## Introduction

We introduce Ground-Fusion, a low-cost sensor fusion simultaneous localization and mapping (SLAM) system for ground vehicles. Our system features efficient initialization, effective sensor anomaly detection and handling, real-time dense color mapping, and robust localization in diverse environments. We tightly integrate RGB-D images, inertial measurements, wheel odometer and GNSS signals within a factor graph to achieve accurate and reliable localization both indoors and outdoors. To ensure successful initialization, we propose an efficient strategy that comprises three different methods: stationary, visual, and dynamic, tailored to handle diverse cases. Furthermore, we develop mechanisms to detect sensor anomalies and degradation, handling them adeptly to maintain system accuracy. 

The preprint version of paper is [arxiv](http://arxiv.org/abs/2402.14308).
The dataset is at [https://github.com/sjtuyinjie/M2DGR-plus](https://github.com/SJTU-ViSYS/M2DGR-plus) and [https://github.com/SJTU-ViSYS/M2DGR](https://github.com/SJTU-ViSYS/M2DGR).
And the homepage is at [https://sites.google.com/view/ground-fusion/home](https://sites.google.com/view/ground-fusion/home).

<div align=center>
<img src="./fig/challenges.jpg" width="300px">

</div>
<p align="center">Figure 1. We categorize corner cases into three types: visual,
wheel, and GNSS challenges.</p>

## 1. Prerequisites and Installation
### 1.1 Ubuntu and ROS
Tested on Ubuntu 18.04 (with ROS Melodic and OpenCV3) and on Ubuntu 20.04(with ROS Noetic and OpenCV4).

### 1.2 OpenCV
This package requires [OpenCV 3/4](https://github.com/opencv/opencv) and some features of C++11. 

### 1.3 Eigen, Ceres, and PCL
This package requires [Eigen 3.3.7](), [Ceres 1.14](https://ceres-solver.googlesource.com/ceres-solver),[Sophus](https://github.com/strasdat/Sophus.git ) and [PCL 1.11](https://github.com/PointCloudLibrary/pcl).

### 1.4 Gnss_comm
This package also requires [gnss_comm](https://github.com/HKUST-Aerial-Robotics/gnss_comm) for ROS message definitions and some utility functions.

### 1.5 Build
~~~
mkdir -p ~/Groundfusion_ws/src
cd ~/Groundfusion_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/gnss_comm
git clone https://github.com/SJTU-ViSYS/Ground-Fusion
cd Ground-Fusion/thirdparty


cd ../..
catkin_make -j12
~~~

## 2. Run examples


### 2.1 Ground-challenge dataset
download at [Ground-challenge](https://github.com/sjtuyinjie/Ground-Challenge) and give a kind star.

~~~
# [launch] open a terminal and type:
source devel/setup.bash
roslaunch vins groundfusion.launch\

# [run localization] open another terminal:
source devel/setup.bash
rosrun vins vins_node src/Ground-Fusion/config/realsense/groundchallenge.yaml

# [dense map]open third terminal:
source devel/setup.bash
rosrun dense_map dense_map_node src/Ground-Fusion/config/realsense/groundchallenge.yaml
~~~


### 2.2 M2DGR-P dataset
download at [M2DGR-P](https://github.com/sjtuyinjie/M2DGR-plus) and give a kind star.


~~~
# [launch] open a terminal and type:
source devel/setup.bash
roslaunch vins groundfusion.launch\

# [run localization] open another terminal:
source devel/setup.bash
rosrun vins vins_node src/Ground-Fusion/config/realsense/m2dgrp.yaml

# [dense map]open third terminal:
source devel/setup.bash
rosrun dense_map dense_map_node src/Ground-Fusion/config/realsense/m2dgrp.yaml
~~~



## 3. Parameter configuration

## 4. Acknowledgement
Thanks support from National Key R&D Program (2022YFB3903802), NSFC(62073214), and Midea Group. This project is based on [GVINS](https://github.com/HKUST-Aerial-Robotics/GVINS), and has borrowed some codes from open-source projects [VIW-Fusion](https://github.com/TouchDeeper/VIW-Fusion) and [VINS-RGBD](https://github.com/STAR-Center/VINS-RGBD), thanks for your great contribution!

## 5. License
The source code of Ground-Fusion is released under GPLv3 license. Do not use this project for any commercial purpose unless permitted by authors. Yin Jie is still working on improving the system. For any technical issues, please contact him at <1195391308@qq.com>.

If you use this work in an academic work, please cite:
~~~
@article{yin2021m2dgr,
  title={M2dgr: A multi-sensor and multi-scenario slam dataset for ground robots},
  author={Yin, Jie and Li, Ang and Li, Tao and Yu, Wenxian and Zou, Danping},
  journal={IEEE Robotics and Automation Letters},
  volume={7},
  number={2},
  pages={2266--2273},
  year={2021},
  publisher={IEEE}
}
@article{yin2024ground,
  title={Ground-Fusion: A Low-cost Ground SLAM System Robust to Corner Cases},
  author={Yin, Jie and Li, Ang and Xi, Wei and Yu, Wenxian and Zou, Danping},
  journal={arXiv preprint arXiv:2402.14308},
  year={2024}
}
~~~



