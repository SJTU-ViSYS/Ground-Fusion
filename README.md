# [ICRA2024]Ground-Fusion: A Low-cost Ground SLAM System Robust to Corner Cases 

<div align="center">

First Author: [**Jie Yin 殷杰**](https://sjtuyinjie.github.io/)
&emsp;
📝 [[Paper]](https://ieeexplore.ieee.org/document/10610070) / [[Arxiv]](https://arxiv.org/abs/2402.14308)
&emsp;
➡️ [[Dataset]](https://github.com/SJTU-ViSYS/M2DGR-plus)
&emsp;
⭐️ [[Presentation Video]](https://www.bilibili.com/video/BV1xx421m75k/?spm_id_from=333.337.search-card.all.click&vd_source=0804300aea4065df90adde5398ee74b7)
&emsp;
🔥[[News]](https://mp.weixin.qq.com/s/CfnfxHvn9pbYc4599_JoSg)

[![Author](https://img.shields.io/badge/Author-Jie%20Yin-blue)](https://sjtuyinjie.github.io/)
[![Paper](https://img.shields.io/badge/Paper-GroundFusion-yellow)](https://ieeexplore.ieee.org/document/10610070)
[![Preprint](https://img.shields.io/badge/Ppreprint-Arxiv-purple)](https://arxiv.org/abs/2112.13659/)
[![Dataset](https://img.shields.io/badge/Dataset-M2DGR+%2B-green)](https://github.com/SJTU-ViSYS/M2DGR-plus)
[![License](https://img.shields.io/badge/License-GPLv3-cyan)]()
[![Video](https://img.shields.io/badge/Video-red)](https://www.bilibili.com/video/BV1xx421m75k/?spm_id_from=333.337.search-card.all.click&vd_source=0804300aea4065df90adde5398ee74b7)
[![News](https://img.shields.io/badge/News-orange)](https://mp.weixin.qq.com/s/CfnfxHvn9pbYc4599_JoSg)

[![stars](https://img.shields.io/github/stars/SJTU-ViSYS/Ground-Fusion.svg)](https://github.com/SJTU-ViSYS/Ground-Fusion)
[![forks](https://img.shields.io/github/forks/SJTU-ViSYS/Ground-Fusion.svg)](https://github.com/SJTU-ViSYS/Ground-Fusion)
[![open issues](https://img.shields.io/github/issues-raw/SJTU-ViSYS/Ground-Fusion)](https://github.com/SJTU-ViSYS/Ground-Fusion/issues)
[![issue resolution](https://img.shields.io/github/issues-closed-raw/SJTU-ViSYS/Ground-Fusion)](https://github.com/SJTU-ViSYS/Ground-Fusion/issues)

</div>

[![ICRA2024 Presentation](cover.jpg)](https://www.bilibili.com/video/BV1xx421m75k/?spm_id_from=333.337.search-card.all.click&vd_source=0804300aea4065df90adde5398ee74b7)



> [!TIP]
> Check out the [**presentation video above**](https://www.bilibili.com/video/BV1xx421m75k/?spm_id_from=333.337.search-card.all.click&vd_source=0804300aea4065df90adde5398ee74b7) for a quick overview of this work!

## News 
- **🔥`2025/06/16`**: Introducing our new paper on IROS2025:"Towards Robust Sensor-Fusion Ground SLAM: A Comprehensive Benchmark and A Resilient Framework" is accepted to IROS2025!code: **[Ground-Fusion++](https://github.com/sjtuyinjie/Ground-Fusion2)**, dataset: **[M3DGR](https://github.com/sjtuyinjie/M3DGR)**! 


## NOTICE
### We warmly welcome and highly recommend the integration of the Ground-Fusion system into your projects for several compelling reasons:

1. **🔥Comprehensive Sensor Suite**: The Ground-Fusion system is equipped with a multitude of sensors (RGBD-IMU-Wheel-GNSS), facilitating an easy onset for enhancements to any module. This richness in sensory input streamlines the process of adapting and refining components within the system.

2. **⭐️Open-Source Ecosystem**: Both the Ground-Fusion algorithm and associated datasets such as [**M2DGR-plus**](https://github.com/SJTU-ViSYS/M2DGR-plus) and the [**Ground-Challenge**](https://github.com/sjtuyinjie/Ground-Challenge) are openly available, forming a comprehensive bechmark suite. **Welcome to beat Ground-Fusion on M2DGR and Ground-Challenge!**

3. **🚀Proven Performance**: The Ground-Fusion algorithm has been rigorously validated across diverse datasets, establishing itself as SOTA in lidar-less SLAM algorithms. Outperforming Ground-Fusion on these benchmarks would significantly bolster the credibility of your proposed method.




## Introduction



We introduce Ground-Fusion, a low-cost sensor fusion simultaneous localization and mapping (SLAM) system for ground vehicles. Our system features efficient initialization, effective sensor anomaly detection and handling, real-time dense color mapping, and robust localization in diverse environments. We tightly integrate RGB-D images, inertial measurements, wheel odometer and GNSS signals within a factor graph to achieve accurate and reliable localization both indoors and outdoors. To ensure successful initialization, we propose an efficient strategy that comprises three different methods: stationary, visual, and dynamic, tailored to handle diverse cases. Furthermore, we develop mechanisms to detect sensor anomalies and degradation, handling them adeptly to maintain system accuracy. 

The preprint version of paper is [here](http://arxiv.org/abs/2402.14308).
The dataset is at [**M2DGR-plus**](https://github.com/SJTU-ViSYS/M2DGR-plus), [**Ground-Challenge**](https://github.com/sjtuyinjie/Ground-Challenge) and [**M2DGR**](https://github.com/SJTU-ViSYS/M2DGR).


<div align=center>
<img src="./fig/challenges.jpg" width="300px">

</div>
<p align="center">Figure 1. We categorize corner cases into three types: visual,
wheel, and GNSS challenges.</p>

## ⚙️ 1. Prerequisites & Installation

### 1.1 OS & ROS Support  
✅ Ubuntu 18.04 + ROS Melodic + OpenCV3  
✅ Ubuntu 20.04 + ROS Noetic + OpenCV4


### 1.2 OpenCV
This package requires [OpenCV 3/4](https://github.com/opencv/opencv) and some features of C++11. 

### 1.3 Eigen, Ceres, and PCL
This package requires [Eigen 3.3.7](https://github.com/PX4/eigen), [Ceres 1.14](https://ceres-solver.googlesource.com/ceres-solver),[Sophus](https://github.com/strasdat/Sophus.git ) and [PCL 1.10 or 1.11](https://github.com/PointCloudLibrary/pcl).
You need to download they in your thirdparty folder, and then:
~~~
sudo apt-get update
sudo apt-get install -y cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev 
cd thirdparty/eigen
mkdir -p build && cd build
cmake ..
sudo make install
cd ../../ceres-solver
mkdir -p build && cd build
cmake ..
make -j$(nproc) 
sudo make install
sudo apt-get install -y libflann-dev libvtk6-dev libboost-all-dev ros-noetic-pcl-ros (for ubuntu20.04) libfmt-dev
cd ../../pcl
mkdir -p build && cd build
cmake ..
make -j$(nproc)
sudo make install
cd ../../Sophus
mkdir -p build && cd build
cmake ..
make -j$(nproc) 
sudo make install
~~~


### 1.4 Gnss_comm
This package also requires [gnss_comm](https://github.com/HKUST-Aerial-Robotics/gnss_comm) for ROS message definitions and some utility functions.

### 1.5 Configure gcc (For Ubuntu 20.04)
~~~
sudo apt-get install g++-8
sudo apt-get install gcc-8
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 20
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 20
~~~

### 1.6 Build Ground-Fusion
After install all 3rd parties：
~~~
mkdir -p ~/Groundfusion_ws/src
cd ~/Groundfusion_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/gnss_comm
git clone https://github.com/SJTU-ViSYS/Ground-Fusion
cd ../..
catkin_make -j12
~~~

> 💡 Tip: If you encounter `Sophus` issues, try building both template and non-template versions.





## 🚀 2. Run Examples


### 2.1 Ground-challenge dataset
Download [Ground-challenge](https://github.com/sjtuyinjie/Ground-Challenge) dataset and give a star.

~~~
# [launch] open a terminal and type:
source devel/setup.bash
roslaunch vins groundfusion.launch

# [run localization] open another terminal:
source devel/setup.bash
rosrun vins vins_node src/Ground-Fusion/config/realsense/groundchallenge.yaml

# [dense map]open third terminal:
source devel/setup.bash
rosrun dense_map dense_map_node src/Ground-Fusion/config/realsense/groundchallenge.yaml

# [play rosbag]open forth terminal:
rosbag play office3.bag
~~~

> ⚠️ Mapping node is resource-intensive, affecting the real-time performance of the entire system. So it's suggested that do not run this node unless necessary. We are working on optimizing the mapping node currently




### 2.2 M2DGR-Plus dataset
Download [M2DGR-Plus](https://github.com/sjtuyinjie/M2DGR-plus) dataset and give a star.


~~~
# [launch] open a terminal and type:
source devel/setup.bash
roslaunch vins groundfusion.launch

# [run localization] open another terminal:
source devel/setup.bash
rosrun vins vins_node src/Ground-Fusion/config/realsense/m2dgrp.yaml

# [dense map]open third terminal:
source devel/setup.bash
rosrun dense_map dense_map_node src/Ground-Fusion/config/realsense/m2dgrp.yaml

# [play rosbag]open forth terminal:
rosbag play anamoly.bag


~~~


> ⚠️ **Known Issues**:  
> - Ground-Fusion may perform better *without GNSS* on M2DGR-Plus due to its low (1Hz) GNSS frequency.  
> - Initialization thresholds may need tuning to avoid early-phase drift.  
> - A more advanced version of Ground-Fusion is under development — stay tuned!

---

## 🙏 3.Acknowledgements

This work is supported by:

- National Key R&D Program (2022YFB3903802)  
- NSFC (62073214)  
- Midea Group

> This project is built on [GVINS](https://github.com/HKUST-Aerial-Robotics/GVINS), with inspirations from [VIW-Fusion](https://github.com/TouchDeeper/VIW-Fusion) and [VINS-RGBD](https://github.com/STAR-Center/VINS-RGBD) — huge thanks!

---





## 📄 4. Citation

```bibtex
@inproceedings{yin2024ground,
  title={Ground-fusion: A low-cost ground slam system robust to corner cases},
  author={Yin, Jie and Li, Ang and Xi, Wei and Yu, Wenxian and Zou, Danping},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={8603--8609},
  year={2024},
  organization={IEEE}
}
@article{zhang2025towards,
  title={Towards Robust Sensor-Fusion Ground SLAM: A Comprehensive Benchmark and A Resilient Framework},
  author={Zhang, Deteng and Zhang, Junjie and Sun, Yan and Li, Tao and Yin, Hao and Xie, Hongzhao and Yin, Jie},
  journal={arXiv preprint arXiv:2507.08364},
  year={2025}
}
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
@inproceedings{yin2023ground,
  title={Ground-challenge: A multi-sensor slam dataset focusing on corner cases for ground robots},
  author={Yin, Jie and Yin, Hao and Liang, Conghui and Jiang, Haitao and Zhang, Zhengyou},
  booktitle={2023 IEEE International Conference on Robotics and Biomimetics (ROBIO)},
  pages={1--5},
  year={2023},
  organization={IEEE}
}
```

## ⭐️ 5. Star History

[![Star History Chart](https://api.star-history.com/svg?repos=SJTU-ViSYS/Ground-Fusion&type=Timeline)](https://star-history.com/#Ashutosh00710/github-readme-activity-graph&Timeline)


Thanks to everyone for star this project.

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://reporoster.com/stars/dark/SJTU-ViSYS/Ground-Fusion" />
  <source media="(prefers-color-scheme: light)" srcset="https://reporoster.com/stars/SJTU-ViSYS/Ground-Fusion" />
  <img alt="github-stargazers" src="https://github.com/SJTU-ViSYS/Ground-Fusion/stargazers" />
</picture>

<p align="right"><a href="#top">🔝Back to top</a></p>
