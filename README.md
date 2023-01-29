# Ground-Fusion: A Low-cost SLAM System Robust to Corner Cases for Ground Vehicles

## Project Author: [Jie Yin](https://github.com/sjtuyinjie?tab=repositories) 

## Notice: 
### 1.Codes and datasets will be made public upon paper acceptance
### 2.We aim to build a low-cost multi-sensor fusion SLAM baseline system with a corresponding benchmark dataset. If there are any improving suggestions, we welcome the cooperation to further develop new features to ensure that the algorithm can be deployed in real-world applications. We hope our system can go towards an ultimate solution of SLAM problem.
### 3.For any commercial or academic communications, feel free to contact Jie Yin at 1195391308@qq.com 




## ABSTRACT:

We present Ground-Fusion: a low-cost SLAM system for ground vehicles featuring adaptive sensor selection, real-time dense color mapping, and robust localization with high accuracy. Our system tightly integrates low-cost sensory data including RGB-D images, inertial measurements, and wheel odometer within a factor graph, and adopts an adaptive sensor selection strategy for diverse corner cases such as severe visual occlusion, wheel drift, and even wheel suspension. Experimental results on both public and self-collected datasets show that our system outperforms exiting low-cost SLAM systems and achieves comparable performance to LiDAR SLAM systems. The code and datasets will be publicly released upon paper publication.

## MAIN CONTRIBUTIONS :



* We develop an optimization-based SLAM framework that tightly integrates RGB-D images, inertial measurements and wheel odometer. The system supports not only real-time localization and dense color mapping, but also line feature tracking and dynamic object detection.

* An adaptive sensor selection strategy is adopted to strengthen the robustness and accuracy of the system in various corner cases. 

* We collect a novel multi-sensor SLAM dataset with rich sensory measurements in diverse scenarios, which will be made publicly available on [https://github.com/sjtuyinjie/M2DGR-plus](https://github.com/sjtuyinjie/M2DGR-plus).


