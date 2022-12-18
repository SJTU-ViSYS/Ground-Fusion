# Ground-Fusion 

codes and datasets will be made public upon paper acceptance!

## Project Author: [Jie Yin](https://github.com/sjtuyinjie?tab=repositories) at 1195391308@qq.com


## ABSTRACT:

We propose Ground-Fusion: a sensor-fusion SLAM system that features adaptive sensor selection and robust tracking with high accuracy in corner cases including sensor failures. Firstly, we implement a baseline system that first ever tightly couples the sensory information of RGB-D images, inertial information and wheel odometer measurements within a factor graph. Secondly, we further explore the merits of each sensor. More specifically, we adopt an adaptive sensor selection strategy in the initialization and tracking phase for diverse corner cases like severe visual occlusion, wheel drift or even wheel suspension. Furthermore, we collect a large-scale outdoor SLAM dataset with a full sensor suite as a novel benchmark, which challenges current cutting-edge SLAM systems. Finally, extensive experiments on public and self-collected datasets demonstrate that our system shows competitive performance in both accuracy and robustness comparable to LiDAR SLAM. For the benefit of the research community, the codes and datasets will be released at this website upon paper publication.

## MAIN CONTRIBUTIONS :

* We construct the first-ever optimization-based SLAM framework which tightly couples RGB-D images, inertial measurements and wheel odometer. Besides being capable of real-time tracking and dense mapping, the system supports line feature tracking and dynamic object detection as well.

 * An adaptive sensor selection strategy is adopted to strengthen the robustness and accuracy of the system in various corner cases. 

 * We collect a novel multi-sensor SLAM dataset with rich sensory measurements in diverse scenarios, which will be made public upon paper publication.


* Extensive experiments are conducted on public and self-captured datasets, whose results show that our system remarkably outperforms baseline methods in accuracy and robustness.
