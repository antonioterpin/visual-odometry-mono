# Visual odometry monocular pipeline

##### Table of Contents
1. [Introduction](#intro)
2. [Getting started](#getstarted)
3. [Features](#features)
4. [Performances](#performances)
5. [References](#references)
6. [Contributing](#contribute)
7. [License](#license)
8. [Acknowledgments](#acknowledgments)

<a name="intro"></a>
## Introduction
This project is the assembly of all of the concepts learnt during the (Vision Algorithms for Mobile Robotics)[http://rpg.ifi.uzh.ch/teaching.html] class at ETH Zürich. In particular, the building blocks developed during the exercise sessions have been improved and assembled to build a monocular visual odometry pipeline, with very satisfying results.

![Optimal estimate KITTI](https://github.com/antonioterpin/visual-odometry-mono/blob/develop/images/cover.jpg)


<a name="getstarted"><a/>
## Getting started
### Prerequisites
The code run and has been tested with MATLAB R2020b. The following licenses are required:

|Toolbox|
|:----- |
|image_toolbox|
|optimization_toolbox|
|statistics_toolbox|
|video_and_image_blockset|

The above set of toolbox has been determined with the `license('inuse')` command at the end of the simulation. The code was developed having access at all the toolboxes.

### Installing
1. Clone the repo. In the following it is supposed the repository has been cloned in `/users/aterpin/visual-odometry-mono`.
2. Download the dataset (at least one). You can find the tested one in the Dataset subsection of the Performance section.
..a. Remark: If you want to use your own dataset, you can extend the `InputBlock` class.
  b. We will further assume you unzipped the [KITTI dataset](http://rpg.ifi.uzh.ch/docs/teaching/2016/kitti00.zip) in a folder called `/users/aterpin/data/kitti`.
3. Update the configuration file.
  a. Suggested .json configuration files can be found in the repository [config](https://github.com/antonioterpin/visual-odometry-mono/tree/main/config) folder.
  b. In `main.m`, change the `configFile` variable to the desired .json configuration file.
    + For instance, `configFile = 'config/kitti/config.json'`
  c. Whichever configuration file are you using, you should update the path to the dataset.
```
{
  "InputBlock": {
    "Handler": "Kitti",
    "Path": "../data/kitti"      <- update this line to the relative (or absolute) path to your dataset
  },
  ...
```

### Running the simulation
Just run `main.m`.

<a name="features"><a/>
## Features
A wiki and thorough documentation for this project is currently under development. You can refer to the [project report](https://github.com/antonioterpin/visual-odometry-mono/blob/main/report.pdf) to have an detailed overview of the pipeline.

The pipeline implements the following additional features:
* the pipeline achieves a good global trajectory estimate;
* improvements to combat the scale drift are proposed and implemented;
* the estimated trajectory and the ground truth are quantitatively compared;
* a  continuously  integrated  bundle  adjustment  is  implemented and running on the pipeline; and
* the pipeline is a flexible platform that allows further experiments and improvements, in which is easy to integrate other blocks

<a name="performances"><a/>
## Performances
### Dataset
The pipeline has been tested with the following datasets. For each you can find a link to the official webpage (if any), a quick download link (KITTI and Malaga are quite heavy!) and a video to a recorded performance. Enjoy the run 🏎
| Dataset | Webpage | Download | Video |
|:----- |:-----|:-----|:-----|
| KITTI | [KITTI webpage](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) | [kitti00.zip](http://rpg.ifi.uzh.ch/docs/teaching/2016/kitti00.zip)| [Youtube video](https://www.youtube.com/watch?v=wylit5A9JqQ) |
| Malaga | [Malaga webpage](https://www.mrpt.org/MalagaUrbanDataset) | [malaga07.zip](http://rpg.ifi.uzh.ch/docs/teaching/2016/malaga-urban-dataset-extract-07.zip)| [Youtube video](https://www.youtube.com/watch?v=2y6KvV_5wEw) |
| Parking | | [parking.zip](http://rpg.ifi.uzh.ch/docs/teaching/2016/parking.zip) | [Youtube video](https://www.youtube.com/watch?v=-XjXqDtPnvQ) |
  
The videos are recorded with the following hardware specifications:
| | Acer Aspire E 15 |
|:----- |:-----|
| CPU | 2,7 GHz Intel Core i7-7500U |
| RAM | 12 GB DDR4 |
| OS | Ubuntu 18.04 |
| # threads | 1 |

### Quantitative evaluation
The estimated trajectory for the KITTI dataset (the most challenging one, and the one mostly tuned) has been quantitatively compared to the ground truth using the open source evaluation framework provided by the [Robot Perception Group](https://github.com/uzh-rpg/rpg_trajectory_evaluation).
  
<img src="https://github.com/antonioterpin/visual-odometry-mono/blob/main/images/trajectory_side_sim3_-1.png" width="30%"></img>

Rotation error | Relative yaw error
:-------------------------:|:-------------------------:
![Rotation error](https://github.com/antonioterpin/visual-odometry-mono/blob/main/images/rotation_error_sim3_-1.png)|![Relative yaw error](https://github.com/antonioterpin/visual-odometry-mono/blob/main/images/rel_yaw_error.png)
Translation error | Scale error
![Translation error](https://github.com/antonioterpin/visual-odometry-mono/blob/main/images/translation_error_sim3_-1.png) | ![Scale drift](https://github.com/antonioterpin/visual-odometry-mono/blob/main/images/scale_error_sim3_-1.png)
Relative translation error | Relative translation error (%)
![Relative translation error](https://github.com/antonioterpin/visual-odometry-mono/blob/main/images/rel_translation_error.png) | ![Relative translation error percentage](https://github.com/antonioterpin/visual-odometry-mono/blob/main/images/rel_translation_error_perc.png)

<a name="references"><a/>
## References
<a id="1">[1]</a> L. Kneip, D. Scaramuzza, and R. Siegwart. A novel parametrization ofthe perspective-three-point problem for a direct computation of absolutecamera position and orientation.  In CVPR 2011, pages 2969–2976, June 2011.

<a id="2">[2]</a> Bruce D. Lucas and Takeo Kanade. An iterative image registration technique with an application to stereo vision. In Proceedings of the 7th International Joint Conference on Artificial Intelligence - Volume2, IJCAI’81, page 674–679, San Francisco, CA, USA, 1981. Morgan Kaufmann Publishers Inc.

<a id="3">[3]</a> Simon Baker, Ralph Gross, and Iain Matthews. Lucas-kanade 20 years on: A unifying framework: Part 3. Int. J. Comput. Vis, 56, 12 2003.

<a id="4">[4]</a> R. I. Hartley. In defense of the eight-point algorithm. IEEE Transactionson Pattern Analysis and Machine Intelligence, 19(6):580–593, June 1997.

<a id="5">[5]</a> Martin A. Fischler and Robert C. Bolles. Random sample consensus: A paradigm for model fitting with applications to image analysis and automated cartography. Commun. ACM, 24(6):381–395, June 1981

<a id="6">[6]</a> D. Nister. An efficient solution to the five-point relative pose problem. IEEE Transactions on Pattern Analysis and Machine Intelligence, 26(6):756–770, June 2004

<a id="7">[7]</a> Heng Yang, Pasquale Antonante, Vasileios Tzoumas, and Luca Carlone. Graduated non-convexity for robust spatial perception: From non-minimal solvers to global outlier rejection. IEEE Robotics and Automation Letters, 5(2):1127–1134, Apr 2020.

<a id="8">[8]</a> Y.I Abdel-Aziz, H.M. Karara, and Michael Hauck. Direct linear transformation from comparator coordinates into object space coordinates in close-range photogrammetry. Photogrammetric Engineering Remote Sensing, 81(2):103–107, 2015.

<a id="9">[9]</a> Zichao Zhang and Davide Scaramuzza. A tutorial on quantitativetrajectory evaluation for visual(-inertial) odometry. In IEEE/RSJ Int. Conf. Intell. Robot. Syst. (IROS), 2018.

<a name="contribute"><a/>
## Contributing
At the moment it is not possible to contribute to the project, since it has to be submitted as our original work. Afterwards we will make it possible in a easy way. Save your coffees until then ☕️.

<a name="contributors"><a/>
### Contributors
#### Authors
+ [Antonio Terpin](mailto:aterpin@ethz.ch) - MSc Robotics, Systems and Control ETHz (Zürich), BSc Electronic Engineering École normale di Udine (Italy)
+ [Antonio Arbues](mailto:aarbues@ethz.ch) - MSc Robotics, Systems and Control ETHz (Zürich), BSc Mechanical Engineering Politecnico di Milano (Italy)

<a name="license"><a/>
## License
This project is licensed under the MIT License - see the [LICENSE](https://github.com/antonioterpin/visual-odometry-mono/blob/main/LICENSE) file for details.

<a name="acknowledgments"><a/>
## Acknowledgments
This project has been initially developed within the Vision Algorithms for Mobile Robotics 2020 class at ETHz, by [Antonio Terpin](mailto:aterpin@ethz.ch) and [Antonio Arbues](mailto:aarbues@ethz.ch). A full list of the contributors can be found at the [contributors](#contributors) section. The authors wants to thank Daniel Gehrig (Robot Perception Group) for the insightful hints and comments during the development of this project.
