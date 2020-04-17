# SFND 3D Object Tracking
## Classroom Report
This repository represents my submission for the final project of the Camera module. This module involves the development of a TTC (time-to-collision) estimator which utilizes both monocular camera images and lidar data.

Udacity's Sensor-fusion engineer nanodegree program.

## FP.1 Match 3D Objects
The matching of 3D objects is done in the function `matchBoundingBoxes()` in `camFusion_Student.cpp`

## FP.2 Compute Lidar-based TTC
The lidar-based computation of the TTC is done in the function `computeTTCLidar()`. 
A for loop calculates the minimum y-coordinates of the point clouds each for the current and previous image, respectively.
In order to deal with the outlier problem, we assume the following model: We assume that the rear end of the preceding vehicle is a straight line of gradient zero, i.e. $y=const.$
Then we can calculate the normal distribution of all point cloud elements in y-direction.

$$\mathcal{N}(y;\bar{y},\sigma) \sim \exp\left( -\frac{(y-\bar{y})^2}{\sqrt{2\pi} \sigma}\right) $$

The unbiased estimate for the distribution is

$$ \sigma = \sqrt{\frac{1}{n-1} \sum_{i=0}^n (y_i-\bar{y})^2} $$

where $\bar{y}$ is the mean value of the sample. We say that the minimum y-points are outliers when they are smaller than $-2\sigma+\bar{y}$.

Example: 


The technical implementation is done as follows:



## FP.3 Associate Keypoint Correspondence with Bounding Boxes
This procedure is done in the function `matchBoundingBoxes()`. Its central idea revolves around defining a matrix with dimensions xxx. Since we are not importing the Eigen library for matrix compuations, we utilize an occupancy matrix, `cv::Mat`, which is just appropriate enough for our uses. It is initialized to zero, and for each pairing, we increment the th entry by one. 
Finally, in a second loop, we determine the maximum of each row (equivalently column). The index of the corresponding max (i.e. argmax) is considered the best match and inserted into `bbBestMatch`.



## FP.4 Compute Camera-based TTC
The camera-based computation of TTC ihas been implemented in the function `computeTTCCamera()`. 
The statistical robustness of our procedure is guaranteed by the median distance ratio.



## FP.5 Performance Evaluation 1
Examples where the TTC estimate of Lidar sensor is not plausible.

Estimating the distance to the preceding car is error prone, as the lens of the camera is wide-angular, and 
However, assuming that the car on the left-hand lane is a VW Golf Stationwagon or similar, it's length according to specification is about 4.5 meters. Further assuming that its rear end is one half of the car's length ahead of us, and its front end is about the same distance as the preceding target vehicle's rear end, we can estimate the distance to the preceding car as

    4.5m + 0.5 * 4.5m = 7.75m

Hence, we estimate the distance to the rear end of the preceding car at the beginning of the scene to about 7-8 meters.



## FP.6 Performance Evaluation 2
Detector / Descriptor combinations.

**Keypoint detectors:** SIFT, (Harris), (Shi-Tomasi), FAST, BRISK, ORB, and AKAZE, 

**Keypoint descriptors:** SIFT, BRISK, BRIEF, ORB, FREAK, AKAZE

The exact results are contained in the [Results.ods](Results.ods) file (OpenOffice Calc).


|              | SIFT| BRISK| BRIEF| ORB | FREAK|AKAZE|
|:------------:|:---:|:----:|:----:|:---:|:----:|:---:|
| SIFT         |     |      |      |  -  |      |  -  |
| (Harris)     |     |      |      |     |      |  -  |
| (Shi-Tomasi) |     |      |      |     |      |  -  |
| FAST         |     |      |      |     |      |  -  |
| BRISK        |     |      |      |     |      |  -  |
| ORB          |     |      |      |     |      |  -  |
| AKAZE        |  -  |  -   |   -  |  -  |   -  |     |

One source of error seems to arise from the small denominator.



## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.