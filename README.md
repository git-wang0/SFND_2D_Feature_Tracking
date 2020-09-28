# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

## Results
detectorType_list = {"SHITOMASI", "SIFT", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE"}

descriptorType_list = {"BRISK", "FREAK", "BRIEF", "AKAZE", "SIFT", "ORB"}

### MP.1 Ring Buffer
This is achieved by removing element in the beginning and pushing a new element. Code below:

                    if(dataBuffer.size()>=2)
                        dataBuffer.erase(dataBuffer.begin());

                    dataBuffer.push_back(frame);


### MP.2 Keypoint Detection
Implement different detectors.

                    if (detectorType.compare("SHITOMASI") == 0)
                    {
                        detKeypointsShiTomasi(keypoints, imgGray, false);
                    }
                    else if (detectorType.compare("HARRIS") == 0)
                    {
                        detKeypointsHarris(keypoints, imgGray, false);
                    }
                    else
                    {
                        detKeypointsModern(keypoints, imgGray, detectorType, false);
                    }

### MP.3 Keypoint Removal
Remove keypoints outside of a pre-defined rectangle.

                    if (bFocusOnVehicle)
                    {
                        vector<cv::KeyPoint> filteredKeypoints;
                        for (auto kp : keypoints) {
                            if (vehicleRect.contains(kp.pt)) filteredKeypoints.push_back(kp);
                        }
                        keypoints = filteredKeypoints;
                    }

### MP.2 Keypoint Detection

### MP.2 Keypoint Detection

### MP.2 Keypoint Detection

### MP.2 Keypoint Detection

### MP.2 Keypoint Detection

### MP.2 Keypoint Detection



## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.