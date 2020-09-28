# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

## Results
detectorType_list = {"SHITOMASI", "SIFT", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE"}

descriptorType_list = {"BRISK", "FREAK", "BRIEF", "AKAZE", "SIFT", "ORB"}

### MP.1 Data Buffer Optimization
This is achieved by removing element in the beginning and pushing a new element. Code below:

                    if(dataBuffer.size()>=2)
                        dataBuffer.erase(dataBuffer.begin());

                    dataBuffer.push_back(frame);


### MP.2 Keypoint Detection
Implement different detectors.

Code in main function:

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

Code in matching2D_student.cpp:

    if (detectorType.compare("FAST") == 0) {
        auto fast = cv::FastFeatureDetector::create();
        // double t = (double)cv::getTickCount();
        fast->detect(img, keypoints);
    }
    else if (detectorType.compare("BRISK") == 0) {
        auto brisk = cv::BRISK::create();
        // double t = (double)cv::getTickCount();
        brisk->detect(img, keypoints);
    }
    else if (detectorType.compare("ORB") == 0) {
        auto orb = cv::ORB::create();
        // double t = (double)cv::getTickCount();
        orb->detect(img, keypoints);
    }
    else if (detectorType.compare("AKAZE") == 0) {
        auto akaze = cv::AKAZE::create();
        // double t = (double)cv::getTickCount();
        akaze->detect(img, keypoints);
    }
    else if (detectorType.compare("SIFT") == 0) {
        auto sift = cv::xfeatures2d::SIFT::create();
        sift->detect(img, keypoints);
    }
    else {
        // Specified detectorType is unsupported
        throw invalid_argument(detectorType + " is not a valid detectorType");
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

### MP.4 Keypoint Descriptor
Implement descriptors in matching2D_student.cpp:

    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("BRIEF") == 0)
    {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if (descriptorType.compare("ORB") == 0)
    {
        extractor = cv::ORB::create();
    }
    else if (descriptorType.compare("FREAK") == 0)
    {
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if (descriptorType.compare("AKAZE") == 0)
    {
        extractor = cv::AKAZE::create();
    }
    else if (descriptorType.compare("SIFT") == 0)
    {
        extractor = cv::xfeatures2d::SIFT::create();
    }
    else
    {
        throw invalid_argument(descriptorType + " is not a valid descriptorType");
    }


### MP.5 Descriptor Matching & MP.6 Descriptor Distance Ratio
Implement FLANN matching:

    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        if (descriptorType.compare("DES_HOG") == 0)
        {
            matcher = cv::FlannBasedMatcher::create();
        }

        // with all other binary descriptorTypes
        else if (descriptorType.compare("DES_BINARY") == 0)
        {
            const cv::Ptr<cv::flann::IndexParams>& indexParams = cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2);
            matcher = cv::makePtr<cv::FlannBasedMatcher>(indexParams);
        }

        else {
            throw invalid_argument(descriptorType + " is not a valid descriptorCategory");
        }

Implement KNN selection (with distance ratio test):

    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

        int k = 2;
        vector<vector<cv::DMatch>> knn_matches;
        matcher->knnMatch(descSource, descRef, knn_matches, k);
        
        // Filter matches using descriptor distance ratio test
        double minDescDistRatio = 0.8;
        for (auto it : knn_matches) {
            // The returned knn_matches vector contains some nested vectors with size < 2 !?
            if ( 2 == it.size() && (it[0].distance < minDescDistRatio * it[1].distance) ) {
                matches.push_back(it[0]);
            }
        }
    }



### MP.7 Performance evaluation: number of keypoints
In the Task7_Nkeypoint.csv, I summarized the number of keypoints for different detector for each image. Among all the detectors, the FAST has the best performance, and it gives constantly ~400 keypoints for each image. 

### MP.8 & 9 Performance evaluation
I looped through the following detectors and descriptors:

    std::vector<std::string> detectorType_list = {"SHITOMASI", "SIFT", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE"};
    std::vector<std::string> descriptorType_list = {"BRISK", "FREAK", "BRIEF", "AKAZE", "SIFT", "ORB"};
    
    for (int detIndex = 0; detIndex < detectorType_list.size(); detIndex++){
        for (int dscpIndex = 0; dscpIndex < descriptorType_list.size(); dscpIndex++){
            string detectorType = detectorType_list[detIndex];
            string descriptorType = descriptorType_list[dscpIndex]; // BRIEF, ORB, FREAK, AKAZE, SIFT
            if (((detectorType == "AKAZE") || (descriptorType == "AKAZE")) && (detectorType != descriptorType))
                continue;
            else if ((detectorType == "SIFT") && (descriptorType == "ORB"))
                continue;
            else
            {
                ...


Note 1: AKAZE detector only works with AKAZE descriptor

Note 2: SIFT detector does not work with ORB descriptor

In the Task89_match.csv, I summarized the following metrics for each combination of detector and descriptor: number of keypoints, number of matched points, time of detector, time of descriptor.

Based on the data, I recommend top3 detector-descriptor combinations:

FAST-BRIEF

FAST-ORB

FAST-BRISK




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