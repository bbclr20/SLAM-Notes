#ifndef _POSE_ESTIMATION_2D2D_HPP_
#define _POSE_ESTIMATION_2D2D_HPP_

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

/*
Find the keypoints and the matches between two images.

@param const Mat& img_1 (in): image1.
@param const Mat& img_2 (in): image2.
@param std::vector<KeyPoint>& keypoints_1 (out): the keypoints in image1.
@param std::vector<KeyPoint>& keypoints_2 (out): the keypoints in image2.
@param std::vector<DMatch>& matches (out): the matches between the keypoints.

@return: void
*/
void find_feature_matches (
    const Mat& img_1, const Mat& img_2,
    std::vector<KeyPoint>& keypoints_1,
    std::vector<KeyPoint>& keypoints_2,
    std::vector< DMatch >& matches 
);


/*
Compute the R and t between two cameras.

@param std::vector<KeyPoint> keypoints_1 (in): the keypoints in image1.
@param std::vector<KeyPoint> keypoints_2 (in): the keypoints in image2.
@param std::vector<DMatch> matches(in):  the matches between the keypoints.
@param Mat& R (out): rotation matrix.
@param Mat& t (out): translation.

@return: void
*/
void pose_estimation_2d2d (
    std::vector<KeyPoint> keypoints_1,
    std::vector<KeyPoint> keypoints_2,
    std::vector< DMatch > matches,
    Mat& R, Mat& t
);

/*
Convert the position of a keypoint from pixel coordinate to camera coordinate.

@param const Point2d& p (in): position in pixel coordinate.
@param const Mat& K (in): camera matrix.

@return: the position of the keypoint in the camera coordinate.
*/
Point2d pixel2cam (
    const Point2d& p, 
    const Mat& K
);

#endif