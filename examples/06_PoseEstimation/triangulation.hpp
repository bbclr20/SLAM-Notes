#ifndef _TRIANGULATION_HPP_
#define _TRIANGULATION_HPP_

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
@param std::vector<DMatch>& matches (out): the matches of the keypoints.

@return: void
*/
void find_feature_matches (
    const Mat& img_1, 
    const Mat& img_2,
    std::vector<KeyPoint>& keypoints_1,
    std::vector<KeyPoint>& keypoints_2,
    std::vector< DMatch >& matches
);

/*
Compute the R and t between two cameras.

@param const std::vector<KeyPoint> keypoints_1 (in): the keypoints in image1.
@param const std::vector<KeyPoint> keypoints_2 (in): the keypoints in image2.
@param const std::vector<DMatch> matches(in):  the matches between the keypoints.
@param Mat& R (out): rotation matrix.
@param Mat& t (out): translation.

@return: void
*/
void pose_estimation_2d2d (
    const std::vector<KeyPoint> keypoints_1,
    const std::vector<KeyPoint> keypoints_2,
    const std::vector<DMatch> matches,
    Mat& R,
    Mat& t
);

/*
Convert the position of a keypoint from pixel coordinate to camera coordinate.

@param const Point2d& p (in): position in pixel coordinate.
@param const Mat& K (in): camera matrix.

@return: the position of the keypoint in the camera coordinate.
*/
Point2f pixel2cam(const Point2d& p, const Mat& K);

void triangulation (
    const vector<KeyPoint>& keypoint_1,
    const vector<KeyPoint>& keypoint_2,
    const std::vector< DMatch >& matches,
    const Mat& R, const Mat& t,
    vector<Point3d>& points
);

/*
Compute the position obsered from camera1.

@param const vector<KeyPoint>& keypoint_1 (in): position in pixel coordinate.
@param const vector<KeyPoint>& keypoint_2 (in): position in pixel coordinate.
@param const std::vector<DMatch>& matches (in): the matching result of keypoint_1 and keypoint_2.
@param const Mat& R (in), t (in): the transformation which transforms the observation from camera1 to camera2.
@param vector<Point3d>& points (out): the x,y,z obsered from camera1.

@return: void
*/
void triangulation ( 
    const vector<KeyPoint>& keypoint_1, 
    const vector<KeyPoint>& keypoint_2, 
    const std::vector<DMatch>& matches,
    const Mat& R,
    const Mat& t, 
    vector<Point3d>& points
);

#endif
