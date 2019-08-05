#ifndef _POSE_ESTIMATION_3D2D_HPP_
#define _POSE_ESTIMATION_3D2D_HPP_

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>

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
    const Mat& img_1, const Mat& img_2,
    std::vector<KeyPoint>& keypoints_1,
    std::vector<KeyPoint>& keypoints_2,
    std::vector< DMatch >& matches
);

/*
Convert the position of a keypoint from pixel coordinate to camera coordinate.

@param const Point2d& p (in): position in pixel coordinate.
@param const Mat& K (in): camera matrix.

@return: the position of the keypoint in the camera coordinate.
*/
Point2d pixel2cam (const Point2d& p, const Mat& K);

void bundleAdjustment (
    const vector<Point3f> points_3d,
    const vector<Point2f> points_2d,
    const Mat& K,
    Mat& R, Mat& t
);

#endif
