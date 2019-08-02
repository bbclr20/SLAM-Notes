#include"pose_estimation_2d2d.hpp"

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
        std::vector<DMatch>& matches) 
{
    //
    // find the keypoints, compute the desciptor and match the keypoints
    //
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create("BruteForce-Hamming");
    
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    vector<DMatch> match;
    matcher->match(descriptors_1, descriptors_2, match);

    //
    // compute the max and min distance of the matches and filter the matches
    //
    double min_dist=10000, max_dist=0;

    for (int i=0; i<descriptors_1.rows; i++) {
        double dist = match[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }
    printf ("== Max dist: %f ==\n", max_dist);
    printf ("== Min dist: %f == \n", min_dist);

    // filter the keypoints by Hamming distance
    // the threshold is set to 30 if the value of 2*min_dist is too small
    for (int i=0; i<descriptors_1.rows; i++) {
        if (match[i].distance <= max (2*min_dist, 30.0)) {
            matches.push_back ( match[i] );
        }
    }
}

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
        std::vector<DMatch> matches,
        Mat& R, 
        Mat& t)
{
    //
    // compute the fundamental matrix "F"
    // the unit of points1 and points2 are pixel
    //
    vector<Point2f> points1;
    vector<Point2f> points2;
    for (int i = 0; i < (int)matches.size(); i++) {
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }

    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat(points1, points2, CV_FM_8POINT);
    cout << "\nFundamental_matrix is: \n" << fundamental_matrix <<endl;

    //
    // compute the essential matrix "E"
    //
    // camera matrix
    Mat K = (Mat_<double>(3, 3) << 
             520.9,      0.0,     325.1, 
             0.0,      521.0,     249.7, 
             0.0,        0.0,       1.0
            );

    Point2d principal_point(325.1, 249.7); // cx, cy (optical center of the camera)
    double focal_length = 521;			   // f form TUM dataset
    Mat essential_matrix;
    essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);
    cout << "\nEssential_matrix is: \n" << essential_matrix << endl;

    //
    // compute R and t
    //
    recoverPose (
        essential_matrix, 
        points1, 
        points2, 
        R, 
        t, 
        focal_length, 
        principal_point
    );
    cout<<"\nR is: \n" << R << endl;
    cout<<"\nt is: \n" << t << endl;    

    //
    // compute the homography matrix
    //
    Mat homography_matrix;
    homography_matrix = findHomography(points1, points2, RANSAC, 3);
    cout << "\nHomography_matrix is: \n" << homography_matrix << endl;
}

/*
Convert the position of a keypoint from pixel coordinate to camera coordinate.

@param const Point2d& p (in): position in pixel coordinate.
@param const Mat& K (in): camera matrix.

@return: the position of the keypoint in the camera coordinate.
*/
Point2d pixel2cam (const Point2d& p, const Mat& K) {
    return Point2d
           (
               (p.x - K.at<double> (0, 2)) / K.at<double>(0, 0),
               (p.y - K.at<double> (1, 2)) / K.at<double>(1, 1)
           );
}

int main(int argc, char** argv) {
    //
    // extract the matching keypoints from two images
    //
    string img1 = "../data/1.png";
    string img2 = "../data/2.png";

    if (argc != 3) {
        cout << "usage: pose_estimation_2d2d img1 img2" << endl;
        cout << "using default images!!" << endl;
        cout << "image1: " << img1 << endl;
        cout << "image2: " << img2 << endl;
    } else {
        img1 = argv[1];
        img2 = argv[2];
        cout << "image1: " << img1 << endl;
        cout << "image2: " << img2 << endl;
    }
    Mat img_1 = imread(img1, CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(img2, CV_LOAD_IMAGE_COLOR);

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    cout << "Totally " << matches.size() << "matches are found" << endl;
    
    Mat imageMatches;
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, imageMatches);
    imshow("Matching keypoints", imageMatches);
    waitKey(0);

    //
    // compute F,E, R and t
    //
    Mat R,t;
    pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);

    // verify that E=t^R*scale
    Mat t_x = (Mat_<double> ( 3,3 ) <<
               0,                      -t.at<double> ( 2,0 ),    t.at<double> ( 1,0 ),
               t.at<double> ( 2,0 ),    0,                      -t.at<double> ( 0,0 ),
              -t.at<double> ( 1,0 ),    t.at<double> ( 0,0 ),    0 
              );
    cout << "\nt^R:\n\n" << t_x*R << endl;

    // verify the epipolar constraint
    Mat K = (Mat_<double> (3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    for (DMatch m: matches ){
        Point2d pt1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        Mat y1 = (Mat_<double>(3, 1) << pt1.x, pt1.y, 1);
        Point2d pt2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
        Mat y2 = (Mat_<double>(3, 1) << pt2.x, pt2.y, 1);
        Mat d = y2.t() * t_x * R * y1;
        cout << "epipolar constraint = " << d << endl;
    }
    return 0;
}
