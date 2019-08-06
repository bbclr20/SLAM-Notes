#include"pose_estimation_3d2d.hpp"

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

/*
Do bundle adjustment.

@param const vector<Point3f> points_3d (in): object points.
@param const vector<Point2f> points_2d (in): image points.
@param const Mat& K (in): camera matrix.
@param Mat& R (in, out): rotation matrix.
@param Mat& t (in, out): translation.

@return void
*/
void bundleAdjustment (
    const vector<Point3f> points_3d,
    const vector<Point2f> points_2d,
    const Mat& K,
    Mat& R, 
    Mat& t)
{
    //
    // init g2o
    //
    const int POSE_DIM = 6;
    const int LANDMARK_DIM = 3;
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<POSE_DIM, LANDMARK_DIM>> Block;  
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();
    Block* solver_ptr = new Block(unique_ptr<Block::LinearSolverType>(linearSolver));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(unique_ptr<Block>(solver_ptr)); 
    
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    //
    // set R, T as vertices
    //
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    Eigen::Matrix3d R_mat;
    R_mat <<
          R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
          R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
          R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
    Eigen::Vector3d T_vec;
    T_vec << Eigen::Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0));
    pose->setId(0);
    pose->setEstimate( g2o::SE3Quat (
                            R_mat,
                            T_vec
                        ) );
    optimizer.addVertex(pose);

    //
    // set the positions of the landmarks as vertices
    //
    int index = 1;
    for (const Point3f p:points_3d) {  
        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
        point->setId(index++);
        point->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));
        point->setMarginalized(true);
        optimizer.addVertex(point);
    }

    //
    // add camera intrinsics to parameters
    //
    g2o::CameraParameters* camera = new g2o::CameraParameters (
        K.at<double>(0, 0), 
        Eigen::Vector2d(K.at<double>(0,2), K.at<double>(1,2)),
        0
    );
    camera->setId(0);
    optimizer.addParameter(camera);

    //
    // add edges
    //
    index = 1;
    for (const Point2f p:points_2d) {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId(index);
        edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(index)));
        edge->setVertex(1, pose);
        edge->setMeasurement(Eigen::Vector2d(p.x, p.y));
        edge->setParameterId(0,0);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
        index++;
    }

    //
    // start optimization
    //
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "optimization costs time: " << time_used.count() << " seconds." << endl;

    cout << endl << "after optimization:" << endl;
    cout << "T=" << endl << Eigen::Isometry3d(pose->estimate()).matrix() << endl;
}

int main (int argc, char** argv) {
    //
    // read images
    //
    string img1 = "../data/1.png";
    string img2 = "../data/2.png";
    string depth_img = "../data/1_depth.png";

    if (argc != 4) {
        cout << "usage: pose_estimation_2d2d img1 img2 depth_img" << endl;
        cout << "using default images!!" << endl;
        cout << "image1: " << img1 << endl;
        cout << "image2: " << img2 << endl;
        cout << "depth_img: " << depth_img << endl;
    } else {
        img1 = argv[1];
        img2 = argv[2];
        depth_img = argv[3];
        cout << "image1: " << img1 << endl;
        cout << "image2: " << img2 << endl;
        cout << "depth_img: " << depth_img << endl;
    }
    Mat img_1 = imread(img1, CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(img2, CV_LOAD_IMAGE_COLOR);

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    cout << "Totally " << matches.size() << " matches are found" << endl;

    //
    // get 3d-2d pairs
    //
    Mat d1 = imread(depth_img, CV_LOAD_IMAGE_UNCHANGED); // Uint16
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    vector<Point3f> pts_3d;
    vector<Point2f> pts_2d;
    for (DMatch m: matches) {
        ushort d = d1.ptr<unsigned short> (int ( keypoints_1[m.queryIdx].pt.y )) [ int ( keypoints_1[m.queryIdx].pt.x ) ];
        if (d == 0)   // bad depth
            continue;
        float dd = d/1000.0;
        Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        pts_3d.push_back(Point3f(p1.x*dd, p1.y*dd, dd));  // p1 3d
        pts_2d.push_back(keypoints_2[m.trainIdx].pt);     // p2 2d
    }
    cout << "3d-2d pairs: " << pts_3d.size() << endl;

    //
    // use OpenCV to solve PnP(EPNP，DLS)
    //
    Mat r, t;
    solvePnP (
        pts_3d,       // objectPoints
        pts_2d,       // imagePoints
        K,            // cameraMatrix
        Mat(),        // distCoeffs
        r,            // rotation vector
        t,            // translation vector
        false,        // useExtrinsicGuess
        SOLVEPNP_EPNP // method
    );
    Mat R;
    cv::Rodrigues(r, R); //  convert r to R with Rodrigues equation

    cout << "r=\n" << r << endl;
    cout << "R=\n" << R << endl;
    cout << "t=\n" << t << endl;

    //
    // BA (bundle adjustment)
    //
    cout << endl << "calling bundle adjustment" << endl;
    bundleAdjustment (pts_3d, pts_2d, K, R, t);
}
