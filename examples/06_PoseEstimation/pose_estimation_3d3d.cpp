#include"pose_estimation_3d3d.hpp"

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
               (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
               (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
           );
}

/*
Compute the R and t between two cameras.

@param const vector<Point3f>& pts1 (in):
@param const vector<Point3f>& pts2 (in):
@param Mat& R (out):
@param Mat& t (out):

@return void
*/
void pose_estimation_3d3d (
    const vector<Point3f>& pts1,
    const vector<Point3f>& pts2,
    Mat& R,
    Mat& t)
{
    //
    // find center of mass
    //
    Point3f p1, p2;
    int N = pts1.size();
    for (int i=0; i<N; i++) {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = Point3f(Vec3f(p1) / N);
    p2 = Point3f(Vec3f(p2) / N);
    
    vector<Point3f>  q1(N), q2(N); // remove the center
    for (int i=0; i<N; i++) {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i=0; i<N; i++) {
        W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z ) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
    }
    cout << "W=" << endl << W << endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    
    if (U.determinant() * V.determinant() < 0) {
        for (int x=0; x<3; ++x) {
            U(x, 2) *= -1;
        }
	}
    
    cout << "U=" << endl << U << endl;
    cout << "V=" << endl << V << endl;

    Eigen::Matrix3d R_ = U* (V.transpose());
    Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);

    // convert to cv::Mat
    R = ( Mat_<double> ( 3,3 ) <<
          R_(0, 0), R_(0, 1), R_(0, 2),
          R_(1, 0), R_(1, 1), R_(1, 2),
          R_(2, 0), R_(2, 1), R_(2, 2)
        );
    t = (Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
}

// g2o edge
class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeProjectXYZRGBDPoseOnly( const Eigen::Vector3d& point ) : _point(point) {}

    virtual void computeError()
    {
        const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
        // measurement is p, point is p'
        _error = _measurement - pose->estimate().map( _point );
    }

    virtual void linearizeOplus()
    {
        g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        g2o::SE3Quat T(pose->estimate());
        Eigen::Vector3d xyz_trans = T.map(_point);
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];

        _jacobianOplusXi(0,0) = 0;
        _jacobianOplusXi(0,1) = -z;
        _jacobianOplusXi(0,2) = y;
        _jacobianOplusXi(0,3) = -1;
        _jacobianOplusXi(0,4) = 0;
        _jacobianOplusXi(0,5) = 0;

        _jacobianOplusXi(1,0) = z;
        _jacobianOplusXi(1,1) = 0;
        _jacobianOplusXi(1,2) = -x;
        _jacobianOplusXi(1,3) = 0;
        _jacobianOplusXi(1,4) = -1;
        _jacobianOplusXi(1,5) = 0;

        _jacobianOplusXi(2,0) = -y;
        _jacobianOplusXi(2,1) = x;
        _jacobianOplusXi(2,2) = 0;
        _jacobianOplusXi(2,3) = 0;
        _jacobianOplusXi(2,4) = 0;
        _jacobianOplusXi(2,5) = -1;
    }

    bool read ( istream& in ) {}
    bool write ( ostream& out ) const {}
protected:
    Eigen::Vector3d _point;
};

/*
*/
void bundleAdjustment (
    const vector<Point3f>& pts1,
    const vector<Point3f>& pts2,
    Mat& R, 
    Mat& t)
{
    //
    // init g2o
    //
    const int POSE_DIM = 6;
    const int LANDMARK_DIM = 3;
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<POSE_DIM, LANDMARK_DIM>> Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>();
    Block* solver_ptr = new Block(unique_ptr<Block::LinearSolverType>(linearSolver));
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(unique_ptr<Block>(solver_ptr));
    
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    //
    // add vertex to graph
    //
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    pose->setId(0);
    pose->setEstimate( g2o::SE3Quat(
                            Eigen::Matrix3d::Identity(),
                            Eigen::Vector3d(0, 0, 0)
                    ));
    optimizer.addVertex(pose);

    //
    // add edges to graph
    //
    int index = 1;
    vector<EdgeProjectXYZRGBDPoseOnly*> edges;
    for (size_t i=0; i<pts1.size(); i++) {
        EdgeProjectXYZRGBDPoseOnly* edge = new EdgeProjectXYZRGBDPoseOnly(
            Eigen::Vector3d(pts2[i].x, pts2[i].y, pts2[i].z) );
        edge->setId(index);
        edge->setVertex(0, dynamic_cast<g2o::VertexSE3Expmap*> (pose));
        edge->setMeasurement( Eigen::Vector3d(
            pts1[i].x, pts1[i].y, pts1[i].z) );
        edge->setInformation(Eigen::Matrix3d::Identity() * 1e4);
        optimizer.addEdge(edge);
        index++;
        edges.push_back(edge);
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose( true );
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "optimization costs time: " << time_used.count() << " seconds." << endl;

    cout << endl << "after optimization:" << endl;
    cout << "T=" << endl << Eigen::Isometry3d(pose->estimate()).matrix() << endl;
}

int main (int argc, char** argv) {
    //
    // extract the matching keypoints from two images
    //
    string img1 = "../data/1.png";
    string img2 = "../data/2.png";
    string depth_img1 = "../data/1_depth.png";
    string depth_img2 = "../data/2_depth.png";
    if (argc != 5) {
        cout << "usage: pose_estimation_2d2d img1 img2 depth_img1 depth_img2" << endl;
        cout << "using default images!!" << endl;
    } else {
        img1 = argv[1];
        img2 = argv[2];
        depth_img1 = argv[3];
        depth_img1 = argv[4];
    }
    Mat img_1 = imread(img1, CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(img2, CV_LOAD_IMAGE_COLOR);

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    cout << "Totally " << matches.size() << " matching keypoints are found" << endl;
    
    //
    // get 3D positions
    //
    Mat depth1 = imread(depth_img1, CV_LOAD_IMAGE_UNCHANGED);
    Mat depth2 = imread(depth_img2, CV_LOAD_IMAGE_UNCHANGED);
    Mat K = (Mat_<double> ( 3,3 ) << 
                520.9,   0,   325.1, 
                  0,   521.0, 249.7, 
                  0,     0,     1
            );
    vector<Point3f> pts1, pts2;
    for (DMatch m: matches) {
        ushort d1 = depth1.at<unsigned short>(keypoints_1[m.queryIdx].pt.y, keypoints_1[m.queryIdx].pt.x);
        ushort d2 = depth2.at<unsigned short>(keypoints_2[m.trainIdx].pt.y, keypoints_2[m.trainIdx].pt.x);
        
        if (d1 == 0 || d2 == 0) continue; // bad depth

        Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        Point2d p2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
        float dd1 = float (d1)/1000.0;
        float dd2 = float (d2)/1000.0;
        pts1.push_back(Point3f(p1.x * dd1, p1.y * dd1, dd1));
        pts2.push_back(Point3f(p2.x * dd2, p2.y * dd2, dd2));
    }

    cout << "Totally " << pts1.size() << " 3d-3d pairs" << endl;
    Mat R, t;
    pose_estimation_3d3d(pts1, pts2, R, t);
    cout << endl << "ICP via SVD results: " << endl;
    cout << "R = " << endl << R << endl;
    cout << "t = " << endl << t << endl;
    cout << "R_inv = " << endl << R.t() << endl;
    cout << "t_inv = " << endl << -R.t() *t << endl;

    //
    // bundle adjustment
    //
    cout << endl << "calling bundle adjustment" << endl;
    bundleAdjustment(pts1, pts2, R, t);

    // verify p1 = R*p2 + t
    for (int i=0; i<5; i++) {
        cout << endl << "p1 = " << pts1[i] << endl;
        cout << endl << "p2 = " << pts2[i] << endl;
        cout << "(R*p2+t) = " <<
            R * (Mat_<double>(3,1)<<pts2[i].x, pts2[i].y, pts2[i].z) + t
            << endl;
        cout << endl;
    }
}
