#include<iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
    //
    // read two images and compute the keypoints and descriptors
    //
    string imageFile1;
    string imageFile2;

    if(argc != 3) {
        cout << "Using default images" << endl;
        imageFile1 = "../data/house1.jpg";
        imageFile2 = "../data/house2.jpg";
        cout << "Read image 1: " << imageFile1  << endl;
        cout << "Read image 2: " << imageFile2 << endl;        
    } else {
        imageFile1 = argv[1];
        imageFile2 = argv[2];
        cout << "Image 1: " << imageFile1  << endl;
        cout << "Image 2: " << imageFile2 << endl;        
    }

    Mat image1 = imread(imageFile1, CV_LOAD_IMAGE_COLOR);
    Mat image2 = imread(imageFile2, CV_LOAD_IMAGE_COLOR);

    // prepare ORB
    std::vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;
    Ptr<ORB> orb = ORB::create(
                            500,  // nfeatures 
                            1.2f, // pyramid decimation ratio
                            8,    // pyramid levels 
                            31,   // size of the border
                            0,    // not support currently
                            2,    // WTA_K
                            ORB::HARRIS_SCORE, 
                            31,   // size of the pathch to determine the orientation
                            20    // 20%
                        );

    // fint keypoints and descriptors
    orb->detect(image1, keypoints1);
    orb->detect(image2, keypoints2);
    orb->compute(image1, keypoints1, descriptors1);
    orb->compute(image2, keypoints2, descriptors2);
    
    // // draw keypoints
    // Mat outImage1, outImage2;
    // drawKeypoints(image1, keypoints1, outImage1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    // drawKeypoints(image2, keypoints2, outImage2, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    // imshow("Keypoints of the image1", outImage1);
    // imshow("Keypoints of the image2", outImage2);
    // waitKey(0);
    // destroyAllWindows();

    //
    // feature matching
    //

    // find the max and min distance
    vector<DMatch> matches;
    BFMatcher matcher(NORM_HAMMING); // WTA_K == 3 or 4, cv2.NORM_HAMMING2 should be used !!
    matcher.match(descriptors1, descriptors2, matches);

    double min_dist = 10000.0;
    double max_dist = 0.0;

    for (int i=0; i<descriptors1.rows; i++) {
        double dist = matches[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }
    printf("== Max dist: %f ==\n", max_dist);
    printf("== Min dist: %f ==\n", min_dist);

    std::vector<DMatch> goodMatches;
    for (int i=0; i<descriptors1.rows; i++) {
        // set the threshold to 2*min_dist
        // set the threshold to 30 if min_dist is too small
        if (matches[i].distance <= max(2*min_dist, 30.0)) {
            goodMatches.push_back(matches[i]);
        }
    }
    
    //
    // display the result
    //
    Mat imageAllMatches;
    Mat imageGoodMatches;
    drawMatches(image1, keypoints1, image2, keypoints2, matches, imageAllMatches);
    drawMatches(image1, keypoints1, image2, keypoints2, goodMatches, imageGoodMatches);
    imshow("All matching keypoints", imageAllMatches);
    imshow("Good matching keypoints", imageGoodMatches);
    waitKey(0);
    destroyAllWindows();

    return 0;    
}