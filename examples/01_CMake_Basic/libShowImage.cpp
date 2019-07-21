#include<opencv2/opencv.hpp>

using namespace std;

void showImage(string filename) {
    cv::Mat image = cv::imread(filename);
    cv::imshow(filename, image);
    cv::waitKey(0);
    cv::destroyAllWindows();
}
