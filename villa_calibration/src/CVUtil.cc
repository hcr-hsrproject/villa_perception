#include "CVUtil.h"
#include "LinearAlgebraShortcuts.h"

#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;
using namespace Eigen;
using namespace std;

void drawCrosshair(Mat &img, int x, int y, int rad, int thickness
                    , Scalar color) {
    line(img, Point(x - rad, y), Point(x + rad, y), color, thickness);
    line(img, Point(x, y - rad), Point(x, y + rad), color, thickness);
}

//Mat3b img         -   Image to draw on
//MatrixXd points   -   Points to draw, Cartesian, 2D
void drawCrosshairs(Mat &img, MatrixXd points, int rad, int thickness
                    , Scalar color) {    
    for(int i =0; i < points.cols(); i++)
        drawCrosshair(img, points(0,i), points(1,i), rad, thickness, color);
}

void showPoints(Mat &frameBGR, MatrixXd inPts, Scalar &s) {
    drawCrosshairs(frameBGR, inPts, 5, 2, s);
    imshow("Calibration Output", frameBGR);
}

std::vector<cv::Mat> loadImages(std::vector<std::string> files) {
    vector<Mat> images;
    for(size_t i = 0; i < files.size(); i++) {
        images.push_back(imread(files[i], CV_LOAD_IMAGE_COLOR));
    }
    return images;
}

void detectCorners(
    Size s
    , vector<Mat> inImages
    , vector<Mat> &outImages
    , vector<MatrixXd> &outCorners) {
    for(size_t i = 0; i < inImages.size(); i++) {
        Mat c;
        bool found = findChessboardCorners(inImages[i], s, c);
        if(found) {
            Mat gray_image;
            cvtColor(inImages[i], gray_image, CV_BGR2GRAY);
            //cornerSubPix(gray_image, c, s, Size(-1, -1)
            //    , TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            drawChessboardCorners(inImages[i], s, c, found);
            outImages.push_back(inImages[i]);
            MatrixXd d = convert2ChanCVMatToEigenMatF(c);
            outCorners.push_back(d);
            //cout << "detectCorners: c   " << c.rows << "    " << c.cols <<
            //    "    " << c.channels() << endl;
            //cout << c << endl;
            //cout << d << endl;
            //drawChessboardCorners(inImages[i], s, c, found);
            //imshow("Calibration Output", inImages[i]);
            //waitKey();
            //cout << "detectCorners: c" << endl;
            //cout << convertToEigenMat(c) << endl;
        }
    }
}
