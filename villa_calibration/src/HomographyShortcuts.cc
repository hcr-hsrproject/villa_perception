#include "HomographyShortcuts.h"
#include "HomCartShortcuts.h"
#include "HomographyCostFunctor.h"
#include "LinearAlgebraShortcuts.h"
#include "TransformShortcuts.h"

#include "ceres/ceres.h"

#include <opencv/cv.h>

using namespace ceres;
using namespace cv;
using namespace Eigen;
using namespace std;

MatrixXd homographyDLTConstraint(MatrixXd x, MatrixXd xPrime) {
    MatrixXd zero(1,3);
    zero(0,0) = 0.0;    zero(0,1) = 0.0;    zero(0,2) = 0.0;

    MatrixXd xT = x.transpose();
    double xP = xPrime(0,0);
    double yP = xPrime(1,0);
    double wP = xPrime(2,0);

    MatrixXd a(2,9);
    a.block(0,0,1,3) = zero;
    a.block(0,3,1,3) = -wP * xT;
    a.block(0,6,1,3) = yP * xT;
    a.block(1,0,1,3) = wP * xT;
    a.block(1,3,1,3) = zero;
    a.block(1,6,1,3) = -xP * xT;
    
    return a;
}

MatrixXd homographyDLT(MatrixXd x, MatrixXd xPrime) {
    MatrixXd A(x.cols() * 2, 9);

    for(int i = 0; i < x.cols(); i++) {
        A.block(i * 2, 0, 2, 9) = 
            homographyDLTConstraint(
                x.block(0, i, 3, 1),
                xPrime.block(0, i, 3, 1)
                );
    }

    //cout << "A: " << A << endl;

    return A;
}

MatrixXd computeDLTHomography(MatrixXd x, MatrixXd xPrime) {
    //cout << "computeDLTHomography:" << endl;
    //cout << "   computeDLTHomography:   x" << endl;
    //cout << x << endl;
    //cout << "   computeDLTHomography:   xPrime" << endl;
    //cout << xPrime << endl;
    MatrixXd hCol = rightNullSpace(homographyDLT(x, xPrime));
    MatrixXd h(3,3);
    int ctr = 0;
    for(int row = 0; row < 3; row++)
        for(int col = 0; col < 3; col++)
            h(row, col) = hCol(ctr++);

    //cout << "hCol: " << hCol << endl;    
    return h;
}

vector<MatrixXd> computeHomographies(
    ModelChessboard chessboard, vector<MatrixXd> camPoints) {
    vector<MatrixXd> homographies;
    for(int i = 0; i < camPoints.size(); i++) {
        MatrixXd h = 
            computeDLTHomography(
                chessboard.getModelCBH2D(),
                addARowOfConst(camPoints[i], 1.0));
        homographies.push_back(h);
    }
    return homographies;
}

Eigen::MatrixXd optimizeHomography(
    Eigen::MatrixXd modelPts, Eigen::MatrixXd imagePts
    , Eigen::MatrixXd homography) {
    double h[9];
    size_t ctr = 0;
    for(size_t row = 0; row < 3; row++)
        for(size_t col = 0; col < 3; col++)
            h[ctr++] = homography(row,col);

    Problem problem;
    DynamicNumericDiffCostFunction<HomographyCostFunctor> *cost_function =
      new DynamicNumericDiffCostFunction<HomographyCostFunctor>(
          new HomographyCostFunctor(modelPts, imagePts));
    cost_function->AddParameterBlock(9);
    cost_function->SetNumResiduals(modelPts.cols());
    problem.AddResidualBlock(cost_function, NULL, h);

    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    cout << summary.BriefReport() << endl;

    Eigen::MatrixXd retVal(3,3);
    ctr = 0;
    for(size_t row = 0; row < 3; row++)
        for(size_t col = 0; col < 3; col++)
            retVal(row,col) = h[ctr++];

    return retVal;
}

vector<MatrixXd> optimizeHomographies(
    MatrixXd modelPts, vector<MatrixXd> imagePts, vector<MatrixXd> homography) {
    vector<MatrixXd> retVal;
    for(size_t i = 0; i < imagePts.size(); i++)
        retVal.push_back(optimizeHomography(modelPts, imagePts[i]
            , homography[i]));
    return retVal;
}

vector<RigidTrans> extractRTs(
    CameraIntrinsics k, vector<MatrixXd> homographies) {
    vector<RigidTrans> rts;

    MatrixXd invK = k.getMat().inverse();

    for(int i = 0; i < homographies.size(); i++) {
        MatrixXd h = invK * homographies[i];
        Vector3d r1(h(0,0), h(1,0), h(2,0));
        Vector3d r2(h(0,1), h(1,1), h(2,1));
        Vector3d t(h(0,2), h(1,2), h(2,2));

        double r1N = r1.norm();
        double r2N = r2.norm();
        double tScale = (r1N + r2N) / 2.0;
        r1 = r1 / r1N;
        r2 = r2 / r2N;
        t = t / tScale;

        Vector3d r3 = r1.cross(r2);
        Matrix3d r(3,3);
        r(0,0) = r1(0); r(1,0) = r1(1); r(2,0) = r1(2);
        r(0,1) = r2(0); r(1,1) = r2(1); r(2,1) = r2(2);
        r(0,2) = r3(0); r(1,2) = r3(1); r(2,2) = r3(2);

        JacobiSVD<MatrixXd> svd(r, Eigen::ComputeFullU | Eigen::ComputeFullV);
        r = svd.matrixU() * svd.matrixV().transpose();

        MatrixXd rV = positiveBoundedRotVect(rotVect(r));

        rts.push_back(RigidTrans(rV, t));
    }
    return rts;
}
