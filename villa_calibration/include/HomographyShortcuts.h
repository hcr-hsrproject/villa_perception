#ifndef HOMOGRAPHY_SHORTCUTS_H
#define HOMOGRAPHY_SHORTCUTS_H

#include "CameraIntrinsics.h"
#include "ModelChessboard.h"
#include "RigidTrans.h"

#include <Eigen/Dense>
#include <vector>

Eigen::MatrixXd homographyDLTConstraint(
    Eigen::MatrixXd x, Eigen::MatrixXd xPrime);
Eigen::MatrixXd homographyDLT(Eigen::MatrixXd x, Eigen::MatrixXd xPrime);
Eigen::MatrixXd computeDLTHomography(Eigen::MatrixXd x, Eigen::MatrixXd xPrime);

std::vector<Eigen::MatrixXd> computeHomographies(
    ModelChessboard chessboard, std::vector<Eigen::MatrixXd> camPoints);

Eigen::MatrixXd optimizeHomography(
    Eigen::MatrixXd modelPts, Eigen::MatrixXd imagePts
    , Eigen::MatrixXd homography);
std::vector<Eigen::MatrixXd> optimizeHomographies(
    Eigen::MatrixXd modelPts, std::vector<Eigen::MatrixXd> imagePts
        , std::vector<Eigen::MatrixXd> homography);

std::vector<RigidTrans> extractRTs(
    CameraIntrinsics k, std::vector<Eigen::MatrixXd> homographies);


#endif
