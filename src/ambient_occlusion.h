#pragma once

#include "aabb.h"
#include <geogram/mesh/mesh_AABB.h>

template <typename AABBTreeType>
void ambient_occlusion(const Eigen::MatrixXd& V,
                       const Eigen::MatrixXi& F,
                       const AABBTreeType& tree,
                       const Eigen::MatrixXd& P,
                       const Eigen::MatrixXd& N,
                       const int num_samples,
                       Eigen::VectorXd& S);
