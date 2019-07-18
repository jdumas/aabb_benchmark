#pragma once

#include <Eigen/Core>
#include <igl/AABB.h>
#include <igl/embree/EmbreeIntersector.h>
#include <geogram/mesh/mesh_AABB.h>

struct AABBTree;

///
/// Compute the squared distances between a point set and a triangle mesh.
///
/// @param[in]  V             #V x 3 matrix of mesh vertices positions.
/// @param[in]  F             #F x 3 matrix of mesh face indices.
/// @param[in]  tree          AABB tree of the input mesh.
/// @param[in]  P             #P x 3 matrix of query points positions.
/// @param[out] S             #P x 1 vector of squared distances.
///
/// @tparam     AABBTreeType  Type of the AABB tree.
///
template <typename AABBTreeType>
void squared_distances(const Eigen::MatrixXd &V,
                       const Eigen::MatrixXi &F,
                       const AABBTreeType &tree,
                       const Eigen::MatrixXd &P,
                       Eigen::VectorXd &S);
