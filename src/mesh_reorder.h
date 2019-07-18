#pragma once

#include <Eigen/Core>
#include <geogram/mesh/mesh.h>

///
/// Igl-style wrapper around Geogram's mesh reordering.
///
/// @param[in, out] V       #V x 3 input mesh vertices, modified by the function call.
/// @param[out]     IV      #V list of original indices.
/// @param[in]      morton  Whether to use Morton (true) or Hilbert (false).
///
void mesh_reorder(Eigen::MatrixXd &V, Eigen::VectorXi &IV, bool morton = true);

///
/// Reorder both vertices and facets of a mesh, remapping indices accordingly.
///
/// @param[in,out] V       #V x 3 mesh vertices.
/// @param[in,out] F       #F x 3 mesh facets.
/// @param[out]    IV      #V list of original vertex indices.
/// @param[out]    IF      #F list of original facet indices.
/// @param[in]     morton  Whether to use Morton (true) or Hilbert (false).
///
void mesh_reorder(Eigen::MatrixXd &V,
                  Eigen::MatrixXi &F,
                  Eigen::VectorXi &IV,
                  Eigen::VectorXi &IF,
                  bool morton = true);
