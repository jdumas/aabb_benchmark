#pragma once

#include <Eigen/Core>
#include <igl/AABB.h>
#include <igl/embree/EmbreeIntersector.h>
#include <geogram/mesh/mesh_AABB.h>

struct AABBTree;

///
/// Compute the intersection between a triangle and a ray.
///
/// @param[in]  a              Triangle corner.
/// @param[in]  b              Triangle corner.
/// @param[in]  c              Triangle corner.
/// @param[in]  ray_origin     Ray origin.
/// @param[in]  ray_direction  Ray direction (assumed to be normalized).
/// @param[out] hit_position   Position of the intersection point.
/// @param[out] hit_normal     Normal to the surface at intersection.
/// @param[out] hit_param      Ray parameter at intersection (distance to intersection).
///
/// @return     True in case of intersection.
///
bool triangle_intersect_ray(const Eigen::Vector3d &a,
                            const Eigen::Vector3d &b,
                            const Eigen::Vector3d &c,
                            const Eigen::Vector3d &ray_origin,
                            const Eigen::Vector3d &ray_direction,
                            Eigen::Vector3d &hit_position,
                            Eigen::Vector3d &hit_normal,
                            double &hit_param);

///
/// Compute the intersection between an axis-aligned box and a ray.
///
/// @param[in]  box            The box.
/// @param[in]  ray_origin     Ray origin.
/// @param[in]  ray_direction  Ray direction (assumed to be normalized).
///
/// @return     True in case of intersection.
///
bool box_intersect_ray(const Eigen::AlignedBox3d &box,
                       const Eigen::Vector3d &ray_origin,
                       const Eigen::Vector3d &ray_direction);

///
/// Compute the intersection between a parallelogram and a ray. This version uses a precomputed AABB
/// tree to accelerate queries.
///
/// @param[in]  V              #V x 3 matrix of mesh vertices positions.
/// @param[in]  F              #F x 3 matrix of mesh face indices.
/// @param[in]  tree           AABB tree of the input mesh.
/// @param[in]  ray_origin     Ray origin.
/// @param[in]  ray_direction  Ray direction (assumed to be normalized).
/// @param[out] hit_position   Position of the intersection point.
/// @param[out] hit_normal     Normal to the surface at intersection.
/// @param[out] hit_param      Ray parameter at intersection (distance to intersection).
///
/// @tparam     AABBTreeType   Type of the AABB tree.
///
/// @return     True in case of intersection.
///
template <typename AABBTreeType>
bool aabb_intersect_ray(const Eigen::MatrixXd &V,
                        const Eigen::MatrixXi &F,
                        const AABBTreeType &tree,
                        const Eigen::Vector3d &ray_origin,
                        const Eigen::Vector3d &ray_direction,
                        Eigen::Vector3d &hit_position,
                        Eigen::Vector3d &hit_normal,
                        double &hit_param);
