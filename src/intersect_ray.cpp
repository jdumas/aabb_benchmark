////////////////////////////////////////////////////////////////////////////////
#include "intersect_ray.h"
#include "aabb.h"
#include "aabb_binary.h"
#include <Eigen/Dense>
#include <geogram/mesh/mesh_geometry.h>
#include <limits>
#include <numeric>
#undef IGL_STATIC_LIBRARY
#include <igl/ray_mesh_intersect.h>
////////////////////////////////////////////////////////////////////////////////

bool triangle_intersect_ray(const Eigen::Vector3d &a,
                            const Eigen::Vector3d &b,
                            const Eigen::Vector3d &c,
                            const Eigen::Vector3d &ray_origin,
                            const Eigen::Vector3d &ray_direction,
                            Eigen::Vector3d &hit_position,
                            Eigen::Vector3d &hit_normal,
                            double &hit_param)
{
    igl::Hit hit;
    Eigen::Matrix<double, 3, 3> V;
    V.row(0) = a.transpose();
    V.row(1) = b.transpose();
    V.row(2) = c.transpose();
    Eigen::RowVector3i F(0, 1, 2);
    auto ret = igl::ray_mesh_intersect(ray_origin, ray_direction, V, F, hit);
    if (ret) {
        hit_param = hit.t;
        hit_position = ray_origin + hit_param * ray_direction;
        hit_normal = (b - a).cross(c - a).normalized();
    }
    return ret;
}

////////////////////////////////////////////////////////////////////////////////

bool box_intersect_ray(const Eigen::AlignedBox3d &box,
                       const Eigen::Vector3d &ray_origin,
                       const Eigen::Vector3d &ray_direction)
{
    double tmin = 0;
    double tmax = std::numeric_limits<double>::infinity();
    for (int dim = 0; dim < 3; ++dim) {
        if (std::abs(ray_direction(dim)) < 1e-5) {
            if (ray_origin[dim] < box.min()[dim] || ray_origin[dim] > box.max()[dim]) {
                return false;
            }
        }
        else {
            double t1 = (box.min()[dim] - ray_origin[dim]) / ray_direction[dim];
            double t2 = (box.max()[dim] - ray_origin[dim]) / ray_direction[dim];
            tmin = std::max(std::min(t1, t2), tmin);
            tmax = std::min(std::max(t1, t2), tmax);
        }
    }
    return (tmin <= tmax);
}

////////////////////////////////////////////////////////////////////////////////

template <>
bool aabb_intersect_ray<AABBTree>(const Eigen::MatrixXd &V,
                                  const Eigen::MatrixXi &F,
                                  const AABBTree &tree,
                                  const Eigen::Vector3d &ray_origin,
                                  const Eigen::Vector3d &ray_direction,
                                  Eigen::Vector3d &hit_position,
                                  Eigen::Vector3d &hit_normal,
                                  double &hit_param)
{
    return tree.shoot_ray(V, F, ray_origin, ray_direction, hit_position, hit_normal, hit_param);
}

////////////////////////////////////////////////////////////////////////////////

template <>
bool aabb_intersect_ray<AABBTreeBinary>(const Eigen::MatrixXd &V,
                                        const Eigen::MatrixXi &F,
                                        const AABBTreeBinary &tree,
                                        const Eigen::Vector3d &ray_origin,
                                        const Eigen::Vector3d &ray_direction,
                                        Eigen::Vector3d &hit_position,
                                        Eigen::Vector3d &hit_normal,
                                        double &hit_param)
{
    return tree.shoot_ray(V, F, ray_origin, ray_direction, hit_position, hit_normal, hit_param);
}

////////////////////////////////////////////////////////////////////////////////

template <>
bool aabb_intersect_ray<igl::AABB<Eigen::MatrixXd, 3>>(const Eigen::MatrixXd &V,
                                                       const Eigen::MatrixXi &F,
                                                       const igl::AABB<Eigen::MatrixXd, 3> &tree,
                                                       const Eigen::Vector3d &ray_origin,
                                                       const Eigen::Vector3d &ray_direction,
                                                       Eigen::Vector3d &hit_position,
                                                       Eigen::Vector3d &hit_normal,
                                                       double &hit_param)
{
    igl::Hit hit;
    bool ret = tree.intersect_ray(V, F, ray_origin.transpose(), ray_direction.transpose(), hit);
    if (ret) {
        hit_param = hit.t;
        hit_position = ray_origin + hit.t * ray_direction;
        Eigen::Vector3d a = V.row(F.row(hit.id)[0]).transpose();
        Eigen::Vector3d b = V.row(F.row(hit.id)[1]).transpose();
        Eigen::Vector3d c = V.row(F.row(hit.id)[2]).transpose();
        hit_normal = (b - a).cross(c - a).normalized();
    }
    return ret;
}

////////////////////////////////////////////////////////////////////////////////

template <>
bool aabb_intersect_ray<igl::embree::EmbreeIntersector>(const Eigen::MatrixXd &V,
                                                        const Eigen::MatrixXi &F,
                                                        const igl::embree::EmbreeIntersector &tree,
                                                        const Eigen::Vector3d &ray_origin,
                                                        const Eigen::Vector3d &ray_direction,
                                                        Eigen::Vector3d &hit_position,
                                                        Eigen::Vector3d &hit_normal,
                                                        double &hit_param)
{
    igl::Hit hit;
    bool ret = tree.intersectRay(ray_origin.transpose().cast<float>(),
                                 ray_direction.transpose().cast<float>(), hit);
    if (ret) {
        hit_param = hit.t;
        hit_position = ray_origin + hit.t * ray_direction;
        Eigen::Vector3d a = V.row(F.row(hit.id)[0]).transpose();
        Eigen::Vector3d b = V.row(F.row(hit.id)[1]).transpose();
        Eigen::Vector3d c = V.row(F.row(hit.id)[2]).transpose();
        hit_normal = (b - a).cross(c - a).normalized();
    }
    return ret;
}

////////////////////////////////////////////////////////////////////////////////

template <>
bool aabb_intersect_ray<GEO::MeshFacetsAABB>(const Eigen::MatrixXd &V,
                                             const Eigen::MatrixXi &F,
                                             const GEO::MeshFacetsAABB &tree,
                                             const Eigen::Vector3d &ray_origin,
                                             const Eigen::Vector3d &ray_direction,
                                             Eigen::Vector3d &hit_position,
                                             Eigen::Vector3d &hit_normal,
                                             double &hit_param)
{
    // Multiply by big constant because AABB has a segment isect routine
    // (not ray isect routine), so it would ignore intersections further
    // away than (R.origin + R.direction), and we want them!
    GEO::vec3 origin(ray_origin.data());
    GEO::vec3 direction(ray_direction.data());
    GEO::vec3 p2 = origin + 10000.0 * direction;
    double t;
    GEO::index_t f;
    if (tree.segment_nearest_intersection(origin, p2, t, f)) {
        t *= 10000.0;
        if (t > 0) {
            hit_param = t;
            hit_position = ray_origin + t * ray_direction;
            GEO::vec3 normal = GEO::normalize(GEO::Geom::mesh_facet_normal(*tree.mesh(), f));
            hit_normal << normal[0], normal[1], normal[2];
            return true;
        }
    }
    return false;
}
