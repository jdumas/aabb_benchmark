#include "ambient_occlusion.h"
#include "aabb.h"
#include "aabb_binary.h"
#include "intersect_ray.h"
#include <igl/ambient_occlusion.h>

template <typename TreeType>
void ambient_occlusion(const Eigen::MatrixXd& V,
                       const Eigen::MatrixXi& F,
                       const TreeType& tree,
                       const Eigen::MatrixXd& P,
                       const Eigen::MatrixXd& N,
                       const int num_samples,
                       Eigen::VectorXd& S)
{
    const auto& shoot_ray = [&V, &F, &tree](const Eigen::Vector3f& origin,
                                            const Eigen::Vector3f& direction) -> bool {
        Eigen::Vector3d offset = (origin + 1e-4f * direction).cast<double>();
        Eigen::Vector3d position, normal;
        double param;
        return aabb_intersect_ray(V, F, tree, offset, direction.cast<double>(), position, normal,
                                  param);
    };
    return igl::ambient_occlusion(shoot_ray, P, N, num_samples, S);
}

// Explicit template instantiation
// clang-format off
template void ambient_occlusion<AABBTree>(Eigen::Matrix<double, -1, -1, 0, -1, -1> const&, Eigen::Matrix<int, -1, -1, 0, -1, -1> const&, AABBTree const&, Eigen::Matrix<double, -1, -1, 0, -1, -1> const&, Eigen::Matrix<double, -1, -1, 0, -1, -1> const&, int, Eigen::Matrix<double, -1, 1, 0, -1, 1>&);
template void ambient_occlusion<igl::AABB<Eigen::Matrix<double, -1, -1, 0, -1, -1>, 3> >(Eigen::Matrix<double, -1, -1, 0, -1, -1> const&, Eigen::Matrix<int, -1, -1, 0, -1, -1> const&, igl::AABB<Eigen::Matrix<double, -1, -1, 0, -1, -1>, 3> const&, Eigen::Matrix<double, -1, -1, 0, -1, -1> const&, Eigen::Matrix<double, -1, -1, 0, -1, -1> const&, int, Eigen::Matrix<double, -1, 1, 0, -1, 1>&);
template void ambient_occlusion<GEO::MeshFacetsAABB>(Eigen::Matrix<double, -1, -1, 0, -1, -1> const&, Eigen::Matrix<int, -1, -1, 0, -1, -1> const&, GEO::MeshFacetsAABB const&, Eigen::Matrix<double, -1, -1, 0, -1, -1> const&, Eigen::Matrix<double, -1, -1, 0, -1, -1> const&, int, Eigen::Matrix<double, -1, 1, 0, -1, 1>&);
template void ambient_occlusion<igl::embree::EmbreeIntersector>(Eigen::Matrix<double, -1, -1, 0, -1, -1> const&, Eigen::Matrix<int, -1, -1, 0, -1, -1> const&, igl::embree::EmbreeIntersector const&, Eigen::Matrix<double, -1, -1, 0, -1, -1> const&, Eigen::Matrix<double, -1, -1, 0, -1, -1> const&, int, Eigen::Matrix<double, -1, 1, 0, -1, 1>&);
template void ambient_occlusion<AABBTreeBinary>(Eigen::Matrix<double, -1, -1, 0, -1, -1> const&, Eigen::Matrix<int, -1, -1, 0, -1, -1> const&, AABBTreeBinary const&, Eigen::Matrix<double, -1, -1, 0, -1, -1> const&, Eigen::Matrix<double, -1, -1, 0, -1, -1> const&, int, Eigen::Matrix<double, -1, 1, 0, -1, 1>&);
// clang-format on
