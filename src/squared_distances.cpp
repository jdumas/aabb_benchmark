////////////////////////////////////////////////////////////////////////////////
#include "squared_distances.h"
#include "aabb.h"
#include "aabb_binary.h"
#include "distances.h"
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_geometry.h>
////////////////////////////////////////////////////////////////////////////////

namespace {

inline void get_point_facet_nearest_point(const GEO::Mesh &M,
                                          const GEO::vec3 &p,
                                          GEO::index_t f,
                                          GEO::vec3 &nearest_p,
                                          double &squared_dist)
{
    using namespace GEO;
    geo_debug_assert(M.facets.nb_vertices(f) == 3);
    index_t c = M.facets.corners_begin(f);
    const vec3 &p1 = Geom::mesh_vertex(M, M.facet_corners.vertex(c));
    ++c;
    const vec3 &p2 = Geom::mesh_vertex(M, M.facet_corners.vertex(c));
    ++c;
    const vec3 &p3 = Geom::mesh_vertex(M, M.facet_corners.vertex(c));
    double lambda1, lambda2, lambda3;  // barycentric coords, not used.
    squared_dist =
        Geom::point_triangle_squared_distance(p, p1, p2, p3, nearest_p, lambda1, lambda2, lambda3);
}

}  // namespace

////////////////////////////////////////////////////////////////////////////////

template <>
void squared_distances<igl::AABB<Eigen::MatrixXd, 3>>(const Eigen::MatrixXd &V,
                                                      const Eigen::MatrixXi &F,
                                                      const igl::AABB<Eigen::MatrixXd, 3> &tree,
                                                      const Eigen::MatrixXd &P,
                                                      Eigen::VectorXd &S)
{
    S.resize(P.rows());
    for (int i = 0; i < P.rows(); ++i) {
        int index;
        Eigen::RowVector3d query, nearest_point;
        query = P.row(i);
        S(i) = tree.squared_distance(V, F, query, index, nearest_point);
    }
}

template <>
void squared_distances<AABBTree>(const Eigen::MatrixXd &V,
                                 const Eigen::MatrixXi &F,
                                 const AABBTree &tree,
                                 const Eigen::MatrixXd &P,
                                 Eigen::VectorXd &S)
{
#if 1
    // Without caching prev result
    S.resize(P.rows());
    for (int i = 0; i < P.rows(); ++i) {
        int index;
        Eigen::Vector3d query, nearest_point;
        query = P.row(i).transpose();
        S(i) = tree.squared_distance(V, F, query, index, nearest_point);
    }
#else
    // With caching of prev result
    S.resize(P.rows());
    int prev_facet = -1;
    double sq_dist = std::numeric_limits<double>::max();
    Eigen::Vector3d nearest_point;
    for (int i = 0; i < P.rows(); ++i) {
        Eigen::Vector3d query = P.row(i).transpose();
        if (prev_facet >= 0) {
            get_point_facet_nearest_point(V, F, query, prev_facet, nearest_point, sq_dist);
        }
        S(i) = tree.squared_distance(V, F, query, prev_facet, nearest_point);
    }
#endif
}

template <>
void squared_distances<AABBTreeBinary>(const Eigen::MatrixXd &V,
                                       const Eigen::MatrixXi &F,
                                       const AABBTreeBinary &tree,
                                       const Eigen::MatrixXd &P,
                                       Eigen::VectorXd &S)
{
#if 1
    // Without caching prev result
    S.resize(P.rows());
    for (int i = 0; i < P.rows(); ++i) {
        int index;
        Eigen::Vector3d query, nearest_point;
        query = P.row(i).transpose();
        S(i) = tree.squared_distance(V, F, query, index, nearest_point);
    }
#else
    // With caching of prev result
    S.resize(P.rows());
    int prev_facet = -1;
    double sq_dist = std::numeric_limits<double>::max();
    Eigen::Vector3d nearest_point;
    for (int i = 0; i < P.rows(); ++i) {
        Eigen::Vector3d query = P.row(i).transpose();
        if (prev_facet >= 0) {
            get_point_facet_nearest_point(V, F, query, prev_facet, nearest_point, sq_dist);
        }
        S(i) = tree.squared_distance(V, F, query, prev_facet, nearest_point);
    }
#endif
}

template <>
void squared_distances<GEO::MeshFacetsAABB>(const Eigen::MatrixXd &V,
                                            const Eigen::MatrixXi &F,
                                            const GEO::MeshFacetsAABB &tree,
                                            const Eigen::MatrixXd &P,
                                            Eigen::VectorXd &S)
{
#if 1
    // Without caching prev result
    S.resize(P.rows());
    for (int i = 0; i < P.rows(); ++i) {
        GEO::vec3 query(P(i, 0), P(i, 1), P(i, 2));
        double sq_dist = std::numeric_limits<double>::max();
        GEO::vec3 nearest_point;
        tree.nearest_facet(query, nearest_point, sq_dist);
        S(i) = sq_dist;
    }
#else
    // With caching of prev result
    S.resize(P.rows());
    GEO::vec3 nearest_point;
    double sq_dist = std::numeric_limits<double>::max();
    GEO::index_t prev_facet = GEO::NO_FACET;
    for (int i = 0; i < P.rows(); ++i) {
        GEO::vec3 query(P(i, 0), P(i, 1), P(i, 2));
        if (prev_facet != GEO::NO_FACET) {
            get_point_facet_nearest_point(*tree.mesh(), query, prev_facet, nearest_point, sq_dist);
        }
        tree.nearest_facet_with_hint(query, prev_facet, nearest_point, sq_dist);
        S(i) = sq_dist;
    }
#endif
}
