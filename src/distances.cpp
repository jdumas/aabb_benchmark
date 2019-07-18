#include <geogram/basic/geometry_nd.h>
#include <Eigen/Geometry>
#include <cassert>
#undef IGL_STATIC_LIBRARY
#include <igl/point_simplex_squared_distance.h>

#define sqr(x) (x) * (x)

double inner_point_box_squared_distance(const Eigen::Vector3d& p, const Eigen::AlignedBox3d& B)
{
    assert(B.contains(p));
    double result = sqr(p[0] - B.min()[0]);
    result = std::min(result, sqr(p[0] - B.max()[0]));
    for (int c = 1; c < 3; ++c) {
        result = std::min(result, sqr(p[c] - B.min()[c]));
        result = std::min(result, sqr(p[c] - B.max()[c]));
    }
    return result;
}

double point_box_signed_squared_distance(const Eigen::Vector3d& p, const Eigen::AlignedBox3d& B)
{
    bool inside = true;
    double result = 0.0;
    for (int c = 0; c < 3; c++) {
        if (p[c] < B.min()[c]) {
            inside = false;
            result += sqr(p[c] - B.min()[c]);
        }
        else if (p[c] > B.max()[c]) {
            inside = false;
            result += sqr(p[c] - B.max()[c]);
        }
    }
    if (inside) {
        result = -inner_point_box_squared_distance(p, B);
    }
    return result;
}

double point_box_center_squared_distance(const Eigen::Vector3d& p, const Eigen::AlignedBox3d& B)
{
    double result = 0.0;
    for (int c = 0; c < 3; ++c) {
        double d = p[c] - 0.5 * (B.min()[c] + B.max()[c]);
        result += sqr(d);
    }
    return result;
}

void get_point_facet_nearest_point(const Eigen::MatrixXd& V,
                                   const Eigen::MatrixXi& F,
                                   const Eigen::Vector3d& p,
                                   int f,
                                   Eigen::Vector3d& nearest_p,
                                   double& squared_dist)
{
    assert(F.cols() == 3);
#if 0
    igl::point_simplex_squared_distance<3>(p, V, F, f, squared_dist, nearest_p);
#else
    GEO::vec3 query(p.data());
    GEO::vec3 pts[3];
    for (int lv = 0; lv < 3; ++lv) {
        int i = F(f, lv);
        pts[lv] = GEO::vec3(V(i, 0), V(i, 1), V(i, 2));
    }
    double lambda1, lambda2, lambda3;  // barycentric coords, not used.
    GEO::vec3 x;
    squared_dist = GEO::Geom::point_triangle_squared_distance(query, pts[0], pts[1], pts[2], x,
                                                              lambda1, lambda2, lambda3);
    nearest_p << x[0], x[1], x[2];
#endif
}
