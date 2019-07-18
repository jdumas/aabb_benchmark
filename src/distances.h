#include <Eigen/Geometry>
#include <cassert>

///
/// Computes the squared distance between a point and a Box.
///
/// @param[in]  p     the point
/// @param[in]  B     the box
///
/// @return     the squared distance between @p p and @p B
/// @pre        p is inside B
///
double inner_point_box_squared_distance(const Eigen::Vector3d& p, const Eigen::AlignedBox3d& B);

///
/// Computes the squared distance between a point and a Box with negative sign if the point is
/// inside the Box.
///
/// @param[in]  p     the point
/// @param[in]  B     the box
///
/// @return     the signed squared distance between @p p and @p B
///
double point_box_signed_squared_distance(const Eigen::Vector3d& p, const Eigen::AlignedBox3d& B);

///
/// Computes the squared distance between a point and the center of a box.
///
/// @param[in]  p     the point
/// @param[in]  B     the box
///
/// @return     the squared distance between @p p and the center of @p B /
///
double point_box_center_squared_distance(const Eigen::Vector3d& p, const Eigen::AlignedBox3d& B);

///
/// Finds the nearest point in a mesh facet from a query point.
///
/// @param[in]  V              #V x 3 mesh vertices positions.
/// @param[in]  F              #F x 3 mesh face indices.
/// @param[in]  p             the query point
/// @param[in]  f             index of the facet in @p F
/// @param[out] nearest_p     the point of facet @p f nearest to @p p
/// @param[out] squared_dist  the squared distance between @p p and @p nearest_p
///
void get_point_facet_nearest_point(const Eigen::MatrixXd& V,
                                   const Eigen::MatrixXi& F,
                                   const Eigen::Vector3d& p,
                                   int f,
                                   Eigen::Vector3d& nearest_p,
                                   double& squared_dist);
