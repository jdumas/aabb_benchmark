////////////////////////////////////////////////////////////////////////////////
#include "aabb_binary.h"
#include "distances.h"
#include "intersect_ray.h"
#include <igl/barycenter.h>
#include <igl/list_to_matrix.h>
#include <igl/slice.h>
#include <numeric>
#include <stack>
#include <limits>
#include <iostream>
#include <cmath>
////////////////////////////////////////////////////////////////////////////////

namespace {

Eigen::AlignedBox3d bbox_triangle(const Eigen::Vector3d &a,
                                  const Eigen::Vector3d &b,
                                  const Eigen::Vector3d &c)
{
    Eigen::AlignedBox3d box;
    box.extend(a);
    box.extend(b);
    box.extend(c);
    return box;
}

}  // namespace

////////////////////////////////////////////////////////////////////////////////

int AABBTreeBinary::number_of_nodes(int num_leaves)
{
    if (num_leaves == 0) {
        return 0;
    }
    int d = std::ceil(std::log2(num_leaves));
    return (1 << (d + 1));
}

////////////////////////////////////////////////////////////////////////////////

AABBTreeBinary::AABBTreeBinary(const Eigen::MatrixXd &V, Eigen::MatrixXi &F, bool sort)
    : boxes_(number_of_nodes(F.rows()) + 1)
{
    int num_internal_nodes = number_of_nodes(F.rows());

    // Compute the centroids of all the triangles in the input mesh
    Eigen::MatrixXd centroids;
    igl::barycenter(V, F, centroids);

    // Top-down approach: split each set of primitives into 2 sets of roughly equal size,
    // based on sorting the centroids along one direction or another.
    std::vector<int> triangles(F.rows());
    std::iota(triangles.begin(), triangles.end(), 0);

    Eigen::MatrixXi F2(F.rows(), F.cols());

    std::function<void(int, int, int)> top_down = [&](int index, int b, int e) {
        // If the index is > num_internal_nodes, then we are at a leaf
        if (e - b == 1) {
            int f = triangles[b];
            F2.row(b) = F.row(f);
            Eigen::Vector3d a = V.row(F(f, 0)).transpose();
            Eigen::Vector3d b = V.row(F(f, 1)).transpose();
            Eigen::Vector3d c = V.row(F(f, 2)).transpose();
            boxes_[index] = bbox_triangle(a, b, c);
            return;
        }

        // Otherwise, we need to sort centroids along the longest dimension, and split recursively
        if (sort) {
            Eigen::AlignedBox3d centroid_box;
            for (int k = b; k < e; ++k) {
                Eigen::Vector3d c = centroids.row(triangles[k]).transpose();
                centroid_box.extend(c);
            }
            Eigen::Vector3d extent = centroid_box.diagonal();
            int longest_dim = 0;
            for (int dim = 1; dim < 3; ++dim) {
                if (extent(dim) > extent(longest_dim)) {
                    longest_dim = dim;
                }
            }
            std::sort(triangles.begin() + b, triangles.begin() + e, [&](int f1, int f2) {
                return centroids(f1, longest_dim) < centroids(f2, longest_dim);
            });
        }

        // Then we can create a new internal node
        int left = 2 * index;
        int right = 2 * index + 1;
        int m = b + (e - b) / 2;
        top_down(left, b, m);
        top_down(right, m, e);
        boxes_[index] = boxes_[left].extend(boxes_[right]);
    };

    top_down(1, 0, triangles.size());

    F = F2;
}

////////////////////////////////////////////////////////////////////////////////

bool AABBTreeBinary::shoot_ray(const Eigen::MatrixXd &V,
                               const Eigen::MatrixXi &F,
                               const Eigen::Vector3d &ray_origin,
                               const Eigen::Vector3d &ray_direction,
                               Eigen::Vector3d &hit_position,
                               Eigen::Vector3d &hit_normal,
                               double &hit_param) const
{
    // Optimized version that leverages the BVH
    bool hit_exists = false;
    hit_param = std::numeric_limits<double>::infinity();
    std::stack<std::tuple<int, int, int>> q;
    q.emplace(1, 0, F.rows());
    while (!q.empty()) {
        auto [index, b, e] = q.top();
        q.pop();
        if (box_intersect_ray(boxes_[index], ray_origin, ray_direction)) {
            if (e - b == 1) {
                // Node is a leaf, we need to test the intersection with its triangle
                Eigen::Vector3d p1 = V.row(F(b, 0)).transpose();
                Eigen::Vector3d p2 = V.row(F(b, 1)).transpose();
                Eigen::Vector3d p3 = V.row(F(b, 2)).transpose();
                Eigen::Vector3d position, normal;
                double param;
                if (triangle_intersect_ray(p1, p2, p3, ray_origin, ray_direction, position, normal,
                                           param)) {
                    if (param < hit_param) {
                        hit_position = position;
                        hit_normal = normal;
                        hit_param = param;
                        hit_exists = true;
                    }
                }
            }
            else {
                // Internal node, we need to test intersection with both children
                int m = b + (e - b) / 2;
                q.emplace(2 * index, b, m);
                q.emplace(2 * index + 1, m, e);
            }
        }
    }
    return hit_exists;
}

////////////////////////////////////////////////////////////////////////////////

void AABBTreeBinary::get_nearest_facet_hint(const Eigen::MatrixXd &V,
                                            const Eigen::MatrixXi &F,
                                            const Eigen::Vector3d &query,
                                            int &nearest_f,
                                            Eigen::Vector3d &nearest_point,
                                            double &sq_dist) const
{
    if (F.rows() == 0) {
        return;
    }

    // Find a good initial value for nearest_f by traversing
    // the boxes and selecting the child such that the center
    // of its bounding box is nearer to the query point.
    // For a large mesh (20M facets) this gains up to 10%
    // performance as compared to picking nearest_f randomly.
    int index = 1;
    int b = 0;
    int e = F.rows();
    while (e - b > 1) {
        int m = b + (e - b) / 2;
        if (point_box_center_squared_distance(query, boxes_[2 * index]) <
            point_box_center_squared_distance(query, boxes_[2 * index + 1])) {
            index = 2 * index;
            e = m;
        }
        else {
            index = 2 * index + 1;
            b = m;
        }
    }
    nearest_f = b;
    assert(nearest_f >= 0);

    nearest_point = V.row(F(nearest_f, 0)).transpose();
    sq_dist = (nearest_point - query).squaredNorm();
}

////////////////////////////////////////////////////////////////////////////////

double AABBTreeBinary::squared_distance(const Eigen::MatrixXd &V,
                                        const Eigen::MatrixXi &F,
                                        const Eigen::Vector3d &query,
                                        int &nearest_facet,
                                        Eigen::Vector3d &nearest_point) const
{
    double sq_dist = std::numeric_limits<double>::max();
    if (F.rows() == 0) {
        return sq_dist;
    }
    get_nearest_facet_hint(V, F, query, nearest_facet, nearest_point, sq_dist);
    squared_distance_recursive(V, F, query, nearest_facet, nearest_point, sq_dist, 1, 0, F.rows());
    return sq_dist;
}

double AABBTreeBinary::squared_distance_with_hint(const Eigen::MatrixXd &V,
                                                  const Eigen::MatrixXi &F,
                                                  const Eigen::Vector3d &query,
                                                  int &nearest_facet,
                                                  Eigen::Vector3d &nearest_point,
                                                  double &sq_dist) const
{
    if (F.rows() == 0) {
        return sq_dist;
    }
    if (nearest_facet < 0) {
        get_nearest_facet_hint(V, F, query, nearest_facet, nearest_point, sq_dist);
    }
    squared_distance_recursive(V, F, query, nearest_facet, nearest_point, sq_dist, 1, 0, F.rows());
    return sq_dist;
}

void AABBTreeBinary::squared_distance_recursive(const Eigen::MatrixXd &V,
                                                const Eigen::MatrixXi &F,
                                                const Eigen::Vector3d &query,
                                                int &nearest_f,
                                                Eigen::Vector3d &nearest_point,
                                                double &sq_dist,
                                                int node_index,
                                                int b,
                                                int e) const
{
    if (e - b == 1) {
        // This is a leaf: compute point-facet distance and replace current if nearer.
        Eigen::Vector3d cur_nearest_point;
        double cur_sq_dist;
        get_point_facet_nearest_point(V, F, query, b, cur_nearest_point, cur_sq_dist);
        if (cur_sq_dist < sq_dist) {
            nearest_f = b;
            nearest_point = cur_nearest_point;
            sq_dist = cur_sq_dist;
        }
        return;
    }

    int cl = 2 * node_index;
    int cr = 2 * node_index + 1;
    int m = b + (e - b) / 2;
    double dl = point_box_signed_squared_distance(query, boxes_[cl]);
    double dr = point_box_signed_squared_distance(query, boxes_[cr]);

    // Traverse the "nearest" child first, so that it has more chances
    // to prune the traversal of the other child.
    if (dl < dr) {
        if (dl < sq_dist) {
            squared_distance_recursive(V, F, query, nearest_f, nearest_point, sq_dist, cl, b, m);
        }
        if (dr < sq_dist) {
            squared_distance_recursive(V, F, query, nearest_f, nearest_point, sq_dist, cr, m, e);
        }
    }
    else {
        if (dr < sq_dist) {
            squared_distance_recursive(V, F, query, nearest_f, nearest_point, sq_dist, cr, m, e);
        }
        if (dl < sq_dist) {
            squared_distance_recursive(V, F, query, nearest_f, nearest_point, sq_dist, cl, b, m);
        }
    }
}
