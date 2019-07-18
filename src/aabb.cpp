////////////////////////////////////////////////////////////////////////////////
#include "aabb.h"
#include "distances.h"
#include "intersect_ray.h"
#include <igl/barycenter.h>
#include <numeric>
#include <stack>
#include <limits>
#include <iostream>
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

AABBTree::AABBTree(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, bool sort)
{
    // Compute the centroids of all the triangles in the input mesh
    Eigen::MatrixXd centroids;
    igl::barycenter(V, F, centroids);

    // Top-down approach: split each set of primitives into 2 sets of roughly equal size,
    // based on sorting the centroids along one direction or another.
    std::vector<int> triangles(F.rows());
    std::iota(triangles.begin(), triangles.end(), 0);

    std::function<int(int, int, int)> top_down = [&](int i, int j, int parent) {
        // Scene is empty, so is the aabb tree
        if (j - i == 0) {
            return -1;
        }

        // If there is only 1 triangle left, then we are at a leaf
        if (j - i == 1) {
            Node node;
            int f = triangles[i];
            Eigen::Vector3d a = V.row(F(f, 0)).transpose();
            Eigen::Vector3d b = V.row(F(f, 1)).transpose();
            Eigen::Vector3d c = V.row(F(f, 2)).transpose();
            node.bbox = bbox_triangle(a, b, c);
            node.parent = parent;
            node.left = node.right = -1;
            node.triangle = triangles[i];
            nodes_.push_back(node);
            return (int)(nodes_.size() - 1);
        }

        // Otherwise, we need to sort centroids along the longest dimension, and split recursively
        if (sort) {
            Eigen::AlignedBox3d centroid_box;
            for (int k = i; k < j; ++k) {
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
            std::sort(triangles.begin() + i, triangles.begin() + j, [&](int f1, int f2) {
                return centroids(f1, longest_dim) < centroids(f2, longest_dim);
            });
        }

        // Then we can create a new internal node
        int current = nodes_.size();
        nodes_.resize(current + 1);
        int midpoint = (i + j) / 2;
        int left = top_down(i, midpoint, current);
        int right = top_down(midpoint, j, current);
        Node &node = nodes_[current];
        node.left = left;
        node.right = right;
        node.parent = parent;
        node.triangle = -1;
        node.bbox = nodes_[node.left].bbox.extend(nodes_[node.right].bbox);

        return current;
    };

    root_ = top_down(0, triangles.size(), -1);
}

////////////////////////////////////////////////////////////////////////////////

bool AABBTree::shoot_ray(const Eigen::MatrixXd &V,
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
    std::stack<int> q;
    q.push(root_);
    while (!q.empty()) {
        const Node &node = nodes_[q.top()];
        q.pop();
        if (box_intersect_ray(node.bbox, ray_origin, ray_direction)) {
            if (node.left < 0) {
                // Node is a leaf, we need to test the intersection with its triangle
                Eigen::Vector3d a = V.row(F(node.triangle, 0)).transpose();
                Eigen::Vector3d b = V.row(F(node.triangle, 1)).transpose();
                Eigen::Vector3d c = V.row(F(node.triangle, 2)).transpose();
                Eigen::Vector3d position, normal;
                double param;
                if (triangle_intersect_ray(a, b, c, ray_origin, ray_direction, position, normal,
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
                q.push(node.left);
                q.push(node.right);
            }
        }
    }
    return hit_exists;
}

////////////////////////////////////////////////////////////////////////////////

void AABBTree::get_nearest_facet_hint(const Eigen::MatrixXd &V,
                                      const Eigen::MatrixXi &F,
                                      const Eigen::Vector3d &query,
                                      int &nearest_f,
                                      Eigen::Vector3d &nearest_point,
                                      double &sq_dist) const
{
    // Find a good initial value for nearest_f by traversing
    // the boxes and selecting the child such that the center
    // of its bounding box is nearer to the query point.
    // For a large mesh (20M facets) this gains up to 10%
    // performance as compared to picking nearest_f randomly.
    int current = root_;
    while (nodes_[current].left >= 0) {
        const Node &node = nodes_[current];
        if (point_box_center_squared_distance(query, nodes_[node.left].bbox) <
            point_box_center_squared_distance(query, nodes_[node.right].bbox)) {
            current = node.left;
        }
        else {
            current = node.right;
        }
    }
    nearest_f = nodes_[current].triangle;
    assert(nearest_f >= 0);

    nearest_point = V.row(F(nearest_f, 0)).transpose();
    sq_dist = (nearest_point - query).squaredNorm();
}

////////////////////////////////////////////////////////////////////////////////

double AABBTree::squared_distance(const Eigen::MatrixXd &V,
                                  const Eigen::MatrixXi &F,
                                  const Eigen::Vector3d &query,
                                  int &nearest_facet,
                                  Eigen::Vector3d &nearest_point) const
{
    double sq_dist = std::numeric_limits<double>::max();
    get_nearest_facet_hint(V, F, query, nearest_facet, nearest_point, sq_dist);
    squared_distance_recursive(V, F, query, nearest_facet, nearest_point, sq_dist, root_);
    return sq_dist;
}

double AABBTree::squared_distance_with_hint(const Eigen::MatrixXd &V,
                                            const Eigen::MatrixXi &F,
                                            const Eigen::Vector3d &query,
                                            int &nearest_facet,
                                            Eigen::Vector3d &nearest_point,
                                            double &sq_dist) const
{
    if (nearest_facet < 0) {
        get_nearest_facet_hint(V, F, query, nearest_facet, nearest_point, sq_dist);
    }
    squared_distance_recursive(V, F, query, nearest_facet, nearest_point, sq_dist, root_);
    return sq_dist;
}

void AABBTree::squared_distance_recursive(const Eigen::MatrixXd &V,
                                          const Eigen::MatrixXi &F,
                                          const Eigen::Vector3d &query,
                                          int &nearest_f,
                                          Eigen::Vector3d &nearest_point,
                                          double &sq_dist,
                                          int node_index) const
{
    const Node &node = nodes_[node_index];

    if (node.left < 0) {
        // This is a leaf: compute point-facet distance and replace current if nearer.
        Eigen::Vector3d cur_nearest_point;
        double cur_sq_dist;
        get_point_facet_nearest_point(V, F, query, node.triangle, cur_nearest_point, cur_sq_dist);
        if (cur_sq_dist < sq_dist) {
            nearest_f = node.triangle;
            nearest_point = cur_nearest_point;
            sq_dist = cur_sq_dist;
        }
        return;
    }

    double dl = point_box_signed_squared_distance(query, nodes_[node.left].bbox);
    double dr = point_box_signed_squared_distance(query, nodes_[node.right].bbox);

    // Traverse the "nearest" child first, so that it has more chances
    // to prune the traversal of the other child.
    if (dl < dr) {
        if (dl < sq_dist) {
            squared_distance_recursive(V, F, query, nearest_f, nearest_point, sq_dist, node.left);
        }
        if (dr < sq_dist) {
            squared_distance_recursive(V, F, query, nearest_f, nearest_point, sq_dist, node.right);
        }
    }
    else {
        if (dr < sq_dist) {
            squared_distance_recursive(V, F, query, nearest_f, nearest_point, sq_dist, node.right);
        }
        if (dl < sq_dist) {
            squared_distance_recursive(V, F, query, nearest_f, nearest_point, sq_dist, node.left);
        }
    }
}
