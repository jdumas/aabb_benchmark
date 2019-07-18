#pragma once

#include <Eigen/Geometry>

struct AABBTree {
protected:
    struct Node {
        Eigen::AlignedBox3d bbox;
        int parent;    // Index of the parent node (-1 for root)
        int left;      // Index of the left child (-1 for a leaf)
        int right;     // Index of the right child (-1 for a leaf)
        int triangle;  // Index of the node triangle (-1 for internal nodes)
    };

    std::vector<Node> nodes_;
    int root_;

public:
    AABBTree() = default;  // Default empty constructor
    AABBTree(const Eigen::MatrixXd &V,
             const Eigen::MatrixXi &F,
             bool sort = true);  // Build a BVH from an existing mesh

    /////////////////
    // Ray casting //
    /////////////////

    bool shoot_ray(const Eigen::MatrixXd &V,
                   const Eigen::MatrixXi &F,
                   const Eigen::Vector3d &ray_origin,
                   const Eigen::Vector3d &ray_direction,
                   Eigen::Vector3d &hit_position,
                   Eigen::Vector3d &hit_normal,
                   double &hit_param) const;

    //////////////////////
    // Distance queries //
    //////////////////////

    void get_nearest_facet_hint(const Eigen::MatrixXd &V,
                                const Eigen::MatrixXi &F,
                                const Eigen::Vector3d &query,
                                int &nearest_f,
                                Eigen::Vector3d &nearest_point,
                                double &sq_dist) const;

    double squared_distance(const Eigen::MatrixXd &V,
                            const Eigen::MatrixXi &F,
                            const Eigen::Vector3d &query,
                            int &nearest_facet,
                            Eigen::Vector3d &nearest_point) const;

    double squared_distance_with_hint(const Eigen::MatrixXd &V,
                                      const Eigen::MatrixXi &F,
                                      const Eigen::Vector3d &query,
                                      int &nearest_facet,
                                      Eigen::Vector3d &nearest_point,
                                      double &sq_dist) const;

protected:
    void squared_distance_recursive(const Eigen::MatrixXd &V,
                                    const Eigen::MatrixXi &F,
                                    const Eigen::Vector3d &query,
                                    int &nearest_facet,
                                    Eigen::Vector3d &nearest_point,
                                    double &sq_dist,
                                    int node_index) const;
};
