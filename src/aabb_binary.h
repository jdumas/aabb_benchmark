#pragma once

#include <Eigen/Geometry>

struct AABBTreeBinary {
protected:
    std::vector<Eigen::AlignedBox3d> boxes_;

public:
    AABBTreeBinary() = default;  // Default empty constructor

    // Build a BVH from an existing mesh
    AABBTreeBinary(const Eigen::MatrixXd &V,
                   Eigen::MatrixXi &F,
                   bool sort = true);

    // Computes the number of internal + leaf nodes, given the number of leaves
    static int number_of_nodes(int num_leaves);

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
                                    int node_index,
                                    int b,
                                    int e) const;
};
