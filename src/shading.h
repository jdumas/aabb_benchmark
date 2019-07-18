#pragma once

#include <Eigen/Dense>

typedef std::function<double(const Eigen::Vector3d &, const Eigen::Vector3d &)> ShootRayFunc;

struct Material {
    Eigen::Vector3d ambient = Eigen::Vector3d(0, 0.5, 0);
    Eigen::Vector3d diffuse = Eigen::Vector3d(0.5, 0.5, 0.5);
    Eigen::Vector3d specular = Eigen::Vector3d(0.2, 0.2, 0.2);
    double shininess = 256.0;  // specular exponent
};

///
/// Determine the color of a surface point as seen by an input ray, given the surface material and
/// description of point lights in the scene.
///
/// @param[in]  ray_origin         Ray origin.
/// @param[in]  ray_direction      Ray direction (assumed to be normalized).
/// @param[in]  hit_position       Surface point position.
/// @param[in]  hit_normal         Surface point normal.
/// @param[in]  light_positions    #L x 3 matrix of light positions.
/// @param[in]  light_intensities  (1|#L) x 3 matrix of light intensities in RGB.
/// @param[in]  light_ambient      Ambient light color.
/// @param[in]  material           Material parameters.
/// @param[in]  shoot_ray          Optional shoot_ray function for casting shadow rays.
/// @param[in]  blinn_phong        Whether to use Blinn-Phon (true) or Phong model for specular
///                                component.
///
/// @return     Shaded color of the surface point.
///
Eigen::Vector3d shading(const Eigen::Vector3d &ray_origin,
                        const Eigen::Vector3d &ray_direction,
                        const Eigen::Vector3d &hit_position,
                        const Eigen::Vector3d &hit_normal,
                        const Eigen::MatrixXd &light_positions,
                        const Eigen::MatrixXd &light_intensities = Eigen::RowVector3d(16, 16, 16),
                        const Eigen::Vector3d &light_ambient = {0.2, 0.2, 0.2},
                        const Material &material = {},
                        ShootRayFunc shoot_ray = nullptr,
                        bool blinn_phong = true);
