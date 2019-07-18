#include "shading.h"
#include <cmath>

Eigen::Vector3d shading(const Eigen::Vector3d &ray_origin,
                        const Eigen::Vector3d &ray_direction,
                        const Eigen::Vector3d &hit_position,
                        const Eigen::Vector3d &hit_normal,
                        const Eigen::MatrixXd &light_positions,
                        const Eigen::MatrixXd &light_intensities,
                        const Eigen::Vector3d &light_ambient,
                        const Material &material,
                        ShootRayFunc shoot_ray,
                        bool blinn_phong)
{
    // Ambient light contribution
    Eigen::Vector3d ambient = material.ambient.array() * light_ambient.array();

    // Punctual lights contribution (direct lighting)
    Eigen::Vector3d lights_color(0, 0, 0);
    for (int i = 0; i < light_positions.rows(); ++i) {
        Eigen::Vector3d light_position = light_positions.row(i).transpose();
        Eigen::Vector3d Li = (light_position - hit_position).normalized();
        Eigen::Vector3d N = hit_normal;

        // Shoot a shadow ray to determine if the light should affect the
        // intersection point
        if (shoot_ray) {
            double t = shoot_ray(hit_position + 1e-6 * Li, Li);
            if (std::isfinite(t)) {
                double dist = (ray_origin - light_position).norm();
                if (t < dist) {
                    // Light is occluded, skip
                    continue;
                }
            }
        }

        // Diffuse contribution
        Eigen::Vector3d diffuse = material.diffuse * std::max(Li.dot(N), 0.0);

        // Specular contribution
        Eigen::Vector3d specular;
        if (blinn_phong) {
            // Blinn-Phong model
            Eigen::Vector3d Hi = (Li - ray_direction).normalized();
            specular = material.specular * std::pow(std::max(N.dot(Hi), 0.0), material.shininess);
        }
        else {
            // Phong model (the shininess is adjusted to give approximatively similar results)
            Eigen::Vector3d R = (Li - 2.0 * Li.dot(hit_normal) * hit_normal).normalized();
            specular = material.specular *
                       std::pow(std::max(R.dot(ray_direction), 0.0), material.shininess / 4);
        }

        // Attenuate lights according to the squared distance to the lights
        Eigen::Vector3d D = light_position - hit_position;
        Eigen::Vector3d intensity =
            light_intensities.row(std::min(i, (int)light_intensities.rows() - 1));
        lights_color += (diffuse + specular).cwiseProduct(intensity) / D.squaredNorm();
    }

    // Rendering equation
    Eigen::Vector3d C = ambient + lights_color;

    return C;
}
