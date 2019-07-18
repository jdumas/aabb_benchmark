////////////////////////////////////////////////////////////////////////////////
#include "intersect_ray.h"
#include "shading.h"
#include "utils.h"
#include <igl/read_triangle_mesh.h>
#include <igl/write_triangle_mesh.h>
#include <igl/png/writePNG.h>
#include <igl/Timer.h>
#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>
#include <limits>
////////////////////////////////////////////////////////////////////////////////

using json = nlohmann::json;

// Default arguments
struct Args {
    std::string input = "mesh.obj";
    std::string output = "output.png";
    int width = 800;
    int height = 600;
    Method method = Method::Igl;
};

template <Method M>
auto run(const Args &args, json &entry)
{
    // Read input
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::read_triangle_mesh(args.input, V, F);

    // Normalize (x, y) into [-1, 1]^2 + center mesh around the origin
    {
        Eigen::RowVector3d Vmin = V.colwise().minCoeff();
        Eigen::RowVector3d Vmax = V.colwise().maxCoeff();
        double scaling = 0.5 * (Vmax - Vmin).head<2>().maxCoeff();
        V = (V.rowwise() - 0.5 * (Vmax + Vmin)) / scaling;
    }

    using TreeType = typename TreeTraits<M>::type;
    TreeType tree;
    GEO::Mesh mesh;
    tree_from_mesh<M>(V, F, mesh, tree);

    // Shoot-ray function
    auto shoot_ray = [&](const Eigen::Vector3d &origin, const Eigen::Vector3d &direction,
                         Eigen::Vector3d &hit_position, Eigen::Vector3d &hit_normal) {
        double hit_param;
        return aabb_intersect_ray(V, F, tree, origin, direction, hit_position, hit_normal,
                                  hit_param);
    };

    // Scene lights
    // clang-format off
    Eigen::MatrixXd lights(7, 3);
    lights <<
        8, 8, 0,
        6, -8, 0,
        4, 8, 0,
        2, -8, 0,
        0, 8, 0,
        -2, -8, 0,
        -4, 8, 0;
    // clang-format on

    // Pixels to ray transformations
    double aspect_ratio = double(args.width) / double(args.height);
    Eigen::Vector3d grid_origin(-std::max(aspect_ratio, 1.), -std::max(1. / aspect_ratio, 1.), 0.);
    double grid_spacing = 2.0 / std::min(args.width, args.height);

    // Shading
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R(args.width, args.height);
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> G(args.width, args.height);
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> B(args.width, args.height);
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> A(args.width, args.height);

    igl::Timer timer;
    timer.start();
    for (int x = 0; x < args.width; ++x) {
        for (int y = 0; y < args.height; ++y) {
            Eigen::Vector3d ray_direction(0, 0, -1);
            Eigen::Vector3d ray_origin = Eigen::Vector3d(0, 0, 10) + grid_origin +
                                         Eigen::Vector3d(x + 0.5, y + 0.5, 0) * grid_spacing;

            Eigen::Vector3d hit_position, hit_normal;
            if (shoot_ray(ray_origin, ray_direction, hit_position, hit_normal)) {
                Eigen::Vector3d C =
                    shading(ray_origin, ray_direction, hit_position, hit_normal, lights);

                R(x, y) = C(0) * 255;
                G(x, y) = C(1) * 255;
                B(x, y) = C(2) * 255;
                A(x, y) = 255;
            }
            else {
                R(x, y) = G(x, y) = B(x, y) = A(x, y) = 255;
            }
        }
    }
    timer.stop();

    entry["mesh"] = args.input;
    entry["method"] = method_name(M);
    entry["time"] = timer.getElapsedTime();

    // Write output image
    igl::png::writePNG(R, G, B, A, args.output);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    Args args;

    // Parse arguments
    CLI::App app{"ray_tracing"};
    app.add_option("input,-i,--input", args.input, "Input mesh.")->check(CLI::ExistingFile);
    app.add_option("output,-o,--output", args.output, "Output image.");
    app.add_option("-W,--width", args.width, "Image width.")
        ->check(CLI::PositiveNumber & !CLI::Range(0, 0));
    app.add_option("-H,--height", args.height, "Image height.")
        ->check(CLI::PositiveNumber & !CLI::Range(0, 0));
    app.add_option("-m,--method", args.method, "Acceleration method")
        ->transform(CLI::CheckedTransformer(map, CLI::ignore_case));
    try {
        app.parse(argc, argv);
    }
    catch (const CLI::ParseError &e) {
        return app.exit(e);
    }

    json entry;

    // clang-format off
    switch (args.method) {
        case Method::Igl: run<Method::Igl>(args, entry); break;
        case Method::Embree: run<Method::Embree>(args, entry); break;
        case Method::Geogram: run<Method::Geogram>(args, entry); break;
        case Method::Ours: run<Method::Ours>(args, entry); break;
        case Method::Morton: run<Method::Morton>(args, entry); break;
        case Method::Hilbert: run<Method::Hilbert>(args, entry); break;
        case Method::OursBinary: run<Method::OursBinary>(args, entry); break;
        case Method::MortonBinary: run<Method::MortonBinary>(args, entry); break;
        case Method::HilbertBinary: run<Method::HilbertBinary>(args, entry); break;
    }
    // clang-format on

    std::cout << entry.dump(4) << std::endl;

    return 0;
}
