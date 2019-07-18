////////////////////////////////////////////////////////////////////////////////
#include "squared_distances.h"
#include "utils.h"
#include <igl/read_triangle_mesh.h>
#include <igl/write_triangle_mesh.h>
#include <igl/png/writePNG.h>
#include <igl/Timer.h>
#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>
#include <limits>
#include <random>
////////////////////////////////////////////////////////////////////////////////

using json = nlohmann::json;

// Default arguments
struct Args {
    std::string input = "mesh.obj";
    int samples = 1000;
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

    // Sample points in the unit cube
    Eigen::MatrixXd P(args.samples, 3);
    std::default_random_engine gen;
    std::uniform_real_distribution<double> dist(0, 1);
    for (int i = 0; i < args.samples; ++i) {
        for (int j = 0; j < 3; ++j) {
            P(i, j) = dist(gen);
        }
    }
    Eigen::VectorXi IP;
    mesh_reorder(P, IP);

    // Query distances
    Eigen::VectorXd S;

    igl::Timer timer;
    timer.start();
    squared_distances(V, F, tree, P, S);
    timer.stop();

    entry["mesh"] = args.input;
    entry["method"] = method_name(M);
    entry["time"] = timer.getElapsedTime();

    return S;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    Args args;

    // Parse arguments
    CLI::App app{"distance_to_mesh"};
    app.add_option("input,-i,--input", args.input, "Input mesh.")->check(CLI::ExistingFile);
    app.add_option("-s,--samples", args.samples, "Number of samples.")
        ->check(CLI::PositiveNumber & !CLI::Range(0, 0));
    app.add_option("-m,--method", args.method, "Acceleration method")
        ->transform(CLI::CheckedTransformer(map, CLI::ignore_case));
    try {
        app.parse(argc, argv);
    }
    catch (const CLI::ParseError &e) {
        return app.exit(e);
    }

    Eigen::VectorXd S;

    json entry;

    // clang-format off
    switch (args.method) {
        case Method::Igl: S = run<Method::Igl>(args, entry); break;
        case Method::Geogram: S = run<Method::Geogram>(args, entry); break;
        case Method::Ours: S = run<Method::Ours>(args, entry); break;
        case Method::Morton: S = run<Method::Morton>(args, entry); break;
        case Method::Hilbert: S = run<Method::Hilbert>(args, entry); break;
        case Method::OursBinary: S = run<Method::OursBinary>(args, entry); break;
        case Method::MortonBinary: S = run<Method::MortonBinary>(args, entry); break;
        case Method::HilbertBinary: S = run<Method::HilbertBinary>(args, entry); break;
        default: throw std::runtime_error("Not implemented"); break;
    }
    // clang-format on

    json dummy;
    Eigen::VectorXd R = run<Method::Geogram>(args, dummy);
    entry["max_error"] = (S - R).cwiseAbs().maxCoeff();

    std::cout << entry.dump(4) << std::endl;

    return 0;
}
