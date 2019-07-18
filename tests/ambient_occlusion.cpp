#include "ambient_occlusion.h"
#include "utils.h"
#include <igl/Timer.h>
#include <igl/avg_edge_length.h>
#include <igl/embree/ambient_occlusion.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/per_vertex_normals.h>
#include <igl/read_triangle_mesh.h>
#include <igl/embree/EmbreeIntersector.h>
#include <geogram/mesh/mesh_AABB.h>
#include <CLI/CLI.hpp>
#include <iostream>
#undef IGL_STATIC_LIBRARY  // Missing template instantiation in libigl, temporary fix
#include <igl/ambient_occlusion.h>

// Mesh
Eigen::MatrixXd V;
Eigen::MatrixXi F;
Eigen::VectorXd AO;

// It allows to change the degree of the field when a number is pressed
bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier)
{
    using namespace Eigen;
    using namespace std;
    const RowVector3d color(0.9, 0.85, 0.9);
    switch (key) {
        case '1':
            // Show the mesh without the ambient occlusion factor
            viewer.data().set_colors(color);
            break;
        case '2': {
            // Show the mesh with the ambient occlusion factor
            MatrixXd C = color.replicate(V.rows(), 1);
            for (unsigned i = 0; i < C.rows(); ++i)
                C.row(i) *= AO(i);  // std::min<double>(AO(i)+0.2,1);
            viewer.data().set_colors(C);
            break;
        }
        case '.':
            viewer.core().lighting_factor += 0.1;
            break;
        case ',':
            viewer.core().lighting_factor -= 0.1;
            break;
        default:
            break;
    }
    viewer.core().lighting_factor = std::min(std::max(viewer.core().lighting_factor, 0.f), 1.f);

    return false;
}

template <Method M>
void run(Eigen::MatrixXd& V, Eigen::MatrixXi& F, int samples, Eigen::VectorXd& AO)
{
    using TreeType = typename TreeTraits<M>::type;
    TreeType tree;
    GEO::Mesh mesh;
    tree_from_mesh<M>(V, F, mesh, tree);

    Eigen::MatrixXd N;
    igl::per_vertex_normals(V, F, N);

    igl::Timer timer;
    timer.start();
    ambient_occlusion(V, F, tree, V, N, samples, AO);
    timer.stop();
    double t = timer.getElapsedTime();
    AO = 1.0 - AO.array();
    std::cout << "-- Took " << t << " s" << std::endl;
}

int main(int argc, char* argv[])
{
    // Default arguments
    struct {
        std::string input = TUTORIAL_SHARED_PATH "/fertility.off";
        Method method = Method::Igl;
        int samples = 500;
    } args;

    // Parse arguments
    CLI::App app{"ambient_occlusion"};
    app.add_option("input,-i,--input", args.input, "Input mesh.")->check(CLI::ExistingFile);
    app.add_option("-s,--samples", args.samples, "Number of samples.")->check(CLI::PositiveNumber);
    app.add_option("-m,--method", args.method, "Acceleration method")
        ->transform(CLI::CheckedTransformer(map, CLI::ignore_case));
    try {
        app.parse(argc, argv);
    }
    catch (const CLI::ParseError& e) {
        return app.exit(e);
    }

    std::cout << "Press 1 to turn off Ambient Occlusion" << std::endl
              << "Press 2 to turn on Ambient Occlusion" << std::endl
              << "Press . to turn up lighting" << std::endl
              << "Press , to turn down lighting" << std::endl;

    // Load a triangle mesh
    igl::read_triangle_mesh(args.input, V, F);

    // Compute AO
    // clang-format off
    switch (args.method) {
        case Method::Igl: run<Method::Igl>(V, F, args.samples, AO); break;
        case Method::Embree: run<Method::Embree>(V, F, args.samples, AO); break;
        case Method::Geogram: run<Method::Geogram>(V, F, args.samples, AO); break;
        case Method::Ours: run<Method::Ours>(V, F, args.samples, AO); break;
        case Method::Morton: run<Method::Morton>(V, F, args.samples, AO); break;
        case Method::Hilbert: run<Method::Hilbert>(V, F, args.samples, AO); break;
        case Method::OursBinary: run<Method::OursBinary>(V, F, args.samples, AO); break;
        case Method::MortonBinary: run<Method::MortonBinary>(V, F, args.samples, AO); break;
        case Method::HilbertBinary: run<Method::HilbertBinary>(V, F, args.samples, AO); break;
    }
    // clang-format on

    // Show mesh
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.callback_key_down = &key_down;
    key_down(viewer, '2', 0);
    viewer.data().show_lines = false;
    viewer.core().lighting_factor = 0.0f;
    viewer.launch();
}
