////////////////////////////////////////////////////////////////////////////////
#include "mesh_reorder.h"
#include <geogram/basic/command_line.h>
#include <geogram/basic/command_line_args.h>
#include <geogram/basic/logger.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_reorder.h>
////////////////////////////////////////////////////////////////////////////////

namespace {

#ifdef WIN32
int setenv(const char *name, const char *value, int overwrite)
{
    int errcode = 0;
    if (!overwrite) {
        size_t envsize = 0;
        errcode = getenv_s(&envsize, NULL, 0, name);
        if (errcode || envsize) return errcode;
    }
    return _putenv_s(name, value);
}
#endif

}  // namespace

////////////////////////////////////////////////////////////////////////////////

// Initialize geogram library + clean up a bit its defaults behavior
void init_geogram()
{
    static bool first_time = true;

    if (first_time) {
        first_time = false;
    }
    else {
        return;
    }

    // Do not install custom signal handlers
    setenv("GEO_NO_SIGNAL_HANDLER", "1", 1);

    // Init logger first so we can hide geogram output from init
    GEO::Logger::initialize();

    // Do not show geogram output
    GEO::Logger::instance()->unregister_all_clients();
    GEO::Logger::instance()->set_quiet(true);

#if 0
    // Use the following code to disable multi-threading in geogram (for debugging purposes).
    GEO::Process::enable_multithreading(false);
    GEO::Process::set_max_threads(1);
#endif

    // Initialize global instances (e.g., logger), register MeshIOHandler, etc.
    GEO::initialize(GEO::GEOGRAM_NO_HANDLER);

    // Import standard command line arguments, and custom ones
    GEO::CmdLine::import_arg_group("standard");
    GEO::CmdLine::import_arg_group("pre");
    GEO::CmdLine::import_arg_group("algo");
    // GEO::CmdLine::import_arg_group("sys");
    GEO::CmdLine::set_arg("sys:assert", "throw");
}

////////////////////////////////////////////////////////////////////////////////

void to_geogram_mesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, GEO::Mesh &M)
{
    M.clear();
    // Setup vertices
    M.vertices.create_vertices((int)V.rows());
    for (int i = 0; i < (int)M.vertices.nb(); ++i) {
        GEO::vec3 &p = M.vertices.point(i);
        p[0] = V(i, 0);
        p[1] = V(i, 1);
        p[2] = V.cols() >= 3 ? V(i, 2) : 0;
    }
    // Setup faces
    if (F.rows() == 0) {
        // Nothing to do
    }
    else if (F.cols() == 3) {
        M.facets.create_triangles((int)F.rows());
    }
    else if (F.cols() == 4) {
        M.facets.create_quads((int)F.rows());
    }
    else {
        throw std::runtime_error("Mesh format not supported");
    }
    for (int c = 0; c < (int)M.facets.nb(); ++c) {
        for (int lv = 0; lv < F.cols(); ++lv) {
            M.facets.set_vertex(c, lv, F(c, lv));
        }
    }
}

void from_geogram_mesh(const GEO::Mesh &M, Eigen::MatrixXd &V, Eigen::MatrixXi &F)
{
    V.resize(M.vertices.nb(), 3);
    for (int i = 0; i < (int)M.vertices.nb(); ++i) {
        GEO::vec3 p = M.vertices.point(i);
        V.row(i) << p[0], p[1], p[2];
    }
    assert(M.facets.are_simplices());
    F.resize(M.facets.nb(), 3);
    for (int c = 0; c < (int)M.facets.nb(); ++c) {
        for (int lv = 0; lv < 3; ++lv) {
            F(c, lv) = M.facets.vertex(c, lv);
        }
    }
}
