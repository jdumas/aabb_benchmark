////////////////////////////////////////////////////////////////////////////////
#include "mesh_reorder.h"
#include "utils.h"
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_reorder.h>
////////////////////////////////////////////////////////////////////////////////

void mesh_reorder(Eigen::MatrixXd &V, Eigen::VectorXi &IV, bool morton)
{
    init_geogram();

    GEO::Mesh mesh;
    Eigen::MatrixXi F(0, 3);
    to_geogram_mesh(V, F, mesh);

    // Reordered to original vertex id
    GEO::Attribute<int> indices(mesh.vertices.attributes(), "id");
    for (int i = 0; i < mesh.vertices.nb(); ++i) {
        indices[i] = i;
    }

    if (mesh.vertices.nb() > 0) {
        GEO::mesh_reorder(mesh, morton ? GEO::MESH_ORDER_MORTON : GEO::MESH_ORDER_HILBERT);
    }

    from_geogram_mesh(mesh, V, F);

    IV.resize(V.rows());
    for (int i = 0; i < mesh.vertices.nb(); ++i) {
        IV(i) = indices[i];
    }
}

void mesh_reorder(
    Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::VectorXi &IV, Eigen::VectorXi &IF, bool morton)
{
    init_geogram();

    GEO::Mesh mesh;
    to_geogram_mesh(V, F, mesh);

    // Reordered to original vertex id
    GEO::Attribute<int> vid(mesh.vertices.attributes(), "vid");
    GEO::Attribute<int> fid(mesh.facets.attributes(), "fid");
    for (int i = 0; i < mesh.vertices.nb(); ++i) {
        vid[i] = i;
    }
    for (int i = 0; i < mesh.facets.nb(); ++i) {
        fid[i] = i;
    }

    if (mesh.vertices.nb() > 0) {
        GEO::mesh_reorder(mesh, morton ? GEO::MESH_ORDER_MORTON : GEO::MESH_ORDER_HILBERT);
    }

    from_geogram_mesh(mesh, V, F);

    IV.resize(V.rows());
    for (int i = 0; i < mesh.vertices.nb(); ++i) {
        IV(i) = vid[i];
    }
    IF.resize(F.rows());
    for (int i = 0; i < mesh.facets.nb(); ++i) {
        IF(i) = fid[i];
    }
}
