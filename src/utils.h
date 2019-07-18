#pragma once

////////////////////////////////////////////////////////////////////////////////
#include "mesh_reorder.h"
#include "aabb.h"
#include "aabb_binary.h"
#include <igl/AABB.h>
#include <igl/embree/EmbreeIntersector.h>
#include <geogram/mesh/mesh_AABB.h>
#include <vector>
#include <string>
////////////////////////////////////////////////////////////////////////////////

///
/// Initialize the geogram library.
///
void init_geogram();

///
/// Converts a triangle mesh to a Geogram mesh.
///
/// @param[in]  V     #V x 3 input mesh vertices.
/// @param[in]  F     #F x 3 input mesh faces.
/// @param[out] M     Output Geogram mesh.
///
void to_geogram_mesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, GEO::Mesh &M);

///
/// Converts a Geogram mesh into matrix format.
///
/// @param[in]  M     Input Geogram mesh.
/// @param[out] V     #V x 3 output mesh vertices.
/// @param[out] F     #F x 3 output mesh faces.
///
void from_geogram_mesh(const GEO::Mesh &M, Eigen::MatrixXd &V, Eigen::MatrixXi &F);

////////////////////////////////////////////////////////////////////////////////

enum class Method : int {
    Igl,
    Embree,
    Geogram,
    Ours,
    Morton,
    Hilbert,
    OursBinary,
    MortonBinary,
    HilbertBinary
};

// clang-format off
inline std::vector<std::pair<std::string, Method>> map{
    {"igl", Method::Igl},
    {"embree", Method::Embree},
    {"geogram", Method::Geogram},
    {"ours", Method::Ours},
    {"morton", Method::Morton},
    {"hilbert", Method::Hilbert},
    {"ours_binary", Method::OursBinary},
    {"morton_binary", Method::MortonBinary},
    {"hilbert_binary", Method::HilbertBinary},
};
template<Method> struct TreeTraits;
template<> struct TreeTraits<Method::Igl> { using type = igl::AABB<Eigen::MatrixXd, 3>; };
template<> struct TreeTraits<Method::Embree> { using type = igl::embree::EmbreeIntersector; };
template<> struct TreeTraits<Method::Geogram> { using type = GEO::MeshFacetsAABB; };
template<> struct TreeTraits<Method::Ours> { using type = AABBTree; };
template<> struct TreeTraits<Method::Morton> { using type = AABBTree; };
template<> struct TreeTraits<Method::Hilbert> { using type = AABBTree; };
template<> struct TreeTraits<Method::OursBinary> { using type = AABBTreeBinary; };
template<> struct TreeTraits<Method::MortonBinary> { using type = AABBTreeBinary; };
template<> struct TreeTraits<Method::HilbertBinary> { using type = AABBTreeBinary; };
// clang-format on

// Method --> name
inline std::string method_name(Method method) {
    for (const auto &kv : map) {
        if (kv.second == method) { return kv.first; }
    }
    return "";
}

// Populate a tree given a mesh (V, E)
template <Method M, typename TreeType = typename TreeTraits<M>::type>
void tree_from_mesh(Eigen::MatrixXd &V, Eigen::MatrixXi &F, GEO::Mesh &mesh, TreeType &tree)
{
    if constexpr (M == Method::Igl) {
        tree.init(V, F);
    }
    else if constexpr (M == Method::Embree) {
        tree.init(V.cast<float>(), F.cast<int>());
    }
    else if constexpr (M == Method::Geogram) {
        init_geogram();
        to_geogram_mesh(V, F, mesh);
        tree = GEO::MeshFacetsAABB(mesh);
    }
    else if constexpr (M == Method::Ours) {
        tree = AABBTree(V, F, true);
    }
    else if constexpr (M == Method::Morton || M == Method::Hilbert) {
        Eigen::VectorXi IV, IF;
        mesh_reorder(V, F, IV, IF, M == Method::Morton);
        tree = AABBTree(V, F, false);
    }
    else if constexpr (M == Method::OursBinary) {
        tree = AABBTreeBinary(V, F, true);
    }
    else if constexpr (M == Method::MortonBinary || M == Method::HilbertBinary) {
        Eigen::VectorXi IV, IF;
        mesh_reorder(V, F, IV, IF, M == Method::Morton);
        tree = AABBTreeBinary(V, F, false);
    }
}
