cmake_minimum_required(VERSION 3.11)
project(aabb_benchmark)

# CMP0063: Honor visibility properties for all target types.
if(POLICY CMP0063)
	cmake_policy(SET CMP0063 NEW)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
include(External)
include(UseColors)

gazebo_import(geogram tbb libigl cli11 json eigen)

# Data folder
set(DATA_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/data)

# Main lib
add_library(benchlib
	src/aabb.cpp
	src/aabb_binary.cpp
	src/ambient_occlusion.cpp
	src/distances.cpp
	src/intersect_ray.cpp
	src/mesh_reorder.cpp
	src/squared_distances.cpp
	src/shading.cpp
	src/utils.cpp
)

target_link_libraries(benchlib PUBLIC
	CLI11::CLI11
	geogram::geogram
	nlohmann_json::nlohmann_json
	igl::core
	igl::embree
	igl::opengl_glfw
	igl::paths
	igl::png
	tbb::tbb
)

target_include_directories(benchlib PUBLIC src)

target_compile_features(benchlib PUBLIC cxx_std_17)
target_compile_definitions(benchlib PUBLIC "-DDATA_FOLDER=\"${DATA_FOLDER}\"")

# Individual test apps
add_executable(ray_tracing tests/ray_tracing.cpp)
target_link_libraries(ray_tracing benchlib)

add_executable(ambient_occlusion tests/ambient_occlusion.cpp)
target_link_libraries(ambient_occlusion benchlib)

add_executable(distance_to_mesh tests/distance_to_mesh.cpp)
target_link_libraries(distance_to_mesh benchlib)
