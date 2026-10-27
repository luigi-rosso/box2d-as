# Box2D as a dependency, so its top level pthread flags, samples and heavy validation stay off. With SIMD on,
# Box2D's CMake adds -msimd128 -msse2 and core.h maps wasm onto the SSE2 path.
cmake_minimum_required(VERSION 3.22)
project(box2d_web C CXX)

set(BOX2D_SOURCE "$ENV{HOME}/Projects/box2d" CACHE PATH "Upstream Box2D checkout")

add_subdirectory(${BOX2D_SOURCE} box2d EXCLUDE_FROM_ALL)

set(SCENES "${CMAKE_CURRENT_SOURCE_DIR}/../../../verify/scenes")
add_executable(box2d_web driver.c ${SCENES}/scenes.c)
target_include_directories(box2d_web PRIVATE ${SCENES})
target_compile_options(box2d_web PRIVATE -ffp-contract=off)
target_link_libraries(box2d_web PRIVATE box2d)
target_link_options(box2d_web PRIVATE
	-sMODULARIZE=1 -sEXPORT_ES6=1 -sEXPORT_NAME=Box2D
	-sALLOW_MEMORY_GROWTH=1 -sINITIAL_MEMORY=64MB -sMAXIMUM_MEMORY=4GB
	-sSTACK_SIZE=4MB
	-sEXPORTED_RUNTIME_METHODS=HEAPU8
	-sENVIRONMENT=node,web,worker)
