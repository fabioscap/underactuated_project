cmake_minimum_required(VERSION 2.8.12)

project(isMPC_hpipm)

find_package(DART 6.1.1 REQUIRED COMPONENTS utils-urdf gui-osg)

add_compile_options(-std=c++17)
add_compile_options(-O3)

include_directories(${DART_INCLUDE_DIRS})

add_subdirectory(labrob_qpsolvers)

file(GLOB srcs "*.cpp" "*.hpp" "*.h" "qpOASES/*.cpp" "*qpOASES/.hpp" "hpipm/include/*.h" "blasfeo/include/*.h")

add_executable(${PROJECT_NAME} ${srcs})

target_link_libraries(${PROJECT_NAME} labrob_qpsolvers ${DART_LIBRARIES})
