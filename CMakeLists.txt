project(MPC)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++11 -O3)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/MPC.cpp src/main.cpp src/logging.cpp src/acado.cpp)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)
include_directories(src/Eigen-3.3)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(mpc ${sources})
# add external mpc library from acado generator
add_library(ACADO STATIC IMPORTED)
set_target_properties(ACADO PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/acado_mpc_export/libacado_exported_rti.a")
# target_link_libraries(${PROJECT_NAME} ACADO)

target_link_libraries(mpc ipopt z ssl uv uWS ACADO)
