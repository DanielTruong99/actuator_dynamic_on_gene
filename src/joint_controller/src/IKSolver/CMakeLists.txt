add_library(ik_solver STATIC IKSolver.cpp IKSolver.h)

target_include_directories(ik_solver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(ik_solver pinocchio::pinocchio Eigen3::Eigen)
