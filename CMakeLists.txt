cmake_minimum_required(VERSION 3.16)
project(dog_sim)

set(CMAKE_CXX_STANDARD 17)

set(MUJOCO_PATH "$ENV{HOME}/mujoco-3.14.0")

include_directories(${MUJOCO_PATH}/include)
link_directories(${MUJOCO_PATH}/lib)

find_package(glfw3 REQUIRED)

add_executable(dog_sim main.cpp)
target_link_libraries(dog_sim
    mujoco
    glfw
    pthread
    dl
)
