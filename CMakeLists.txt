cmake_minimum_required(VERSION 3.20)
project(propwash LANGUAGES C CXX)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

add_subdirectory(extern)

# ---------------------------------------------------------------- pw_sim
# The simulation core: physics + betaflight glue. GPL-3.0.
add_library(pw_sim STATIC
  core/sim/bf.cpp
  core/sim/sim.cpp
  core/sim/physics.cpp
  core/sim/util/SimplexNoise.cpp
)
target_include_directories(pw_sim PUBLIC core core/sim)
target_link_libraries(pw_sim PUBLIC bf_sitl)
target_compile_options(pw_sim PRIVATE -Wall -Wextra -Wno-unused-parameter)

# ---------------------------------------------------------- propwash-core
add_executable(propwash-core core/main.cpp)
target_link_libraries(propwash-core PRIVATE pw_sim ${BF_LINK_OPTIONS})

# ---------------------------------------------------------------- tester
add_executable(pw-tester tools/tester/main.cpp)
target_link_libraries(pw-tester PRIVATE pw_sim ${BF_LINK_OPTIONS})

enable_testing()
add_test(NAME headless_scenario COMMAND pw-tester)
