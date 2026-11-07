cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(s32k344_wheel_node)

target_sources(app PRIVATE src/main.c src/lab_logic.c)

if(CONFIG_LAB_USE_SOLUTION)
  target_sources(app PRIVATE src/student_solution.c)
else()
  target_sources(app PRIVATE src/student.c)
endif()

if(CONFIG_ARCH_POSIX)
  target_sources(app PRIVATE src/hal_sim.c)
else()
  target_sources(app PRIVATE src/hal_s32k344.c)
endif()
