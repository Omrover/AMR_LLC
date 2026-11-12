# tlaloc_llc: controlador de bajo nivel del Mobile Manipulator (Zephyr 4.5).
#   west build -b tlaloc AMR_LLC/firmware/tlaloc_llc        (hardware)
#   west build -b native_sim/native/64 AMR_LLC/firmware/tlaloc_llc   (SIL)
cmake_minimum_required(VERSION 3.20.0)

# Board out-of-tree: AMR_LLC/firmware/boards/omrover/tlaloc
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tlaloc_llc)

set(LIB ${CMAKE_CURRENT_SOURCE_DIR}/../lib)
if(DEFINED ENV{AMR_ODOM_DIR})
  set(ODOM $ENV{AMR_ODOM_DIR})
else()
  set(ODOM ${CMAKE_CURRENT_SOURCE_DIR}/../../../AMR_localization/amr_odom)
endif()

target_include_directories(app PRIVATE
  src ${LIB}/amr_contract ${LIB}/amr_drive ${LIB}/roboclaw ${LIB}/bno08x ${LIB}/gim6010 ${ODOM})

target_sources(app PRIVATE
  src/main.c
  src/app_config.c
  src/llc.c
  src/gripper.c
  ${LIB}/amr_contract/amr_contract.c
  ${LIB}/amr_drive/amr_drive.c
  ${LIB}/roboclaw/roboclaw.c
  ${LIB}/bno08x/bno08x.c
  ${LIB}/gim6010/gim6010.c
  ${LIB}/gim6010/gim_arm.c
  ${ODOM}/omni.c
  ${ODOM}/ekf.c
)

if(CONFIG_ARCH_POSIX)
  target_sources(app PRIVATE
    src/hal_sim.c
    ${LIB}/roboclaw/roboclaw_sim.c
    ${LIB}/bno08x/bno08x_sim.c
    ${LIB}/gim6010/gim6010_sim.c)
else()
  target_sources(app PRIVATE src/hal_tlaloc.c)
endif()
