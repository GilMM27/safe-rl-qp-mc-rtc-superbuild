option(WITH_HUMAN_MOCAP "Install the tagged MS-Human-700 MuJoCo asset" OFF)

if(NOT WITH_HUMAN_MOCAP)
  return()
endif()

include(FetchContent)

set(HUMAN_MOCAP_MENAGERIE_COMMIT 4d038b3feae26ec82b46a4d586379114012a8ac7)
set(HUMAN_MOCAP_DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mc_mujoco/human_mocap")

FetchContent_Declare(
  mujoco_menagerie_human
  GIT_REPOSITORY https://github.com/google-deepmind/mujoco_menagerie.git
  GIT_TAG "${HUMAN_MOCAP_MENAGERIE_COMMIT}"
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(mujoco_menagerie_human)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(HUMAN_MOCAP_SOURCE_ROOT "${mujoco_menagerie_human_SOURCE_DIR}/ms_human_700")
set(HUMAN_MOCAP_FLATTENED_MODEL "${CMAKE_CURRENT_BINARY_DIR}/MS-Human-700-Locomotion.xml")
execute_process(
  COMMAND
    "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_LIST_DIR}/../tools/flatten_mjcf.py"
    --input "${HUMAN_MOCAP_SOURCE_ROOT}/MS-Human-700-Locomotion.xml"
    --output "${HUMAN_MOCAP_FLATTENED_MODEL}"
    --source-root "${HUMAN_MOCAP_SOURCE_ROOT}"
    --install-root "${HUMAN_MOCAP_DESTINATION}"
  COMMAND_ERROR_IS_FATAL ANY
)

configure_file(
  "${CMAKE_CURRENT_LIST_DIR}/human_mocap.in.yaml"
  "${CMAKE_CURRENT_BINARY_DIR}/human_mocap.yaml"
  @ONLY
)

install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/human_mocap.yaml"
  DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mc_mujoco"
)

install(
  DIRECTORY "${mujoco_menagerie_human_SOURCE_DIR}/ms_human_700/"
  DESTINATION "${HUMAN_MOCAP_DESTINATION}"
)

install(
  FILES "${HUMAN_MOCAP_FLATTENED_MODEL}"
  DESTINATION "${HUMAN_MOCAP_DESTINATION}"
)

install(
  FILES "${CMAKE_CURRENT_LIST_DIR}/human_mocap_parts.yaml"
  DESTINATION "${HUMAN_MOCAP_DESTINATION}"
)
