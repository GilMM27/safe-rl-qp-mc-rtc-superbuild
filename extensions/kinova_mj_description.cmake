if(NOT WITH_Kinova)
  return()
endif()

include(FetchContent)

set(KINOVA_MJ_DESCRIPTION_COMMIT c96a32d28fb5da84da38c1da4d749e7a13212855)
set(KINOVA_MJ_DESCRIPTION_DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mc_mujoco/kinova_gen3")

FetchContent_Declare(
  mujoco_menagerie
  GIT_REPOSITORY https://github.com/google-deepmind/mujoco_menagerie.git
  GIT_TAG "${KINOVA_MJ_DESCRIPTION_COMMIT}"
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(mujoco_menagerie)

configure_file(
  "${CMAKE_CURRENT_LIST_DIR}/kinova_mj_description.in.yaml"
  "${CMAKE_CURRENT_BINARY_DIR}/kinova.yaml"
  @ONLY
)

install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/kinova.yaml"
  DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mc_mujoco"
)

install(
  DIRECTORY "${mujoco_menagerie_SOURCE_DIR}/kinova_gen3/"
  DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mc_mujoco/kinova_gen3"
)
