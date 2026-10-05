find_package(Python3 REQUIRED COMPONENTS Interpreter)
execute_process(COMMAND "${Python3_EXECUTABLE}" "${BRIDGE_SCRIPT}" --source "${SOURCE_DIR}"
                RESULT_VARIABLE result)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "Failed to install independent SMPL-H bridge")
endif()
