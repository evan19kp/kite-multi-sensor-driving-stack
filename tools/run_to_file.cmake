execute_process(COMMAND ${EXE}
  RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
message("${out}")
if(NOT rc STREQUAL "0")
  message(FATAL_ERROR "exit code ${rc}, expected 0: ${err}")
endif()
string(REPLACE "\r\n" "\n" out "${out}")
file(CONFIGURE OUTPUT ${OUT} CONTENT "${out}" @ONLY NEWLINE_STYLE UNIX)
