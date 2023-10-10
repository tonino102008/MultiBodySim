# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#

set(DOXYFILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/../MultiBodySim.doxygen)
set(DOXYFILE ${CMAKE_CURRENT_BINARY_DIR}/doc/MultiBodySim.doxygen)

configure_file(${DOXYFILE_IN} ${DOXYFILE} @ONLY)

message("Doxygen build started.")

add_custom_target(doc ALL
	COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_IN}
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../
	COMMENT "Generating API documentation with Doxygen"
	VERBATIM)