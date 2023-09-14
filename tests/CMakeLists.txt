# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#

set(TEST_NAME Test1)
set(TEST_SOURCE Test1.cpp)

add_executable(${TEST_NAME} ${TEST_SOURCE})

target_link_libraries(${TEST_NAME} PUBLIC
	gtest_main
	Quaternions
	Integrators
)

add_test(
	NAME ${TEST_NAME}
	COMMAND ${TEST_NAME}
)