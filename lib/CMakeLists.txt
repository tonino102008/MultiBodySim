# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#

add_subdirectory(Quaternions)
add_subdirectory(TimeSim)

if(BUILD_TESTS)
	set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
	add_subdirectory(googletest)
endif(BUILD_TESTS)