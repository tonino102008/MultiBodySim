# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#
cmake_minimum_required (VERSION 3.8)

# Enable Hot Reload for MSVC compilers if supported.
if(POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

# Set project name
project ("MultiBodySim")

option(BUILD_TESTS "Build test programs" OFF)

# C++17 Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(LIBS Quaternions)
set(LIBS_DIR lib)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)

if (WIN32 OR MSVC)
  #add_compile_options(/fopenmp /O2 /fp:fast /arch:AVX2)
endif (WIN32 OR MSVC)
if (UNIX)
  #add_compile_options(-O2 -Ofast -march=native)
  #add_compile_definitions(EIGEN_DONT_PARALLELIZE)
endif (UNIX)

foreach(LIBRARY ${LIBS})
    include_directories("${LIBS_DIR}/${LIBS}/include")
endforeach(LIBRARY)
include_directories(include)

add_subdirectory(lib)
add_subdirectory(src)

if(BUILD_TESTS)
	enable_testing()
	add_subdirectory(tests)
endif(BUILD_TESTS)