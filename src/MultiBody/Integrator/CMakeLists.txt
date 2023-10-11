# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#

add_subdirectory(EulerForward)

add_library(Integrator INTERFACE)

target_link_libraries(Integrator INTERFACE EulerForward Eigen3::Eigen)