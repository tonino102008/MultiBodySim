# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#

set(INTEGRATORS_SOURCES Integrator.cpp)

add_subdirectory(EulerForward)

# Aggiungere l'origine all'eseguibile di questo progetto.
add_library(Integrators ${INTEGRATORS_SOURCES})

target_link_libraries(Integrators EulerForward Eigen3::Eigen)