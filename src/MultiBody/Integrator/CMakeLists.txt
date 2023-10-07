# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#

set(INTEGRATOR_SOURCES Integrator.cpp)

add_subdirectory(EulerForward)

# Aggiungere l'origine all'eseguibile di questo progetto.
add_library(Integrator ${INTEGRATOR_SOURCES})

target_link_libraries(Integrator EulerForward Eigen3::Eigen)