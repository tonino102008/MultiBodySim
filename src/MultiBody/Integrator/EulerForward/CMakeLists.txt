# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#

set(EULERFORWARD_SOURCES EulerForward.cpp)

# Aggiungere l'origine all'eseguibile di questo progetto.
add_library(EulerForward ${EULERFORWARD_SOURCES})

target_link_libraries(EulerForward RigidBody External Constraints TimeSim Eigen3::Eigen)