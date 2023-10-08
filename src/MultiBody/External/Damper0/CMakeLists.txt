# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#

set(DAMPER0_SOURCES Damper0.cpp)

# Aggiungere l'origine all'eseguibile di questo progetto.
add_library(Damper0 ${DAMPER0_SOURCES})

target_link_libraries(Damper0 RigidBody Eigen3::Eigen)