# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#

set(SPRING0_SOURCES Spring0.cpp)

# Aggiungere l'origine all'eseguibile di questo progetto.
add_library(Spring0 ${SPRING0_SOURCES})

target_link_libraries(Spring0 RigidBody Eigen3::Eigen)