# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#

set(RIGIDBODY_SOURCES RigidBody.cpp)

# Aggiungere l'origine all'eseguibile di questo progetto.
add_library(RigidBody ${RIGIDBODY_SOURCES})

target_link_libraries(RigidBody Quaternions Eigen3::Eigen)