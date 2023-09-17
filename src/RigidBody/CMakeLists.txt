# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#

set(RIGIDBODY_SOURCES RigidBody.cpp)

add_subdirectory(Integrators)
add_subdirectory(Quaternions)
#add_subdirectory(VectorN)
add_subdirectory(MatrixN)

# Aggiungere l'origine all'eseguibile di questo progetto.
add_library(RigidBody ${RIGIDBODY_SOURCES})

target_link_libraries(RigidBody Integrators Quaternions MatrixN) # VectorN)