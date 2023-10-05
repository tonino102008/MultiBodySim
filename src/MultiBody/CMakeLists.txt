# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#

set(MULTIBODY_SOURCES MultiBody.cpp)

add_subdirectory(Constraints)
add_subdirectory(RigidBody)
add_subdirectory(External)
add_subdirectory(EulerForward)

# Aggiungere l'origine all'eseguibile di questo progetto.
add_library(MultiBody ${MULTIBODY_SOURCES})

target_link_libraries(MultiBody Constraints RigidBody External TimeSim Eigen3::Eigen)