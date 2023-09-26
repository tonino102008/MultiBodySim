# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#

set(QUATERNIONS_SOURCES Quaternion.cpp)

# Aggiungere l'origine all'eseguibile di questo progetto.
add_library(Quaternions ${QUATERNIONS_SOURCES})

target_link_libraries(Quaternions Eigen3::Eigen)