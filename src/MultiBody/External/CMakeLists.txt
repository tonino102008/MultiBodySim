# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#

set(EXTERNAL_SOURCES External.cpp)

add_subdirectory(Damper)
add_subdirectory(Spring)

# Aggiungere l'origine all'eseguibile di questo progetto.
add_library(External ${EXTERNAL_SOURCES})

target_link_libraries(External Damper Spring Eigen3::Eigen)