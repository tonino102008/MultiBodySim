# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#

set(DAMPER_SOURCES Damper.cpp)

# Aggiungere l'origine all'eseguibile di questo progetto.
add_library(Damper ${DAMPER_SOURCES})

target_link_libraries(Damper Eigen3::Eigen)