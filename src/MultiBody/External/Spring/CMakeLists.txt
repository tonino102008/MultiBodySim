# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#

set(SPRING_SOURCES Spring.cpp)

# Aggiungere l'origine all'eseguibile di questo progetto.
add_library(Spring ${SPRING_SOURCES})

target_link_libraries(Spring Eigen3::Eigen)