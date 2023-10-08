# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#

set(EQUALITYC0_SOURCES EqualityC0.cpp)

# Aggiungere l'origine all'eseguibile di questo progetto.
add_library(EqualityC0 ${EQUALITYC0_SOURCES})

target_link_libraries(EqualityC0 Eigen3::Eigen)