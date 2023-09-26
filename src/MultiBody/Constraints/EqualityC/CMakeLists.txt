# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#

set(EQUALITYC_SOURCES EqualityC.cpp)

# Aggiungere l'origine all'eseguibile di questo progetto.
add_library(EqualityC ${EQUALITYC_SOURCES})

target_link_libraries(EqualityC Eigen3::Eigen)