# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#

set(VECTORN_SOURCES VectorN.cpp)

# Aggiungere l'origine all'eseguibile di questo progetto.
add_library(VectorN ${VECTORN_SOURCES})