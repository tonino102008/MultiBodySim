# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#

set(MATRIXN_SOURCES MatrixN.cpp)

# Aggiungere l'origine all'eseguibile di questo progetto.
add_library(MatrixN ${MATRIXN_SOURCES})