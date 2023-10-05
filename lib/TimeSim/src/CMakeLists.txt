# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#

set(TIMESIM_SOURCES TimeSim.cpp)

# Aggiungere l'origine all'eseguibile di questo progetto.
add_library(TimeSim ${TIMESIM_SOURCES})

target_link_libraries(TimeSim)