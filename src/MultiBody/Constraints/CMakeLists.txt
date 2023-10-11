# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#

add_subdirectory(EqualityC)
add_subdirectory(EqualityC0)

# Aggiungere l'origine all'eseguibile di questo progetto.
add_library(Constraints INTERFACE)

target_link_libraries(Constraints INTERFACE EqualityC EqualityC0 Eigen3::Eigen)