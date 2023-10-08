# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#

set(CONSTRAINTS_SOURCES Constraint.cpp)

add_subdirectory(EqualityC)
add_subdirectory(EqualityC0)

# Aggiungere l'origine all'eseguibile di questo progetto.
add_library(Constraints ${CONSTRAINTS_SOURCES})

target_link_libraries(Constraints EqualityC EqualityC0 Eigen3::Eigen)