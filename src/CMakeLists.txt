﻿# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#
cmake_minimum_required (VERSION 3.8)

# Set project name
project ("MultiBodySim")

add_subdirectory(MultiBody)

set(SOURCES MultiBodySim.cpp)

# Aggiungere l'origine all'eseguibile di questo progetto.
add_executable (MultiBodySim ${SOURCES})

target_link_libraries(MultiBodySim MultiBody Eigen3::Eigen)