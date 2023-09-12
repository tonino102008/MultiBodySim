# CMakeList.txt: progetto CMake per MultiBodySim. Includere l'origine e definire
# qui la logica specifica del progetto.
#
cmake_minimum_required (VERSION 3.8)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("MultiBodySim")

# Aggiungere l'origine all'eseguibile di questo progetto.
add_executable (MultiBodySim "MultiBodySim.cpp" "MultiBodySim.h" "Integrators/Integrator.h" "Integrators/Integrator.cpp" "Quaternions/Quaternion.h" "Quaternions/Quaternion.cpp")

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET MultiBodySim PROPERTY CXX_STANDARD 17)
endif()

# TODO: Aggiungere i test e, se necessario, installare le destinazioni.
