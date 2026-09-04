  # ---------------------------------------------------------------------------
  # Plateforme de développement et de test
  #   Système     : macOS (Apple Silicon, arm64), windows 11
  #   Compilateur : Apple Clang 21.0.0 (c++ fourni par Xcode), MinGW-w64 (G++)
  #   Qt          : 6.11.2 (installateur en ligne, ~/Qt/6.11.2/macos)
  # Configuration spécifique : aucune requise, hormis QTLocation.txt si Qt n'est
  #   pas trouvé automatiquement par CMake. Ce fichier est optionnel et ignoré
  #   par git : il fixe CMAKE_PREFIX_PATH vers l'installation Qt locale.
  # ---------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)

# Chemin vers votre installation de Qt. Renommez QTLocation.txt.example en
# QTLocation.txt et adaptez le chemin. Ce fichier est optionnel : il est inutile
# si Qt est déjà trouvable par CMake (Homebrew, paquets Linux).
include(QTLocation.txt OPTIONAL)

project(PIN_2026_Alpine VERSION 0.1 LANGUAGES CXX)
enable_testing()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Qt6 6.5 REQUIRED COMPONENTS Widgets)
message(STATUS "Qt ${Qt6_VERSION} trouvé dans ${Qt6_DIR}")

# Sources communes (partagées entre le rendu et le setup)
set(COMMON_SOURCES
        common/Types.h
        common/Position.h
        common/Robot.h
        common/Particle.h
        common/State.h
        common/TimeLine.h
        common/json.hpp
        common/json_util.h
        common/Movement.h
        common/StateUtils.cpp
        common/StateUtils.h
        common/Constants.h
        common/Constraint.h
        common/angle_utils.h
        simulation/assigne/weight_calculation.h
        simulation/assigne/weight_calculation.cpp
        common/doctest.h
)

add_library(Common STATIC ${COMMON_SOURCES})
#target_include_directories(Common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/common)

# Exécutable 1 : Robots (application de rendu, GUI Qt)
set(RENDER_SOURCES
        render/main.cpp
        render/robotsui.cpp
        render/robotsui.h
        render/robotsui.ui
        render/robotwidget.h
        render/robotwidget.cpp
        render/worldtoscreenconverter.h
        render/worldtoscreenconverter.cpp
)

qt_add_executable(Robots
        MANUAL_FINALIZATION
        ${RENDER_SOURCES}
)

target_link_libraries(Robots PRIVATE Common Qt6::Widgets)

target_include_directories(Robots PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/render
)

set_target_properties(Robots PROPERTIES
        MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
        MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
        MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
        MACOSX_BUNDLE TRUE
        WIN32_EXECUTABLE TRUE
)

install(TARGETS Robots
        BUNDLE DESTINATION .
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

qt_finalize_executable(Robots)

# Exécutable 2 : Setup (application console, calcule une TimeLine)
add_executable(Setup
        simulation/main.cpp
        simulation/world/direction.cpp
        simulation/world/direction.h
        simulation/world/particles.cpp
        simulation/world/particles.h
        simulation/io/io.cpp
        simulation/io/io.h
        simulation/world/collision.cpp
        simulation/world/collision.h
        simulation/world/event.h
        simulation/trajectory/trajectory.h
        simulation/trajectory/trajectory_simulator.cpp
        simulation/trajectory/trajectory_simulator.h
        simulation/trajectory/simple_trajectory.cpp
        simulation/trajectory/simple_trajectory.h
        simulation/trajectory/arc_trajectory.cpp
        simulation/trajectory/arc_trajectory.h
        simulation/trajectory/trajectory_score.cpp
        simulation/trajectory/trajectory_score.h
        simulation/trajectory/trajectory_timeline.cpp
        simulation/trajectory/trajectory_timeline.h
        simulation/trajectory/trajectory_collision.h
        simulation/trajectory/trajectory_collision.cpp
        simulation/trajectory/trajectory_planner.h
        simulation/trajectory/trajectory_planner.cpp
        simulation/trajectory/trajectory_command.h
        simulation/trajectory/trajectory_command.cpp
        simulation/world/explosion.h
        simulation/engine/command.cpp
        simulation/engine/command.h
        simulation/engine/interval.cpp
        simulation/engine/interval.h
)

target_link_libraries(Setup PRIVATE Common)

target_include_directories(Setup PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/simulation
)

install(TARGETS Setup
        RUNTIME DESTINATION bin)

# Exécutable 3 : Smoke (vérification rapide de la fonction d'évaluation)
add_executable(Smoke
        simulation/tests/smoke.cpp
        simulation/trajectory/trajectory_score.cpp
        simulation/trajectory/trajectory_score.h
        simulation/tests/test_helpers.h
)

target_link_libraries(Smoke PRIVATE Common)

target_include_directories(Smoke PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/simulation
)
add_executable(ScoreTests
        simulation/tests/trajectory_tests.cpp
        simulation/trajectory/trajectory_score.cpp
        simulation/trajectory/trajectory_score.h
        simulation/trajectory/trajectory_collision.cpp
        simulation/trajectory/trajectory_collision.h
        simulation/tests/test_helpers.h
)

target_link_libraries(ScoreTests PRIVATE Common)

target_include_directories(ScoreTests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/simulation
)

add_test(NAME ScoreTests COMMAND ScoreTests)

# Tests des trajectoires simples
add_executable(SimpleTrajectoryTests
        simulation/trajectory/simple_trajectory_tests.cpp
        simulation/trajectory/simple_trajectory.cpp
        simulation/trajectory/simple_trajectory.h
        simulation/trajectory/trajectory.h
)

target_link_libraries(SimpleTrajectoryTests PRIVATE Common)

target_include_directories(SimpleTrajectoryTests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/simulation
)

add_test(NAME SimpleTrajectoryTests COMMAND SimpleTrajectoryTests)

add_executable(TrajectorySimulatorTests
        simulation/trajectory/trajectory_simulator_tests.cpp
        simulation/trajectory/trajectory_simulator.cpp
        simulation/trajectory/trajectory_collision.cpp
        simulation/trajectory/trajectory_planner.cpp
        simulation/trajectory/trajectory_simulator.h
        simulation/trajectory/simple_trajectory.cpp
        simulation/trajectory/simple_trajectory.h
        simulation/trajectory/arc_trajectory.cpp
        simulation/trajectory/arc_trajectory.h
        simulation/trajectory/trajectory_score.cpp
        simulation/trajectory/trajectory_score.h
        simulation/trajectory/trajectory.h
)

target_link_libraries(TrajectorySimulatorTests PRIVATE Common)

target_include_directories(TrajectorySimulatorTests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/simulation
)

add_test(NAME TrajectorySimulatorTests COMMAND TrajectorySimulatorTests)

add_executable(TrajectoryDemo
        simulation/trajectory/trajectory_demo.cpp
        simulation/trajectory/trajectory_simulator.cpp
        simulation/trajectory/trajectory_simulator.h
        simulation/trajectory/trajectory_collision.cpp
        simulation/trajectory/trajectory_collision.h
        simulation/trajectory/trajectory_planner.cpp
        simulation/trajectory/trajectory_planner.h
        simulation/trajectory/simple_trajectory.cpp
        simulation/trajectory/simple_trajectory.h
        simulation/trajectory/arc_trajectory.cpp
        simulation/trajectory/arc_trajectory.h
        simulation/trajectory/trajectory_score.cpp
        simulation/trajectory/trajectory_score.h
        simulation/trajectory/trajectory_timeline.cpp
        simulation/trajectory/trajectory_timeline.h
        simulation/trajectory/trajectory.h
        simulation/io/io.cpp
        simulation/io/io.h
)

target_link_libraries(TrajectoryDemo PRIVATE Common)

target_include_directories(TrajectoryDemo PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/simulation
)