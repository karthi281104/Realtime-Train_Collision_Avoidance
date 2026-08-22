cmake_minimum_required(VERSION 3.25)
project(TrainCollisionAvoidance VERSION 0.1.0 LANGUAGES CXX)

# ─── C++23 standard ───────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ─── Compiler flags ───────────────────────────────────────────────────────────
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    add_compile_options(
        -Wall -Wextra -Wpedantic
        -Wshadow -Wconversion -Wsign-conversion
        -march=native -O2
    )
endif()

# ─── Output directories ───────────────────────────────────────────────────────
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# ─── Global include ───────────────────────────────────────────────────────────
include_directories(${PROJECT_SOURCE_DIR}/include)

# ─── Threads (POSIX) ──────────────────────────────────────────────────────────
find_package(Threads REQUIRED)

# ─── Source files ─────────────────────────────────────────────────────────────
set(CORE_SOURCES
    src/core/Types.cpp
    src/core/Config.cpp
    src/core/Logger.cpp
    src/core/SimulationClock.cpp
)

set(RAILWAY_SOURCES
    src/railway/Track.cpp
    src/railway/Station.cpp
    src/railway/Junction.cpp
    src/railway/Signal.cpp
    src/railway/RailwayNetwork.cpp
    src/railway/RouteManager.cpp
)

set(TRAIN_SOURCES
    src/train/Train.cpp
    src/train/PassengerTrain.cpp
    src/train/TrainPhysics.cpp
    src/train/TrainStateManager.cpp
)

set(SIMULATION_SOURCES
    src/simulation/MovementEngine.cpp
    src/simulation/SimulationEngine.cpp
)

set(PREDICTION_SOURCES
    src/prediction/CollisionPredictor.cpp
    src/prediction/StateEstimator.cpp
)

set(SAFETY_SOURCES
    src/safety/RiskAssessor.cpp
    src/safety/ConflictManager.cpp
    src/safety/ConflictResolver.cpp
)

set(COMMUNICATION_SOURCES
    src/communication/CommChannel.cpp
    src/communication/MessageBus.cpp
)

set(MONITORING_SOURCES
    src/monitoring/Dashboard.cpp
    src/monitoring/EventLogger.cpp
    src/monitoring/ScenarioManager.cpp
)

set(ALL_SOURCES
    ${CORE_SOURCES}
    ${RAILWAY_SOURCES}
    ${TRAIN_SOURCES}
    ${SIMULATION_SOURCES}
    ${PREDICTION_SOURCES}
    ${SAFETY_SOURCES}
    ${COMMUNICATION_SOURCES}
    ${MONITORING_SOURCES}
    src/ControlCenter.cpp
)

# ─── Main executable ──────────────────────────────────────────────────────────
add_executable(train_sim src/main.cpp ${ALL_SOURCES})
target_link_libraries(train_sim PRIVATE Threads::Threads)

# ─── Tests ────────────────────────────────────────────────────────────────────
enable_testing()

add_executable(test_physics
    tests/test_physics.cpp
    src/core/Types.cpp
    src/core/Logger.cpp
    src/train/Train.cpp
    src/train/TrainPhysics.cpp
    src/train/PassengerTrain.cpp
    src/train/TrainStateManager.cpp
)
target_link_libraries(test_physics PRIVATE Threads::Threads)
add_test(NAME Physics COMMAND test_physics)

add_executable(test_railway
    tests/test_railway.cpp
    ${RAILWAY_SOURCES}
    src/core/Types.cpp
    src/core/Logger.cpp
    src/core/Config.cpp
)
target_link_libraries(test_railway PRIVATE Threads::Threads)
add_test(NAME Railway COMMAND test_railway)

add_executable(test_prediction
    tests/test_prediction.cpp
    ${CORE_SOURCES}
    ${RAILWAY_SOURCES}
    ${TRAIN_SOURCES}
    ${PREDICTION_SOURCES}
    ${SAFETY_SOURCES}
)
target_link_libraries(test_prediction PRIVATE Threads::Threads)
add_test(NAME Prediction COMMAND test_prediction)

# ─── Install ──────────────────────────────────────────────────────────────────
install(TARGETS train_sim DESTINATION bin)
