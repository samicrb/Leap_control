cmake_minimum_required(VERSION 3.16)
project(doosan_gesture_demo CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/W4 /permissive- /Zc:__cplusplus)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Optional external SDKs.
#
# The project is structured so that LeapC and DRFL are cleanly wrapped by
# adapter classes. When the SDKs are not installed on the build machine the
# project still builds with stub adapters, so the state machine / gesture
# logic can be exercised on a developer laptop.
#
# To enable real SDK integration set:
#   -DLEAPC_SDK_ROOT="C:/Program Files/Ultraleap/LeapSDK"
#   -DDRFL_SDK_ROOT="C:/Doosan/DRFL"
# ---------------------------------------------------------------------------
set(LEAPC_SDK_ROOT "" CACHE PATH "Path to Ultraleap Gemini LeapSDK (contains include/LeapC.h)")
set(DRFL_SDK_ROOT  "" CACHE PATH "Path to Doosan DRFL 1.33.2 SDK (contains include/DRFLEx.h)")

set(HAVE_LEAPC FALSE)
if(LEAPC_SDK_ROOT AND EXISTS "${LEAPC_SDK_ROOT}/include/LeapC.h")
    set(HAVE_LEAPC TRUE)
    message(STATUS "LeapC SDK found at ${LEAPC_SDK_ROOT}")
else()
    message(WARNING "LeapC SDK not found - building with STUB sensor adapter")
endif()

set(HAVE_DRFL FALSE)
if(DRFL_SDK_ROOT AND EXISTS "${DRFL_SDK_ROOT}/include/DRFLEx.h")
    set(HAVE_DRFL TRUE)
    message(STATUS "DRFL SDK found at ${DRFL_SDK_ROOT}")
else()
    message(WARNING "DRFL SDK not found - building with STUB robot adapter")
endif()

# ---------------------------------------------------------------------------
# Source files
# ---------------------------------------------------------------------------
set(APP_SOURCES
    src/main.cpp
    src/app/Application.cpp
    src/config/Config.cpp
    src/config/RuntimeConfigReloader.cpp
    src/sensor/LeapSource.cpp
    src/gesture/GestureInterpreter.cpp
    src/state/StateMachine.cpp
    src/robot/DrflRobotController.cpp
    src/robot/WorkspaceGuard.cpp
    src/gripper/ToolIoGripperController.cpp
    src/input/KeyboardButton.cpp
    src/logging/EventLogger.cpp
    src/logging/MotionLogger.cpp
    src/ui/ConsoleUI.cpp
    src/util/Logger.cpp
)

add_executable(doosan_gesture_demo ${APP_SOURCES})
target_include_directories(doosan_gesture_demo PRIVATE src)

if(HAVE_LEAPC)
    target_compile_definitions(doosan_gesture_demo PRIVATE HAVE_LEAPC=1)
    target_include_directories(doosan_gesture_demo PRIVATE "${LEAPC_SDK_ROOT}/include")
    target_link_directories(doosan_gesture_demo PRIVATE "${LEAPC_SDK_ROOT}/lib/x64")
    target_link_libraries(doosan_gesture_demo PRIVATE LeapC)
endif()

if(HAVE_DRFL)
    target_compile_definitions(doosan_gesture_demo PRIVATE HAVE_DRFL=1)
    target_include_directories(doosan_gesture_demo PRIVATE "${DRFL_SDK_ROOT}/include")

    # Resolve the DRFL import library. The 1.33.3 release at
    # github.com/DoosanRobotics/API-DRFL ships "DRFLWin64.lib" under
    # library/Windows/64bits/; older bundles use lib/x64/DRFL.lib or
    # DRFLx64.lib. Search all known layouts so the build works regardless
    # of how the SDK was unpacked.
    find_library(DRFL_LIBRARY
        NAMES DRFLWin64 DRFLx64 DRFL64 DRFL
        PATHS
            "${DRFL_SDK_ROOT}/library/Windows/64bits"
            "${DRFL_SDK_ROOT}/library/Windows/32bits"
            "${DRFL_SDK_ROOT}/lib/x64"
            "${DRFL_SDK_ROOT}/lib"
        NO_DEFAULT_PATH
    )
    if(NOT DRFL_LIBRARY)
        message(FATAL_ERROR
            "DRFL library not found under ${DRFL_SDK_ROOT}. "
            "Expected DRFLWin64.lib (1.33.3) or DRFL.lib (older).")
    endif()
    message(STATUS "DRFL library: ${DRFL_LIBRARY}")
    target_link_libraries(doosan_gesture_demo PRIVATE "${DRFL_LIBRARY}")

    # Copy the runtime DLLs next to the binary so the demo launches
    # without manual PATH gymnastics. Both 1.33.3's flat layout and the
    # older lib/x64 layout are handled.
    if(WIN32)
        set(_drfl_dll_candidates
            "${DRFL_SDK_ROOT}/library/Windows/64bits/DRFLWin64.dll"
            "${DRFL_SDK_ROOT}/library/Windows/64bits/PocoFoundation64.dll"
            "${DRFL_SDK_ROOT}/library/Windows/64bits/PocoNet64.dll"
            "${DRFL_SDK_ROOT}/bin/DRFL.dll"
            "${DRFL_SDK_ROOT}/lib/x64/DRFL.dll"
        )
        foreach(_dll IN LISTS _drfl_dll_candidates)
            if(EXISTS "${_dll}")
                add_custom_command(TARGET doosan_gesture_demo POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                            "${_dll}"
                            "$<TARGET_FILE_DIR:doosan_gesture_demo>"
                )
            endif()
        endforeach()
    endif()
endif()

if(WIN32)
    target_link_libraries(doosan_gesture_demo PRIVATE ws2_32)
endif()

# ---------------------------------------------------------------------------
# Tests (headless - state machine / gesture interpreter only)
# ---------------------------------------------------------------------------
add_executable(state_machine_smoke_test
    test/state_machine_smoke_test.cpp
    src/state/StateMachine.cpp
    src/config/Config.cpp
    src/util/Logger.cpp
)
target_include_directories(state_machine_smoke_test PRIVATE src)

# Vector-norm limiter regression test (header-only target).
add_executable(vector_norm_limiter_test
    test/vector_norm_limiter_test.cpp
)
target_include_directories(vector_norm_limiter_test PRIVATE src)

enable_testing()
add_test(NAME state_machine_smoke      COMMAND state_machine_smoke_test)
add_test(NAME vector_norm_limiter      COMMAND vector_norm_limiter_test)

# Copy config next to binary so the demo can be launched directly.
add_custom_command(TARGET doosan_gesture_demo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_SOURCE_DIR}/config/demo_config.ini"
            "$<TARGET_FILE_DIR:doosan_gesture_demo>/demo_config.ini"
)
