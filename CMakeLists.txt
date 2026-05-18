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
# qbRobotics qbAPI sources (https://github.com/NMMI/qbAPI). The SoftClaw
# is controlled FROM THIS PC via the qbRobotics USB / RS-485 adapter -
# the Doosan controller is NOT involved in gripper commands. If left
# empty the project still builds, QbSoftClawGripper compiles as a stub
# and the factory falls back to NoopGripper when required = false.
# The folder is expected to contain qbmove_communications.h and the
# matching qbmove_communications.cpp (default layout: <root>/src).
set(QBROBOTICS_SDK_ROOT
    "${CMAKE_SOURCE_DIR}/external/qbrobotics_sdk"
    CACHE PATH
    "Path to qbRobotics qbAPI source tree (https://github.com/NMMI/qbAPI)")

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

# qbRobotics qbAPI lookup. Accept either:
#   <root>/qbmove_communications.h           (top-level)
#   <root>/src/qbmove_communications.h       (default NMMI/qbAPI layout)
#   <root>/include/qbmove_communications.h   (custom)
set(HAVE_QBROBOTICS_SDK FALSE)
set(QB_SDK_INCLUDE_DIR "")
set(QB_SDK_SOURCES)
if(QBROBOTICS_SDK_ROOT)
    foreach(_qb_layout "" "src" "include")
        set(_qb_dir "${QBROBOTICS_SDK_ROOT}/${_qb_layout}")
        if(EXISTS "${_qb_dir}/qbmove_communications.h")
            set(QB_SDK_INCLUDE_DIR "${_qb_dir}")
            if(EXISTS "${_qb_dir}/qbmove_communications.cpp")
                list(APPEND QB_SDK_SOURCES "${_qb_dir}/qbmove_communications.cpp")
            elseif(EXISTS "${_qb_dir}/qbmove_communications.c")
                list(APPEND QB_SDK_SOURCES "${_qb_dir}/qbmove_communications.c")
            endif()
            if(EXISTS "${_qb_dir}/commands.h")
                # commands.h is the protocol-constants header; include only.
            endif()
            set(HAVE_QBROBOTICS_SDK TRUE)
            break()
        endif()
    endforeach()
endif()
if(HAVE_QBROBOTICS_SDK)
    message(STATUS "qbRobotics qbAPI found at ${QB_SDK_INCLUDE_DIR} "
                   "(real SoftClaw control enabled).")
else()
    message(WARNING
        "qbRobotics qbAPI not found - building QbSoftClawGripper as a STUB. "
        "Clone https://github.com/NMMI/qbAPI into "
        "${QBROBOTICS_SDK_ROOT} (or set -DQBROBOTICS_SDK_ROOT=<path>) to "
        "enable real SoftClaw control. See docs/SOFTCLAW_INTEGRATION.md.")
endif()

# ---------------------------------------------------------------------------
# Source files
# ---------------------------------------------------------------------------
set(APP_SOURCES
    src/main.cpp
    src/app/Application.cpp
    src/config/Config.cpp
    src/sensor/LeapSource.cpp
    src/gesture/GestureInterpreter.cpp
    src/state/StateMachine.cpp
    src/robot/DrflRobotController.cpp
    src/robot/WorkspaceGuard.cpp
    src/gripper/GripperFactory.cpp
    src/gripper/NoopGripper.cpp
    src/gripper/QbSoftClawGripper.cpp
    src/gripper/ToolIoGripperController.cpp
    src/input/KeyboardButton.cpp
    src/ui/ConsoleUI.cpp
    src/util/Logger.cpp
)
if(HAVE_QBROBOTICS_SDK)
    list(APPEND APP_SOURCES ${QB_SDK_SOURCES})
endif()

add_executable(doosan_gesture_demo ${APP_SOURCES})
target_include_directories(doosan_gesture_demo PRIVATE src)

if(HAVE_QBROBOTICS_SDK)
    target_compile_definitions(doosan_gesture_demo PRIVATE HAVE_QBROBOTICS_SDK=1)
    target_include_directories(doosan_gesture_demo PRIVATE "${QB_SDK_INCLUDE_DIR}")
    if(WIN32)
        # qbAPI uses Win32 HANDLE / CreateFile under the hood. Kernel32
        # is implicitly linked by MSVC; the explicit link is here for
        # clarity should the build move to a different toolchain.
        target_link_libraries(doosan_gesture_demo PRIVATE kernel32)
    endif()
endif()

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

# GripperFactory selection + NoopGripper fallback test. The qbAPI is
# deliberately NOT linked here so the QbSoftClawGripper exercises its
# "SDK missing / hardware absent" branches.
add_executable(gripper_factory_test
    test/gripper_factory_test.cpp
    src/gripper/GripperFactory.cpp
    src/gripper/NoopGripper.cpp
    src/gripper/QbSoftClawGripper.cpp
    src/gripper/ToolIoGripperController.cpp
    src/config/Config.cpp
    src/util/Logger.cpp
)
target_include_directories(gripper_factory_test PRIVATE src)

enable_testing()
add_test(NAME state_machine_smoke COMMAND state_machine_smoke_test)
add_test(NAME gripper_factory     COMMAND gripper_factory_test)

# Copy config next to binary so the demo can be launched directly.
add_custom_command(TARGET doosan_gesture_demo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_SOURCE_DIR}/config/demo_config.ini"
            "$<TARGET_FILE_DIR:doosan_gesture_demo>/demo_config.ini"
)
