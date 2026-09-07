cmake_minimum_required(VERSION 3.15)
project(LivoxHmiNative LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# IMPORTANT:
# The vendor Livox SDK2 source tree is kept intact.
# We link the SDK STATICALLY into LivoxHmiBridge on Windows so the
# application does not depend on a second livox_lidar_sdk_shared.dll.
add_subdirectory(Livox-SDK2/sdk_core ${CMAKE_BINARY_DIR}/livox_sdk2)

add_library(LivoxHmiBridge SHARED
    LivoxHmiBridge.cpp
    LivoxHmiBridge.h
)

target_include_directories(LivoxHmiBridge PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Livox-SDK2/include
)

# Static vendor SDK: one native bridge DLL is deployed beside the .NET EXE.
target_link_libraries(LivoxHmiBridge PRIVATE livox_lidar_sdk_static)

if(WIN32)
    target_link_libraries(LivoxHmiBridge PRIVATE ws2_32 iphlpapi)

    set_target_properties(LivoxHmiBridge PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/bin/Debug"
        RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin/Release"
        RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/bin/RelWithDebInfo"
        RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${CMAKE_BINARY_DIR}/bin/MinSizeRel"
    )
endif()

# ABI sanity test for the C#/native packed structs.
add_executable(LivoxHmiBridgeAbiCheck bridge_abi_check.cpp LivoxHmiBridge.h)
target_include_directories(LivoxHmiBridgeAbiCheck PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

enable_testing()
add_test(NAME LivoxHmiBridgeAbiCheck COMMAND LivoxHmiBridgeAbiCheck)
