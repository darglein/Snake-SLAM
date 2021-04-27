cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

set(LIB_NAME Snake)
project(${LIB_NAME} VERSION 1.0.0 LANGUAGES CXX)


set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
include(helper_macros)
include(ExternalProject)
DefaultBuildType(RelWithDebInfo)


message("Build Options")

OptionsHelper(SNAKE_DEBUG_ASAN "Enable the address sanitizer. Does not work in combination with TSAN." OFF)
OptionsHelper(SNAKE_DEBUG_TSAN "Enable the thread sanitizer. Does not work in combination with ASAN." OFF)
OptionsHelper(SNAKE_PCH "Generate a precompiled header" OFF)
OptionsHelper(SNAKE_MOBILE "Optimize for mobile hardware" OFF)

############# Compile flags ###############

if (SNAKE_DEBUG_ASAN)
    SET(CMAKE_BUILD_TYPE Debug)
    list(APPEND SNAKE_CXX_FLAGS "-fsanitize=address")
    list(APPEND SNAKE_CXX_FLAGS "-fno-omit-frame-pointer")

    list(APPEND SNAKE_LD_FLAGS "-fsanitize=address")
endif ()

if (SNAKE_DEBUG_TSAN)
    SET(CMAKE_BUILD_TYPE Debug)
    list(APPEND SNAKE_CXX_FLAGS "-fsanitize=thread")
    list(APPEND SNAKE_CXX_FLAGS "-fno-omit-frame-pointer")

    list(APPEND SNAKE_LD_FLAGS "-fsanitize=thread")
    list(APPEND SNAKE_LD_FLAGS "-static-libsan")
endif ()



message("\nSearching for installed libaries...")


set(SAIGA_BUILD_SAMPLES OFF)
set(SAIGA_MODULE_VULKAN OFF)
set(SAIGA_NO_INSTALL ON)
set(SAIGA_USE_SUBMODULES ON)
set(SAIGA_BUILD_TESTS OFF)
set(SAIGA_BUILD_SHARED ON)
set(SAIGA_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin CACHE PATH "override saiga runtime output" FORCE)
add_subdirectory(External/saiga)

#saiga
PackageHelperTarget(saiga_core SAIGA_FOUND)
PackageHelperTarget(saiga_vision SAIGA_FOUND)
PackageHelperTarget(saiga_opengl SAIGA_FOUND)
PackageHelperTarget(saiga_cuda SAIGA_CUDA_FOUND)

if (NOT SAIGA_FOUND)
    message(FATAL_ERROR "Saiga not found D:")
endif ()

if (SAIGA_CUDA_FOUND)
    set(SNAKE_CUDA 1)
    message(STATUS "Enable GPU Feature Detection")
endif ()

#openmp
find_package(OpenMP REQUIRED)
PackageHelperTarget(OpenMP::OpenMP_CXX OPENMP_FOUND)

#opencv
find_package(OpenCV REQUIRED QUIET)
PackageHelper(OpenCV "${OpenCV_FOUND}" "${OpenCV_INCLUDE_DIRS}" "${OpenCV_LIBRARIES}")

find_package(Filesystem REQUIRED)
SET(SAIGA_USE_FILESYSTEM 1)
PackageHelperTarget(std::filesystem FILESYSTEM_FOUND)


include_directories(.)
include_directories(${PACKAGE_INCLUDES})

############# COMPILER FLAGS ###############


if (MSVC)
    #multiprocessor compilation for visual studio
    SET(SNAKE_CXX_FLAGS "${SNAKE_CXX_FLAGS} /MP")
else ()
    list(APPEND SNAKE_CXX_FLAGS "-Wall" "-Werror=return-type")
    list(APPEND SNAKE_CXX_FLAGS "-Wno-strict-aliasing")
    list(APPEND SNAKE_CXX_FLAGS "-Wno-sign-compare")
    list(APPEND SNAKE_CXX_FLAGS "-ftemplate-backtrace-limit=0")
    list(APPEND SNAKE_CXX_FLAGS "-march=native")
    if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        list(APPEND SNAKE_CXX_FLAGS "-fsized-deallocation")
    endif ()
endif ()


message(STATUS SNAKE_CXX_FLAGS "${SNAKE_CXX_FLAGS}")

set(LIBS ${LIBS} ${LIB_TARGETS})

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(Snake)







