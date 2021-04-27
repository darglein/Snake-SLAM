
set(APP_NAME snake)

set(SAIGA_BUILDCONFIG_DIR "${CMAKE_CURRENT_BINARY_DIR}/include")
set(SAIGA_BUILDCONFIG_FILE "${SAIGA_BUILDCONFIG_DIR}/buildconfig.h")
configure_file (
    "${CMAKE_CURRENT_SOURCE_DIR}/buildconfig.h.in"
    "${SAIGA_BUILDCONFIG_FILE}"
    )



add_library(snake STATIC)
#INCLUDE_DIRECTORIES(${SAIGA_BUILDCONFIG_DIR})
target_include_directories(snake PUBLIC "." "System/" ${SAIGA_BUILDCONFIG_DIR})
target_link_libraries(snake PUBLIC ${LIBS})
target_compile_options(snake PRIVATE ${SNAKE_CXX_FLAGS})
target_link_options(snake PRIVATE ${SNAKE_LD_FLAGS})

#############  source #############

# Group source files in IDEs
FILE(GLOB_RECURSE ALL_SRC  "*.cpp")
FILE(GLOB_RECURSE ALL_HEADER  "*.h")



source_group(TREE ${CMAKE_CURRENT_LIST_DIR} FILES  ${ALL_SRC} ${ALL_HEADER})

# Source files in "src/" to level directory
FILE(GLOB_RECURSE PROG_SRC  "*.cpp")
FILE(GLOB_RECURSE PROG_HEADER  "*.h" ${SAIGA_BUILDCONFIG_FILE})
target_sources(snake PRIVATE ${PROG_SRC} PUBLIC ${PROG_HEADER})

#add_subdirectory(Culling)
#add_subdirectory(IMU)
#add_subdirectory(Initialization)
#add_subdirectory(Input)
#add_subdirectory(LocalMapping)
#add_subdirectory(LoopClosing)
#add_subdirectory(Map)
#add_subdirectory(OpenGLViewer)
#add_subdirectory(Optimizer)
#add_subdirectory(PostMapOptimization)
#if(TORCH_FOUND)
#    add_subdirectory(Torch)
#endif()
#add_subdirectory(Tracking)




#############  output dir ############# 

set_target_properties( ${APP_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

set_target_properties(${APP_NAME} PROPERTIES CXX_STANDARD 17)

if(MSVC)
    #visual studio working dir
    set_target_properties(${APP_NAME} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
endif()


set_property(TARGET ${APP_NAME} PROPERTY CXX_STANDARD 17)





if(SNAKE_PCH)
    target_precompile_headers(
        ${APP_NAME}
        PRIVATE
        "${PROJECT_SOURCE_DIR}/Snake/System/SnakeGlobal.h"
        )
endif()



add_executable(snake_slam main.cpp)
target_link_libraries(snake_slam snake)
target_compile_options(snake_slam PRIVATE ${SNAKE_CXX_FLAGS})
target_link_options(snake_slam PRIVATE ${SNAKE_LD_FLAGS})