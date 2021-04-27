
# Collect all files in this directory
FILE(GLOB_RECURSE SRC  "*.cpp")
FILE(GLOB_RECURSE HEADER  "*.h")

target_sources(snake PRIVATE ${SRC} PUBLIC ${HEADER})
