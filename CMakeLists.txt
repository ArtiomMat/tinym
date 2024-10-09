cmake_minimum_required(VERSION 3.25)
project(tinyvm)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Source directory
set(SRC_DIR  ${CMAKE_SOURCE_DIR}/tinyvm)

# Add the platform source files and append them
if(WIN32)
  set(LIBRARIES  -lws2_32 -lole32 -lcomctl32 -lgdi32 -lwindowscodecs)
elseif(UNIX)
  set(LIBRARIES -lm -lX11)
endif()

# Source files
file(GLOB BOX_SRC "${SRC_DIR}/box/*.c")
file(GLOB SAND_SRC "${SRC_DIR}/sand/*.c")
file(GLOB SHARED_SRC "${SRC_DIR}/*.c")

# Add an executable
add_executable(
  ${PROJECT_NAME}
  ${BOX_SRC} ${SAND_SRC} ${SHARED_SRC})

target_compile_options(${PROJECT_NAME} PRIVATE 
  -Wall -Wextra -pedantic -ansi -O3 -flto=auto
  ${PLATFORM_FLAGS})

target_link_options(${PROJECT_NAME} PRIVATE ${PLATFORM_LINKS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBRARIES} ${GLOBAL_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${SRC_DIR})
