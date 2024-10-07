cmake_minimum_required(VERSION 3.25)
project(tinyvm)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# The cross platform files
file(GLOB BOX_SRC "${CMAKE_SOURCE_DIR}/src/box/*.c")
file(GLOB SAND_SRC "${CMAKE_SOURCE_DIR}/src/sand/*.c")

# Add the platform source files and append them
if(WIN32)
  set(LIBRARIES  -lws2_32 -lole32 -lcomctl32 -lgdi32 -lwindowscodecs)
elseif(UNIX)
  set(LIBRARIES -lm -lX11)
endif()

# Add an executable
add_executable(${PROJECT_NAME} ${BOX_SRC} ${SAND_SRC})

target_compile_options(${PROJECT_NAME} PRIVATE 
  -Wall -Wextra -pedantic -ansi -O3 -flto=auto
  ${PLATFORM_FLAGS})

target_link_options(${PROJECT_NAME} PRIVATE ${PLATFORM_LINKS})
# target_link_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/lib)
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBRARIES} ${GLOBAL_LIBRARIES})
# target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/src)
