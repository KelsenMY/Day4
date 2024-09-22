cmake_minimum_required(VERSION 3.10)

project(myproject LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)


find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)


file(GLOB SOURCES "*.cpp")

add_executable(myproject ${SOURCES})


target_link_libraries(myproject ncurses ${JSONCPP_LIBRARIES})


include_directories(${JSONCPP_INCLUDE_DIRS})