cmake_minimum_required(VERSION 2.8)
cmake_policy(SET CMP0020 NEW)

project(reactor)

include(CTest)

#include (GenerateExportHeader)
#set (CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS true)

set(CMAKE_AUTOMOC true)
set(CMAKE_AUTOUIC true)
set(CMAKE_AUTORCC true)
set(CMAKE_INCLUDE_CURRENT_DIR true)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

set(CMAKE_INSTALL_RPATH ".;${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH true)

#set (CMAKE_CXX_STANDARD 11)
#set (CMAKE_CXX_STANDARD_REQUIRED true)
if(UNIX)
    add_compile_options("-std=c++14" "-Wall")
endif(UNIX)

set (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY AUTOGEN_TARGETS_FOLDER AutoGen)

set(WITH_TRAJCLIENT_V1 false CACHE BOOL "build TrajClient v1")
set(WITH_TRAJCLIENT_V3 false CACHE BOOL "build TrajClient v3")
set(WITH_CS8_EMULATOR false CACHE BOOL "build CS8 emulator")

add_subdirectory(src)
#add_subdirectory(data)
