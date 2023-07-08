set(TARGET_EXE_NAME ${PROJECT_NAME}_tests)

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

add_executable(${TARGET_EXE_NAME}
        ${CMAKE_SOURCE_DIR}/tests/main.cpp
        ${CMAKE_SOURCE_DIR}/thirdparty/pugixml/pugixml.cpp)

set(INCLUDE_DIR
        "${CMAKE_SOURCE_DIR}"
        "${CMAKE_SOURCE_DIR}/vtc"
        "${CMAKE_SOURCE_DIR}/thirdparty")

target_include_directories(
        ${TARGET_EXE_NAME}
        PRIVATE ${INCLUDE_DIR})

configure_file(${CMAKE_SOURCE_DIR}/vtc/xils.config.xml ${PROJECT_BINARY_DIR}/tests COPYONLY)
