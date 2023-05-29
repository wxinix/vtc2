set(TEST_EXE ${PROJECT_NAME}_tests)

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

add_executable(${TEST_EXE}
        ${CMAKE_SOURCE_DIR}/tests/tests.cpp
        ${CMAKE_SOURCE_DIR}/thirdparty/pugixml/pugixml.cpp)

set(INCLUDE_DIR
        "${CMAKE_SOURCE_DIR}"
        "${CMAKE_SOURCE_DIR}/vtc"
        "${CMAKE_SOURCE_DIR}/thirdparty")

target_include_directories(
        ${TEST_EXE}
        PRIVATE ${INCLUDE_DIR})

configure_file(${CMAKE_SOURCE_DIR}/vtc/xils.config.xml ${PROJECT_BINARY_DIR}/tests COPYONLY)
