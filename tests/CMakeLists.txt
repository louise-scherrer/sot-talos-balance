# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST

CONFIG_FILES_CMAKE(test-paths.h)

# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

SET(test_distribute_deps
  distribute-wrench
  sot-core::sot-core parameter-server
  )

#test paths and names (without .cpp extension)
SET(tests
  dummy
  test_distribute
  )


IF(UNIX)
  LINK_DIRECTORIES(${DYNAMIC_GRAPH_PLUGINDIR})
ENDIF(UNIX)

ADD_DEFINITIONS(-DDEBUG=2)

FOREACH(test ${tests})
  ADD_UNIT_TEST(${test} ${test}.cpp)
  TARGET_LINK_LIBRARIES(${test} ${PROJECT_NAME} ${${test}_deps})
  TARGET_INCLUDE_DIRECTORIES(${test} PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
ENDFOREACH(test)

TARGET_LINK_LIBRARIES(test_distribute eiquadprog::eiquadprog)

IF(BUILD_PYTHON_INTERFACE)
  ADD_SUBDIRECTORY(python)
ENDIF(BUILD_PYTHON_INTERFACE)
