# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST

# Relative plug-in directory (i.e. lib/plugin for instance)
# as exposed by the dynamic-graph pkg-config file.
SET(PLUGINDIR ${DYNAMIC_GRAPH_PLUGINDIRNAME})
SET(FULLPLUGINDIR ${CMAKE_INSTALL_PREFIX}/${PLUGINDIR})
SET(SOT_PLUGIN_OUT_DIR ${CMAKE_BINARY_DIR}/src)

CONFIG_FILES_CMAKE(test-paths.h)

IF(UNIX)
  ADD_DEFINITIONS(-pthread)
ENDIF(UNIX)

# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

# Definition of a macro that corresponds to the suffix for a dynamic lib
ADD_DEFINITIONS(-DTESTS_DYNLIBSUFFIX="${CMAKE_SHARED_LIBRARY_SUFFIX}")

# The sot-core plugin libraries that tests have to be linked against
# the name of the variable for test EXECUTABLE_NAME is
# TEST_${EXECUTABLE_NAME}_LIBS
SET(TEST_test_distribute_LIBS
  distribute-wrench
  )

SET(TEST_test_distribute_EXT_LIBS
  sot-core::sot-core ${CMAKE_INSTALL_PREFIX}/lib/plugin/parameter-server.so
  )

#test paths and names (without .cpp extension)
SET (tests
  dummy
  test_distribute
  )

#IF(UNIX)
#  ADD_LIBRARY(pluginabstract
#    SHARED
#    tools/plugin.cc)

#  ADD_EXECUTABLE(test_abstract_interface
#    tools/test_abstract_interface.cpp
#    )
#  TARGET_LINK_LIBRARIES(test_abstract_interface
#    pluginabstract ${CMAKE_DL_LIBS} ${Boost_LIBRARIES})

#  LINK_DIRECTORIES(${SOT_PLUGIN_OUT_DIR} ${FULLPLUGINDIR})
#ENDIF(UNIX)

IF(UNIX)
  LINK_DIRECTORIES(${CMAKE_INSTALL_PREFIX}/lib/plugin)
ENDIF(UNIX)


ADD_DEFINITIONS(-DDEBUG=2)

# Add MatrixAbstractLayer compilation flags
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/unittest)

FOREACH(test ${tests})
  GET_FILENAME_COMPONENT(EXECUTABLE_NAME ${test} NAME)

  ADD_EXECUTABLE(${EXECUTABLE_NAME} ${test}.cpp)

  TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${PROJECT_NAME})
  ADD_DEPENDENCIES     (${EXECUTABLE_NAME} ${PROJECT_NAME})

  IF( TEST_${EXECUTABLE_NAME}_LIBS )
    TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${TEST_${EXECUTABLE_NAME}_LIBS})
    ADD_DEPENDENCIES     (${EXECUTABLE_NAME} ${TEST_${EXECUTABLE_NAME}_LIBS})
  ENDIF( TEST_${EXECUTABLE_NAME}_LIBS )

  IF( TEST_${EXECUTABLE_NAME}_EXT_LIBS )
    TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${TEST_${EXECUTABLE_NAME}_EXT_LIBS})
  ENDIF( TEST_${EXECUTABLE_NAME}_EXT_LIBS )

  IF (UNIX)
    TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${CMAKE_DL_LIBS})
  ENDIF(UNIX)

  TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} dynamic-graph::dynamic-graph)
  PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} pinocchio)
  IF(BUILD_PYTHON_INTERFACE)
    TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME}
      dynamic-graph-python::dynamic-graph-python)
  ENDIF(BUILD_PYTHON_INTERFACE)

  # Link against Boost.
  TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME}
    ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
  ADD_TEST(${test} ${EXECUTABLE_NAME})

  IF (UNIX)
    SET(EXTRA_LD_LIBRARY_PATH $ENV{LD_LIBRARY_PATH})
    SET_PROPERTY(TEST ${test} PROPERTY
      ENVIRONMENT "LD_LIBRARY_PATH=${CMAKE_BINARY_DIR}/src:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/plugin:${BOOST_ROOT}/lib:${EXTRA_LD_LIBRARY_PATH}")
  ENDIF(UNIX)

ENDFOREACH(test)

IF(BUILD_PYTHON_INTERFACE)
  ADD_SUBDIRECTORY(python)
ENDIF(BUILD_PYTHON_INTERFACE)
