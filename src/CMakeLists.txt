# Copyright 2018, Gepetto team, LAAS-CNRS

# Verbosity level
IF (NOT (\"${CMAKE_VERBOSITY_LEVEL}\" STREQUAL \"\"))
  ADD_DEFINITIONS(-DVP_DEBUG_MODE=${CMAKE_VERBOSITY_LEVEL} -DVP_DEBUG)
ENDIF (NOT (\"${CMAKE_VERBOSITY_LEVEL}\" STREQUAL \"\"))

IF(UNIX)
  ADD_DEFINITIONS(-pthread)
ENDIF(UNIX)

SET(${LIBRARY_NAME}_SOURCES
  utils/statistics.cpp
  robot/robot-wrapper.cpp
  sdk_qualisys/Network.cpp
  sdk_qualisys/RTPacket.cpp
  sdk_qualisys/RTProtocol.cpp
  sdk_qualisys/Markup.cpp
  )

ADD_LIBRARY(${LIBRARY_NAME} SHARED ${${LIBRARY_NAME}_SOURCES})

SET_TARGET_PROPERTIES(${LIBRARY_NAME}
  PROPERTIES
  SOVERSION ${PROJECT_VERSION}
  INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})

TARGET_LINK_LIBRARIES(${LIBRARY_NAME} dynamic-graph::dynamic-graph)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} sot-core::sot-core)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} pinocchio)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} parametric-curves)

IF(UNIX)
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${CMAKE_DL_LIBS})
  IF(NOT APPLE)
    TARGET_LINK_LIBRARIES(${LIBRARY_NAME} pthread)
  ENDIF(NOT APPLE)
ENDIF(UNIX)

TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${Boost_LIBRARIES})

IF (NOT INSTALL_PYTHON_INTERFACE_ONLY)
  INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)



#This project will create many plugins as shared libraries, listed here
SET(plugins
  example
  delay
  boolean-identity
  round-double-to-int
  int-identity
  pose-quaternion-to-matrix-homo
  euler-to-quat
  quat-to-euler
  state-transformation
  simple-reference-frame
  simple-state-integrator
  dummy-walking-pattern-generator
  simple-zmp-estimator
  simple-distribute-wrench
  simple-pid
  simple-pidd
  simple-controller-6d
  distribute-wrench
  dcm-com-controller
  dcm-controller
  dummy-dcm-estimator
  com-admittance-controller
  simple-admittance-controller
  coupled-admittance-controller
  admittance-controller-end-effector
  ankle-admittance-controller
  foot-force-difference-controller
  joint-position-controller
  nd-trajectory-generator
  talos-base-estimator
  talos-control-manager
  dcm-estimator
  qualisys-client
  ft-calibration
  ft-wrist-calibration
  ankle-joint-selector
  saturation
  hip-flexibility-compensation
  )

#set(ADDITIONAL_feature-task_LIBS feature-generic task)


#Plugins compilation, link, and installation
#Compiles a plugin. The plugin library is ${LIBRARY_NAME}
FOREACH(plugin ${plugins})
  #retrieve plugin name
  GET_FILENAME_COMPONENT(LIBRARY_NAME ${plugin} NAME)

  # only one source file per plugin
  ADD_LIBRARY(${LIBRARY_NAME}
    SHARED
    ${plugin}.cpp)

  #remove the "lib" prefix from the plugin output name
  SET_TARGET_PROPERTIES(${LIBRARY_NAME}
    PROPERTIES
    PREFIX "")

  SET_TARGET_PROPERTIES(${LIBRARY_NAME}
    PROPERTIES
    SOVERSION ${PROJECT_VERSION}
    INSTALL_RPATH ${CMAKE_INSTALL_LIBDIR}/plugin)

  # Link with sot-talos-balance library
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${SOTTALOSBALANCE_LIB_NAME})
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${Boost_LIBRARIES})

  ADD_DEPENDENCIES(${LIBRARY_NAME} ${SOTTALOSBALANCE_LIB_NAME})

  IF(ADDITIONAL_${LIBRARY_NAME}_LIBS)
    ADD_DEPENDENCIES(${LIBRARY_NAME} ${ADDITIONAL_${LIBRARY_NAME}_LIBS})
    TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${ADDITIONAL_${LIBRARY_NAME}_LIBS})
  ENDIF(ADDITIONAL_${LIBRARY_NAME}_LIBS)

  # Linux dynamic loading library flags
  IF(UNIX)
    TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${CMAKE_DL_LIBS})
    IF(NOT APPLE)
      TARGET_LINK_LIBRARIES(${LIBRARY_NAME} dl pthread)
    ENDIF(NOT APPLE)
  ENDIF(UNIX)

  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} dynamic-graph::dynamic-graph)
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} sot-core::sot-core)
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} pinocchio)

  # build python submodule
  # (replace minus with underscore to make Python happy)
  IF(BUILD_PYTHON_INTERFACE)
    STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${LIBRARY_NAME})
    SOT_TALOS_BALANCE_PYTHON_MODULE(
      ${PYTHON_LIBRARY_NAME}
      ${LIBRARY_NAME}
      sot-talos-balance-${PYTHON_LIBRARY_NAME}-wrap
      )
  ENDIF(BUILD_PYTHON_INTERFACE)
  # Install plugins
  IF (NOT INSTALL_PYTHON_INTERFACE_ONLY)
    INSTALL(TARGETS ${LIBRARY_NAME}
      DESTINATION ${CMAKE_INSTALL_LIBDIR}/plugin)
  ENDIF (NOT INSTALL_PYTHON_INTERFACE_ONLY)
ENDFOREACH(plugin)

# Bindings Python
IF(BUILD_PYTHON_INTERFACE)
  SOT_TALOS_BALANCE_PYTHON_MODULE("" ${SOTTALOSBALANCE_LIB_NAME} wrap)

  # Install empty __init__.py files in intermediate directories.
  INSTALL(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${SOTTALOSBALANCE_PYNAME}/__init__.py
    DESTINATION ${PYTHON_SITELIB}/${SOTTALOSBALANCE_PYNAME}
    )
ENDIF(BUILD_PYTHON_INTERFACE)
