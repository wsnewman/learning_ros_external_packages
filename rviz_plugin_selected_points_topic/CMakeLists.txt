## BEGIN_TUTORIAL
## This CMakeLists.txt file for rviz_plugin_tutorials builds both the TeleopPanel tutorial and the ImuDisplay tutorial.
##
## First start with some standard catkin stuff.
cmake_minimum_required(VERSION 2.8.3)
project(rviz_plugin_selected_points_topic)
find_package(catkin REQUIRED COMPONENTS rviz)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

string(REPLACE "/" ";" ROS_VER_RAW ${CMAKE_PREFIX_PATH_AS_IS})
list(GET ROS_VER_RAW -1 ROS_VER)

if(${ROS_VER} MATCHES "noetic")

  ## This plugin includes Qt widgets, so we must include Qt like so:
  # find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  find_package(Qt5 COMPONENTS Core Gui UiTools Widgets REQUIRED)
  # include(${QT_USE_FILE})
  include_directories(
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5UiTools_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
  )

  ## I prefer the Qt signals and slots to avoid defining "emit", "slots",
  ## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
  add_definitions(-DQT_NO_KEYWORDS)

  ## Here we specify which header files need to be run through "moc",
  ## Qt's meta-object compiler.
  qt5_wrap_cpp(MOC_FILES
    src/selected_points_topic.h
  )

  ## Here we specify the list of source files, including the output of
  ## the previous command which is stored in ``${MOC_FILES}``.
  set(SOURCE_FILES
    src/selected_points_topic.cpp
    ${MOC_FILES}
  )

  ## An rviz plugin is just a shared library, so here we declare the
  ## library to be called ``${PROJECT_NAME}`` (which is
  ## "rviz_plugin_tutorials", or whatever your version of this project
  ## is called) and specify the list of source files we collected above
  ## in ``${SOURCE_FILES}``.
  add_library(${PROJECT_NAME} ${SOURCE_FILES})

  ## Link the library with whatever Qt libraries have been defined by
  ## the ``find_package(Qt4 ...)`` line above, and with whatever libraries
  ## catkin has included.
  ##
  ## Although this puts "rviz_plugin_tutorials" (or whatever you have
  ## called the project) as the name of the library, cmake knows it is a
  ## library and names the actual file something like
  ## "librviz_plugin_tutorials.so", or whatever is appropriate for your
  ## particular OS.
  target_link_libraries(${PROJECT_NAME} ${Qt5_Core_LIBRARIES} ${Qt5_Gui_LIBRARIES} ${Qt5_UiTools_LIBRARIES} ${Qt5_Widgets_LIBRARIES} ${catkin_LIBRARIES} ${ROS_LIBRARIES})
  ## END_TUTORIAL


  ## Install rules

  install(TARGETS
    ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(FILES 
    plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

elseif(${ROS_VER} MATCHES "melodic")
  ## This plugin includes Qt widgets, so we must include Qt like so:
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})

  ## I prefer the Qt signals and slots to avoid defining "emit", "slots",
  ## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
  add_definitions(-DQT_NO_KEYWORDS)

  ## Here we specify which header files need to be run through "moc",
  ## Qt's meta-object compiler.
  qt4_wrap_cpp(MOC_FILES
    src/selected_points_topic.h
  )

  ## Here we specify the list of source files, including the output of
  ## the previous command which is stored in ``${MOC_FILES}``.
  set(SOURCE_FILES
    src/selected_points_topic.cpp
    ${MOC_FILES}
  )

  ## An rviz plugin is just a shared library, so here we declare the
  ## library to be called ``${PROJECT_NAME}`` (which is
  ## "rviz_plugin_tutorials", or whatever your version of this project
  ## is called) and specify the list of source files we collected above
  ## in ``${SOURCE_FILES}``.
  add_library(${PROJECT_NAME} ${SOURCE_FILES})

  ## Link the library with whatever Qt libraries have been defined by
  ## the ``find_package(Qt4 ...)`` line above, and with whatever libraries
  ## catkin has included.
  ##
  ## Although this puts "rviz_plugin_tutorials" (or whatever you have
  ## called the project) as the name of the library, cmake knows it is a
  ## library and names the actual file something like
  ## "librviz_plugin_tutorials.so", or whatever is appropriate for your
  ## particular OS.
  target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})
  ## END_TUTORIAL


  ## Install rules

  install(TARGETS
    ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(FILES 
    plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

endif()
