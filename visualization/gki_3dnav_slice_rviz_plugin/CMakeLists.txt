
# Convert the Qt Signals and Slots for QWidget events
qt4_wrap_cpp(MOC_SOURCES include/gki_3dnav_slice_rviz_plugin/gki_3dnav_slice_display.h)

set(GKI_3DNAV_LIB_NAME gki_3dnav_slice_rviz_plugin)
add_library(${GKI_3DNAV_LIB_NAME}_core
  src/gki_3dnav_slice_display.cpp 
  ${MOC_SOURCES})
target_link_libraries(${GKI_3DNAV_LIB_NAME}_core moveit_rviz_plugin_render_tools ${catkin_LIBRARIES} ${OGRE_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES})

add_library(${GKI_3DNAV_LIB_NAME} src/plugin_init.cpp)
target_link_libraries(${GKI_3DNAV_LIB_NAME} ${GKI_3DNAV_LIB_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${GKI_3DNAV_LIB_NAME}_core ${GKI_3DNAV_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

