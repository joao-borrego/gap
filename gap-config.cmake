# Package configuration file

get_filename_component(SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
include(${SELF_DIR}/${CMAKE_BUILD_TYPE}/gap.cmake)
install(FILES gap-config.cmake DESTINATION ${main_lib_dest})
install(EXPORT ...)
