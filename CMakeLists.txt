cmake_minimum_required(VERSION 2.8.3)
project(aruco_mapping)

find_package(OpenCV REQUIRED)
find_package(aruco REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
             roscpp
             message_generation
             image_transport
             cv_bridge
             tf
             visualization_msgs
             camera_calibration_parsers)

SET(MODULES_DIR "modules" )

#BEGIN RESOLVE GIT REVISION
#https://bravenewmethod.com/2017/07/14/git-revision-as-compiler-definition-in-build-with-cmake/
message(STATUS "Resolving GIT Version...")
set(_build_version "unknown")
find_package(Git)
if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    OUTPUT_VARIABLE _build_version
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  message( STATUS "GIT hash: ${_build_version}")
else()
  message(STATUS "GIT not found")
endif()
string(TIMESTAMP _time_stamp)
#END RESOLVE GIT REVISION

#http://stackoverflow.com/questions/7787823/cmake-how-to-get-the-name-of-all-subdirectories-of-a-directory
function(GETPROJECTS result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*/CMakeLists.txt)
  SET(dirlist "")
  FOREACH(file ${children})
        get_filename_component(subdir ${file} DIRECTORY)
        LIST(APPEND dirlist ${subdir})
  ENDFOREACH()
  SET(${result} ${dirlist} PARENT_SCOPE)
endfunction()


####### HANDLE DEPENDENCIES ######

SET(DEPS_DIR ${MODULES_DIR})
SET(TMP0 ${CMAKE_CURRENT_SOURCE_DIR}/${DEPS_DIR})
GETPROJECTS(DEPS ${TMP0})
message(STATUS "Dependencies found in ${TMP0}: ${DEPS}")
FOREACH(subdir ${DEPS})
if(GIT_FOUND)
  SET(SUBPROJECT_PATH ${DEPS_DIR}/${subdir})
  SET(SUBPROJECT_FULL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/${SUBPROJECT_PATH})
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
    WORKING_DIRECTORY ${SUBPROJECT_FULL_PATH}
    OUTPUT_VARIABLE _sub_build_version
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  SET(SUBPROJECT_GIT_NAME ${subdir}_${_sub_build_version})
  message(STATUS "Subproject generated git name: " ${SUBPROJECT_GIT_NAME})
  if (TARGET ${subdir})
      message(WARNING "Subproject ${SUBPROJECT_GIT_NAME} (${subdir}) has already been built!")
  else()
      add_subdirectory(${SUBPROJECT_PATH})
  endif()
else()
  message(STATUS "GIT not found")
endif()
ENDFOREACH()

######## END HANDLE DEPENDENCIES ######

set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11

SET(SRCPATH src/)

SET(${PROJECT_NAME}_INCLUDE_DIRS
    ${catkin_INCLUDE_DIRS}
    include
    )

SET(SOURCES ${SRCPATH}aruco_mapping_main.cpp
            ${SRCPATH}aruco_mapping.cpp
            )


SET(SOURCES2 ${SRCPATH}aruco_mapping_static_main.cpp
            ${SRCPATH}aruco_mapping_static.cpp
            )

add_message_files(FILES ArucoMarker.msg)

generate_messages(DEPENDENCIES
                  std_msgs
                  geometry_msgs)


add_executable(${PROJECT_NAME} ${SOURCES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} aruco_mapping_gencpp )
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} aruco ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_INCLUDE_DIRS})

add_executable(aruco_mapping_fixed ${SOURCES2})
add_dependencies(aruco_mapping_fixed ${catkin_EXPORTED_TARGETS} aruco_mapping_gencpp )
target_link_libraries(aruco_mapping_fixed ${OpenCV_LIBS} aruco ${catkin_LIBRARIES})
target_include_directories(aruco_mapping_fixed PUBLIC ${${PROJECT_NAME}_INCLUDE_DIRS})

install(TARGETS ${PROJECT_NAME} aruco_mapping_fixed
        LIBRARY DESTINATION "lib"
        RUNTIME DESTINATION "bin"
        )
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}" DESTINATION "include")

catkin_package(
  INCLUDE_DIRS ${${PROJECT_NAME}_INCLUDE_DIRS}
  LIBRARIES
)

