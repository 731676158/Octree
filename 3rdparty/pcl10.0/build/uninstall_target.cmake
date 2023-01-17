if(NOT EXISTS "C:/files/codes/git/Octree/3rdparty/pcl10.0/build/install_manifest.txt")
  message(FATAL_ERROR "Cannot find install manifest: \"C:/files/codes/git/Octree/3rdparty/pcl10.0/build/install_manifest.txt\"")
endif()

file(READ "C:/files/codes/git/Octree/3rdparty/pcl10.0/build/install_manifest.txt" files)
string(REGEX REPLACE "\n" ";" files "${files}")
foreach(file ${files})
  message(STATUS "Uninstalling \"$ENV{DESTDIR}${file}\"")
  if(EXISTS "$ENV{DESTDIR}${file}" OR IS_SYMLINK "$ENV{DESTDIR}${file}")
    exec_program("C:/Program Files/CMake/bin/cmake.exe" ARGS "-E remove \"$ENV{DESTDIR}${file}\""
                 OUTPUT_VARIABLE rm_out RETURN_VALUE rm_retval)
    if(NOT "${rm_retval}" STREQUAL 0)
      message(FATAL_ERROR "Problem when removing \"$ENV{DESTDIR}${file}\"")
    endif()
  else()
    message(STATUS "File \"$ENV{DESTDIR}${file}\" does not exist.")
  endif()
endforeach()

# remove pcl directory in include (removes all files in it!)
message(STATUS "Uninstalling \"C:/Program Files (x86)/PCL/include/pcl-1.10\"")
if(EXISTS "C:/Program Files (x86)/PCL/include/pcl-1.10")
  exec_program("C:/Program Files/CMake/bin/cmake.exe"
               ARGS "-E remove_directory \"C:/Program Files (x86)/PCL/include/pcl-1.10\""
               OUTPUT_VARIABLE rm_out RETURN_VALUE rm_retval)
  if(NOT "${rm_retval}" STREQUAL 0)
    message(FATAL_ERROR "Problem when removing \"C:/Program Files (x86)/PCL/include/pcl-1.10\"")
  endif()
else()
  message(STATUS "Directory \"C:/Program Files (x86)/PCL/include/pcl-1.10\" does not exist.")
endif()

# remove pcl directory in share (removes all files in it!)
# created by CMakeLists.txt for PCLConfig.cmake
if(EXISTS "C:/Program Files (x86)/PCL/cmake")
  file(GLOB_RECURSE CMAKE_CONFIG_FOLDER_FILES FOLLOW_SYMLINKS
       LIST_DIRECTORIES false
       "C:/Program Files (x86)/PCL/cmake/*")
  list(LENGTH CMAKE_CONFIG_FOLDER_FILES CMAKE_CONFIG_FOLDER_FILES_NUMBER)
  if(CMAKE_CONFIG_FOLDER_FILES_NUMBER EQUAL 0)
    message(STATUS "Uninstalling \"C:/Program Files (x86)/PCL/cmake\"")
    exec_program("C:/Program Files/CMake/bin/cmake.exe"
                 ARGS "-E remove_directory \"C:/Program Files (x86)/PCL/cmake\""
                 OUTPUT_VARIABLE rm_out RETURN_VALUE rm_retval)
    if(NOT "${rm_retval}" STREQUAL 0)
      message(FATAL_ERROR "Problem when removing \"C:/Program Files (x86)/PCL/cmake\"")
    endif()
  endif()
else()
  message(STATUS "Directory \"C:/Program Files (x86)/PCL/cmake\" does not exist.")
endif()

# remove pcl directory in share/doc (removes all files in it!)
if(OFF)
  message(STATUS "Uninstalling \"C:/Program Files (x86)/PCL/share/doc/pcl-1.10\"")
  if(EXISTS "C:/Program Files (x86)/PCL/share/doc/pcl-1.10")
    exec_program("C:/Program Files/CMake/bin/cmake.exe"
                 ARGS "-E remove_directory \"C:/Program Files (x86)/PCL/share/doc/pcl-1.10\""
                 OUTPUT_VARIABLE rm_out RETURN_VALUE rm_retval)
    if(NOT "${rm_retval}" STREQUAL 0)
      message(FATAL_ERROR "Problem when removing \"C:/Program Files (x86)/PCL/share/doc/pcl-1.10\"")
    endif()
  else()
    message(STATUS "Directory \"C:/Program Files (x86)/PCL/share/doc/pcl-1.10\" does not exist.")
  endif()
endif()
