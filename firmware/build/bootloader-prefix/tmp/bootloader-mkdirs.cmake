# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/matthew/programs/esp/esp-idf/components/bootloader/subproject"
  "/home/matthew/projects/luke_unitree/firmware/build/bootloader"
  "/home/matthew/projects/luke_unitree/firmware/build/bootloader-prefix"
  "/home/matthew/projects/luke_unitree/firmware/build/bootloader-prefix/tmp"
  "/home/matthew/projects/luke_unitree/firmware/build/bootloader-prefix/src/bootloader-stamp"
  "/home/matthew/projects/luke_unitree/firmware/build/bootloader-prefix/src"
  "/home/matthew/projects/luke_unitree/firmware/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/matthew/projects/luke_unitree/firmware/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/matthew/projects/luke_unitree/firmware/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
