# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/rob/esp-idf/components/bootloader/subproject"
  "/home/rob/Documents/Github/shed-scripts/T500 Still/Flow Meter/build/bootloader"
  "/home/rob/Documents/Github/shed-scripts/T500 Still/Flow Meter/build/bootloader-prefix"
  "/home/rob/Documents/Github/shed-scripts/T500 Still/Flow Meter/build/bootloader-prefix/tmp"
  "/home/rob/Documents/Github/shed-scripts/T500 Still/Flow Meter/build/bootloader-prefix/src/bootloader-stamp"
  "/home/rob/Documents/Github/shed-scripts/T500 Still/Flow Meter/build/bootloader-prefix/src"
  "/home/rob/Documents/Github/shed-scripts/T500 Still/Flow Meter/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/rob/Documents/Github/shed-scripts/T500 Still/Flow Meter/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/rob/Documents/Github/shed-scripts/T500 Still/Flow Meter/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
