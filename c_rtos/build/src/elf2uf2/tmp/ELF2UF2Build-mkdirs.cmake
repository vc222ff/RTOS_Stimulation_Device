# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/victor/Programming/2DT304/2dt304-wearable-posture-correction-device/OLD/c_rtos_old/lib/pico-sdk/tools/elf2uf2")
  file(MAKE_DIRECTORY "/home/victor/Programming/2DT304/2dt304-wearable-posture-correction-device/OLD/c_rtos_old/lib/pico-sdk/tools/elf2uf2")
endif()
file(MAKE_DIRECTORY
  "/home/victor/Programming/2DT304/2dt304-wearable-posture-correction-device/OLD/c_rtos_old/build/elf2uf2"
  "/home/victor/Programming/2DT304/2dt304-wearable-posture-correction-device/OLD/c_rtos_old/build/src/elf2uf2"
  "/home/victor/Programming/2DT304/2dt304-wearable-posture-correction-device/OLD/c_rtos_old/build/src/elf2uf2/tmp"
  "/home/victor/Programming/2DT304/2dt304-wearable-posture-correction-device/OLD/c_rtos_old/build/src/elf2uf2/src/ELF2UF2Build-stamp"
  "/home/victor/Programming/2DT304/2dt304-wearable-posture-correction-device/OLD/c_rtos_old/build/src/elf2uf2/src"
  "/home/victor/Programming/2DT304/2dt304-wearable-posture-correction-device/OLD/c_rtos_old/build/src/elf2uf2/src/ELF2UF2Build-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/victor/Programming/2DT304/2dt304-wearable-posture-correction-device/OLD/c_rtos_old/build/src/elf2uf2/src/ELF2UF2Build-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/victor/Programming/2DT304/2dt304-wearable-posture-correction-device/OLD/c_rtos_old/build/src/elf2uf2/src/ELF2UF2Build-stamp${cfgdir}") # cfgdir has leading slash
endif()
