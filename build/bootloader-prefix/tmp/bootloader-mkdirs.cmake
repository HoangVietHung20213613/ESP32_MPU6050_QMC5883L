# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Espressif/frameworks/esp-idf-v5.3.1/components/bootloader/subproject"
  "C:/da2/esp-idf-mpu6050-dmp-main/esp-idf-mpu6050-dmp-main/ESP32_MPU6050_QMC5883L/build/bootloader"
  "C:/da2/esp-idf-mpu6050-dmp-main/esp-idf-mpu6050-dmp-main/ESP32_MPU6050_QMC5883L/build/bootloader-prefix"
  "C:/da2/esp-idf-mpu6050-dmp-main/esp-idf-mpu6050-dmp-main/ESP32_MPU6050_QMC5883L/build/bootloader-prefix/tmp"
  "C:/da2/esp-idf-mpu6050-dmp-main/esp-idf-mpu6050-dmp-main/ESP32_MPU6050_QMC5883L/build/bootloader-prefix/src/bootloader-stamp"
  "C:/da2/esp-idf-mpu6050-dmp-main/esp-idf-mpu6050-dmp-main/ESP32_MPU6050_QMC5883L/build/bootloader-prefix/src"
  "C:/da2/esp-idf-mpu6050-dmp-main/esp-idf-mpu6050-dmp-main/ESP32_MPU6050_QMC5883L/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/da2/esp-idf-mpu6050-dmp-main/esp-idf-mpu6050-dmp-main/ESP32_MPU6050_QMC5883L/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/da2/esp-idf-mpu6050-dmp-main/esp-idf-mpu6050-dmp-main/ESP32_MPU6050_QMC5883L/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
