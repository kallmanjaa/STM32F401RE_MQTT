cmake_minimum_required(VERSION 3.10)

ENABLE_LANGUAGE (ASM)
set(UNIX 1)

project(NucleoF401RE)

SET(CMAKE_C_FLAGS  "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wextra  -funwind-tables -g3 -DDEBUG -DTRACE -DSTM32F401xE -std=gnu11 -nostartfiles -Xlinker --gc-sections -Wl,-Map=stm32_app.map " CACHE INTERNAL "c compiler flags")

SET(CMAKE_CXX_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wextra  -g3 -DDEBUG -DTRACE -DSTM32F401xE -T${CMAKE_CURRENT_SOURCE_DIR}/ldscripts/libs.ld -T${CMAKE_CURRENT_SOURCE_DIR}/ldscripts/mem.ld -T${CMAKE_CURRENT_SOURCE_DIR}/ldscripts/sections.ld -nostartfiles -Xlinker --gc-sections -Wl,-Map=stm32_app.map -specs=nano.specs -specs=nosys.specs " CACHE INTERNAL "cxx compiler flags")

SET(CMAKE_ASM_FLAGS "-mcpu=cortex-m4 -mthumb" CACHE INTERNAL "asm compiler flags")
SET (CMAKE_EXE_LINKER_FLAGS ${CMAKE_CXX_FLAGS} CACHE INTERNAL "exe link flags" )

add_subdirectory(src)
