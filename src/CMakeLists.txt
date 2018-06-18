set(prog stm32_app)

FILE(GLOB stm32f4xxHal ${CMAKE_SOURCE_DIR}/system/src/stm32f4xx/*.c)
FILE(GLOB stm32f4xxHalLegacy ${CMAKE_SOURCE_DIR}/system/src/stm32f4xx/Legacy/*.c)
FILE(GLOB cmsis ${CMAKE_SOURCE_DIR}/system/src/cmsis/*.c)
FILE(GLOB cortexm ${CMAKE_SOURCE_DIR}/system/src/cortexm/*.c)
FILE(GLOB diag ${CMAKE_SOURCE_DIR}/system/src/diag/*.c)
FILE(GLOB newlib ${CMAKE_SOURCE_DIR}/system/src/newlib/*.c)
FILE(GLOB cmsisASM ${CMAKE_SOURCE_DIR}/system/src/cmsis/*.S)
FILE(GLOB W5100 ${CMAKE_SOURCE_DIR}/Ethernet/W5100/*.c)
FILE(GLOB Ethernet ${CMAKE_SOURCE_DIR}/Ethernet/*.c)
FILE(GLOB MQTTClient ${CMAKE_SOURCE_DIR}/MQTT/*.c)
FILE(GLOB MQTTPacket ${CMAKE_SOURCE_DIR}/MQTT/MQTTPacket/src/*.c)

add_executable(${prog} ${CMAKE_CURRENT_SOURCE_DIR}/main.c ${CMAKE_CURRENT_SOURCE_DIR}/stm32f4xx_hal_msp.c ${CMAKE_CURRENT_SOURCE_DIR}/stm32f4xx_it.c ${stm32f4xxHal} ${stm32f4xxHalLegacy} ${cmsis} ${cmsisASM} ${cortexm} ${diag} ${newlib} ${Ethernet} ${W5100} ${MQTTClient} ${MQTTPacket})

target_include_directories (${prog} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include 	
											${CMAKE_CURRENT_SOURCE_DIR}/../system/include
											${CMAKE_CURRENT_SOURCE_DIR}/../system/include/cmsis
											${CMAKE_CURRENT_SOURCE_DIR}/../system/include/stm32f4xx
											${CMAKE_CURRENT_SOURCE_DIR}/../system/include/stm32f4xx/Legacy	
											${CMAKE_CURRENT_SOURCE_DIR}/../system/include/Device
											${CMAKE_CURRENT_SOURCE_DIR}/../system/include/arm
											${CMAKE_CURRENT_SOURCE_DIR}/../system/include/cortexm
											${CMAKE_CURRENT_SOURCE_DIR}/../system/include/diag
											${CMAKE_CURRENT_SOURCE_DIR}/../system/src/newlib
											${CMAKE_CURRENT_SOURCE_DIR}/../Ethernet/W5100
											${CMAKE_CURRENT_SOURCE_DIR}/../Ethernet
											${CMAKE_CURRENT_SOURCE_DIR}/../MQTT/MQTTPacket/src
											${CMAKE_CURRENT_SOURCE_DIR}/../MQTT
											${CMAKE_CURRENT_SOURCE_DIR}
											)
