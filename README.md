#Build project:

$mkdir build 

$cd build

$rm -rf * && cmake -DCMAKE_TOOLCHAIN_FILE=/path/to/toolchain_makeFile/toolchain.cmake ../ && make -j4

Binary : stm32_app

Is found in cd /build/src directory


#Flashing the binary on Nucleo stm32F401RE board

	$openocd -f /usr/share/openocd/scripts/board/st_nucleo_f4.cfg -c "program stm32_app verify reset exit"


#Install MQTT on Raspberry pi or On your PC

Start the broker:

$mosquitto -c /etc/mosquitto/mosquitto.conf

start the Mqtt client to subcribe to your stm32 board 

$mosquitto_sub -h hostIP -p port -t topic/name -u username -P pwd
