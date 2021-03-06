include(nuttx/impl_nuttx)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-none-eabi.cmake)

set(config_module_list
	drivers/device
	drivers/boards/fmu
	drivers/led
	drivers/stm32
	drivers/l3gd20
	drivers/mpu6000
	drivers/lsm303d
	drivers/ms5611
	
	#modules/gpio_led
	platforms/common 
	
	#
	# Library modules
	#
	modules/systemlib
	modules/uORB
	drivers/rgbled
	#test
	systemcmds/tests
	
	systemcmds/mtd
	
	#
	# Libraries
	#
	#lib/mathlib/CMSIS
	lib/mathlib
	lib/conversion
	lib/mathlib/math/filter
	
	platforms/nuttx/px4_layer
	platforms/nuttx
	
)

set(config_extra_builtin_cmds
	serdis
	sercon
	)

set(config_extra_libs
	${CMAKE_SOURCE_DIR}/src/lib/mathlib/CMSIS/libarm_cortexM4lf_math.a
#	uavcan
#	uavcan_stm32_driver
	)


add_custom_target(sercon)
set_target_properties(sercon PROPERTIES
	MAIN "sercon" STACK "2048")

add_custom_target(serdis)
set_target_properties(serdis PROPERTIES
	MAIN "serdis" STACK "2048")
