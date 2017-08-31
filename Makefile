
all:
	make -C cpu1_bsp
	cp cpu1_bsp/ps7_cortexa9_1/lib/libxil.a pilot_cpu1/libs
	#cp cpu1_bsp/ps7_cortexa9_1/include/* pilot_cpu1/include/standalone/
	make -C pilot_cpu1/build
	./create_boot.sh amp_system/output.bif

bsp:
	make -C cpu1_bsp
	cp cpu1_bsp/ps7_cortexa9_1/lib/libxil.a pilot_cpu1/libs
#	cp cpu1_bsp/ps7_cortexa9_1/include/* pilot_cpu1/include/standalone/
	./create_boot.sh amp_system/output.bif

pilot:
	make -C pilot_cpu1/build
	./create_boot.sh amp_system/output.bif

clean:
	make -C pilot_cpu1/build clean
	make -C cpu1_bsp clean
bsp-clean:
	make -C cpu1_bsp clean
pilot-clean:
	make -C pilot_cpu1/build clean
