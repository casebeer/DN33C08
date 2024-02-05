
load:
	mpremote connect /dev/tty.usbmodem14211\
		fs cp -r \
			./main.py ./dn33c08.py \
			./dn33c08_modbus.py ./dma.py ./seven_segment \
			: \
			+ reset
