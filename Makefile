
UART=/dev/tty.usbmodem14211
load:
	mpremote connect $(UART) \
		fs cp -r \
			./main.py ./dn33c08.py ./dn33c08_modbus.py \
			./dma ./seven_segment \
			: \
			+ reset

mount:
	mpremote connect $(UART) mount . run main.py
