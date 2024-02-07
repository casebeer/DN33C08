'''
Eletechsup DN33C08 Pi Pico 2040 "PLC" expansion board demo

'''

import time
from machine import Pin, UART

import seven_segment

#import dn33c08_modbus

# CONSTANTS
INPUT_PINS = [3, 4, 5, 6, 7, 8, 14, 15] # Input pins #1-8 (`PULL_UP`)
OUTPUT_PINS = [13, 12, 28, 27, 26, 19, 17, 16] # Output relay pins #1-8
BUTTON_PINS = [18, 20, 21, 22] # Key/button pins #1–4 (`PULL_UP`)
RS485_RW_PIN = 2 # HDX control pin. 0=read, 1=write. Important: Delay 15 ms before returning to read mode.
RS485_WRITE_COOLDOWN_MS = 15

# 7-segment LED shift registers
# 2x SM74HC595D like SN74HC595 shift registers
# 4x FJ3461AH 7-segment LEDs
# 12 bits of SR outputs used
# first 4 bits in are digit-enable pins (active low)
# next 8 bits are segment-enable (A-G + decimal point) pins shared amongs all digits (active high)
#
# Bit order:
# digit4
# digit3
# digit2
# digit1
# G
# C
# DP
# D
# E
# F
# B
# A
#  _     8
# |_|  6   7
# |_|.   1 
#      5   2
#        4  3
# 
# Digit order: 1 2 3 4
#
SR0_RCLK = SR1_RCLK = 9 #12 # latch - high copies shift register to storage register
SR0_SRCLK = SR1_SRCLK = 10 #14 # shift register load clock, loads value of DATA on falling edge
SR0_SER = 11 #15 # data
#SR0_OE = SR1_OE = GND # output enable
#SR0_SRCLR = SR1_SRCLR = 36 # VCC # clear shift register/master reset (high)

SR_LATCH = 9  #SR0_RCLK
SR_CLK   = 10 #SR0_SRCLK
SR_DATA  = 11 #SR0_SER

#SR_OUTPUT_ENABLE = SR0_OE
#SR_RESET = SR0_SRCLR

UART0_BAUD = 9600
UART0_TX_PIN = 0
UART0_RX_PIN = 1

PICO_LED_PIN = 25

MODBUS_DEVICE_ADDRESS = 1

def main():
  pin_demos()
  shift_register_display_demo()

def shift_register_display_demo():
  '''Run demo on the 8-segement LED display'''
  display = seven_segment.Display(
    latch = Pin(SR_LATCH),
    clock = Pin(SR_CLK),
    data = Pin(SR_DATA),
    status = Pin(25), # Pico built-in LED
  )
  display.message("3.141")
  time.sleep(3)
  display.scroll_message(counter(), loop=False, framerate=8)

def counter(start=0):
  '''
  Generator producing a character stream of space-separated stringified
  decimal numbers counting up from `start`.
  '''
  def helper():
    n = start
    while True:
      yield f"{n} "
      n += 1
  for s in helper():
    for c in s:
      yield c

def pin_demos():
  '''Configure input/output pin demo for DN33C08'''
  # Set interrupts on input pins
  for input_no, pin in enumerate(inputs):
    pin.irq(
      lambda pin, input_no=input_no: print(f"Input pin {input_no} {pin} {'HIGH' if pin.value() else 'LOW'}!"),
      trigger=Pin.IRQ_FALLING # n.b. only IRQ_{RISING,FALLING} are implemented on rp2040
    )
  
  def button_handler(pin, button_no):
    print(f"Button pin {button_no} {pin} {'HIGH' if pin.value() else 'LOW'}!"),
    out = outputs[button_no]
    out.value(not out.value())
  # Set interrupts on button pins
  for button_no, pin in enumerate(buttons):
    pin.irq(
      lambda pin, button_no=button_no: button_handler(pin, button_no),
      trigger=Pin.IRQ_FALLING
    )

  return

  # Loop through relays twice
  for i in range(2):
    for output_no, pin in enumerate(outputs):
      pin.on()
      print(f"Relay output {output_no} ON")
      time.sleep(1)
      pin.off()

def start_modbus():
  '''
  Configure DN33C08 to act as modbus server/device

  We'll answer queries from a modbus client directed to address
  `MODBUS_DEVICE_ADDRESS` over UART0.
  '''
  dn33c08_modbus.serve_modbus(
    device_address=MODBUS_DEVICE_ADDRESS,
    uart_id=0,
    tx_pin_no=UART0_TX_PIN,
    rx_pin_no=UART0_RX_PIN,
    ctrl_pin_no=RS485_RW_PIN,
    inputs=inputs,
    outputs=outputs,
    buttons=buttons,
  )

# PIN CONFIGURATION
inputs =  [Pin(pin, Pin.IN, Pin.PULL_UP)  for pin in INPUT_PINS]
outputs = [Pin(pin, Pin.OUT, value=False) for pin in OUTPUT_PINS]
buttons = [Pin(pin, Pin.IN, Pin.PULL_UP)  for pin in BUTTON_PINS]

rs485_write_pin = Pin(RS485_RW_PIN, Pin.OUT, value=0)
led_pin = Pin(PICO_LED_PIN, Pin.OUT)

uart0 = UART(0, baudrate=UART0_BAUD, tx=Pin(UART0_TX_PIN), rx=Pin(UART0_RX_PIN))

sr_data = Pin(SR_DATA, Pin.OUT, value=0)
#sr_output_enable = Pin(SR_OUTPUT_ENABLE, Pin.OUT, value=0)
sr_clk = Pin(SR_CLK, Pin.OUT, value=0)
sr_latch = Pin(SR_LATCH, Pin.OUT, value=0)
#sr_reset = Pin(SR_RESET, Pin.OUT, value=0)

# Run with `mpremote mount . run dn22c08.py`
# Note we need to mount the CWD since we import a local python file for modbus
# We also need the micropython-modbus dependency installed:
#     mpremote mip install github:brainelectronics/micropython-modbus
if __name__ == '__main__':
  main()
  #start_modbus()
  #shift_registers()


  #
  # sideset = SR_LATCH, SR_CLK

  #shift_registers()

  #while True:
  #  time.sleep(1)
