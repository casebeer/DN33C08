'''
Eletechsup DN33C08 Pi Pico 2040 "PLC" expansion board demo

- Pin mappings from DN33C08 video on YouTube
- TODO: Add support for 4 digit 7-segment display
- TODO: Get formal board documentation from Eletechsup

'''

import time

from machine import Pin, UART

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
SR0_SER = 11#15 # data
#SR0_OE = SR1_OE = GND # output enable
#SR0_SRCLR = SR1_SRCLR = 36 # VCC # clear shift register/master reset (high)

SR_DATA = SR0_SER
SR_LATCH = SR0_RCLK
SR_CLK = SR0_SRCLK
#SR_OUTPUT_ENABLE = SR0_OE
#SR_RESET = SR0_SRCLR

UART0_BAUD = 9600
UART0_TX_PIN = 0
UART0_RX_PIN = 1

PICO_LED_PIN = 25

MODBUS_DEVICE_ADDRESS = 1


def main():
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

def shift_registers():
  # 7-segment digits
  # left to right, active LOW
  digit1 = 0b0111
  digit2 = 0b1011
  digit3 = 0b1101
  digit4 = 0b1110

  # 7-segment numerals
  # Pattern below, active HIGH
  #  _     8
  # |_|  6   7
  # |_|.   1 
  #      5   2
  #        4  3
  #
  symbols = {
    '0': 0b11111010 << 4,
    '1': 0b01000010 << 4,
    '2': 0b11011001 << 4,
    '3': 0b11001011 << 4,
    '4': 0b01100011 << 4,
    '5': 0b10101011 << 4,
    '6': 0b10111011 << 4,
    '7': 0b11000010 << 4,
    '8': 0b11111011 << 4,
    '9': 0b11101011 << 4,
    'a': 0b11110011 << 4,
    'b': 0b00111011 << 4,
    'c': 0b00011001 << 4,
    'd': 0b01011011 << 4,
    'e': 0b10111001 << 4,
    'f': 0b10110001 << 4,
    'g': 0b11101011 << 4, # nine
    'h': 0b00110011 << 4,
    'i': 0b00000010 << 4,
    'j': 0b01001010 << 4,
    'l': 0b00111000 << 4,
    'n': 0b00010011 << 4,
    'o': 0b00011011 << 4,
    'p': 0b11110001 << 4,
    'q': 0b11111110 << 4, # ambiguous with "0."
    'r': 0b00010001 << 4,
    's': 0b10101011 << 4, # five
    't': 0b00111001 << 4,
    'u': 0b00011010 << 4,
    'y': 0b01101011 << 4,
    'z': 0b11011001 << 4, # two
    '.': 0b00000100 << 4,
    '_': 0b00001000 << 4,
    '-': 0b00000001 << 4,
    '‾': 0b10000000 << 4, # overbar
    '≡': 0b10001001 << 4, # triple bar
    '=': 0b00001001 << 4,
    '═': 0b10000001 << 4, # double horizontal box drawing, used for "upper" equals
    '|': 0b00110000 << 4, # pipe, ambiguous with "1"
    '‖': 0b01110010 << 4, # double vertical bar
    ' ': 0b00000000 << 4,
    '[': 0b10111000 << 4,
    ']': 0b11001010 << 4,
  }
  

  digits = [ digit1, digit2, digit3, digit4 ]
  alnum = "0123456789....abcdefghijklmnopqrstuvwxyz._-‾≡=|‖[] "
  alnum = "_-‾-"
  alnum = "_=≡=_ "
  alnum = "_=≡═‾═≡=_"
  MISSING_SYMBOL_REPLACEMENT = '-'

  # five char buffer, but we only show leftmost four, and use final char
  # as a lookahead so we can merge '.' characters into the previous char
  chars = [0, 0, 0, 0, 0]

  for char in repeat(alnum.lower()): # + " " * 4):
    if char == '.':
      # special case the dot, since we need to look ahead and
      # merge with current final character (usually)
      if chars[-1] & symbols.get('.', 0xff) == 0:
        # don't merge with another dot or dotted char!
        chars[-1] = chars[-1] | symbols.get('.', 0)
        continue
    chars.append(symbols.get(char, symbols.get(MISSING_SYMBOL_REPLACEMENT, 0)))
    chars = chars[-5:]
    start = time.ticks_ms()
    # TODO: move shift register/charlieplexing code to PIO
    while time.ticks_diff(time.ticks_ms(), start) < 100:
      for i in range(4):
        display(digits[i] | chars[i])

def repeat(iterable):
  while True:
    for item in iterable:
      yield item

def bits(bitmap):
  for i in range(12):
    yield (bitmap >> i) & 1

def display(bitmap):
  sr_latch.off()
  sr_data.off()
  for data in bits(bitmap):
    sr_clk.off()
    sr_data.value(data)
    #print(f"{data}: {sr_data.value()} {sr_clk.value()}")

    #time.sleep_ms(500)
    sr_clk.on()
    #print(f"{i}: {sr_data.value()} {sr_clk.value()}")

    #time.sleep_ms(500)
  sr_latch.on()
  #sr_output_enable.on()


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
  shift_registers()
