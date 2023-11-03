'''
Eletechsup DN33C08 Pi Pico 2040 "PLC" expansion board demo

- Pin mappings from DN33C08 video on YouTube
- TODO: Add support for 4 digit 7-segment display
- TODO: Get formal board documentation from Eletechsup

'''

import time

from array import array
from machine import Pin, UART
from rp2 import PIO, StateMachine, asm_pio
import uctypes

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
  shift_registers()

def pin_demos():
  # Set interrupts on input pins
  for input_no, pin in enumerate(inputs):
    if input_no == 5:
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
bitarray = array('I', [0] * 4)
def shift_registers():
  global bitarray
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
  alnum = "_-‾-"
  alnum = "_=≡=_ "
  alnum = "_=≡═‾═≡=_"
  alnum = "0123456789....abcdefghijklmnopqrstuvwxyz._-‾≡=|‖[] "
  MISSING_SYMBOL_REPLACEMENT = '-'

  # five char buffer, but we only show leftmost four, and use final char
  # as a lookahead so we can merge '.' characters into the previous char
  chars = [0, 0, 0, 0, 0]

  pio = False
  pio = True
  dma = True
  dma = False

  print(f"pio={pio} dma={dma}")

  # SR_LATCH = 9  #SR0_RCLK
  # SR_CLK   = 10 #SR0_SRCLK
  # SR_DATA  = 11 #SR0_SER
  if pio:
    pins = (Pin(SR_LATCH), Pin(SR_CLK), Pin(SR_DATA))
  else:
    pins = (inputs[7], inputs[6], inputs[5])
  #pins = (inputs[7], inputs[6], inputs[5])
  sm0 = StateMachine(
    0,
    run_leds,
    freq=1908, #1250000, #1908,
    set_base=Pin(25), # led #pins[0], # latch
    sideset_base=pins[0], 
    out_base=pins[2], 
  ) 
  def irq_handler(sm):
    global dma_handler
    print(f"PIO IRQ: {sm.irq().flags()} put:{['{:012b}'.format(w) for w in bitarray]}")
    sm.put(bitarray)
    return
    if dma_handler:
      sm.put(dma_handler.buffer)
      return
      dma_handler.feed_dma()
      print(f"\tAfter IRQ dma feed: stalled: {is_pio_stalled(0,0)}")
    else:
      print(f"DMA handler not set up yet.")
  # Have MOV x, STATUS trigger when TX FIFO has < 1 items left
  set_execctrl_status_sel(pio=0, stateMachine=0, rxFifo=False, level=2)
  sm0.irq(handler=irq_handler)
  sm0.active(1)

  #display(digit4 | symbols['9'])
  #displayPio(sm0, digit4 | symbols['9'])
  #return

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

    #buffer = [ 0xaaaaaaaa for i in range(4) ]
    buffer = [ 0x003f for i in range(4) ]
    buffer = [ 0xfffffc00 for i in range(4) ]
    buffer = [ 0xfffff000 for i in range(4) ]
    buffer = [ 0xfc0 for i in range(4) ]
    buffer = [ 0xff0 for i in range(4) ]
    buffer = [ d | c for d, c in zip(digits, chars[:4]) ]
    arr = array('H', buffer)
    if pio and dma:
      start = time.ticks_ms()
      while time.ticks_diff(time.ticks_ms(), start) < 100:
        dma_display(array('H', buffer))
        #for bits in buffer:
        #    sm0.put(bits)
        break
      time.sleep_ms(1000)
    else:
      # TODO: move shift register/charlieplexing code to PIO
      start = time.ticks_ms()
      bitarray = arr
      while time.ticks_diff(time.ticks_ms(), start) < 500:
          if pio:
            #sm0.put(arr)
            #print(f'loop put: {arr}')
            #sm0.put(bitarray)
            continue
          for bits in bitarray:
            if pio:
              #displayPio(sm0, bits)
              sm0.put(bits)
            else:
              display(bits)


# DMA Addresses
DMA_BASE = 0x50000000

DMA_COUNT = 12
DMA_CHANNEL_OFFSETS = [ 0x40 * i for i in range(DMA_COUNT) ]
DMA_CHANNELS = [ DMA_BASE + offset for offset in DMA_CHANNEL_OFFSETS ]

DMA_READ_ADDR_OFFSET = 0x0
DMA_WRITE_ADDR_OFFSET = 0x4
DMA_TRANS_COUNT_OFFSET = 0x8
DMA_CTRL_TRIG_OFFSET = 0xc

# DMA CTRL register bit offset
DMA_CTRL_TREQ_SEL = 15  # 6 bits, 20-15. DMA DREQ to use to trigger transfers
DMA_CTRL_CHAIN_TO = 11 # 4 bits, 14-11. DMA channel # to chain to, set to self to disable chaining
DMA_CTRL_RING_SEL = 10 # 0 = read addresses wrap, 1 = write addresses wrap
DMA_CTRL_RING_SIZE = 6 # 4 bits, 9-6. 0 = don't wrap. Specifies _number of address bits_ to wrap.
DMA_CTRL_DATA_SIZE = 2 # 2 bits, 3-2. 0 = byte, 1 = halfword (2 bytes), 2 = word (4 bytes)
DMA_CTRL_INCR_WRITE = 5 # 1 = write addr incrs with each xfer. 0 = write to same address repeatedly
DMA_CTRL_INCR_READ = 4 # 1 = read addr incrs with each xfer. 0 = read from same address repeatedly
DMA_CTRL_EN = 0

# DREQ definitions
DREQ_PIO0_TX0 = 0
DREQ_PIO0_RX0 = 4
DREQ_PIO1_TX0 = 8
DREQ_PIO1_RX0 = 12

# PIO FIFO addresses
PIO0_BASE = 0x50200000
PIO1_BASE = 0x50300000
PIO_BASE = [ PIO0_BASE, PIO1_BASE ]

PIO_TXF0_OFFSET  = 0x010
PIO_TXF1_OFFSET  = 0x014
PIO_TXF2_OFFSET  = 0x018
PIO_TXF3_OFFSET  = 0x01c
PIO_RXF0_OFFSET  = 0x020

PIO0_TXF0 = PIO0_BASE + PIO_TXF0_OFFSET
PIO0_TXF1 = PIO0_BASE + PIO_TXF1_OFFSET
PIO0_TXF2 = PIO0_BASE + PIO_TXF2_OFFSET
PIO0_TXF3 = PIO0_BASE + PIO_TXF3_OFFSET
PIO1_TXF0 = PIO1_BASE + PIO_TXF0_OFFSET
PIO0_RXF0 = PIO0_BASE + PIO_RXF0_OFFSET
PIO1_RXF0 = PIO1_BASE + PIO_RXF0_OFFSET

# We need 4 * 12 bits = 48 bits = 6 bytes of data to charlieplex all four
# display digits. We'll put 12 bits in each of the four FIFO words, 
# and do one pull per display digit since we don't need the extra bits we'd
# get from packing data into the 32 bit words.
#
from machine import mem32

def configure_dma(channel, read_addr, write_addr, transfer_count, ctrl):
  mem32[channel + DMA_READ_ADDR_OFFSET] = read_addr
  mem32[channel + DMA_WRITE_ADDR_OFFSET] = write_addr
  mem32[channel + DMA_TRANS_COUNT_OFFSET] = transfer_count 
  mem32[channel + DMA_CTRL_TRIG_OFFSET] = ctrl

def make_dma_ctrl(
  channel,
  treq,
  enable,
  incr_read,
  incr_write,
  ring_size_bits,
  wrap_write_addrs,
  ):
  config = 0
  config |= (treq & 0x3f) << DMA_CTRL_TREQ_SEL # set treq
  config |= (channel & 0xf) << DMA_CTRL_CHAIN_TO # set chain to self to disable chaining
  config |= int(wrap_write_addrs) << DMA_CTRL_RING_SEL # set ring r/w to wrap read addrs
  config |= (ring_size_bits & 7) << DMA_CTRL_RING_SIZE # set ring size in bits to wrap at 4 reads (not bytes)
  config |= 2 << DMA_CTRL_DATA_SIZE # set data size
  config |= int(incr_write) << DMA_CTRL_INCR_WRITE # set no incr write
  config |= int(incr_read) << DMA_CTRL_INCR_READ # set incr read
  config |= int(enable) << DMA_CTRL_EN # enable

  return config

class Dma(object):
  def __init__(self, channel, buffer):
    self.buffer = buffer
    self.channel = channel
    print(f"Setting up dma with buffer: {buffer}")
  def feed_dma(self):
    print(f"Feeding DMA... stalled: {is_pio_stalled(0,0)}")
    configure_dma(
      DMA_CHANNELS[self.channel],
      read_addr = uctypes.addressof(self.buffer),
      write_addr = PIO0_TXF0,
      transfer_count = 4, #2**20,
      ctrl = make_dma_ctrl(
        channel=self.channel,
        treq=DREQ_PIO0_TX0,
        enable=True,
        incr_read=True,
        incr_write=False,
        ring_size_bits = 2,
        wrap_write_addrs = False,
      ),
    )

dma_handler = None

def dma_display(buffer):
  global dma_handler
  dma_handler = Dma(channel=0, buffer=buffer)
  #dma_handler.feed_dma()

def displayPio(sm0, bitmap):
  #print(f"Tx FIFO bytes: {sm0.tx_fifo()}")
  sm0.put(bitmap)
  #time.sleep(1)
  #sm0.put(array('H', [bitmap]))
  #sm0.put(array('H', [bitmap] * 400))

# tx fifo 4x 32 bit
# rx fifo 4x 32 bit
# osr 32 bit
# isr 32 bit
# x 32 bit
# y 32 bit
#
# PIO STATUS_SEL

SM0_EXECCTRL_OFFSET = 0xcc
SM1_EXECCTRL_OFFSET = 0xe4
SM2_EXECCTRL_OFFSET = 0xfc
SM3_EXECCTRL_OFFSET = 0x114
SM_EXECCTRL_OFFSET = [ SM0_EXECCTRL_OFFSET, SM1_EXECCTRL_OFFSET, SM2_EXECCTRL_OFFSET, SM3_EXECCTRL_OFFSET ]

# EXECTRL bit offsets
EXECCTRL_EXEC_STALLED = 31
EXECCTRL_STATUS_SEL = 4 # 0 = TX FIFO, 1 = RX FIFO. MOV x, STATUS  will be All-ones if FIFO level < N, otherwise all-zeroes
EXECCTRL_STATUS_N = 0 # 4 bits, 3–0. FIFO level below which STATUS_SEL should be true

def set_execctrl_status_sel(pio=0, stateMachine=0, rxFifo=False, level=1):
  reg = PIO_BASE[pio] + SM_EXECCTRL_OFFSET[stateMachine]
  reg_mask = (1 << EXECCTRL_STATUS_SEL) | (0xf << EXECCTRL_STATUS_N)

  ctrl = 0
  ctrl |= int(rxFifo) << EXECCTRL_STATUS_SEL # 0 = tx, 1 = rx
  ctrl |= (1 & 0xf) << EXECCTRL_STATUS_N # 0 bit shift

  mem32[reg] &= ~reg_mask # clear bit we're going to set
  mem32[reg] |= ctrl # set bits

def is_pio_stalled(pio=0, stateMachine=0):
  reg = PIO_BASE[pio] + SM_EXECCTRL_OFFSET[stateMachine]
  return bool(mem32[reg] & (1 << EXECCTRL_EXEC_STALLED))

@asm_pio(
  out_init=PIO.OUT_LOW,
  set_init=PIO.OUT_LOW,
  sideset_init=(PIO.OUT_LOW, PIO.OUT_LOW), # PIO.OUT_LOW, 
  out_shiftdir=PIO.SHIFT_RIGHT,
  #pull_thresh=12,
  #autopull=True,
)
def run_leds():
  '''
  Autopull 12 bits of data and shift out toggling clock pin once per
  output bit. When done, set latch high to output digit. 
  '''
  wrap_target()
  set(pins, 1)
  mov(y, status)
  jmp(not_y, "start")
  irq(block, 0) # send irq and block until mp clears it. Needed since no DMA irqs in MP

  label("start")
  mov(x, null)
  pull(block)
  nop()              .side(0b00) # clear latch in prep for next set of data
  set(x, 11)                     # reset bit counter for 12 bits

  label("loop")
  out(pins, 1)       .side(0b00) [3]
  jmp(x_dec, "loop") .side(0b10) [3]

  nop()              .side(0b01) [3] # set latch high
  set(pins, 0)
  nop() [3]
  nop() [3]
  nop() [3]
  nop() [3]
  nop() [3]
  nop() [3]
  nop() [3]
  nop() [3]
  nop() [3]
  nop() [3]
  nop() [3]
  nop() [3]
  nop() [3]
  nop() [3]
  nop() [3]
  nop() [3]
  nop() [3]
  nop() [3]
  nop() [3]
  nop() [3]
  wrap()

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

def repeat(iterable):
  while True:
    for item in iterable:
      yield item

def bits(bitmap):
  for i in range(12):
    yield (bitmap >> i) & 1



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


  # pins = SR_DATA
  # sideset = SR_LATCH, SR_CLK

  #shift_registers()

  #while True:
  #  time.sleep(1)
