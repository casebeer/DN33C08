'''
Eletechsup DN33C08 Pi Pico 2040 "PLC" expansion board demo

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
  dma = False
  dma = True

  print(f"pio={pio} dma={dma}")

  # SR_LATCH = 9  #SR0_RCLK
  # SR_CLK   = 10 #SR0_SRCLK
  # SR_DATA  = 11 #SR0_SER
  if pio:
    pins = (Pin(SR_LATCH), Pin(SR_CLK), Pin(SR_DATA), Pin(25))
  else:
    pins = (inputs[7], inputs[6], inputs[5], Pin(25))
  #pins = (inputs[7], inputs[6], inputs[5], Pin(25)) # leds only
  #pins = (Pin(SR_LATCH), Pin(SR_CLK), Pin(25), Pin(4)) # flash data
  sm0 = StateMachine(
    0,
    run_leds,
    freq=125000, #1908,
    set_base=pins[3], # led #pins[0], # latch
    sideset_base=pins[0], 
    out_base=pins[2], 
  ) 
  sm0.restart()
  def irq_handler(sm):
    global dma_handler
    if dma:
      if dma_handler:
        print(f"PIO IRQ: {sm.irq().flags()} put:{['{:012b}'.format(w) for w in dma_handler.buffer]}")
        #sm.put(dma_handler.buffer)
        #return
        dma_handler.feed_dma()
        #print(f"\tAfter IRQ dma feed: stalled: {is_pio_stalled(0,0)}")
      else:
        print(f"DMA handler not set up yet.")
    else:
      print(f"PIO IRQ: {sm.irq().flags()} put:{['{:012b}'.format(w) for w in bitarray]}")
      sm.put(bitarray)
      return
   
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
    buffer = [ 0x000, 0xffffffff, 0x00000000, 0xffffffff, ] # 16 * 4 = 64 bits; 2 * 4 = 8 bytes; 2**3 = 8 addrs byte-wise; 
    buffer = [ 0, 0, 0, 0x000001ff]
    buffer = [ 0xffffffff, 0xffffffff, 0xffffffff, 0 ]
    buffer = [ 0, 0x000f,  0, 0] 
    buffer = [ 0, 0, 0, 1, ]
    buffer = [ 0, 0, 1, 0, ]
    buffer = [ 1, 0, 0, 0, ]
    buffer = [ 0, (1 << 0), 0, 0, ]
    buffer = [ d | c for d, c in zip(digits, chars[:4]) ]
    #arr = bytearray([0x00] * 4 * 3 + [ 0x00, 0x00, 0x01, 0xff ] )
    arr = array('H', buffer)
    print(arr)
    print(len(arr))
    bitarray = arr
    if pio and dma:
      start = time.ticks_ms()
      while time.ticks_diff(time.ticks_ms(), start) < 100:
        dma_display(bitarray)
        #for bits in buffer:
        #    sm0.put(bits)
        break
      time.sleep_ms(250)
    else:
      start = time.ticks_ms()
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
DMA_AL1_CTRL_OFFSET = 0x10
DMA_CHAN_ABORT_OFFSET = 0x444

# DMA CTRL register bit offset
DMA_CTRL_BUSY = 24 # 1 = dma is busy, do not trigger
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
  mem32[DMA_CHANNELS[channel] + DMA_READ_ADDR_OFFSET] = read_addr
  mem32[DMA_CHANNELS[channel] + DMA_WRITE_ADDR_OFFSET] = write_addr
  mem32[DMA_CHANNELS[channel] + DMA_TRANS_COUNT_OFFSET] = transfer_count 
  mem32[DMA_CHANNELS[channel] + DMA_CTRL_TRIG_OFFSET] = ctrl

def reset_dma_ctrl(channel):
  #mem32[channel + DMA_TRANS_COUNT_OFFSET] = 0
  mem32[DMA_CHANNELS[channel] + DMA_AL1_CTRL_OFFSET ] &= 0 # reset CTRL without re-triggering using Alias1


def make_dma_ctrl(
  channel,
  treq,
  enable,
  incr_read,
  incr_write,
  ring_size_bits,
  wrap_write_addrs,
  data_size=2,
  ):
  config = 0
  config |= (treq & 0x3f) << DMA_CTRL_TREQ_SEL # set treq
  config |= (channel & 0xf) << DMA_CTRL_CHAIN_TO # set chain to self to disable chaining
  config |= int(wrap_write_addrs) << DMA_CTRL_RING_SEL # set ring r/w to wrap read addrs
  config |= (ring_size_bits & 0xf) << DMA_CTRL_RING_SIZE # set ring size in bits to wrap at (in bytes)
  config |= data_size << DMA_CTRL_DATA_SIZE # set data size, 0 = 1 byte, 1 = 2 bytes, 2 = 4 bytes
  config |= int(incr_write) << DMA_CTRL_INCR_WRITE # set no incr write
  config |= int(incr_read) << DMA_CTRL_INCR_READ # set incr read
  config |= int(enable) << DMA_CTRL_EN # enable

  return config

def dma_ctrl(channel):
  return mem32[DMA_CHANNELS[channel] + DMA_CTRL_TRIG_OFFSET]

def dma_busy(channel):
  return bool((dma_ctrl(channel) & (1 << DMA_CTRL_BUSY)))

def pause_dma(channel):
  mem32[DMA_CHANNELS[channel] + DMA_CTRL_TRIG_OFFSET] &= ~(1 << DMA_CTRL_EN)
  
SUBSYSTEM_RESET_BASE = 0x4000c000
SUBSYSTEM_RESET_OFFSET = 0
SUBSYSTEM_RESET_DONE_OFFSET = 8 

SUBSYSTEM_RESET = SUBSYSTEM_RESET_BASE + SUBSYSTEM_RESET_OFFSET
SUBSYSTEM_RESET_DONE = SUBSYSTEM_RESET_BASE + SUBSYSTEM_RESET_DONE_OFFSET

SUBSYSTEM_RESET_BIT_DMA = 2
SUBSYSTEM_RESET_BIT_PIO0 = 10
SUBSYSTEM_RESET_BIT_PIO1 = 11

def reset_dma_subsystem():
  print("Resetting DMA subsystem...")
  dma_mask = 1 << SUBSYSTEM_RESET_BIT_DMA
  mem32[SUBSYSTEM_RESET] |= dma_mask
  while mem32[SUBSYSTEM_RESET_DONE] & dma_mask:
    print(f"Waiting for DMA subsystem to reset... {mem32[SUBSYSTEM_RESET_DONE]:08x}")

DMA_ABORT_WAIT_LIMIT = 20
def abort_dma(channel):
  chan_abort_reg = DMA_BASE + DMA_CHAN_ABORT_OFFSET
  if dma_busy(channel):
    print(f"Aborting DMA channel {channel}...")
    pause_dma(channel)
    mem32[chan_abort_reg] |= (1 << channel)
    counter = 0
    while dma_busy(channel):
      if counter > DMA_ABORT_WAIT_LIMIT:
        reset_dma_subsystem()
        break
      print(f"Waiting for DMA channel {channel} to not be busy {dma_ctrl(channel):08x}, chan_abort:{mem32[chan_abort_reg]:08x}...")
      time.sleep_ms(100)
      mem32[chan_abort_reg] |= (1 << channel)
      counter += 1
  return mem32[chan_abort_reg]

class Dma(object):
  def __init__(self, channel, buffer):
    self.buffer = buffer
    self.channel = channel
    print(f"Setting up dma with buffer: {buffer}")
    print(f"\tDMA is {'BUSY' if dma_busy(self.channel) else 'not busy'} during setup. ctrl reg = 0x{dma_ctrl(self.channel):08x}")
    #reset_dma_ctrl(self.channel)
    abort_dma(self.channel)
    #print(f"CHAN_ABORT: {abort_dma(0):08x}")
      #abort_dma(self.channel)
  def feed_dma(self):
    #print(f"Feeding DMA... stalled: {is_pio_stalled(0,0)}")
    print(f"Feeding DMA\n\tbefore: ctrl reg = 0x{dma_ctrl(self.channel):08x}")

    configure_dma(
      channel= self.channel,
      read_addr = uctypes.addressof(self.buffer),
      write_addr = PIO0_TXF0,
      transfer_count = 2**20,
      ctrl = make_dma_ctrl(
        channel=self.channel,
        treq=DREQ_PIO0_TX0,
        enable=True,
        incr_read=True,
        incr_write=False,
        ring_size_bits = 4,
        data_size = 1,
        wrap_write_addrs = False,
      ),
    )
    print(f"\tafter:  ctrl reg = 0x{dma_ctrl(self.channel):08x}")

dma_handler = None

def dma_display(buffer):
  global dma_handler
  dma_handler = Dma(channel=0, buffer=buffer)
  dma_handler.feed_dma()

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
  mov(y, status)    #.side(0b00)
  jmp(not_y, "start")
  irq(block, 0) # send irq and block until mp clears it. Needed since no DMA irqs in MP
  set(pins, 1)

  label("start")
  mov(x, null)
  pull(block)
  set(x, 11)                     # reset bit counter for 12 bits
  nop()              .side(0b00) # clear latch in prep for next set of data

  label("loop")
  out(pins, 1)       .side(0b00)
  jmp(x_dec, "loop") .side(0b10)

  nop()              .side(0b01) # set latch high
  set(pins, 0)
  wrap()
  return
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


  # 
  # sideset = SR_LATCH, SR_CLK

  #shift_registers()

  #while True:
  #  time.sleep(1)
