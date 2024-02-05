
import time
from array import array
from machine import Pin
from rp2 import PIO, StateMachine, asm_pio

from dma import Dma, set_execctrl_status_sel
from .symbols import symbols

SR_LATCH = 9  #SR0_RCLK
SR_CLK   = 10 #SR0_SRCLK
SR_DATA  = 11 #SR0_SER

pio = False
pio = True
dma = False
dma = True

bitarray = array('I', [0] * 4)
def shift_registers():
  init_shift_registers()
  run_display_demo()
def init_shift_registers():

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
        print("DMA handler not set up yet.")
    else:
      print(f"PIO IRQ: {sm.irq().flags()} put:{['{:012b}'.format(w) for w in bitarray]}")
      sm.put(bitarray)
      return

  # Have MOV x, STATUS trigger when TX FIFO has < 1 items left
  set_execctrl_status_sel(pio=0, stateMachine=0, rxFifo=False, level=2)
  sm0.irq(handler=irq_handler)
  sm0.active(1)

def run_display_demo():
  global bitarray
  # 7-segment digits
  # left to right, active LOW
  digit1 = 0b0111
  digit2 = 0b1011
  digit3 = 0b1101
  digit4 = 0b1110

  digits = [ digit1, digit2, digit3, digit4 ]
  alnum = "_-‾-"
  alnum = "_=≡=_ "
  alnum = "_=≡═‾═≡=_"
  alnum = "0123456789....abcdefghijklmnopqrstuvwxyz._-‾≡=|‖[] "
  MISSING_SYMBOL_REPLACEMENT = '-'

  # five char buffer, but we only show leftmost four, and use final char
  # as a lookahead so we can merge '.' characters into the previous char
  chars = [0, 0, 0, 0, 0]

  #display(digit4 | symbols['9'])
  #displayPio(sm0, digit4 | symbols['9'])
  #return

  # loop the 'alnum' message forever
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

    # serialize current active char buffer's first four chars
    # so we can send bits to shift registers
    buffer = [ d | c for d, c in zip(digits, chars[:4]) ]
    #arr = bytearray([0x00] * 4 * 3 + [ 0x00, 0x00, 0x01, 0xff ] )
    arr = array('H', buffer)
    print(arr)
    print(len(arr))
    bitarray = arr

    # push the new bits out to the shift registers
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

class Display(object):
  def __init__(self, latch, clock, data, status):
    self.dma_handler = None
  def update(self, message):
    self.dma_handler = dma_display(buffer)
  def clear(self):
    pass
  def handle_irq(self):
    if self.dma_handler:
      self.dma_handler.feed_dma()

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

dma_handler = None

def dma_display(buffer):
  global dma_handler
  dma_handler = Dma(channel=0, buffer=buffer)
  dma_handler.feed_dma()
  return dma_handler

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

def repeat(iterable):
  while True:
    for item in iterable:
      yield item

def bits(bitmap):
  for i in range(12):
    yield (bitmap >> i) & 1


