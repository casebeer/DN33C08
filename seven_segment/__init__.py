
import time
from array import array
import uctypes
import machine
from machine import Pin
import rp2
from rp2 import PIO, StateMachine, asm_pio
import micropython
micropython.alloc_emergency_exception_buf(100)

#from dma import Dma, set_execctrl_status_sel
import dma
import dma.pio
from .symbols import symbols

SR_LATCH = 9  #SR0_RCLK
SR_CLK   = 10 #SR0_SRCLK
SR_DATA  = 11 #SR0_SER

'''
def init_shift_registers():
  # SR_LATCH = 9  #SR0_RCLK
  # SR_CLK   = 10 #SR0_SRCLK
  # SR_DATA  = 11 #SR0_SER
  pins = (Pin(SR_LATCH), Pin(SR_CLK), Pin(SR_DATA), Pin(25))
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
    if dma_handler:
      print(f"PIO IRQ: {sm.irq().flags()} put:{['{:012b}'.format(w) for w in dma_handler.buffer]}")
      #sm.put(dma_handler.buffer)
      #return
      dma_handler.feed_dma()
      #print(f"\tAfter IRQ dma feed: stalled: {is_pio_stalled(0,0)}")
    else:
      print("DMA handler not set up yet.")

  # Have MOV x, STATUS trigger when TX FIFO has < 1 items left
  set_execctrl_status_sel(pio=0, stateMachine=0, rxFifo=False, level=2)
  sm0.irq(handler=irq_handler)
  sm0.active(1)
'''

#def run_display_demo():
def shift_registers():
  pins = (Pin(SR_LATCH), Pin(SR_CLK), Pin(SR_DATA), Pin(25))
  display = Display(*pins)
  #init_shift_registers()
  #run_display_demo()

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

    # push the new bits out to the shift registers
    #dma_display(arr)
    display.update(arr)
    #time.sleep_ms(250)
    time.sleep_ms(1000)

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

  ### trigger PIO IRQ when TX FIFO is low
  ### requires setting EXECCTL correctly
  ### superseded by DMA IRQs in new Micropython version
  #mov(y, status)    #.side(0b00)
  #jmp(not_y, "start")
  #set(pins, 1)
  #irq(block, 0) # send irq and block until mp clears it. Needed since no DMA irqs in MP
  #label("start")
  #set(pins, 0)

  mov(x, null)
  pull(block)
  set(x, 11)                     # reset bit counter for 12 bits
  nop()              .side(0b00) # clear latch in prep for next set of data

  label("loop")
  out(pins, 1)       .side(0b00)
  jmp(x_dec, "loop") .side(0b10)

  nop()              .side(0b01) # set latch high
  wrap()

class Display():
  state_machine_frequency = 125000 # 1908
  def __init__(self, latch, clock, data, status):
    self.dma_channel = None
    self.dma_config = {}
    self.sm = None
    self.message = None

    self.latch = latch
    self.clock = clock
    self.data = data
    self.status = status

    self.retrigger_count = 0

    # TODO: configure
    self.pio_id = 0
    self.state_machine_id = 0
    self.dma_channel_id = 0

    self.init_state_machine()
    self.init_dma()

  def init_state_machine(self):
    '''
    Configure the PIO state machine
    '''
    self.sm = StateMachine(
      self.state_machine_id,
      run_leds,
      freq=self.state_machine_frequency,
      set_base=self.status, #pins[3], # led #pins[0], # latch
      sideset_base=self.latch, #pins[0],
      out_base=self.data, #pins[2],
    )
    self.sm.restart()

    # TODO: implement DMA interrupts
    # Since Micropython doesn't support DMA interrupts, we need
    # our PIO script manually trigger a PIO interrupt based on
    # checking the TX FIFO buffer's remaining transfers using
    # `MOV x, STATUS`. Configure EXECCTRL to support this for our
    # state machine here:

    # trigger `MOV x, STATUS` on TX FIFO
    dma.Pio(self.pio_id).StateMachine(self.state_machine_id)\
      .ExecCtrl().set_status_sel(rx_fifo=False)

    # trigger `MOV x, STATUS` when < 2 transfers remain
    dma.Pio(self.pio_id).StateMachine(self.state_machine_id)\
      .ExecCtrl().set_status_n(level=2)

    # Have MOV x, STATUS trigger when TX FIFO has < 1 items left
    #dma.pio.set_execctrl_status_sel(pio=0, stateMachine=0, rxFifo=False, level=2)

    #self.sm.irq(handler=self.update_dma)
    #PIO(0).irq(lambda pio: print("PIO IRQ", pio.irq().flags()))
    #self.sm.irq(handler=lambda arg: print("SM IRQ", arg))
    #self.sm.irq(handler=isr)
    self.sm.active(1)
  def init_dma(self):
    self.dma = rp2.DMA()
    self.dma.irq(self.update_dma)

  def update(self, message):
    '''
    Set the displayed message

    `message` must be a serialized 
    '''
    state = machine.disable_irq()
    self.message = message
    machine.enable_irq(state)

    print(f"New message '{message}'. Previous message retriggered {self.retrigger_count} times")
    self.retrigger_count = 0 
    
    # TODO: find free DMA channel
    self.dma_channel = dma.Channel(self.dma_channel_id)
    # set all config values here so we don't have to allocate in the ISR
    self.dma_config = {
      'read_addr': uctypes.addressof(self.message),
      'write_addr': dma.addresses.Pio(0).tx_fifo(0), #PIO0_TXF0, # Pio(0).tx_fifo(0),
      'transfer_count': 2**5,
      'ctrl': dma.Ctrl(
        channel_id = 0, #self.dma_channel.channel_id,
        treq = dma.addresses.Dreq().Pio(0).tx_fifo(0), # DREQ_PIO0_TX0, # Dreq().Pio(0).tx_fifo(0)
        enable = True,
        incr_read = True,
        incr_write = False,
        ring_size_bits = 4,
        data_size = 1,
        wrap_write_addrs = False,
      ).value(),
    }
    #self.update_dma()
    self.dma_channel.configure(**self.dma_config)

    #self.dma_handler = Dma(channel=self.dma_channel, buffer=message)
    #self.dma_handler.feed_dma()

  def update_dma(self, isr_arg):
    '''
    Update the DMA channel's config and trigger transfers to start

    This method is used both to initially configure DMA and to reset
    the DMA channel in the interrupt handler.

    Note that since this is an interrupt handler, we can't allocate
    and we want to minimize time spent, so precompute as much as
    possible.

    https://docs.micropython.org/en/latest/reference/isr_rules.html
    '''
    #print(f"\tISR! {isr_arg}")
    self.dma_channel.configure(**self.dma_config)
    self.retrigger_count += 1
    #print(f"\tafter:  ctrl reg = {self.dma_channel.get_ctrl():32b}")

#dma_handler = None
#
#def dma_display(buffer):
#  global dma_handler
#  dma_handler = Dma(channel=0, buffer=buffer)
#  dma_handler.feed_dma()
#  return dma_handler

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


