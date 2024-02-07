
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

# Set to the number of bits to shift per latch pin cycle.
# Overridden by passing pull_thresh to StateMachine.init call.
SHIFT_REGISTER_BITS_DEFAULT=12
MISSING_SYMBOL_REPLACEMENT = '-'

@asm_pio(
  out_init=PIO.OUT_LOW,
  set_init=PIO.OUT_LOW,
  sideset_init=(PIO.OUT_LOW, PIO.OUT_LOW),
  out_shiftdir=PIO.SHIFT_RIGHT,
  pull_thresh=SHIFT_REGISTER_BITS_DEFAULT,
  autopull=False, # see PIO comment for why
)
def run_leds():
  '''
  Pull `pull_thresh` bits of data and shift out, toggling the clock
  pin once per output bit. When done, set latch high to output digit.

  We're using only the lower `pull_thresh` bits from each 32 bit pull.
  Data should be loaded into the TX FIFO one character per FIFO word.

  Each character is made up of 12 bits: 8 bits indicating which 8-segment
  segments to illuminate (active HIGH), and 4 bits indicating which of
  the four digits to illuminate (active LOW). 
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

  # We can't use autopull because we're depending on JMP !OSRE to
  # differentiate between the inner loop, which clocks bits out, and the
  # outer loop, which toggles the latch pin to activate the display.
  #
  # With autopull, the output shift counter gets reset by the autopull
  # before it ever hits the threshold value, so !OSRE is never false.
  #
  # Because of this, we'd have to track bits shifted manually in the PIO
  # program to know when to end the inner loop and toggle the latch,
  # which in turn would prevent us from being able to re-configure the
  # program for different shift register sizes using just the
  # `pull_thresh` field.
  pull(block)           .side(0b01) # side set latch high after wrap

  # reset bit counter for 12 loop iterations
  #set(x, 11)   #      .side(0b01) # side: set latch high after wrap()

  # Main loop. Shift bits to data pin while toggling clock pin with
  # sideset. Latch should remain low while we modify the data.
  #
  # Once we've shifted out `pull_thresh` bits, the JMP will fail and
  # we'll wrap back to the start, where we can pull another word of data
  # and toggle the latch pin high to activate the display.
  label("loop")
  out(pins, 1)          .side(0b00) # side: toggle clock (and for 1st iter, clear latch)
  jmp(not_osre, "loop") .side(0b10) # side: toggle clock
  #jmp(x_dec, "loop")    .side(0b10) # side: toggle clock

  wrap()

class Display():
  state_machine_frequency = 125000 # int(50e6) # 1908 minimum
  def __init__(self, latch, clock, data, status):
    self.dma_channel = None
    self.dma_config = {}
    self.sm = None
    self.buffer = None

    self.latch = latch
    self.clock = clock
    self.data = data
    self.status = status

    self.retrigger_count = 0

    # TODO: configure
    self.pio_id = 0
    self.state_machine_id = 0
    #self.dma_channel_id = 0
    self.shift_register_bits = 12

    self.init_state_machine()
    self.init_dma()

  def message(self, text):
    '''
    Display a message string (or iterable of characters)
    on the 7-segment display. Will truncate the message
    to the size of the display.
    '''
    self.update_raw(self.Serializer.serialize(text))

  def scroll_message(self, text, loop=True, framerate=4, lead_in=True):
    '''
    Scroll a message string (or iterable of characters) on the 7-segment
    display.  Warning, blocks and may not return.

      text str|iterable of chars Message to display.
      loop bool True if message should loop infinitely
      framerate float FPS
      lead_in bool True if text should be led in by a screen of blanks
    '''
    message = text
    if loop:
      message = repeat(message)

    # potentially infinite loop
    for buffer in self.Serializer.scroll(message, lead_in):
      self.update_raw(buffer)
      time.sleep(1./framerate)

  class Serializer():
    '''
    Serializer for DN22C08's 8-segment LED shift registers
    '''
    # 7-segment digits
    # left to right, active LOW
    digit1 = 0b0111
    digit2 = 0b1011
    digit3 = 0b1101
    digit4 = 0b1110

    digits = [ digit1, digit2, digit3, digit4 ]

    dot_symbol = symbols.get('.', 0)

    @classmethod
    def symbolize(cls, text):
      '''Convert a string into an generator of symbol ints'''
      return cls.merge_dots(
        symbols.get(char.lower(), symbols.get(MISSING_SYMBOL_REPLACEMENT, 0))
          for char in text)

    @classmethod
    def merge_dots(cls, symbols):
      '''
      Generator to merge dot ('.') symbols into the previous digit's
      symbol. Use for 8-segment displays with dedicated dot segments.

      Accepts an iterable of symbol ints and returns a generator of
      symbol ints.
      '''
      if cls.dot_symbol == 0:
        # no dot symbol defined, skip this
        return

      symbol = None
      it = iter(symbols)
      previous = next(it)
      for symbol in it:
        if symbol == cls.dot_symbol:
          if previous & cls.dot_symbol == 0:
            # only merge if previous wasn't another dot or dotted char!
            previous = previous | cls.dot_symbol
            symbol = None
        if previous is not None:
          #print(previous)
          yield previous
        previous = symbol
      if symbol is not None:
        #print(f"symbol: {symbol}")
        yield symbol

    @classmethod
    def serialize(cls, text):
      '''
      Convert a string into a list of serialized digit
      bit-representations for charlieplexing
      '''
      return cls.render(cls.symbolize(text))

    # TODO: scroll direction
    @classmethod
    def scroll(cls, text, lead_in=True):
      '''
      Generator returning a sequence of lists of serialized digit
      bit-representations for charlieplexing.
      '''
      buf = []

      if lead_in:
        buf = [0, 0, 0, 0]

      symbolized = cls.symbolize(text)

      for symbol in symbolized:
        buf.append(symbol)
        buf = buf[-4:] # keep only the last 4 chars

        yield cls.render(buf)

    @classmethod
    def render(cls, symbol_list):
      '''
      Render a list of symbols to the display's digits
      
      Concatenate each symbol's 8 bits (active HIGH, which LED segments
      to light) with 4 bits indicating which of the four digits to light
      (active LOW).
      '''
      buffer = [ d | c for d, c in zip(cls.digits, symbol_list) ]

      # The TX FIFO handles 32-bit words ('L'); we're using one word per
      # displayed digit, even though we only need 12 of the 32 bits.
      return array('L', buffer)

  def init_state_machine(self):
    '''Configure the PIO state machine'''
    self.sm = PIO(self.pio_id).state_machine(self.state_machine_id)
    self.sm.init(
      run_leds,
      freq = self.state_machine_frequency,
      set_base = self.status, # used only for debugging
      sideset_base = self.latch, # self.clock should be the next pin
      out_base = self.data,
      pull_thresh = self.shift_register_bits,
    )

    self.sm.restart()
    self.sm.active(1)

  def init_dma(self):
    '''Configure DMA channel and set IRQ handler'''
    self.dma = rp2.DMA()
    self.dma.irq(self.configure_dma)

  def update_raw(self, buffer):
    '''
    Set the displayed message as a serialized array of packed bytes

    You probably want Display.message() or Display.scroll() instead.
    '''
    state = machine.disable_irq()
    self.buffer = buffer
    machine.enable_irq(state)

    print(f"\tPrevious message retriggered DMA {self.retrigger_count} times")
    print(f"New message: {buffer}")
    self.retrigger_count = -1

    # set all config values here so we don't have to allocate in the ISR
    self.dma_config = {
      'read': uctypes.addressof(self.buffer),
      'write': dma.addresses.Pio(0).tx_fifo(0),
      'count': 2**20,
      'ctrl': self.dma.pack_ctrl(
        treq_sel = dma.addresses.Dreq().Pio(0).tx_fifo(0), # DREQ_PIO0_TX0, # Dreq().Pio(0).tx_fifo(0)
        enable = True,
        irq_quiet = False, # we're using DMA IRQs to re-trigger DMA
        inc_read = True,
        inc_write = False,
        size = 1, # 0=byte, 1=half word, 2=word (default: 2)
        ring_size = 4, # bits
        ring_sel = False, # ``False`` to have the ``ring_size`` apply to the read address or ``True`` to apply to the write address.
      ),
      'trigger': True,
    }

    # TODO: find free DMA channel
    #self.dma_channel = dma.Channel(self.dma_channel_id)
    self.dma_config_old = {
      'read_addr': uctypes.addressof(self.buffer),
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
    # IMPORTANT: must abort dma channel if it's active already
    if self.dma.active():
      self.dma.active(False)
    self.configure_dma()
    #self.dma_channel.configure(**self.dma_config_old)
    #self.dma.config(**self.dma_config)

    #self.dma_handler = Dma(channel=self.dma_channel, buffer=message)
    #self.dma_handler.feed_dma()

  def configure_dma(self, isr_arg=None):
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
    #self.dma_channel.configure(**self.dma_config_old)

    self.dma.config(**self.dma_config)

    self.retrigger_count += 1
    #print(f"\tafter:  ctrl reg = {self.dma_channel.get_ctrl():32b}")

def repeat(iterable):
  '''Helper to infinitely repeat an iterable'''
  while True:
    for item in iterable:
      yield item

