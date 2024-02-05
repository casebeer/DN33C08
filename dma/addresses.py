
#
# Base address for all DMA configuration
_DMA_BASE = 0x50000000

_DMA_COUNT = 12
_DMA_CHANNEL_OFFSETS = [ 0x40 * i for i in range(_DMA_COUNT) ]

# Array of starting addresses for each of the 12 DMA channels
DMA_CHANNELS = [ _DMA_BASE + offset for offset in _DMA_CHANNEL_OFFSETS ]

# Address offsets from the base channel address to access specific
# functions of that DMA channel
DMA_READ_ADDR_OFFSET = 0x0
DMA_WRITE_ADDR_OFFSET = 0x4
DMA_TRANS_COUNT_OFFSET = 0x8
DMA_CTRL_TRIG_OFFSET = 0xc
DMA_AL1_CTRL_OFFSET = 0x10
DMA_CHAN_ABORT_OFFSET = 0x444

class Dma():
  DMA_BASE = _DMA_BASE
  DMA_COUNT = 12
  def address(self):
    return self.DMA_BASE
  class Channel():
    CHANNEL_INCREMENT = 0x40
    CTRL_TRIG_OFFSET = DMA_CTRL_TRIG_OFFSET
    def __init__(self, channel_id):
      self.channel_id = channel_id
    def address(self):
      return Dma().address() + self.channel_id * self.CHANNEL_INCREMENT
    def ctrl_trig(self):
      return self.address() + self.CTRL_TRIG_OFFSET
  class Abort():
    CHAN_ABORT_OFFSET = DMA_CHAN_ABORT_OFFSET
    def address(self):
      return Dma().address() + self.CHAN_ABORT_OFFSET

class PioBase():
  PIO_BASE = (0x50200000, 0x50300000)
  TX_F0 = 0x010
  RX_F0 = 0x020
  TX_INCREMENT = 4
  RX_INCREMENT = 4
  def __init__(self, pio_id):
    self.pio_id = int(pio_id)
  def addr(self):
    '''Return base address of this PIO's registers'''
    return self.PIO_BASE[self.pio_id]
  def tx_fifo(self, num):
    '''Return address of specified TX FIFO'''
    return self.addr() + self.TX_F0 + int(num) * self.TX_INCREMENT
  def rx_fifo(self, num):
    '''Return address of specified RX FIFO'''
    return self.addr() + self.RX_F0 + int(num) * self.RX_INCREMENT

class Pio(PioBase):
  '''
  Provide selected register addresses for the PIO perhipherals

  See RP2040 datasheet 3.7
  '''
  SM0_EXECCTRL = 0x0cc
  EXECCTRL_INCREMENT = 24
  def execctrl(self, num):
    '''
    Return the address of the EXECCTRL register for the specified
    state machine
    '''
    return self.SM0_EXECCTRL + num * self.EXECCTRL_INCREMENT

class Dreq():
  '''
  DREQ "Data Request" channel IDs for providing to the DMA CTRL register
  TREQ_SEL field.
  '''
  class Pio(PioBase):
    PIO_BASE = (0, 8)
    TX_F0 = 0
    RX_F0 = 4
    TX_INCREMENT = 1
    RX_INCREMENT = 1
  class _RxTx():
    BASE = 0
    def __init__(self, num):
      self.num = int(num)
    def tx(self):
      return self.BASE + self.num * 2 + 0
    def rx(self):
      return self.BASE + self.num * 2 + 1
  class Spi(_RxTx):
    BASE = 16
  class Uart(_RxTx):
    BASE = 20
  class Pwm():
    BASE = 24
    def wrap(self, num):
      return self.BASE + int(num)
  class I2c(_RxTx):
    BASE = 32
  ADC = 36
  XIP_STREAM = 37
  XIP_SSITX = 38
  XIP_SSIRX = 39

