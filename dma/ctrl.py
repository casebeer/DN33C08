
class Ctrl():
  '''
  Set up a CTRL byte to configure a DMA channel

  See RP2040 datasheet 2.5.1.3 for more details.

  treq int DREQ channel to use for DMA signalling. Must match 
           perhipheral you're using, e.g. provide DREQ_PIO0_TX0 for
           PIO 0/SM 0/TX FIFO.  See RP2040 datasheet 2.5.3
  
  incr_read  int
  incr_write int Configure whether and how the read and/or write addresses
                 should increment on each transfer. 
  '''
  class Offsets():
    '''
    DMA CTRL register bit offsets
    '''
    busy = 24
    sniff_en = 23
    bswap = 22
    irq_quiet = 21
    treq_sel = 15
    chain_to = 11
    ring_sel = 10
    ring_size = 6
    incr_write = 5
    incr_read = 4
    data_size = 2
    high_priority = 1
    en = 0
  # TODO: change signature to avoid long list of args
  def __init__(
    self,
    channel_id,
    treq,
    wrap_write_addrs,
    ring_size_bits,
    incr_write,
    incr_read,
    enable,
    chain_to = None,
    data_size=2,
    ):
    self.channel_id = channel_id
    self.config = 0

    self.treq = treq

    if chain_to is None:
      # set chain to self to disable chaining
      self.chain_to = self.channel_id
    else:
      self.chain_to = chain_to

    self.wrap_write_addrs = wrap_write_addrs
    self.ring_size_bits = ring_size_bits

    self.incr_read = incr_read
    self.incr_write = incr_write

    self.data_size = data_size

    self.enable = enable

  #mem32[DMA_CHANNELS[channel] + DMA_CTRL_TRIG_OFFSET] = ctrl

  def value(self):
    config = 0
    config |= (self.treq & 0x3f) << self.Offsets.treq_sel
    config |= (self.chain_to & 0xf) << self.Offsets.chain_to

    config |= int(self.wrap_write_addrs) << self.Offsets.ring_sel # set ring r/w to wrap read addrs
    config |= (self.ring_size_bits & 0xf) << self.Offsets.ring_size # set ring size in bits to wrap at (in bytes)

    config |= self.data_size << self.Offsets.data_size # set data size, 0 = 1 byte, 1 = 2 bytes, 2 = 4 bytes
    config |= int(self.incr_write) << self.Offsets.incr_write # set no incr write
    config |= int(self.incr_read) << self.Offsets.incr_read # set incr read

    config |= int(self.enable) << self.Offsets.en # enable this DMA channel

    return config

