
#from ctypes import c_uint32 as uint32

def uint32(value):
  return 0xffffffff & value

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

class RegisterField():
  '''
  Metadata and access helpers for fields within 32-bit bitfield registers
  '''
  def __init__(self, bit_position:int, bit_size:int=1, _type:type=int):
    self.bit_position = uint32(bit_position)
    self.bit_size = uint32(bit_size)
    self.public_name = None
    self._type = _type
  def bitmask(self):
    '''Bitmask for this field'''
    return uint32(~(0xfffffffe << (self.bit_size - 1)) << self.bit_position)
  def __set_name__(self, owner, name):
    self.public_name = name
    #self.private_name = f"_{name}"
  def __get__(self, obj, objtype=None):
    #return getattr(obj, self.private_name)
    return self._type((obj.register_value & self.bitmask()) >> self.bit_position)
  def __set__(self, obj, value):
    #setattr(obj, self.private_name, value)
    obj.register_value &= ~self.bitmask()
    obj.register_value |= (uint32(value) << self.bit_position) & self.bitmask()
  def __repr__(self):
    return f"RegisterField({self.bit_size} bit {self._type.__name__} @ bit {self.bit_position})"

class CtrlRegister():
  '''
  Wrapper for bitfields within the DMA CTRL registers
  '''
  busy = RegisterField(24, _type=bool)
  sniff_en = RegisterField(23, _type=bool)
  bswap = RegisterField(22, _type=bool)
  irq_quiet = RegisterField(21, _type=bool)
  treq_sel = RegisterField(15, 6)
  chain_to = RegisterField(11, 4)
  ring_sel = RegisterField(10, _type=bool)
  ring_size = RegisterField(6, 4)
  incr_write = RegisterField(5, _type=bool)
  incr_read = RegisterField(4, _type=bool)
  data_size = RegisterField(2, 2)
  high_priority = RegisterField(1, _type=bool)
  en = RegisterField(0, _type=bool)
  def __init__(self, register_value=0):
    self.register_value = uint32(register_value)

class CtrlBuilder():
  '''
  Chainable helper to set raw DMA CTRL register fields
  '''
  def __init__(self, register_value):
    self.register = CtrlRegister(register_value)
  def value(self):
    return uint32(self.register.register_value)
  def fields(self):
    for attr, value in vars(type(self.register)).items():
      if isinstance(value, RegisterField):
        yield attr
  def make_setter(self, name):
    def setter(value):
      setattr(self.register, name, value)
      # support chaining API
      return self
    return setter
  def __getattr__(self, name):
    if name in self.fields():
      #return getattr(self.register, name)
      return self.make_setter(name)
    raise AttributeError(f"'{type(self).__name__}' object has no attribute '{name}'")

#from pprint import pprint
#foo = RegisterField(24, _type=bool)
#print(f'{foo.bitmask():32b}')
#bar = RegisterField(15, 6)
#print(f'{bar.bitmask():32b}')
#
#c = CtrlBuilder()
#pprint(type(c.register).__dict__)
#pprint(list(c.fields()))
#pprint('treq_sel' in c.fields())
#pprint(c.treq_sel)
#
#import code
#code.interact(local=locals())
#
