
#from ctypes import c_uint32 as uint32

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

def uint32(value):
  return 0xffffffff & value

class RegisterField():
  '''
  Metadata and access helpers for fields within 32-bit bitfield registers
  '''
  def __init__(self, bit_position:int, bit_size:int=1, type_:type=None):
    self.bit_position = uint32(bit_position)
    self.bit_size = uint32(bit_size)
    self.public_name = None
    if type_ is not None:
      self.type_ = type_
    else:
      if self.bit_size == 1:  
        self.type_ = bool
      else:
        self.type_ = int
  def bitmask(self):
    '''Bitmask for this field'''
    return uint32(~(0xfffffffe << (self.bit_size - 1)) << self.bit_position)
  def __set_name__(self, owner, name):
    self.public_name = name
  def __get__(self, obj, objtype=None):
    return self.type_((obj.register_value & self.bitmask()) >> self.bit_position)
  def __set__(self, obj, value):
    obj.register_value &= ~self.bitmask()
    obj.register_value |= (uint32(value) << self.bit_position) & self.bitmask()
  def __repr__(self):
    return f"RegisterField({'' if self.bit_size == 1 else f'{self.bit_size} bit '}{self.type_.__name__} @ bit {self.bit_position})"

class Alias():
  def __init__(self, target):
    self.target = target
    self.name = None
  def __set_name__(self, owner, name):
    self.name = name
  def __get__(self, obj, objtype=None):
    return getattr(obj, self.target)
  def __set__(self, obj, value):
    setattr(obj, self.target, value)
  def __repr__(self):
    return f"AliasedRegisterField({self.name} => {self.target})"

class BitfieldRegister():
  DEFAULT_REGISTER_VALUE = 0
  def __init__(self, register_value=None):
    if register_value is None:
      self.register_value = self.DEFAULT_REGISTER_VALUE
    else:
      self.register_value = uint32(register_value)

  def fields(self):
    '''Return the names of the register fields wrapped by this object'''
    for attr, value in vars(type(self)).items():
      if isinstance(value, RegisterField) or isinstance(value, Alias):
        yield attr

  def value(self):
    '''Return the value of the register as a uint32'''
    return uint32(self.register_value)

def not_implemented(*args, **kwargs):
  '''Helper to throw errors for abstract base classes'''
  raise NotImplementedError()

class RegisterBuilder():
  '''Abstract base class for chainable register builder helpers'''
  register_class = not_implemented # must override
  def __init__(self, register_value=None):
    self.register = self.register_class(register_value)
  def value(self):
    '''Return the value of the register as a uint32'''
    return uint32(self.register.value())
  def _make_setter(self, name):
    def setter(value=None):
      if value is None:
        # act as getter not setter
        return getattr(self.register, name)
      setattr(self.register, name, value)
      # support chaining API
      return self
    return setter
  def __getattr__(self, name):
    if name in self.register.fields():
      return self._make_setter(name)
    raise AttributeError(f"'{type(self).__name__}' object has no attribute '{name}'")

class CtrlRegister(BitfieldRegister):
  '''
  Wrapper for bitfields within the DMA CTRL registers
  '''
  busy = RegisterField(24)
  sniff_en = RegisterField(23)
  bswap = RegisterField(22)
  irq_quiet = RegisterField(21)
  treq_sel = RegisterField(15, 6)
  chain_to = RegisterField(11, 4)
  ring_sel = RegisterField(10) # False => wrap reads; True => wrap writes
  ring_size = RegisterField(6, 4)
  incr_write = RegisterField(5)
  incr_read = RegisterField(4)
  data_size = RegisterField(2, 2)
  high_priority = RegisterField(1)
  en = RegisterField(0)

  enable = Alias('en')
  wrap_writes = Alias('ring_sel')

  # TODO: Set defaults
  DEFAULT_REGISTER_VALUE = 0

class CtrlBuilder(RegisterBuilder):
  '''
  Chainable helper to set raw DMA CTRL register fields
  '''
  register_class = CtrlRegister


'''
ctrl_builder = CtrlBuilder()\
                 .treq_sel(dma.addresses.Dreq().Pio(0).tx_fifo(0))\
                 .irq_quiet(False)\
                 .incr_write(False)\
                 .incr_read(True)\
                 .ring_sel(False)\
                 .ring_size(4)\
                 .data_size(2)\
                 .en(True)
'''

#from pprint import pprint
#foo = RegisterField(24, type_=bool)
#print(f'{foo.bitmask():32b}')
#bar = RegisterField(15, 6)
#print(f'{bar.bitmask():32b}')
#
#c = CtrlBuilder()
#pprint(type(c.register).__dict__)
##pprint(list(c.fields()))
##pprint('treq_sel' in c.fields())
#pprint(c.treq_sel)
#pprint(c.enable())
#pprint(c.enable(True).enable())
#
#pprint(f"{c.ring_size(7).enable(True).incr_read(True).value():32b}")
#
#import code
#code.interact(local=locals())
#
