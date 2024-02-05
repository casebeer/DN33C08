import uctypes

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

