
from machine import mem32
import dma.addresses

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

#def set_execctrl_status_sel(pio=0, stateMachine=0, rxFifo=False, level=1):
#  reg = PIO_BASE[pio] + SM_EXECCTRL_OFFSET[stateMachine]
#  reg_mask = (1 << EXECCTRL_STATUS_SEL) | (0xf << EXECCTRL_STATUS_N)
#
#  ctrl = 0
#  ctrl |= int(rxFifo) << EXECCTRL_STATUS_SEL # 0 = tx, 1 = rx
#  ctrl |= (1 & 0xf) << EXECCTRL_STATUS_N # 0 bit shift
#
#  mem32[reg] &= ~reg_mask # clear bit we're going to set
#  mem32[reg] |= ctrl # set bits

#def is_pio_stalled(pio=0, stateMachine=0):
#  reg = PIO_BASE[pio] + SM_EXECCTRL_OFFSET[stateMachine]
#  return bool(mem32[reg] & (1 << EXECCTRL_EXEC_STALLED))

class Pio():
  def __init__(self, pio_id):
    self.pio_id = pio_id
  def StateMachine(self, sm_id):
    return self._StateMachine(pio_id=self.pio_id, sm_id=sm_id)
  class _StateMachine():
    def __init__(self, pio_id, sm_id):
      self.pio_id = pio_id
      self.sm_id = sm_id
    def ExecCtrl(self):
      '''Factory to instantiate ExecCtrl() instance with outerclass instance vars'''
      return self._ExecCtrl(self.pio_id, self.sm_id)
    class _ExecCtrl():
      class Offsets():
        exec_stalled = 31
        status_sel = 4
        status_n = 0
      def __init__(self, pio_id, state_machine_id):
        self.pio_id = pio_id
        self.state_machine_id = state_machine_id
      def address(self):
        return dma.addresses.Pio(self.pio_id).execctrl(self.state_machine_id)
      def value(self):
        return mem32[self.address()]
      def is_stalled(self):
        '''Return True if the state machine is stalled'''
        return bool(self.value() & (1 << self.Offsets.exec_stalled))

      def set_status_sel(self, rx_fifo=False):
        '''
        Set the STATUS_SEL field

        This will cause status inquiries from PIO programs (i.e. calling
        `MOV x, STATUS`) to apply to either the RX FIFO (by passing
        rx_fifo=True) or the TX FIFO (by passing rx_fifo=False, the default).

        `MOV x, STATUS` will be set x to all ones if FIFO level < N, else
        all zeroes.
        '''
        reg = self.address()
        reg_mask = (1 << EXECCTRL_STATUS_SEL)

        ctrl = 0
        ctrl |= int(rx_fifo) << EXECCTRL_STATUS_SEL # 0 = tx, 1 = rx

        mem32[reg] &= ~reg_mask # clear bit we're going to set
        mem32[reg] |= ctrl # set bits
        pass

      def set_status_n(self, level=1):
        '''
        Set the FIFO level below which STATUS_SEL should be True
        4 bit integer.
        '''
        reg = self.address()
        reg_mask = (0xf << EXECCTRL_STATUS_N)
        ctrl = 0
        ctrl |= (1 & 0xf) << EXECCTRL_STATUS_N # 0 bit shift

        mem32[reg] &= ~reg_mask # clear bit we're going to set
        mem32[reg] |= ctrl # set bits

