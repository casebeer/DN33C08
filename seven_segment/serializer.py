
from array import array

from .symbols import symbols

MISSING_SYMBOL_REPLACEMENT = '-'

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

  # Configure what bits to emit to activate each digit
  # This implicitly defines the layout of the display, e.g number of
  # digits, order of digits, etc.
  digit_bit_patterns = [ digit1, digit2, digit3, digit4 ]
  number_of_digits = len(digit_bit_patterns)

  # For 8-segment displays, we want to special-case dots so we can merge
  # them into the previous digit's dot-segment. Define the dot's symbol
  # here. Set to 0 to disable dot-merging.
  dot_symbol = symbols.get('.', 0)

  def symbolize(self, text):
    '''Convert a string into an generator of symbol ints'''
    return self.merge_dots(
      symbols.get(char.lower(), symbols.get(MISSING_SYMBOL_REPLACEMENT, 0))
        for char in text)

  def merge_dots(self, symbols):
    '''
    Generator to merge dot ('.') symbols into the previous digit's
    symbol. Use for 8-segment displays with dedicated dot segments.

    Accepts an iterable of symbol ints and returns a generator of
    symbol ints.
    '''
    if self.dot_symbol == 0:
      # no dot symbol defined, skip this
      return

    symbol = None
    it = iter(symbols)
    previous = next(it)
    for symbol in it:
      if symbol == self.dot_symbol:
        if previous & self.dot_symbol == 0:
          # only merge if previous wasn't another dot or dotted char!
          previous = previous | self.dot_symbol
          symbol = None
      if previous is not None:
        #print(previous)
        yield previous
      previous = symbol
    if symbol is not None:
      #print(f"symbol: {symbol}")
      yield symbol

  def serialize(self, text):
    '''
    Convert a string into a list of serialized digit
    bit-representations for charlieplexing
    '''
    return self.render(self.symbolize(text))

  # TODO: scroll direction
  def scroll(self, text, lead_in=True):
    '''
    Generator returning a sequence of lists of serialized digit
    bit-representations for charlieplexing.
    '''
    buf = []

    if lead_in:
      buf = [0] * self.number_of_digits

    symbolized = self.symbolize(text)

    for symbol in symbolized:
      buf.append(symbol)
      buf = buf[-self.number_of_digits:] # keep only the last 4 chars

      yield self.render(buf)

  def render(self, symbol_list):
    '''
    Render a list of symbols to the display's digits
    
    Concatenate each symbol's 8 bits (active HIGH, which LED segments
    to light) with 4 bits indicating which of the four digits to light
    (active LOW).
    '''
    buffer = [ d | c for d, c in zip(self.digit_bit_patterns, symbol_list) ]

    # The TX FIFO handles 32-bit words ('L'); we're using one word per
    # displayed digit, even though we only need 12 of the 32 bits.
    return array('L', buffer)

