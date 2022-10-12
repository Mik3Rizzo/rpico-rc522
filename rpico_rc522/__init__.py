__version__ = "1.0.0"

try:
    from .rc522 import RC522
    from .rc522manager import RC522Manager
    from .utils import get_block_number, get_block_repr, bytes_to_hex
except RuntimeError:
    print("Must be used on Raspberry Pico")