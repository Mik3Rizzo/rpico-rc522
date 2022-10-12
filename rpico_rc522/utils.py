import binascii


def get_block_number(sector_num: int, relative_block_num: int) -> int:
    """
    Returns the block number starting from the relative block number and the sector number.
    :param sector_num: sector number (from 0 to SECTORS_NUMBER - 1)
    :param relative_block_num: relative block number (from 0 to 3)
    :return number of the block (from 0 to SECTORS_NUMBER * 4 - 1)
    """
    return sector_num * 4 + relative_block_num


def get_block_repr(block_number: int) -> str:
    """
    Returns block representation of a given block address, e.g.
    S01B03 for sector trailer in second sector.
    :return string representation
    """
    return f"S{(block_number - (block_number % 4)) // 4}B{block_number % 4}"


def bytes_to_hex(bytes_to_convert: bytes, separator="") -> str:
    """
    Converts bytes to hex string
    :param bytes_to_convert: bytes to convert
    :param separator: separator between hex values
    :return: hex string
    """
    if separator != "":
        return binascii.hexlify(bytes_to_convert, separator).decode("utf-8")
    else:
        return binascii.hexlify(bytes_to_convert).decode("utf-8")
