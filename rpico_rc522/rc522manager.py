import board
from .rc522 import RC522
from .utils import get_block_number, get_block_repr, bytes_to_hex


class RC522Manager:
    """
    High level class that manages an RC522 RFID Reader connected to a Raspberry Pico.

    Default connection:
        - MOSI to GP3
        - MISO to GP4
        - SCK  to GP2
        - SDA  to GP1
        - RST  to GP0
        - 3.3v and Ground
    """
    DEFAULT_BAUDRATE = 1000000

    DEFAULT_SCK_PIN = board.GP2
    DEFAULT_MOSI_PIN = board.GP3
    DEFAULT_MISO_PIN = board.GP4
    DEFAULT_RST_PIN = board.GP0
    DEFAULT_CS_PIN = board.GP1

    DEFAULT_AUTH_METHOD = RC522.ACT_AUTH_A  # use KEY_A
    DEFAULT_KEY = (0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF)
    DEFAULT_AUTH_BITS = (0xFF, 0x07, 0x80)
    DEFAULT_SECTORS_NUMBER = 16

    # RC522 Status
    STATUS_OK = RC522.STATUS_OK
    STATUS_NO_TAG_ERR = RC522.STATUS_NO_TAG_ERR
    STATUS_ERR = RC522.STATUS_ERR

    def __init__(self, sck_pin=DEFAULT_SCK_PIN, mosi_pin=DEFAULT_MOSI_PIN, miso_pin=DEFAULT_MISO_PIN,
                 rst_pin=DEFAULT_RST_PIN, cs_pin=DEFAULT_CS_PIN, baudrate=DEFAULT_BAUDRATE, debug=False):

        self.reader: RC522 = RC522(sck_pin=sck_pin, mosi_pin=mosi_pin, miso_pin=miso_pin, rst_pin=rst_pin,
                                   cs_pin=cs_pin, baudrate=baudrate, debug=debug)

        self.uid: list[int] | None = None
        self.key: list[int] | None = None
        self.auth_method: int | None = None
        self.last_auth_data: tuple[int, int, list[int], list[int]] | None = None
        
        self.debug: bool = debug

    def scan(self, scan_once: bool = False) -> (int, list[int]):
        """
        Scans for a tag once or until a tag appears.
        It restarts Crypto1 and performs anti-collision.
        :param scan_once: True to scan one time, False to scan until a tag appears
        :return status: 0 = OK, 1 = NO_TAG_ERROR, 2 = ERROR
                uid_data: UID of the tag (4 bytes) concatenated with checksum (1 byte), 5 bytes total
        """
        if self.debug:
            print(f"[d] RC522Manager.scan(scan_once={scan_once}) ...")

        self.reader.restart_crypto()
        uid_data = []

        if scan_once:
            # Request tag once
            (status, tag_type) = self.reader.request_tag()
        else:
            # Wait for the tag
            (status, tag_type) = self.reader.wait_for_tag()

        if status == self.STATUS_OK:  # there is a tag
            # Perform anti-collision
            (status, uid_data) = self.reader.anti_collision()

        return status, uid_data

    def select_tag(self, uid_data: list[int]) -> int:
        """
        Selects a tag.
        Resets the auth if the another UID is already set.
        :param uid_data: UID of the tag (4 bytes) concatenated with checksum (1 byte), 5 bytes total
        :return status: 0 = OK, 1 = NO_TAG_ERROR, 2 = ERROR
        """
        if self.debug:
            print(f"[d] RC522Manager.select_tag(uid_data={bytes_to_hex(bytes(uid_data))}) ...")

        uid = uid_data[0:4]
        if self.uid != uid:
            self.reset_auth()

        status = self.reader.select_tag(uid_data)
        if status == self.STATUS_OK:
            self.uid = uid_data[0:4]
            if self.debug:
                print(f"[d] RC522Manager: Selected UID {bytes_to_hex(bytes(self.uid))}")

        return status

    def set_auth(self, auth_method: int = DEFAULT_AUTH_METHOD, key: list[int] = DEFAULT_KEY):
        """
        Sets the authentication info for the current tag.
        :param auth_method: KEY_A (0x60) or KEY_B (0x61)
        :param key: key of the tag
        """
        self.auth_method = auth_method
        self.key = key

        if self.debug:
            print(f"[d] RC522Manager.set_auth() >>> Set key {bytes_to_hex(bytes(self.key))}, method {'A' if auth_method == self.reader.ACT_AUTH_A else 'B'}")

    def reset_auth(self):
        """
        Resets the authentication info.
        """
        self.auth_method = None
        self.key = None
        self.last_auth_data = None

        if self.debug:
            print("[d] RC522Manager.reset_auth() >>> Reset auth info")

    def is_auth_set(self) -> bool:
        """
        :return: True if the authentication info are set.
        """
        return (self.uid is not None) and (self.key is not None) and (self.auth_method is not None)

    def auth(self, block_number: int, force: bool = False) -> int:
        """
        Authenticates a certain block using the saved auth info, only if needed.
        :param block_number: number of the block (from 0 to SECTORS_NUMBER * 4 - 1)
        :param force: True to force the auth even it is already authenticated
        :return status: 0 = OK, 1 = NO_TAG_ERROR, 2 = ERROR
        """
        if self.debug:
            print(f"[d] RC522Manager.auth(block_number={block_number}, force={force}) ...")

        auth_data = (block_number, self.auth_method, self.key, self.uid)
        status = self.STATUS_OK

        if (self.last_auth_data != auth_data) or force:
            if self.debug:
                print(f"[d] RC522Manager: calling reader.auth() on UID {bytes_to_hex(bytes(self.uid))}")
            self.last_auth_data = auth_data
            status = self.reader.auth(self.auth_method, block_number, self.key, self.uid)
        else:
            if self.debug:
                print("[d] RC522Manager: not calling reader.auth() - already authenticated")

        return status

    def read_block(self, block_number: int) -> (int, list[int]):
        """
        Reads a specific block.
        Note: Tag and auth must be set, since it does auth.
        :param block_number: number of the block (from 0 to SECTORS_NUMBER * 4 - 1)
        :return status: 0 = OK, 1 = NO_TAG_ERROR, 2 = ERROR
                read_data: read data
        """
        if self.debug:
            print(f"[d] RC522Manager.read_block(block_number={block_number}) ...")

        status = self.STATUS_ERR
        read_data = []

        if not self.is_auth_set():
            return status, read_data

        # Do authentication
        status = self.auth(block_number)
        if status == self.STATUS_OK:
            (status, read_data) = self.reader.read_block(block_number)
        else:
            print(f"[e] Error reading {get_block_repr(block_number)}")

        return status, read_data

    def write_block(self, block_number: int, new_bytes: list[int]) -> int:
        """
        Writes bytes to a specific block, keeping the old ones if None is passed.
        Note: Tag and auth must be set, since it does auth.

        Example:
            write_block(block_number=1, new_bytes=[None, 0x1a, None, 0x00])
            will write the second and the fourth byte of the second block, leaving the other 14 bytes unaltered.

        :param block_number: number of the block (from 0 to SECTORS_NUMBER * 4 - 1)
        :param new_bytes: list of bytes to be written
        :return status: 0 = OK, 1 = NO_TAG_ERROR, 2 = ERROR
        """
        if self.debug:
            print(f"[d] RC522Manager.write_block(block_number={block_number}, new_bytes={new_bytes}) ...")

        if not self.is_auth_set():
            return self.STATUS_ERR

        # Do authentication
        status = self.auth(block_number)
        if status == self.STATUS_OK:
            # Read previous block
            (status, block_data) = self.reader.read_block(block_number)
            if status == self.STATUS_OK:
                for i in range(len(new_bytes)):
                    # Overwrite block_data if the new_byte is not None
                    if new_bytes[i] is not None:
                        if self.debug:
                            print(f"[d] Changing byte {i} - from {block_data[i]} to {new_bytes[i]}")
                        block_data[i] = new_bytes[i]

                # Write the new block with changed bytes (block_data)
                status = self.reader.write_block(block_number, block_data)
                if self.debug:
                    print(f"[d] Writing {bytes_to_hex(bytes(block_data))} to {get_block_repr(block_number)}")

        return status

    def write_trailer(self, sector_number: int,
                      key_a: list[int] = DEFAULT_KEY,
                      access_bits: list[int] = DEFAULT_AUTH_BITS,
                      user_data: int = 0x69,
                      key_b: list[int] = DEFAULT_KEY) -> int:
        """
        Writes sector trailer (last block) of specified sector. Tag and auth must be set - does auth.
        If value is None, value of byte is kept.
        :param sector_number: number of the sector
        :param key_a: key A of the tag
        :param key_b: key B of the tag
        :param access_bits: access bits
        :param user_data: eventual user data to append after the access bits
        :return status: 0 = OK, 1 = NO_TAG_ERROR, 2 = ERROR
        """
        block_number = get_block_number(sector_number, relative_block_num=3)
        trailer = key_a[:6] + access_bits[:3] + [user_data] + key_b[:6]
        return self.write_block(block_number, trailer)

    def dump(self, sectors_number: int = DEFAULT_SECTORS_NUMBER) -> (int, list[list[int]]):
        """
        Dumps the entire tag.
        :param sectors_number: number of sectors
        :return: status: 0 = OK, 1 = NO_TAG_ERROR, 2 = ERROR
                 dump_data: dump data
        """
        status = self.STATUS_ERR
        dump_data = []
        for i in range(sectors_number * 4):
            (status, block_data) = self.read_block(i)
            dump_data.append(block_data)

        if self.debug:
            print(f"[d] RC522Manager.dump() >>> status={status}, dump_data={dump_data}")

        return status, dump_data
