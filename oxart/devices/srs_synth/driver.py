import logging
import socket

logger = logging.getLogger(__name__)


class SRSSynth:
    """
    Driver for Stanford Research Systems Synth: SG384. Implements IEEE-488.2 standard functions.
    """
    def __init__(self, addr, port=5025, serial_number=None):
        # addr : IP address of *device*
        self.addr = addr

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.addr, port))

        # Store identity as a list of the comma separated fields returned
        self.idn = self.identity().split(',')

        # Some devices may not implement *IDN? in the same way, but most place
        # serial number in this field, allowing us to check that we have the
        # correct device
        if serial_number is not None:
            if self.idn[2] != serial_number:
                raise ValueError("Serial number {} did not match expected ({})"
                                 "".format(self.idn[2], serial_number))

    def close(self):
        self.sock.close()
        self.sock = None

    def send(self, cmd):
        data = cmd + "\n"
        self.sock.send(data.encode())

    def query(self, cmd):
        self.send(cmd)
        with self.sock.makefile() as f:
            response = f.readline().strip()
        return response

    def ping(self):
        # TODO - maybe check errors here?
        self.identity()
        return True

    def get_freq(self):
        return self.query("FREQ?")

    def get_phase(self):
        return self.query("PHAS?")

    def get_amplitude(self):
        # returns amplitude in units of Vpp
        return self.query("AMPR? VPP")

    def set_freq(self,freq):
        return self.send("FREQ {}".format(freq))

    def set_phase(self,phase):
        return self.send("PHAS {}".format(phase))

    def set_amplitude(self,amplitude):
        # sets amplitude in units of Vpp
        return self.send("AMPR {} VPP".format(amplitude))



    # IEEE-488.2 functions
    # ====================

    def identity(self):
        # Returns ASCII data in 4 comma separated fields
        # Specification is vague, but usually follows
        # Field 1: manufacturer
        # Field 2: model number
        # Field 3: serial number
        # Field 4: firmware revision
        return self.query("*IDN?")

    def check_error(self):
        """Reads and clears the most recent error"""
        return self.query("*SYST:ERR?")

    def check_operation_complete(self):
        return bool(self.query("*OPC?"))

    def reset(self):
        """Reset values to default"""
        self.send("*RST")
