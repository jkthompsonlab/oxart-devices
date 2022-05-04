import ctypes
import numpy as np
from picosdk.ps5000 import ps5000 as ps
import matplotlib.pyplot as plt
from picosdk.functions import adc2mV, assert_pico_ok, mV2adc


class PicoScope:
    """ Driver for Picoscope 5444D """
    def __init__(self):
        self.chandle = ctypes.c_int16()
        status = {}

        # Open 5000 series PicoScope
        # Returns handle to chandle for use in future API functions
        status["openunit"] = ps.ps5000OpenUnit(ctypes.byref(self.chandle))

        if status["openunit"] > 0:
            pass
        elif status["openunit"]==0:
            raise Error('Picoscope not found.')
        elif status["openunit"]== -1:
            raise Error('Picoscope could not be initialised.')
        else:
            raise Error('Error initialising picoscope. Unknown status: {}'.format(status["openunit"]))

    def setup_channel(self,channel='A',v_range='5V',coupling='DC'):
        if channel is not in ['A','B','C','D']:
            raise Exception('Invalid channel input. Must be A, B C or D.') 
        if v_range is not in ['10MV','20MV','100MV','200MV','500MV','1V','2V','5V','10V','20V','50V','MAX_RANGES']:
            raise Exception('Invalid voltage range input. Possible values are 10MV, 20MV, 100MV, 200MV, 500MV, 1V, 2V, 5V, 10V, 20V, 50V, MAX_RANGES') 
        if coupling is not in ['DC','AC']:
            raise Exception('Invalid channel input. Must be DC or AC.') 
        coupling_type = ps.PS5000_COUPLING[f"PS5000_{coupling}"]
        channel = ps.PS5000_CHANNEL[f"PS5000_CHANNEL_{channel}"]
        enabled = 1
        ch_range = ps.PS5000_RANGE[f"PS5000_{v_range}"]
        # analogue offset = 0 V
        status["setCh"] = ps.ps5000SetChannel(self.chandle, channel, enabled, coupling_type, ch_range)
        assert_pico_ok(status["setCh"])

    def configure_trigger(self,channel='EXTERNAL',enable=1):
        if channel is not in ['A','B','C','D','EXTERNAL']:
            raise Exception('Invalid channel input. Must be A, B C D, or EXTERNAL.') 
        if enable is not in [0,1]:
            raise Exception('Invalid enable input. Must be 0 or 1.') 
        source = ps.PS5000_CHANNEL[f"PS5000_CHANNEL_{channel}"]
        # find maximum ADC count value
        # pointer to value = ctypes.byref(maxADC)
        maxADC = ctypes.c_int16(32512)
        # Set up single trigger
        threshold = int(mV2adc(500,chARange, maxADC))
        # direction = PS5000_RISING = 2
        # delay = 0 s
        # auto Trigger = 1000 ms
        status["trigger"] = ps.ps5000SetSimpleTrigger(chandle, enable, source, threshold, 2, 0, 1000)
        assert_pico_ok(status["trigger"])

    def get_waveform(self):
        """ Returns waveform data from the scope's memory.
        Does not trigger a measurement 
        """
        self.dev.write(":WAV:DATA?\n".encode())
        return np.array(self.dev.readline().decode().strip('\n,\r ').split(','),
                        dtype=np.float32)

    def get_x_axis(self):
        """ Returns an x-axis with a given number of points """
        self.dev.write(":WAV:POIN?\n".encode())
        num_pts = int(self.dev.readline().decode().strip())
        self.dev.write(":WAV:XOR?\n".encode())
        origin = float(self.dev.readline().strip())

        self.dev.write(":WAV:XINC?\n".encode())
        inc = float(self.dev.readline().strip())

        return np.arange(num_pts) * inc + origin
