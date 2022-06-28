import ctypes
import numpy as np
from picosdk.ps5000a import ps5000a as ps
import matplotlib.pyplot as plt
from picosdk.functions import adc2mV, assert_pico_ok, mV2adc


class PicoScope:
    """ Driver for Picoscope 5444D """
    def __init__(self,amp_resolution=8):
        self.chandle = ctypes.c_int16()
        self.status = {}

        if amp_resolution not in [8,12,14,15,16]:
            raise ValueError(f"Resolution {resolution} not supported. Must be in [8,12,14,15,16].")

        # Resolution set to 12 Bit
        resolution = ps.PS5000A_DEVICE_RESOLUTION[f"PS5000A_DR_{amp_resolution:n}BIT"]

        # Open 5000 series PicoScope
        # Returns handle to chandle for use in future API functions
        self.status["openunit"] = ps.ps5000aOpenUnit(ctypes.byref(self.chandle), None, resolution)
        self.ch_range = {}

        try:
            assert_pico_ok(self.status["openunit"])
        except: # PicoNotOkError:

            powerStatus = self.status["openunit"]

            if powerStatus == 286:
                self.status["changePowerSource"] = ps.ps5000aChangePowerSource(self.chandle, powerStatus)
            elif powerStatus == 282:
                self.status["changePowerSource"] = ps.ps5000aChangePowerSource(self.chandle, powerStatus)
            else:
                raise

            assert_pico_ok(self.status["changePowerSource"])

        self.disable_all_channels()

    def disable_all_channels(self):
        status = {}
        coupling_type = ps.PS5000A_COUPLING[f"PS5000A_DC"]
        enabled = 0
        ch_range = ps.PS5000A_RANGE[f"PS5000A_5V"]
        analogue_offset = 0
        for channel in ['A','B','C','D']:
            channel_obj = ps.PS5000A_CHANNEL[f"PS5000A_CHANNEL_{channel}"]
            status["setCh"] = ps.ps5000aSetChannel(self.chandle, channel_obj, enabled, coupling_type, ch_range, analogue_offset)
            assert_pico_ok(status["setCh"])

    def ping(self):
        self.status["ping"] = ps.ps5000aPingUnit(self.chandle)
        assert_pico_ok(self.status["ping"]) 
        return True


    def setup_channel(self,channel='A',v_range='5V',coupling='DC'):
        if channel not in ['A','B','C','D']:
            raise Exception('Invalid channel input. Must be A, B C or D.') 
        if v_range not in ['10MV','20MV','100MV','200MV','500MV','1V','2V','5V','10V','20V','50V','MAX_RANGES']:
            raise Exception('Invalid voltage range input. Possible values are 10MV, 20MV, 100MV, 200MV, 500MV, 1V, 2V, 5V, 10V, 20V, 50V, MAX_RANGES') 
        if coupling not in ['DC','AC']:
            raise Exception('Invalid channel input. Must be DC or AC.') 
        channel_obj = ps.PS5000A_CHANNEL[f"PS5000A_CHANNEL_{channel}"]
        coupling_type = ps.PS5000A_COUPLING[f"PS5000A_{coupling}"]
        enabled = 1
        self.ch_range[channel] = ps.PS5000A_RANGE[f"PS5000A_{v_range}"]
        print(self.ch_range)
        # analogue offset = 0 V
        analogue_offset = 0
        self.status["setCh"] = ps.ps5000aSetChannel(self.chandle, channel_obj, enabled, coupling_type, self.ch_range[channel], analogue_offset)
        assert_pico_ok(self.status["setCh"])

    def configure_trigger(self,channel='A',enable=1):
        if channel not in ['A','B','C','D','EXTERNAL']:
            raise Exception('Invalid channel input. Must be A, B C D, or EXTERNAL.') 
        if enable not in [0,1]:
            raise Exception('Invalid enable input. Must be 0 or 1.') 
        if channel != 'EXTERNAL':
            source = ps.PS5000A_CHANNEL[f"PS5000A_CHANNEL_{channel}"]
        else:
            source = ps.PS5000A_CHANNEL[f"PS5000A_{channel}"]
        # find maximum ADC count value
        # pointer to value = ctypes.byref(maxADC)
        self.maxADC = ctypes.c_int16()
        self.status["maximumValue"] = ps.ps5000aMaximumValue(self.chandle, ctypes.byref(self.maxADC))
        assert_pico_ok(self.status["maximumValue"])
        # Set up single trigger
        threshold = int(mV2adc(500,self.ch_range[channel], self.maxADC))
        # direction = PS5000_RISING = 2
        direction = 2
        # delay = 0 s
        delay = 0
        # auto Trigger = 1000 ms
        auto_trigger = 1000
        self.status["trigger"] = ps.ps5000aSetSimpleTrigger(self.chandle, enable, source, threshold, direction, delay, auto_trigger)
        assert_pico_ok(self.status["trigger"])

    def run_block(self,pre_trigger_samples=0,post_trigger_samples=1e6, timebase=0):
        # Set number of pre and post trigger samples to be collected
        self.maxSamples = int(pre_trigger_samples + post_trigger_samples)
        post_trigger_samples = int(post_trigger_samples)

        # Get timebase information
        # Warning: When using this example it may not be possible to access all Timebases as all channels are enabled by default when opening the scope.  
        # To access these Timebases, set any unused analogue channels to off.
        timebase = int(timebase)
        # noSamples = maxSamples
        # pointer to timeIntervalNanoseconds = ctypes.byref(timeIntervalns)
        # pointer to maxSamples = ctypes.byref(returnedMaxSamples)
        # segment index = 0
        self.timeIntervalns = ctypes.c_float()
        returnedMaxSamples = ctypes.c_int32()
        self.status["getTimebase2"] = ps.ps5000aGetTimebase2(self.chandle, timebase, self.maxSamples, ctypes.byref(self.timeIntervalns), ctypes.byref(returnedMaxSamples), 0)
        assert_pico_ok(self.status["getTimebase2"])

        # Run block capture
        # number of pre-trigger samples = preTriggerSamples
        # number of post-trigger samples = PostTriggerSamples
        # timebase = 8 = 80 ns (see Programmer's guide for mre information on timebases)
        # time indisposed ms = None (not needed in the example)
        # segment index = 0
        # lpReady = None (using ps5000aIsReady rather than ps5000aBlockReady)
        # pParameter = None
        self.status["runBlock"] = ps.ps5000aRunBlock(self.chandle, pre_trigger_samples, post_trigger_samples, timebase, None, 0, None, None)
        assert_pico_ok(self.status["runBlock"])

    def get_waveform(self,channel="A"):
        """ Returns waveform data from the scope's memory.
        Does not trigger a measurement 
        """
        # Check for data collection to finish using ps5000aIsReady
        ready = ctypes.c_int16(0)
        check = ctypes.c_int16(0)
        while ready.value == check.value:
            self.status["isReady"] = ps.ps5000aIsReady(self.chandle, ctypes.byref(ready))


        # Create buffers ready for assigning pointers for data collection
        bufferMax = (ctypes.c_int16 * self.maxSamples)()
        bufferMin = (ctypes.c_int16 * self.maxSamples)() # used for downsampling which isn't in the scope of this example

        # Set data buffer location for data collection from channel A
        source = ps.PS5000A_CHANNEL[f"PS5000A_CHANNEL_{channel}"]
        # pointer to buffer max = ctypes.byref(bufferAMax)
        # pointer to buffer min = ctypes.byref(bufferAMin)
        # buffer length = maxSamples
        # segment index = 0
        # ratio mode = PS5000A_RATIO_MODE_NONE = 0
        self.status["setDataBuffers"] = ps.ps5000aSetDataBuffers(self.chandle, source, ctypes.byref(bufferMax), ctypes.byref(bufferMin), self.maxSamples, 0, 0)
        assert_pico_ok(self.status["setDataBuffers"])

        # create overflow loaction
        overflow = ctypes.c_int16()
        # create converted type maxSamples
        cmaxSamples = ctypes.c_int32(self.maxSamples)

        # Retried data from scope to buffers assigned above
        # start index = 0
        # pointer to number of samples = ctypes.byref(cmaxSamples)
        # downsample ratio = 0
        # downsample ratio mode = PS5000A_RATIO_MODE_NONE
        # pointer to overflow = ctypes.byref(overflow))
        self.status["getValues"] = ps.ps5000aGetValues(self.chandle, 0, ctypes.byref(cmaxSamples), 0, 0, 0, ctypes.byref(overflow))
        assert_pico_ok(self.status["getValues"])

        # convert ADC counts data to mV
        adc2mVChMax =  adc2mV(bufferMax, self.ch_range[channel], self.maxADC)

        # Create time data
        time = np.linspace(0, (cmaxSamples.value - 1) * self.timeIntervalns.value, cmaxSamples.value)

        return time, adc2mVChMax

    def close(self):
        # Stop the scope
        self.status["stop"] = ps.ps5000aStop(self.chandle)
        assert_pico_ok(self.status["stop"])

        # Close unit Disconnect the scope
        self.status["close"]=ps.ps5000aCloseUnit(self.chandle)
        assert_pico_ok(self.status["close"])