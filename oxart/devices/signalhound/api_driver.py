import logging
import numpy as np
from oxart.devices.signalhound.smdevice.sm_api import *

logger = logging.getLogger(__name__)

from scipy.signal import get_window
# Number of I/Q samples
PRETRIGGER_LEN = 16384
POSTTRIGGER_LEN = 16384
CAPTURE_LEN = PRETRIGGER_LEN + POSTTRIGGER_LEN


class Signalhound:
    """
    Driver for Signalhound spectrum analyser
    """
    def __init__(self, port=5025):
        self.handle = sm_open_device()["device"]



    def close(self):
        sm_abort(self.handle)
        sm_close_device(self.handle)

    def ping(self):
        return True


    def configure_ref_level(self,ref_level=0.0):
        sm_set_ref_level(self.handle, 0.0)

    def configure_sweep(self,centre_freq=6.8e6, span=5e6):
        self.configure_ref_level()
        self.configure_iq_stream(centre_freq=centre_freq, span=span)


    def configure_iq_stream(self,centre_freq=6.8e6, span=5e6):
        sm_set_IQ_data_type(self.handle, SM_DATA_TYPE_32_FC)
        sm_set_IQ_center_freq(self.handle, centre_freq)
        sm_set_IQ_bandwidth(self.handle, SM_TRUE, span)
        sm_set_IQ_sample_rate(self.handle, 64) #TODO
        # Setup an rising edge external trigger
        sm_set_seg_IQ_ext_trigger(self.handle, SM_TRIGGER_EDGE_RISING)
        # Setup a single segment capture where the external trigger position
        # is at 50% (equal number of samples before and after the trigger)
        # 2 second timeout period.
        sm_set_seg_IQ_segment_count(self.handle, 1)
        sm_set_seg_IQ_segment(self.handle, 0, SM_TRIGGER_TYPE_EXT, PRETRIGGER_LEN, POSTTRIGGER_LEN, 2.0)

        sm_configure(self.handle, SM_MODE_IQ_SEGMENTED_CAPTURE)

    def start_wait_for_trigger(self):
        # Start the measurement, the SM200B will begin looking for an external
        #   trigger at this point.
        sm_seg_IQ_capture_start(self.handle, 0)

    def collect_stream_data(self):
        # Block until complete
        sm_seg_IQ_capture_wait(self.handle, 0)
        # Did the capture timeout?
        timed_out = sm_seg_IQ_capture_timeout(self.handle, 0, 0)["timed_out"]
        print("Timed out? ",timed_out)
        # Read the timestamp
        ns_since_epoch = sm_seg_IQ_capture_time(self.handle, 0, 0)["ns_since_epoch"]
        # Read the I/Q data
        iq = sm_seg_IQ_capture_read(self.handle, 0, 0, 0, CAPTURE_LEN)["iq"]
        # Complete the transfer
        sm_seg_IQ_capture_finish(self.handle, 0)
        return iq

    def process_iq_data(self,iq_data):
        # Create window
        window = get_window('hamming', len(iq_data))
        # Normalize window
        window *= len(window) / sum(window)
        # Window, FFT, normalize FFT output
        iq_data_FFT = np.fft.fftshift(np.fft.fft(iq_data * window) / len(window))
        iq_data_freq = np.fft.fftfreq(np.shape(iq_data)[-1])
        return iq_data_freq,(10 * numpy.log10(iq_data_FFT.real ** 2 + iq_data_FFT.imag ** 2))


