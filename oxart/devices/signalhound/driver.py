import logging
import numpy as np
import pyvisa

logger = logging.getLogger(__name__)


class Signalhound:
    """
    Driver for Signalhound spectrum analyser
    """
    def __init__(self, port=5025):
        # Get the VISA resource manager
        rm = pyvisa.ResourceManager()

        # Open a session to the Spike software, Spike must be running at this point
        self.inst = rm.open_resource(f'TCPIP0::localhost::{port}::SOCKET')

        # For SOCKET programming, we want to tell VISA to use a terminating character
        #   to end a read and write operation.
        self.inst.read_termination = '\n'
        self.inst.write_termination = '\n'



    def close(self):
        self.inst.close()
        self.inst = None

    def ping(self):
        # TODO - maybe check errors here?
        return self.inst.query("*IDN?")

    def set_measruement_mode(self,mode='sweep'):
        if mode=='sweep':
            mode_str = 'SA'
        elif mode=='real_time':
            mode_str = 'RTSA'
        elif mode=='zero_span':
            mode_str = 'ZS'
        elif mode=='harmonics':
            mode_str = 'HARMonics'
        elif mode=='scalar_network_analysis':
            mode_str = 'NA'
        elif mode=='phase_noise':
            mode_str = 'PNoise'
        elif mode=='digital_modulation_analysis':
            mode_str = 'DDEMod'
        elif mode=='EMC_precompliance':
            mode_str = 'EMI'
        elif mode=='analog_demod':
            mode_str = 'ADEMod'
        elif mode=='interference_hunting':
            mode_str = 'IH'
        elif mode=='spectrum_emission_mask':
            mode_str = 'SEMask'
        elif mode=='wlan':
            mode_str = 'WLAN'
        elif mode=='bluetooth_low_energy':
            mode_str = 'BLE'
        elif mode=='lte':
            mode_str = 'LTE'
        else:
            raise ValueError(f" Mode {mode} not supported.")

        self.inst.write(f"INSTRUMENT:SELECT {mode_str}")

    def switch_continuous_mode(self,on=True):
        if on == True:
            self.inst.write("INIT:CONT ON")
        else:
            self.inst.write("INIT:CONT OFF")

    def configure_bandwidth(self,resolution="10kHz",video="AUTO ON",shape="GAUSSIAN"):
        if shape not in ["GAUSSIAN","FLATTOP","NUTTALL"]:
            raise ValueError(f" Unknown shape {shape}.")
        self.inst.write(f"SENS:BAND:RES:{resolution}; :BAND:VID:{video}; :BAND:SHAPE {shape}")

    def configure_span(self,span="2MHz",centre="5.8MHz"):
        self.inst.write(f"SENS:FREQ:SPAN {span}; CENT {centre}")

    def configure_amplitude(self,ref_level="22.361MV"):
        self.inst.write(f"SENS:POW:RF:RLEV {ref_level}")

    def configure_sweep(self,resolution="10kHz",span="8MHz",centre="6.6MHz"):
        self.switch_continuous_mode(on=False)
        self.configure_bandwidth(resolution=resolution,video="AUTO ON",shape="GAUSSIAN")
        self.configure_span(span=span,centre=centre)
        self.configure_amplitude(ref_level="22.361MV")
        self.configure_trace()


    def configure_trace(self):
        # Configure the trace. Ensures trace 1 is active and enabled for clear-and-write.
        # These commands are not required to be sent everytime, this is for illustrative purposes only.
        self.inst.write("TRAC:SEL 1") # Select trace 1
        self.inst.write("TRAC:TYPE WRITE") # Set clear and write mode
        self.inst.write("TRAC:UPD ON") # Set update state to on
        self.inst.write("TRAC:DISP ON") # Set un-hidden

    def run_sweep(self):
        # Trigger a sweep, and wait for it to complete
        self.inst.query(":INIT; *OPC?")
        
        # Sweep data is returned as comma separated values
        data = self.inst.query("TRACE:DATA?")

        # Split the returned string into a list
        points = [float(x) for x in data.split(',')]

        # Query information needed to know what frequency each point in the sweep refers to
        start_freq = float(self.inst.query("TRACE:XSTART?"))
        bin_size = float(self.inst.query("TRACE:XINC?"))

        x_vec=np.linspace(start_freq,start_freq+bin_size*len(points),len(points))

        return x_vec, points
