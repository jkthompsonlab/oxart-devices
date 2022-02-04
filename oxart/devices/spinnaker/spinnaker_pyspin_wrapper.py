import numpy as np
import os
import PySpin
import matplotlib.pyplot as plt
import sys
import keyboard
import time
import collections
import threading
import contextlib
import logging
logger = logging.getLogger(__name__)

from ctypes import c_int, c_float, c_long, c_ulong, POINTER


# Constants for SetTrigger()
TRIGGER_INTERNAL = 0
TRIGGER_EXTERNAL = 1

# Constants for SetAcquitisionMode()
ACQUISITION_SINGLE = 2
ACQUISITION_RUN_TILL_ABORT = 5



#%%
class AllCameras:
    def __init__(self):
        self.lock = threading.Lock()

    def initialise_all_cameras(self):
        """Initialises all connected cameras.
        Returns a dictionary of Spinnaker cameras instances indexed by serial number
        """
        #cams = CamerasInstance()
        self.system = PySpin.System.GetInstance()
        cam_list = self.system.GetCameras()
        n_cameras = cam_list.GetSize()
        if n_cameras == 0:
            raise Exception('No cameras found.')
        # serial_nr = "18275682"
        self.cameras = {}
        for cam in cam_list:
            with self.lock:
                cam.Init()
                serial_nr_ptr = PySpin.CStringPtr(cam.GetTLDeviceNodeMap().GetNode('DeviceSerialNumber'))
                serial_nr = serial_nr_ptr.GetValue()
                self.cameras[serial_nr]=SpinnakerCamera(cam,self.lock)
                logger.info("Initialised camera {}".format(serial_nr))
        logger.info("Initialised all cameras.")

    def initialise_camera(self,sn):
        """Initialises all connected cameras.
        Returns a dictionary of Spinnaker cameras instances indexed by serial number
        """
        #cams = CamerasInstance()
        self.system = PySpin.System.GetInstance()
        cam_list = self.system.GetCameras()
        n_cameras = cam_list.GetSize()
        if n_cameras == 0:
            raise Exception('No cameras found.')
        # serial_nr = "18275682"
        cam = cam_list.GetBySerial(sn)
        cam.Init()
        self.cam = SpinnakerCamera(cam)
        logger.info("Initialised camera {}".format(sn))
        #cam_list.Clear()

    def __del__(self):
        self.system.ReleaseInstance()
        #pass


#%%

class SpinnakerCamera:
    """A simple wrapper for PySpin SDK for Spinnaker cameras.
    """

    def __init__(self, camera, lock_fct, framebuffer_len=100,
            camera_handle=None):
        """Initialise the camera interface.
        framebuffer_len: maximum number of stored frames before oldest are discarded
        lib: SDK library instance to use. If none, one will be instantiated
        camera_handle: driver handle of camera to use. If None, assume a single camera
            system.
        """
        self.cam = camera

        # if camera:
        #     self.lock_camera = lambda: lock_fct
        # else:
        #     self.lock_camera = contextlib.suppress
        self.lock_camera = lock_fct

        # horiz = c_int()
        # vert = c_int()
        # with self.lock_camera():
        #     self.lib.get_detector(ctypes.byref(horiz), ctypes.byref(vert))
        # self.ccdWidth = horiz.value
        # self.ccdHeight = vert.value

        # # Set the default ROI to the full sensor
        # self.set_image_region(0, self.ccdWidth-1, 0, self.ccdHeight-1, \
        #                          hBin=1, vBin=1)

        # # Sensible defaults
        # self.set_shutter_open(True)
        # self.set_trigger_mode(TRIGGER_INTERNAL)

        self.frame_buffer = collections.deque([], framebuffer_len)

        self._frame_call_list = []

        self._stopping = threading.Event()

        # Start image acquisition thread
        self._thread = threading.Thread(target=self._acquisition_thread, daemon=True)
        self._thread.start()

        self.acquisition_running = False

    def __del__(self):
        self.close()

    def _get_camera_attribute(self,attribute,node_map=0):
        ''' Read out attribute from camera, with checks whether it exists and is readable 
        node_map: 0: normal node map, 1: TL device nodemap, 2: TL Stream node map '''
        if node_map == 0:
            node_map = self.cam.GetNodeMap()
        elif node_map == 1:
            node_map = self.cam.GetTLDeviceNodeMap()
        elif node_map == 2:
            node_map = self.cam.GetTLStreamNodeMap()
        else:
            raise Exception("Invalid argument. Node_map can only take values 0,1 or 2.")
        node = node_map.GetNode(attribute)
        if not PySpin.IsAvailable(node):
            raise Exception("Attribute {} not available...".format(attribute))
        if not PySpin.isReadable(node):
            raise Exception("Attribute {} not readable...".format(attribute))

        if node.GetPrincipalInterfaceType() == PySpin.intfIInteger:
            return PySpin.CIntegerPtr(node).GetValue()
        elif node.GetPrincipalInterfaceType() == PySpin.intfIFloat:
            return PySpin.CFloatPtr(node).GetValue()
        elif node.GetPrincipalInterfaceType() == PySpin.intfIBoolean:
            return PySpin.CBooleanPtr(node).GetValue()
        elif node.GetPrincipalInterfaceType() == PySpin.intfIEnumeration:
            return PySpin.CEnumerationPtr(node).GetValue()
        elif node.GetPrincipalInterfaceType() == PySpin.intfIString:
            return PySpin.CStringPtr(node).GetValue()




        
    def close(self):
        """Leave the camera in a safe state and shutdown the driver"""
        if self.cam == None:
            return
        self._stopping.set()
        self.stop_acquisition()
        self._thread.join()
        with self.lock_camera:
            self.cam.DeInit()
            self.cam = None

    def set_trigger_mode(self, trig_mode='continuous'):
        """Set the trigger mode between internal and external"""
        #self.lib.set_trigger_mode(int(trig_mode))
        # with lock
        with self.lock_camera:
            if trig_mode == 'continuous':
                node_acquisition_mode = PySpin.CEnumerationPtr(self.cam.GetNodeMap().GetNode('AcquisitionMode'))
                node_acquisition_mode_continuous = node_acquisition_mode.GetEntryByName('Continuous')

                acquisition_mode_continuous = node_acquisition_mode_continuous.GetValue()
                node_acquisition_mode.SetIntValue(acquisition_mode_continuous)

    def set_image_region(self, hStart, hEnd, vStart, vEnd, hBin=1, vBin=1):
        """Set the CCD region to read out and the horizontal and vertical
        binning.
        The region is 0 indexed and inclusive, so the valid ranges for hStart
        is 0..self.ccdWidth-1 etc."""
        self.roiWidth = int((1+hEnd-hStart) / hBin)
        self.roiHeight = int((1+vEnd-vStart) / vBin)

        # with self.lock_camera():
        #     self.lib.set_read_mode(READMODE_IMAGE)
        #     self.lib.set_image(int(hBin), int(vBin),  1+int(hStart),
        #                                               1+int(hEnd),
        #                                               1+int(vStart),
        #                                               1+int(vEnd))


    def set_exposure_time(self, time):
        """Set the CCD exposure time in seconds"""
        #self.lib.set_exposure_time(float(time))
        pass

    def get_acquisition_timings(self):
        """Returns the actual timings the camera will use, after quantisation
        and padding as needed by the camera hardware.
        The timings are returned as a tuple (exposureTime, minCycleTime,
        minKineticTime)"""
        # exposure = c_float()
        # accumulate = c_float()
        # kinetic = c_float()
        # with self.lock_camera():
        #     ret = self.lib.get_acquisition_timings( ctypes.byref(exposure),
        #                                           ctypes.byref(accumulate),
        #                                           ctypes.byref(kinetic))
        #     return (exposure.value, accumulate.value, kinetic.value)
        pass

    def start_acquisition(self, single=False):
        """Start a single or repeated acquisition. If single=False the
        acquisition is repeated as fast as possible, (or on every trigger, if
        in 'external trigger' mode) until stop_acquisition() is called."""

        # if single:
        #     mode = ACQUISITION_SINGLE
        # else:
        #     mode = ACQUISITION_RUN_TILL_ABORT

        # with self.lock_camera():
        #     self.lib.set_acquisition_mode(mode)
        #     self.lib.start_acquisition()
        with self.lock_camera:
            node_acquisition_mode = PySpin.CEnumerationPtr(self.cam.GetNodeMap().GetNode('AcquisitionMode'))
            if not PySpin.IsAvailable(node_acquisition_mode) or not PySpin.IsWritable(node_acquisition_mode):
                print('Unable to set acquisition mode to continuous (node retrieval;). Aborting... \n')
                return False

            node_acquisition_mode_continuous = node_acquisition_mode.GetEntryByName('Continuous')
            if not PySpin.IsAvailable(node_acquisition_mode_continuous) or not PySpin.IsReadable(
                    node_acquisition_mode_continuous):
                print('Unable to set acquisition mode to continuous (entry \'continuous\' retrieval). \
                Aborting... \n')
                return False

            acquisition_mode_continuous = node_acquisition_mode_continuous.GetValue()

            node_acquisition_mode.SetIntValue(acquisition_mode_continuous)

        print('Camera acquisition mode set to continuous...' )

        logger.info("Starting acquisition.")
        if self.acquisition_running:
            return
        self.acquisition_running = True
        with self.lock_camera:
            self.cam.BeginAcquisition()

    def stop_acquisition(self):
        """Stop a repeated acquisition"""
        if not self.acquisition_running:
            return
        self.acquisition_running = False
        with self.lock_camera:
            self.cam.EndAcquisition()
        # with self.lock_camera():
        #     self.lib.abort_acquisition()
        logger.info("Stopped acquisition.")

    def _wait_for_acquisition(self):
        """Wait for a new image to become available"""
        # with self.lock_camera():
        #     self.lib.wait_for_acquisition()
        pass


    def _grab_image(self):
        """Returns next image in the camera buffer as a numpy
        array, or None if no new images"""
        with self.lock_camera:
            if self.cam.IsStreaming():
                try:
                    image = self.cam.GetNextImage(1000)
                    img_array = image.GetNDArray()
                except:
                    print(self.cam.IsInitialized())
                    print('Missed an image.')
                    img_array = []
            else:
                print('Camera not streaming.')
                img_array = []
        #logger.info(np.shape(image.GetNDArray()))
        return img_array
        #return []

    def register_callback(self, f):
        """Register a function to be called from the acquisition thread for each
        new image"""
        self._frame_call_list.append(f)

    def deregister_callback(self, f):
        if f in self._frame_call_list:
            self._frame_call_list.remove(f)

    def _acquisition_thread(self):
        while True:
            # sleep() release the GIL, hence we have true multi-threading here
            time.sleep(50e-3)
            if self._stopping.is_set():
                break
            #logger.info(self.acquisition_running)
            if self.acquisition_running:
                im= self._grab_image()
            else:
                im = None
            if im is None:
                continue
            self.frame_buffer.append(im)
            for f in self._frame_call_list:
                f(im)

    def get_image(self):
        """Returns the oldest image in the buffer as a numpy array, or None if
        no new images"""
        if len(self.frame_buffer) == 0:
            return None
        return self.frame_buffer.popleft()

    def wait_for_image(self):
        """Returns the oldest image in the buffer as a numpy array, blocking until there
        is an image available"""
        while True:
            im = self.get_image()
            if im is not None:
                break
            time.sleep(10e-3)
        return im

    def flush_images(self):
        """Delete all images from the buffer"""
        while len(self.frame_buffer) > 0:
            self.frame_buffer.popleft()

    def get_all_images(self):
        """Returns all of the images in the buffer as an array of numpy arrays,
        or None if no new images"""
        if len(self.frame_buffer):
            ims = []
            while len(self.frame_buffer) > 0:
                ims.append(self.frame_buffer.popleft())
        else:
            ims = None
        return ims

#%%

# my_cam = SpinnakerCamera()
# my_cam.set_trigger_mode()
# my_cam.start_acquisition()
# print('Here!')
# img = my_cam.get_image()
# print(img)
# plt.imshow(img)
# time.sleep(1)
# #my_cam.stop_acquisition()
# print('Done!')

