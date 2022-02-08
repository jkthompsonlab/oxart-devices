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
                self.cameras[serial_nr]=SpinnakerCamera(cam,self.lock,serial_nr)
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

    def __init__(self, camera, lock_fct, serial_nr, framebuffer_len=100,
            camera_handle=None):
        """Initialise the camera interface.
        framebuffer_len: maximum number of stored frames before oldest are discarded
        lib: SDK library instance to use. If none, one will be instantiated
        camera_handle: driver handle of camera to use. If None, assume a single camera
            system.
        """
        

        # if camera:
        #     self.lock_camera = lambda: lock_fct
        # else:
        #     self.lock_camera = contextlib.suppress
        self.lock_camera = lock_fct
        self.serial_nr = serial_nr

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


        self.cam = camera
        self.node_map = self.cam.GetNodeMap()
        self.device_node_map = self.cam.GetTLDeviceNodeMap()
        self.stream_node_map = self.cam.GetTLStreamNodeMap()

    def __del__(self):
        self.close()

    def _get_camera_attribute(self,attribute,node_map="normal"):
        ''' Read out attribute from camera, with checks whether it exists and is readable 
        node_map: 0: normal node map, 1: TL device nodemap, 2: TL Stream node map '''
        if node_map == "normal":
            node_map = self.node_map
        elif node_map == "device":
            node_map = self.device_node_map
        elif node_map == "stream":
            node_map = self.stream_node_map
        else:
            raise Exception("Invalid argument. Node_map can only take values normal, device or stream.")
        node = node_map.GetNode(attribute)
        if not PySpin.IsAvailable(node):
            raise Exception("Attribute {} not available...".format(attribute))
        if not PySpin.IsReadable(node):
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

    def _get_camera_attribute_int_value(self,node,value):
        node_of_value = node.GetEntryByName(value)
        if not PySpin.IsAvailable(node_of_value):
            raise Exception("Value {} not available...".format(value))
        if not PySpin.IsReadable(node_of_value):
            raise Exception("Value {} not readable...".format(value))

        int_value = node_of_value.GetValue()
        return int_value


    def _set_camera_attribute(self,attribute,value,node_map="normal"):
        ''' Read out attribute from camera, with checks whether it exists and is readable 
        node_map: 0: normal node map, 1: TL device nodemap, 2: TL Stream node map '''
        if node_map == "normal":
            node_map = self.node_map
        elif node_map == "device":
            node_map = self.device_node_map
        elif node_map == "stream":
            node_map = self.stream_node_map
        else:
            raise Exception("Invalid argument. Node_map can only take values normal, device or stream.")
        node = node_map.GetNode(attribute)

        if node.GetPrincipalInterfaceType() == PySpin.intfIInteger:
            node_ptr = PySpin.CIntegerPtr(node)
        elif node.GetPrincipalInterfaceType() == PySpin.intfIFloat:
            node_ptr = PySpin.CFloatPtr(node)
        elif node.GetPrincipalInterfaceType() == PySpin.intfIBoolean:
            node_ptr = PySpin.CBooleanPtr(node)
        elif node.GetPrincipalInterfaceType() == PySpin.intfIEnumeration:
            node_ptr = PySpin.CEnumerationPtr(node)
        elif node.GetPrincipalInterfaceType() == PySpin.intfIString:
            node_ptr = PySpin.CStringPtr(node)

        if not PySpin.IsAvailable(node_ptr):
            raise Exception("Attribute {} not available...".format(attribute))
        if not PySpin.IsWritable(node_ptr):
            raise Exception("Attribute {} not writable...".format(attribute))

        if type(value) is int:
            node_ptr.SetValue(value)
        else:
            int_value = self._get_camera_attribute_int_value(node_ptr,value)
            node_ptr.SetIntValue(int_value)

        
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
                self._set_camera_attribute('AcquisitionMode','Continuous')

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

        
        if self.acquisition_running:
            logger.info("Acquisition already in progress.")
            return

        logger.info("Starting acquisition.")
        with self.lock_camera:
            self._set_camera_attribute('StreamBufferCountMode','Manual',node_map="stream")
            self._set_camera_attribute('StreamBufferCountManual',15,node_map="stream")
            self._set_camera_attribute('StreamBufferHandlingMode','NewestFirst',node_map="stream")
            self._set_camera_attribute('AcquisitionMode','Continuous')
        logger.info('Camera acquisition mode set to continuous...' )

        self.acquisition_running = True
        with self.lock_camera:
            self.cam.BeginAcquisition()

    def stop_acquisition(self):
        """Stop a repeated acquisition"""
        if not self.acquisition_running:
            logger.info("Acquisition was not running.")
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
        restart_camera = False
        with self.lock_camera:
            if self.cam.IsStreaming():
                try:
                    image = self.cam.GetNextImage(1000)
                    if image.IsIncomplete():
                        logger.info("Image incomplete with image status {}...".format(image.GetImageStatus()))
                        img_array = None
                        image.Release()
                    else:
                        img_array = image.GetNDArray()
                        image.Release()
                except Exception as e:
                    logger.info('Camera {} is initialised: {}'.format(self.serial_nr,self.cam.IsInitialized()))
                    logger.info('Missed an image, exception occured: \n {}'.format(e))
                    img_array = None
                    restart_camera = True
                    #time.sleep(1)
                    #self.stop_acquisition()
                    #self.start_acquisition()
            else:
                logger.info('Camera not streaming.')
                img_array = None
        #logger.info(np.shape(image.GetNDArray()))
        if restart_camera:
            logger.info('Restarting camera acquistion...')
            self.stop_acquisition()
            self.start_acquisition()
        return img_array


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
                im = self._grab_image()
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

