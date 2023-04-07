#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
axsun_ctl_lw_wrapper.py

TODO

Created by Chris Mason on 04/05/2023
"""

import ctypes
import os
import sys
import threading

from functools import lru_cache
from axsun_common_enums import AxChannelMode, AxErr, AxPipelineMode
from axsun_ctl_lw_enums import AxEdgeSource, AxDevType, AxConnectionType, AxTECState
from typing import Union

from translation_interfaces import VdlStatus


class AxsunCtlLwWrapper:
    """
    Wraps all functions in AxsunOCTControl_LW_C.h.
    The signatures may differ.  This is particularly true for functions that take an output pointer as an argument;
    said functions will return those values rather than accept pointer arguments.  Also, functions that take an input
    "count" argument will not do so since this is superfluous in Python.
    This is a singleton class, there is no reason to ever instantiate two copies of it.
    It also is only usable by the thread that instantiated it.  From the DLL api:
        Thread Safety: The AxsunOCTControl_LW library is thread-safe insofar as shared resources are protected from
        concurrent access within the same process. As such, functions may return AxErr::MUTEX_TIMEOUT if device
        communication is attempted concurrently from different threads. Design your client application to call library
        functions from a single thread.
    """
    _instance = None
    _lock = threading.Lock()

    AxConnectCallbackFunctionC = ctypes.CFUNCTYPE(AxErr, ctypes.c_void_p)

    def __new__(cls, path: str = None):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(AxsunCtlLwWrapper, cls).__new__(cls)
        return cls._instance

    def __init__(self, path: str = None):
        """
        The constructor.
        :param path: Use this to override the path and use whatever library you point it at
        """
        if hasattr(self, "initialized") and self.initialized:
            # abandon if we've already initialized
            print(f"Warning: ignored attempt to re-initialize {type(self)}.")
            return
        # if no path is provided, use our default
        if path is None:
            self.cdll = self._load_default_lib()
        else:
            self.cdll = ctypes.CDLL(path)
        self.allowed_thread_id = threading.get_ident()
        self.initialized = True

    def _load_default_lib(self) -> ctypes.CDLL:
        """
        Loads the default library
        :return: a handle to the library
        """
        if sys.platform == "win32":
            lib_loc = "AxsunLibraries\\Windows\\AxsunOCTControl_LW.dll"
        elif sys.platform == "darwin":
            #lib_ext = ".dylib"
            raise NotImplementedError()
        else:
            #lib_ext = ".so"
            raise NotImplementedError()

        lib_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), lib_loc)
        return ctypes.CDLL(lib_path)

    def _check_thread(self):
        if threading.get_ident() != self.allowed_thread_id:
            raise RuntimeError("Attempted to access Axsun API from a thread that did not create it")

    @staticmethod
    def _filter_enum(err_code: Union[int, AxErr]) -> int:
        if isinstance(err_code, AxErr):
            return err_code.value
        elif isinstance(err_code, int):
            return err_code
        else:
            raise TypeError(f"{type(err_code)} is not a valid error code type")

    def _check_error_ok(self, err_code: Union[int, AxErr]) -> bool:
        """Returns True if error code indicates NO ERROR"""
        filtered_code = self._filter_enum(err_code)
        if filtered_code != AxErr.NO_AxERROR.value:
            err_string = self.axGetErrorExplained(err_code)
            print(f"ERROR: {err_string}")
            # TODO: do we raise exception here?
            return False
        return True

    @lru_cache
    def axGetErrorExplained(self, err_code: Union[int, AxErr]) -> str:
        # Don't need to check thread here
        # self._check_thread()
        func = self.cdll.axGetErrorExplained
        func.argtypes = [ctypes.c_int, ctypes.c_char_p]
        out_string = ctypes.create_string_buffer(256)
        err_code = self._filter_enum(err_code)
        func(int(err_code), out_string)
        return out_string.value.decode('utf-8')

    def axOpenAxsunOCTControl(self, open_all_interfaces: int) -> AxErr:
        raise NotImplementedError()

    def axCloseAxsunOCTControl(self) -> AxErr:
        raise NotImplementedError()

    def axNetworkInterfaceOpen(self, interface_status: int) -> AxErr:
        raise NotImplementedError()

    def axNetworkInterfaceOpen(self, interface_status: int) -> AxErr:
        raise NotImplementedError()

    def axUSBInterfaceOpen(self, interface_status: int) -> AxErr:
        raise NotImplementedError()

    def axSerialInterfaceOpen(self, interface_status: int) -> tuple[AxErr, str]:
        raise NotImplementedError()

    def axImagingCntrlEthernet(self, number_of_images: int, which_daq: int) -> AxErr:
        raise NotImplementedError()

    def axSetImageSyncSource(self, source: Union[int, AxEdgeSource], frequency: float, which_daq: int) -> AxErr:
        raise NotImplementedError()

    def axSetSampleClockSource(self, source: Union[int, AxEdgeSource], which_daq: int) -> AxErr:
        raise NotImplementedError()

    def axSetSweepTriggerSource(self, source: Union[int, AxEdgeSource], which_daq: int) -> AxErr:
        raise NotImplementedError()

    def axSetPipelineMode(self,
                          pipeline_mode: Union[int, AxPipelineMode],
                          polarization_mode: Union[int, AxChannelMode],
                          which_daq: int) -> AxErr:
        raise NotImplementedError()

    def axSetEightBitGain(self, gain: float, which_daq: int) -> AxErr:
        raise NotImplementedError()

    def axSetEightBitOffset(self, gain: float, which_daq: int) -> AxErr:
        raise NotImplementedError()

    def axSetSubsamplingFactor(self, subsampling_factor: int, which_daq: int) -> AxErr:
        raise NotImplementedError()

    def axGetFPGARegister(self, regnum: int, which_daq: int) -> tuple[AxErr, int]:
        raise NotImplementedError()

    def axGetFPGARegisterRange(self,
                               start_regnum: int,
                               end_regnum: int,
                               bytes_allocated: int,
                               which_daq: int) -> tuple[AxErr, list[int]]:
        raise NotImplementedError()

    def axGetFPGARegisterDefaults(self,
                                  elements_allocated: int,
                                  regnums: list[int],
                                  regvals: list[int],
                                  which_daq: int) -> tuple[AxErr, list[int]]:
        raise NotImplementedError()

    def axSetFPGARegisterDefaults(self,
                                  regnums: list[int],
                                  regvals: list[int],
                                  count: int,
                                  which_daq: int) -> AxErr:
        raise NotImplementedError()

    def axSetFPGARegister(self, regnum: int, regval: int, which_daq: int) -> AxErr:
        raise NotImplementedError()

    def axSetFPGARegisterMasked(self, regnum: int, retval: int, bitmask: int, which_daq: int) -> AxErr:
        raise NotImplementedError()

    def axSetFPGARegisterSingleBit(self, regnum: int, which_bit: int, value: int, which_daq: int) -> AxErr:
        raise NotImplementedError()

    def axSetFPGARegisterSingleByte(self, regnum: int, which_byte: int, value: int, which_daq: int) -> AxErr:
        raise NotImplementedError()

    def axSetFPGARegisterSingleNibble(self, regnum: int, which_nibble: int, value: int, which_daq: int) -> AxErr:
        raise NotImplementedError()

    def axSetFPGADataArray(self, regnum: int, data_array: list[int], number_of_words: int, which_daq: int) -> AxErr:
        raise NotImplementedError()

    def axReadDAQPHYRegister(self):
        """ For internal use only, listed here for completeness"""
        raise NotImplementedError()

    def axConnectionHeartbeat(self, heartbeat_state: int, which_device: int) -> AxErr:
        raise NotImplementedError()

    def axDebugCommand(self):
        """ For internal use only, listed here for completeness"""
        raise NotImplementedError()

    def axFirmwareVersion(self, which_device: int) -> tuple[AxErr, list[int]]:
        # Don't need to check thread here
        self._check_thread()
        func = self.cdll.axFirmwareVersion
        func.argtypes = [ctypes.POINTER(ctypes.c_uint32),
                         ctypes.POINTER(ctypes.c_uint32),
                         ctypes.POINTER(ctypes.c_uint32),
                         ctypes.c_int32]
        func.restype = ctypes.c_int
        major = ctypes.c_uint32()
        minor = ctypes.c_uint32()
        patch = ctypes.c_uint32()
        err_code = func(ctypes.byref(major), ctypes.byref(minor), ctypes.byref(patch), which_device)
        if not self._check_error_ok(err_code):
            major = 0
            minor = 0
            patch = 0
        return tuple([AxErr(err_code), list((major, minor, patch))])

    def axFPGAVersion(self, which_device: int) -> tuple[AxErr, list[int]]:
        raise NotImplementedError()

    def axLibraryVersion(self) -> tuple[AxErr, list[int]]:
        raise NotImplementedError()

    def axDeviceType(self, which_device: int) -> tuple[AxErr, AxDevType]:
        raise NotImplementedError()

    def axSerialNumber(self, which_device: int) -> tuple[AxErr, str]:
        raise NotImplementedError()

    def axConnectionType(self, which_device: int) -> tuple[AxErr, AxConnectionType]:
        raise NotImplementedError()

    def axCountDeviceSettings(self, which_device: int) -> tuple[AxErr, list[int]]:
        raise NotImplementedError()

    def axGetFloatSetting(self, which_setting: int, which_device: int) -> tuple[AxErr, float, str]:
        raise NotImplementedError()

    def axGetIntSetting(self, which_setting: int, which_device: int) -> tuple[AxErr, int, str]:
        raise NotImplementedError()

    def axGetBoolSetting(self, which_setting: int, which_device: int) -> tuple[AxErr, bool, str]:
        raise NotImplementedError()

    def axSettingsToDevice(self, float_values: list[float],
                           int_values: list[int],
                           bool_values: list[bool],
                           which_device: int) -> tuple[AxErr, AxDevType]:
        raise NotImplementedError()

    def axCountConnectedDevices(self) -> int:
        raise NotImplementedError()

    def axSetLaserEmission(self, emission_state: int, which_laser: int) -> AxErr:
        raise NotImplementedError()

    def axGetLaserEmission(self, which_laser: int) -> tuple[AxErr, int]:
        raise NotImplementedError()

    def axSetPointerEmission(self, emission_state: int, which_laser: int) -> AxErr:
        raise NotImplementedError()

    def axSetPointerEmission(self, emission_state: int, which_laser: int) -> AxErr:
        raise NotImplementedError()

    def axGetPointerEmission(self, which_laser: int) -> tuple[AxErr, int]:
        raise NotImplementedError()

    def axSetClockDelay(self, delay_code: int, which_laser: int) -> AxErr:
        raise NotImplementedError()

    def axGetClockDelay(self, which_laser: int) -> tuple[AxErr, int]:
        raise NotImplementedError()

    def axHomeVDL(self, which_laser: int) -> AxErr:
        raise NotImplementedError()

    def axStopVDL(self, which_laser: int) -> AxErr:
        raise NotImplementedError()

    def axMoveRelVDL(self, rel_position: float, speed: float, which_laser: int) -> AxErr:
        raise NotImplementedError()

    def axMoveAbsVDL(self, rel_position: float, speed: float, which_laser: int) -> AxErr:
        raise NotImplementedError()

    def axGetVDLStatus(self, which_laser: int) -> tuple[AxErr, VdlStatus]:
        raise NotImplementedError()

    def axGetTECState(self, which_tec: int, which_laser: int) -> tuple[AxErr, AxTECState]:
        raise NotImplementedError()

    def axSetDACTable(self):
        """ For internal use only, listed here for completeness"""
        raise NotImplementedError()

    def axGetDACTable(self):
        """ For internal use only, listed here for completeness"""
        raise NotImplementedError()

    def axSetDriveConfiguration(self, which_config: int, which_laser: int) -> AxErr:
        raise NotImplementedError()

    def axGetDriveConfiguration(self, which_laser: int) -> tuple[AxErr, int]:
        raise NotImplementedError()

    def axRegisterConnectCallback(self, fnp: AxConnectCallbackFunctionC, userdata: ctypes.c_void_p) -> AxErr:
        raise NotImplementedError()


if __name__ == "__main__":
    instance = AxsunCtlLwWrapper()
    print(f"Successfully loaded DLL: {instance.cdll}")
