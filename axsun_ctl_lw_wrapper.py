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

from axsun_common_enums import AxErr
from typing import Union


class AxsunCtlLwWrapper:
    """TODO
    """

    def __init__(self, path:str = None):
        # if no path is provided, use our default
        if path is None:
            self.cdll = self._load_default_lib()
        else:
            self.cdll = ctypes.CDLL(path)

    def _load_default_lib(self):
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

    @staticmethod
    def _filter_err_code(err_code: Union[int, AxErr]) -> int:
        if isinstance(err_code, AxErr):
            return err_code.value
        elif isinstance(err_code, int):
            return err_code
        else:
            raise TypeError(f"{type(err_code)} is not a valid error code type")

    def axGetErrorExplained(self, err_code: AxErr) -> str:
        func = self.cdll.axGetErrorExplained
        func.argtypes = [ctypes.c_int, ctypes.c_char_p]
        out_string = ctypes.create_string_buffer(256)
        err_code = self._filter_err_code(err_code)
        func(int(err_code), out_string)
        return out_string.value.decode('utf-8')


if __name__ == "__main__":
    instance = AxsunCtlLwWrapper()
    print(f"Successfully loaded DLL: {instance.cdll}")
