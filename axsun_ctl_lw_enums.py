#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
axsun_ctl_lw_enums.py

Enums found in AxsunOCTControl_LW_C.h

Created by Chris Mason on 04/06/2023
"""

from enum import Enum


class AxEdgeSource(Enum):
    EXTERNAL = 0
    INTERNAL = 1
    LVDS = 2
    LVCMOS = 3


class AxConnectionType(Enum):
    NONE = 0
    USB = 1
    RS232_PASSTHROUGH = 2
    RS232 = 3
    ETHERNET = 4


class AxDevType(Enum):
    UNDEFINED = 0
    LASER = 40
    CLDAQ = 41
    EDAQ = 42


class AxTECState(Enum):
    TEC_UNINITIALIZED = 0,
    WARMING_UP = 1,
    WAITING_IN_RANGE = 2,
    READY = 3,
    NOT_INSTALLED = 5,
    ERROR_NEVER_GOT_TO_READY = 16,
    ERROR_WENT_OUT_OF_RANGE = 17
