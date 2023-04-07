#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
translation_interfaces.py

TODO

Created by Chris Mason on 04/07/2023
"""


class VdlStatus:
    """Structure representing the status returned from axGetVDLStatus"""

    def __init__(self,
                 current_pos: float,
                 target_pos: float,
                 speed: float,
                 error_from_last_home: int,
                 last_move_time: int,
                 state: int,
                 home_switch: int,
                 limit_switch: int,
                 vdl_error: int):
        self.current_pos = current_pos
        self.target_pos = target_pos
        self.speed = speed
        self.error_from_last_home = error_from_last_home
        self.last_move_time = last_move_time
        self.state = state
        self.home_switch = home_switch
        self.limit_switch = limit_switch
        self.vdl_error = vdl_error
