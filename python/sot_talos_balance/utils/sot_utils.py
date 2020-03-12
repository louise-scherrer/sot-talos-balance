# -*- coding: utf-8 -*-1
"""
2019, LAAS/CNRS
@author: Andrea Del Prete, Gabriele Buondonno
This module contains utilities for the SOT
"""
from __future__ import print_function

import os
from time import sleep

import numpy as np


class Bunch:
    def __init__(self, **kwds):
        self.__dict__.update(kwds)

    def __str__(self, prefix=""):
        res = ""
        for (key, value) in self.__dict__.iteritems():
            if (isinstance(value, np.ndarray) and len(value.shape) == 2 and value.shape[0] > value.shape[1]):
                res += prefix + " - " + key + ": " + str(value.T) + "\n"
            elif (isinstance(value, Bunch)):
                res += prefix + " - " + key + ":\n" + value.__str__(prefix + "    ") + "\n"
            else:
                res += prefix + " - " + key + ": " + str(value) + "\n"
        return res[:-1]


def start_sot():
    os.system('rosservice call /start_dynamic_graph')


def stop_sot():
    os.system('rosservice call /stop_dynamic_graph')


def smoothly_set_signal_to_zero(sig):
    value_type = type(sig.value)
    v = np.array(sig.value)
    for i in range(40):
        v = np.array(0.95 * v)
        sig.value = value_type(v)
        sleep(1)
    print('Setting signal to zero')
    v[:] = 0.0
    sig.value = value_type(v)


def smoothly_set_signal(sig, final_value, duration=5.0, steps=500, prints=10):
    value_type = type(sig.value)
    v = np.array(sig.value)
    vf = np.array(final_value)
    for i in range(steps + 1):
        alpha = 1.0 * i / steps
        sig.value = value_type(np.array(vf * alpha + (1 - alpha) * v))
        sleep(1.0 * duration / steps)
    print('Signal set')
    sig.value = value_type(vf)


def monitor_tracking_error(sig, sigRef, dt, time):
    N = int(time / dt)
    err = np.zeros((N, 6))
    for i in range(N):
        err[i, :] = np.array(sig.value) - np.array(sigRef.value)
        sleep(dt)
    for i in range(6):
        print('Max tracking error for axis %d:         %.2f' % (i, np.max(np.abs(err[:, i]))))
        print('Mean square tracking error for axis %d: %.2f' % (i, np.linalg.norm(err[:, i]) / N))
