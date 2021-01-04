#!/usr/bin/env python

from ctypes import cdll
from ctypes import c_wchar_p, c_bool, c_float, c_uint, c_int, c_char_p
import os
lib_name = 'libmonitoring_core.so'
lib = cdll.LoadLibrary('/home/rtros/workspace/monitoring_ws/devel/lib/libmonitoring_core.so')

def findDLL_monitoring():
    paths = os.environ['PYTHONPATH'].split(':')
    for path in paths:
        p = '/'.join(path.split('/')[:-2]) + '/' + lib_name
        if os.path.isfile(p):
            return p
    return None

class MonitorInterface(object):

    def __init__(self, description, argv, argc=1):
        if argc > 1:
            pass # not supported yet
        path = findDLL_monitoring()
        if not path:
            return
        self.lib = cdll.LoadLibrary(path)
        arc = c_int(argc)
        arv = (c_char_p * len(argv))()
        arv[:] = argv
        des = c_char_p(description)
        aP  = c_bool(False) # make sure this is False
        self.obj = lib.New_Monitor(arc, arv, des, aP)

    def addValue(self, key, value, unit, errorlevel, aggregation=0):
        #print('ADDValue')
        ke = c_char_p(key)
        un = c_char_p(unit)
        er = c_float(errorlevel)
        ag = c_uint(aggregation)
        if type(value) == str:
            va = c_char_p(value)
            self.lib.add_string(self.obj, ke, va, un, er, ag)
        else:
            va = c_float(value)
            self.lib.add_numeric(self.obj, ke, va, un, er, ag)

    def publish_and_reset(self):
        #print('PUB and RESET')
        self.lib.publish_and_reset(self.obj)

    def __del__(self):
        self.lib.delete_monitor(self.obj)
