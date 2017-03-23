#!/usr/bin/env python

import time
import sys
import wx
import numpy as np
import sys
from os import path, getenv
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_SRC + "/sw/ext/pprzlink/lib/v1.0/python")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage 
from settings_xml_parse import PaparazziACSettings

class aircraft:
    def __init__(self, ac_id):
        self.id = ac_id
        self.XY = np.array([-999, -999])
        self.heading = -999
        self.speed = -999

        self.nx_index = 0;
        self.ny_index = 0;
        self.ns_index = 0;
        self.nc_index = 0;

        self.time = -999

list_ids = []
list_aircraft = []
interface = IvyMessagesInterface("GCR Formation")

def message_recv(ac_id, msg):
    if ac_id in list_ids:
        ac = list_aircraft[list_ids.index(ac_id)]

        if msg.name == 'NAVIGATION':
            ac.XY[0] = float(msg.get_field(2))
            ac.XY[1] = float(msg.get_field(3))

        if msg.name == 'GPS':
            ac.heading = int(msg.get_field(3))*np.pi/1800.0
            ac.speed = int(msg.get_field(5))/100.0

        if msg.name == 'BAT':
            ac.time = float(msg.get_field(3))
    return

def formation():
    no_telemetry = 0
    for ac in list_aircraft:
        if ac.heading == -999 or ac.XY[0] == -999:
            print "Waiting for telemetry of aircraft ", ac.id
            no_telemetry = 1

    if no_telemetry:
        return

    i = 0
    for ac in list_aircraft:
        if i == 0:
            nx = list_aircraft[1].XY[0]
            ny = list_aircraft[1].XY[1]
            ns = list_aircraft[1].speed
            nc = list_aircraft[1].heading
        elif i == 1:
            nx = list_aircraft[0].XY[0]
            ny = list_aircraft[0].XY[1]
            ns = list_aircraft[0].speed
            nc = list_aircraft[0].heading

        msgnx = PprzMessage("ground", "DL_SETTING")
        msgnx['ac_id'] = ac.id
        msgnx['index'] = ac.nx_index
        msgnx['value'] = nx
        msgny = PprzMessage("ground", "DL_SETTING")
        msgny['ac_id'] = ac.id
        msgny['index'] = ac.ny_index
        msgny['value'] = ny
        msgns = PprzMessage("ground", "DL_SETTING")
        msgns['ac_id'] = ac.id
        msgns['index'] = ac.ns_index
        msgns['value'] = ns
        msgnc = PprzMessage("ground", "DL_SETTING")
        msgnc['ac_id'] = ac.id
        msgnc['index'] = ac.nc_index
        msgnc['value'] = nc

        interface.send(msgnx)
        interface.send(msgny)
        interface.send(msgns)
        interface.send(msgnc)
        
        i = i+1
    return

def main():
    if len(sys.argv) != 2:
        print "Usage: gvfFormationApp ids.txt"
        return

    ids = np.loadtxt(sys.argv[1])

    global list_ids
    list_ids = np.ndarray.tolist(ids)
    map(int, list_ids)

    for i in range(0, len(ids)):
        list_aircraft.append(aircraft(int(ids[i])))


    # Ivy
    interface.subscribe(message_recv)

    for ac in list_aircraft:
        settings = PaparazziACSettings(ac.id)
        list_of_indexes = ['nx', 'ny', 'ns', 'nc']

        for setting_ in list_of_indexes:
            try:
                index = settings.name_lookup[setting_].index
                if setting_ == 'nx':
                    ac.nx_index = index
                if setting_ == 'ny':
                    ac.ny_index = index
                if setting_ == 'ns':
                    ac.ns_index = index
                if setting_ == 'nc':
                    ac.nc_index = index
            except Exception as e:
                print(e)
                print(setting_ + " setting not found, \
                        have you forgotten gcr.xml in your settings?")

    try:
        while True:
            time.sleep(0.5)
            formation()

    except KeyboardInterrupt:
        interface.shutdown()


if __name__ == '__main__':
    main()
