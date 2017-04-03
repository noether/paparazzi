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

import rigid_formations_lib as rf
from scipy import linalg as la

class rotorcraft:
    def __init__(self, ac_id):
        self.id = ac_id
        self.X = np.array([-999, -999, -999])

list_ids = []
list_rotorcrafts = []
interface = IvyMessagesInterface("Formation Control Rotorcrafts")

def message_recv(ac_id, msg):
    if ac_id in list_ids:
        rc = list_rotorcrafts[list_ids.index(ac_id)]

        if msg.name == 'INS':
            rc.X[0] = float(msg.get_field(0))*0.0039063
            rc.X[1] = float(msg.get_field(1))*0.0039063
            rc.X[2] = float(msg.get_field(2))*0.0039063

    return

def formation(Bb, d, k):
    no_telemetry = 0
    for rc in list_rotorcrafts:
        if rc.X[0] == -999:
            print "Waiting for telemetry of rotorcraft ", rc.id
            no_telemetry = 1

    if no_telemetry:
        return

    X = np.zeros(2*len(list_rotorcrafts))
    i = 0
    for rc in list_rotorcrafts:
        X[i] = rc.X[0]
        X[i+1] = rc.X[1]
        i = i+2
    
    # Computation of useful matrices
    Z = Bb.T.dot(X)
    Dz = rf.make_Dz(Z, 2)
    Dzt = rf.make_Dzt(Z, 2, 1)
    E = rf.make_E(Z, d, 2, 1)

    # Shape control
    U = -k[0]*Bb.dot(Dz).dot(Dzt).dot(E)

    print "Velocity command: " + str(U).replace('[','').replace(']','')
    print "Error distances: " + str(E).replace('[','').replace(']','')

    i = 0
    #for ac in list_aircraft:
    #    msg = PprzMessage("datalink", "FC_ROTOR")
    #    msg['ac_id'] = ac.id
    #    msg['index'] = ac.ux_index
    #    msg['value'] = u[i]

    #    interface.send(msg)

    #    i = i + 1
    return

def main():
    if len(sys.argv) != 6:
        print "Usage: fc_rotor topology.txt desired_distances.txt ids.txt a/v k.txt"
        interface.shutdown()
        return

    B = np.loadtxt(sys.argv[1])
    d = np.loadtxt(sys.argv[2])
    ids = np.loadtxt(sys.argv[3])
    av = sys.argv[4]
    k = np.loadtxt(sys.argv[5])

    Bb = la.kron(B, np.eye(2))

    global list_ids
    list_ids = np.ndarray.tolist(ids)
    map(int, list_ids)

    if np.size(ids) != np.size(B,0):
        print "The number of rotorcrafts in the topology and ids does not match"
        return

    if np.size(d) != np.size(B,1):
        print "The number of links in the topology and desired distances does not match"
        return

    for i in range(0, len(ids)):
        list_rotorcrafts.append(rotorcraft(int(ids[i])))

    # Ivy
    interface.subscribe(message_recv)

    try:
        while True:
            time.sleep(0.02)
            formation(Bb, d, k)

    except KeyboardInterrupt:
        interface.shutdown()


if __name__ == '__main__':
    main()
