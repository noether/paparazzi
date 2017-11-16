#!/usr/bin/env python

import pygame
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
        self.X = np.array([-999.0, -999.0, -999.0])
        self.V = np.array([-999.0, -999.0, -999.0])
        self.timeout = 0

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
            rc.V[0] = float(msg.get_field(3))*0.0000019
            rc.V[1] = float(msg.get_field(4))*0.0000019
            rc.V[2] = float(msg.get_field(5))*0.0000019
            rc.timeout = 0

    return

def get_joy_axis(joystick):
   x = joystick.get_axis(1)
   y = joystick.get_axis(3)
   return x, y

def formation(Bb, d, k, aorv):
    no_ins_msg = 0
    for rc in list_rotorcrafts:
        if rc.X[0] == -999:
            print "Waiting for INS msg of rotorcraft ", rc.id
            no_ins_msg = 1

    if no_ins_msg:
        return

    X = np.zeros(2*len(list_rotorcrafts))
    V = np.zeros(2*len(list_rotorcrafts))
    U = np.zeros(2*len(list_rotorcrafts))

    i = 0
    for rc in list_rotorcrafts:
        X[i] = rc.X[0]
        X[i+1] = rc.X[1]
        V[i] = rc.V[0]
        V[i+1] = rc.V[1]
        i = i+2
    
    # Computation of useful matrices
    Z = Bb.T.dot(X)
    Dz = rf.make_Dz(Z, 2)
    Dzt = rf.make_Dzt(Z, 2, 1)
    E = rf.make_E(Z, d, 2, 1)

    # Shape control
    if aorv == 0:
        U = -k[0]*Bb.dot(Dz).dot(Dzt).dot(E)

        print "Velocity command: " + str(U).replace('[','').replace(']','')
        print "Error distances: " + str(E).replace('[','').replace(']','')

    elif aorv == 1:
        U = -k[1]*V -k[0]*Bb.dot(Dz).dot(Dzt).dot(E)

        #print "Positions: " + str(X).replace('[','').replace(']','')
        #print "Velocities: " + str(V).replace('[','').replace(']','')
        #print "Acceleration command: " + str(U).replace('[','').replace(']','')
        print "Error distances: " + str(E).replace('[','').replace(']','')


    i = 0
    for ac in list_rotorcrafts:
        msg = PprzMessage("datalink", "FC_ROTOR")
        msg['ac_id'] = ac.id
        msg['av'] = aorv
        msg['ux'] = U[i]
        msg['uy'] = U[i+1]
        msg['uz'] = 0

        interface.send(msg)

        i = i+2
    return

def main():
    if len(sys.argv) != 7:
        print "Usage: fc_rotor topology.txt desired_distances.txt ids.txt gains.txt a/v(1/0) joystick(1/0)"
        interface.shutdown()
        return

    B = np.loadtxt(sys.argv[1])
    d = np.loadtxt(sys.argv[2])
    ids = np.loadtxt(sys.argv[3])
    k = np.loadtxt(sys.argv[4])
    aorv = int(sys.argv[5])
    joystick_present = int(sys.argv[6])

    if B.size == 2:
        B.shape = (2,1)

    Bb = la.kron(B, np.eye(2))

    global list_ids
    list_ids = np.ndarray.tolist(ids)
    map(int, list_ids)

    if np.size(ids) != np.size(B,0):
        print "The number of rotorcrafts in the topology and ids do not match"
        return

    if np.size(d) != np.size(B,1):
        print "The number of links in the topology and desired distances do not match"
        return

    for i in range(0, len(ids)):
        list_rotorcrafts.append(rotorcraft(int(ids[i])))

    # Ivy
    interface.subscribe(message_recv)

    # Joystick
    if joystick_present == 1:
        pygame.init()
        stick = pygame.joystick.Joystick(0)
        stick.init()
        clock = pygame.time.Clock()

    try:
        while True:
            time.sleep(0.02)

            for rc in list_rotorcrafts:
                rc.timeout = rc.timeout + 0.02

            formation(Bb, d, k, aorv)

            if joystick_present == 1:
                for e in pygame.event.get():
                    x, y = get_joy_axis(stick)
                    print x, y
                clock.tick(20)

    except KeyboardInterrupt:
        interface.shutdown()
        pygame.quit()


if __name__ == '__main__':
    main()
