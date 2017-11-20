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

import lib_rigid_formations as rf
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

# Joystick
translation = 0
rotation = 0

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

def formation(B, d, mus, k, geo_fence, aorv, joystick_present):
    no_ins_msg = 0
    for rc in list_rotorcrafts:
        if rc.X[0] == -999.0:
            print("Waiting for INS msg of rotorcraft ", rc.id)
            no_ins_msg = 1
        if rc.timeout > 0.2:
            print("The INS msg of rotorcraft ", rc.id, " stopped")
            return
        if (rc.X[0] < geo_fence[0] or rc.X[0] > geo_fence[0]
            or rc.X[1] < geo_fence[1] or rc.X[1] > geo_fence[1]
            or rc.X[2] < geo_fence[2] or rc.X[2] > geo_fence[2]):
            return

    if no_ins_msg == 1:
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
        i = i + 2

    # Computation of useful matrices
    Bb = la.kron(B, np.eye(2))
    Z = Bb.T.dot(X)
    Dz = rf.make_Dz(Z, 2)
    Dzt = rf.make_Dzt(Z, 2, 1)
    Dztstar = rf.make_Dztstar(d, 2, 1)
    Zh = rf.make_Zh(Z, 2)
    E = rf.make_E(Z, d, 2, 1)

    # Shape and motion control
    mu_t = mus[0,:]
    tilde_mu_t = mus[1,:]
    mu_r = mus[2,:]
    tilde_mu_r = mus[3,:]

    global translation
    global rotation
    if joystick_present == 1:
       for e in pygame.event.get():
          translation, rotation = get_joy_axis(stick)
          if translation < 0.3 and translation > -0.3:
              translation = 0
          if rotation < 0.3 and rotation > -0.3:
              rotation = 0

    jmu_t = translation*mu_t
    jtilde_mu_t = translation*mu_t
    jmu_r = rotation*mu_r
    jtilde_mu_r = rotation*mu_r

    Avt = rf.make_Av(B, jmu_t, jtilde_mu_t)
    Avtb = la.kron(Avt, np.eye(2))

    Avr = rf.make_Av(B, jmu_r, jtilde_mu_r)
    Avrb = la.kron(Avr, np.eye(2))

    Avb = Avtb + Avrb

    if aorv == 0:
        U = -k[0]*Bb.dot(Dz).dot(Dzt).dot(E) + Avb.dot(Zh)

        print "Error distances: " + str(E).replace('[','').replace(']','')

    elif aorv == 1:
        U = -k[1]*V -k[0]*Bb.dot(Dz).dot(Dzt).dot(E) + k[1]*Avb.dot(Zh) + la.kron(Av.dot(Dztstar).dot(B.T).dot(Av), np.eye(2)).dot(Zh)

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
    if len(sys.argv) != 8:
        print "Usage: fc_rotor ids.txt topology.txt desired_distances.txt motion_parameters.txt gains.txt geo_fence.txt a/v(1/0) joystick(1/0)"
        interface.shutdown()
        return

    ids = np.loadtxt(sys.argv[1])
    B = np.loadtxt(sys.argv[2])
    d = np.loadtxt(sys.argv[3])
    mus = np.loadtxt(sys.argv[4])
    k = np.loadtxt(sys.argv[5])
    geo_fence = np.loadtxt(sys.argv[6])
    aorv = int(sys.argv[7])
    joystick_present = int(sys.argv[8])

    if B.size == 2:
        B.shape = (2,1)

    global list_ids
    list_ids = np.ndarray.tolist(ids)
    map(int, list_ids)

    if np.size(ids) != np.size(B,0):
        print("The number of rotorcrafts in the topology and ids do not match")
        return

    if np.size(d) != np.size(B,1):
        print("The number of links in the topology and desired distances do not match")
        return

    if np.size(d) != np.size(mus,1):
        print("The number of (columns) motion parameters and relative vectors do not match")
        return

    if 4 != np.size(mus,0):
        print("The number of (rows) motion parameters must be four")
        return

    for i in range(0, len(ids)):
        list_rotorcrafts.append(rotorcraft(int(ids[i])))

    # Ivy
    interface.subscribe(message_recv)

    # Joystick
    global stick
    if joystick_present == 1:
        pygame.init()
        stick = pygame.joystick.Joystick(0)
        stick.init()

    try:
        while True:
            time.sleep(0.02)

            for rc in list_rotorcrafts:
                rc.timeout = rc.timeout + 0.02

            formation(B, d, mus, k, geo_fence, aorv, joystick_present)

    except KeyboardInterrupt:
        interface.shutdown()
        pygame.quit()


if __name__ == '__main__':
    main()
    interface.shutdown()
    pygame.quit()
