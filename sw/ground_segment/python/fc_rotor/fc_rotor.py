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
        self.fa_index = -999

list_ids = []
list_rotorcrafts = []
interface = IvyMessagesInterface("Formation Control Rotorcrafts")

# Joystick and Keyboard
translation = 0.0
translation2 = 0.0
rotation = 0.0
rotation2 = 0.0
scale = 1.0
altitude = 1.0
but_A_pressed = 0
but_B_pressed = 0
but_X_pressed = 0
but_Y_pressed = 0

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
   x2 = joystick.get_axis(0)
   y2 = joystick.get_axis(4)
   A = joystick.get_button(0)
   B = joystick.get_button(1)
   X = joystick.get_button(2)
   Y = joystick.get_button(3)
   return x, y, x2, y2, A, B, X, Y

def formation(B, d, mus, k, geo_fence, dim, joystick_present):
    no_ins_msg = 0
    for rc in list_rotorcrafts:
        if rc.X[0] == -999.0:
            print("Waiting for INS msg of rotorcraft ", rc.id)
            no_ins_msg = 1
        if rc.timeout > 0.5:
            print("The INS msg of rotorcraft ", rc.id, " stopped")
            no_ins_msg = 1
         if (rc.X[0] < geo_fence[0] or rc.X[0] > geo_fence[1]
             or rc.X[1] < geo_fence[2] or rc.X[1] > geo_fence[3]
             or rc.X[2] < geo_fence[4] or rc.X[2] > geo_fence[5]):
             print("The rotorcraft", rc.id, " is out of the fence")
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
    d = scale*d

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
    mu_t2 = mus[4,:]
    tilde_mu_t2 = mus[5,:]
    mu_r2 = mus[6,:]
    tilde_mu_r2 = mus[7,:]

    global translation
    global translation2
    global rotation
    global rotation2
    global scale
    global altitude
    global but_A_pressed
    global but_B_pressed
    global but_X_pressed
    global but_Y_pressed
    # TODO Joystick can be read from ivy and the dead-zones and sensitivity should not be hardcoded
    # Right now, only Xbox PAD is supported
    if joystick_present == 1:
        for e in pygame.event.get():
            translation, rotation, translation2, rotation2, but_A, but_B, but_X, but_Y = get_joy_axis(stick)
            translation = translation*2.5
            rotation = rotation*3.5
            translation2 = -translation2*2.5
            rotation2 = rotation2*2.5
            if translation < 0.3 and translation > -0.3:
               translation = 0
            if rotation < 0.3 and rotation > -0.3:
               rotation = 0
            if translation2 < 0.3 and translation2 > -0.3:
               translation2 = 0
            if rotation2 < 0.3 and rotation2 > -0.3:
               rotation2 = 0

            if but_X == 1 and (not but_X_pressed):
                but_X_pressed = 1
                if scale > 0.2:
                    scale = scale - 0.05
            elif but_X == 0:
                but_X_pressed = 0
            if but_B == 1 and (not but_B_pressed):
                but_B_pressed = 1
                scale = scale + 0.05
            elif but_B == 0:
                but_B_pressed = 0
            if but_Y == 1 and (not but_Y_pressed):
                but_Y_pressed = 1
                if altitude < 4.0:
                    altitude = altitude + 0.1
                    for rc in list_rotorcrafts:
                        msg = PprzMessage("ground", "DL_SETTING")
                        msg['ac_id'] = rc.id
                        msg['index'] = rc.fa_index
                        msg['value'] = altitude
                        interface.send(msg)
            elif but_Y == 0:
                but_Y_pressed = 0
            if but_A == 1 and (not but_A_pressed):
                but_A_pressed = 1
                altitude = altitude - 0.1
                for rc in list_rotorcrafts:
                    msg = PprzMessage("ground", "DL_SETTING")
                    msg['ac_id'] = rc.id
                    msg['index'] = rc.fa_index
                    msg['value'] = altitude
                    interface.send(msg)
            elif but_A == 0:
                but_A_pressed = 0

    jmu_t = translation*mu_t
    jtilde_mu_t = translation*tilde_mu_t
    jmu_r = rotation*mu_r
    jtilde_mu_r = rotation*tilde_mu_r
    jmu_t2 = translation2*mu_t2
    jtilde_mu_t2 = translation2*tilde_mu_t2
    jmu_r2 = rotation2*mu_r2
    jtilde_mu_r2 = rotation2*tilde_mu_r2

    Avt = rf.make_Av(B, jmu_t, jtilde_mu_t)
    Avtb = la.kron(Avt, np.eye(2))
    Avr = rf.make_Av(B, jmu_r, jtilde_mu_r)
    Avrb = la.kron(Avr, np.eye(2))
    Avt2 = rf.make_Av(B, jmu_t2, jtilde_mu_t2)
    Avt2b = la.kron(Avt2, np.eye(2))
    Avr2 = rf.make_Av(B, jmu_r2, jtilde_mu_r2)
    Avr2b = la.kron(Avr2, np.eye(2))

    Avb = Avtb + Avrb + Avt2b + Avr2b

    U = -k[1]*V -k[0]*Bb.dot(Dz).dot(Dzt).dot(E) + k[1]*Avb.dot(Zh) + la.kron(Avr.dot(Dztstar).dot(B.T).dot(Avr), np.eye(2)).dot(Zh)

    #print "Positions: " + str(X).replace('[','').replace(']','')
    #print "Velocities: " + str(V).replace('[','').replace(']','')
    #print "Acceleration command: " + str(U).replace('[','').replace(']','')
    print "Error distances: " + str(E).replace('[','').replace(']','')

    i = 0
    for ac in list_rotorcrafts:
        msg = PprzMessage("datalink", "FC_ROTOR")
        msg['ac_id'] = ac.id
        msg['dim'] = dim
        msg['ux'] = U[i]
        msg['uy'] = U[i+1]
        msg['uz'] = 0

        interface.send(msg)

        i = i+2
    return

def main():
    if len(sys.argv) != 9:
        print "Usage: fc_rotor ids.txt topology.txt desired_distances.txt motion_parameters.txt gains.txt geo_fence.txt 3d/2d(1/0) joystick(1/0)"
        interface.shutdown()
        return

    ids = np.loadtxt(sys.argv[1])
    B = np.loadtxt(sys.argv[2])
    d = np.loadtxt(sys.argv[3])
    mus = np.loadtxt(sys.argv[4])
    k = np.loadtxt(sys.argv[5])
    geo_fence = np.loadtxt(sys.argv[6])
    dim = int(sys.argv[7])
    joystick_present = int(sys.argv[8])

    if np.size(d) == 1:
        mus.shape = (4,1)

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

    if 8 != np.size(mus,0):
        print("The number of (rows) motion parameters must be eight")
        return

    if dim == 1:
        print("3D formation is not supported yet")
        return

    for i in range(0, len(ids)):
        list_rotorcrafts.append(rotorcraft(int(ids[i])))

    # Ivy
    interface.subscribe(message_recv)

    for rc in list_rotorcrafts:
        settings = PaparazziACSettings(rc.id)
        list_of_indexes = ['flight_altitude']

        for setting_ in list_of_indexes:
            try:
                index = settings.name_lookup[setting_].index
                if setting_ == 'flight_altitude':
                    rc.fa_index = index
            except Exception as e:
                print(e)
                print(setting_ + " setting not found, \
                        have you forgotten gvf.xml in your settings?")

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

            formation(B, d, mus, k, geo_fence, dim, joystick_present)

    except KeyboardInterrupt:
        interface.shutdown()
        pygame.quit()


if __name__ == '__main__':
    main()
    interface.shutdown()
    pygame.quit()
