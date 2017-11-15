"Helpful functions for the rigid motion control"

import numpy as np
from scipy import linalg as la

def make_S1(B):
    r, c = B.shape
    S1 = np.zeros_like(B)

    for i in range(0, r):
        for j in range(0, c):
            if B[i,j] == 1:
                S1[i,j] = 1

    return S1

def make_S2(B):
    S2 = make_S1(-1*B)
    return S2

def make_Bd(B):
    r, c = B.shape
    Bd = np.copy(B)

    for i in range(0, c):
        Bd[0, i] = 0

    return Bd

def make_Dz(Z, m):
    edges = Z.size/m
    Dz = np.zeros((Z.size, edges))
    
    j = 0

    for i in range(0, edges):
        Dz[j:j+m, i] =  Z[j:j+m]
        j+=m

    return Dz

def make_Dzt(Z, m, l):
    edges = Z.size/m
    if l == 2:
        return np.eye(edges)

    Zt = np.zeros(edges)
    for i in range(0, edges):
        norm = la.norm(Z[(i*m):(i*m+m)])
        if norm > 0.01:
            Zt[i] = (norm)**(l-2)
    
    return np.diag(Zt)

def make_DPzh(Z, m):
    edges = Z.size/m
    DPzh = np.zeros((2*edges, 2*edges))
    
    Zt = np.diag(make_Dzt(Z, m, 1))
    j = 0

    for i in range(0, edges):
        z = Z[j:j+m] * Zt[i]
        DPzh[j:j+m, j:j+m] = np.identity(m) - \
        np.dot(z[:,np.newaxis], z[np.newaxis, :])
        j+=m

    return DPzh

def make_E(Z, d, m, l):
    edges = Z.size/m
    E = np.zeros(edges)
    if edges == 1:
        E[0] =  la.norm(Z)**l - d**l
    else:
        for i in range(0, edges):
            E[i] = (la.norm(Z[(i*m):(i*m+m)]))**l - d[i]**l

    return E

def make_A(B, mu, tilde_mu):
    agents, edges = B.shape
    A = np.zeros(B.shape)
    for i in range(0, agents):
        for j in range(0, edges):
            if B[i,j] == 1:
                A[i,j] = mu[j]
            elif B[i,j] == -1:
                A[i,j] = tilde_mu[j]

    return A
