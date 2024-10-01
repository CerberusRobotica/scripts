import numpy as np


def calcularvetor(xf, yf, x0, y0):
    vet = np.array([xf - x0, yf - y0])
    return vet


def calculardist(xf, yf, x0, y0, xr, yr):
    a = 0
    if xf != x0:
        a = (yf - y0) / (xf - x0)
    b = -1
    c = y0 - x0 * a
    d = abs(a * xr + b * yr + c) / np.sqrt(a ** 2 + b ** 2)
    #print("distancia:" + str(d))
    return d


def calcularnovovetor(x0, y0, xr, yr, beta, inv):
    v = np.array([xr - x0, yr - y0])
    if xr-x0 == 0:
        return np.array((1,1))
    knorm = np.sqrt(abs(np.linalg.norm(v)**2 - beta**2))
    theta = np.arctan((yr-y0)/(xr-x0)) + inv*(np.arctan(beta/knorm) + 0.00000000000001)
    if xr-x0 < 0:
        theta += np.pi


    vet = np.array([knorm * np.cos(theta), knorm * np.sin(theta)])
    return vet
""" print("AQUIIIIIIIIIvet:")
    print(vet)
    print((knorm*np.sin(theta))/(knorm*np.cos(theta)))"""



def projort(u, v):
    v1 = (np.dot(v, u) / (np.linalg.norm(u)) ** 2) * u
    return v1


def normalizar(alpha, vet):
    vet = alpha * vet / np.linalg.norm(vet)
    return vet

def angulovetores(u, v):
    theta = np.arccos(np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v)))
    if theta < 0:
        theta = (2 * np.pi - abs(theta))
    return theta