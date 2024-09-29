import numpy as np
import VetOp


"""def teste(vet, x0, y0, xr, yr, beta):
    xf, yf = vet + np.array([x0, y0])
    if xf >= x0:
        if xr>=xf:
            if np.sqrt((xr-xf)**2+(yr-yf)**2) > beta:
                return 0
        if xr<x0:
            if np.sqrt((xr-x0)**2+(yr-y0)**2) > beta:
                return 0
    else:
        if xr <= xf:
            if np.sqrt((xr - xf) ** 2 + (yr - yf) ** 2) > beta:
                return 0
        if xr > x0:
            if np.sqrt((xr - x0) ** 2 + (yr - y0) ** 2) > beta:
                return 0

    if VetOp.calculardist(xf, yf, x0, y0, xr, yr) > beta:
        return 0
    return 1"""

def teste(vet, x0, y0, xr, yr, beta):
    xf, yf = vet + np.array([x0, y0])
    if VetOp.calculardist(xf, yf, x0, y0, xr, yr) >= beta:
        return 0
    v = np.array([xr - x0, yr - y0])
    v1 = VetOp.projort(vet, v)
    r = np.linalg.norm(v - v1)
    
    w = np.sqrt(np.abs(beta ** 2 - r ** 2))
    if np.linalg.norm(v1) > np.linalg.norm(vet) + w:

        return 0

    theta = VetOp.angulovetores(vet, v)
    if theta < 0:
        theta = (2 * np.pi - abs(theta))
    if theta > np.pi/2:
        return 0
    """print("colidiu com: ")
    print(VetOp.calculardist(xf, yf, x0, y0, xr, yr))
    print((xr, yr))"""
    """print("v1/vet/w")
    print(np.linalg.norm(v1))
    print(np.linalg.norm(vet))
    print(w)
    print(theta)"""
    return 1



