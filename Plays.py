import Utils
import movetoskill
import numpy as np
import VetOp
import game
import Tatica
import Roles

def PlayAtaque(g):
    if g.playsetup:
        g.idgoleironosso = Utils.getgoleiro(g, "nosso")
        g.id0 = 0
        g.id1 = 2
        if g.idgoleironosso == 0:
            g.id0 = 1
        if g.idgoleironosso == 2:
            g.id1 = 1

        for i in range(g.num_jogadores):
            if i == g.idgoleironosso:
                continue
            if np.sqrt((g.jogador_inimigo[i].x - g.p_bola[0]) ** 2 + (g.jogador_inimigo[i].y - g.p_bola[1]) ** 2) < np.sqrt((g.jogador_inimigo[g.id0].x - g.p_bola[0]) ** 2 + (g.jogador_inimigo[g.id0].y - g.p_bola[1]) ** 2):
                g.id1 = g.id0
                g.id0 = i
        g.playsetup = False
        return g

    if not g.playsetup:
        Roles.Pegarpasserole(g, g.id1, g.id0)
        Roles.Goleirorole(g, g.idgoleironosso, g.id0)
        if Roles.Atacanterole(g, g.id0, g.id1):
            #Roles.Interceptarrole(g, g.id1)
            g.temptempoplay = g.t + 0.5
    return g


def PlayDefesa(g):
    if g.playsetup:
        g.idgoleironosso = Utils.getgoleiro(g, "nosso")
        g.id0 = 0
        g.id1 = 2
        if g.idgoleironosso == 0:
            g.id0 = 1
        if g.idgoleironosso == 2:
            g.id1 = 1
        for i in range(g.num_jogadores):
            if i == g.idgoleironosso:
                continue
            if np.sqrt((g.jogador_inimigo[i].x - g.p_bola[0]) ** 2 + (g.jogador_inimigo[i].y - g.p_bola[1]) ** 2) < np.sqrt((g.jogador_inimigo[g.id0].x - g.p_bola[0]) ** 2 + (g.jogador_inimigo[g.id0].y - g.p_bola[1]) ** 2):
                g.id1 = g.id0
                g.id0 = i
        g.playsetup = False
        return g


    goleirodeles = Utils.getgoleiro(g, "deles")
    idatac = 0
    idpasse = 2
    if goleirodeles == 0:
        idatac = 1
    if goleirodeles == 2:
        idpasse = 1
    for i in range(g.num_jogadores):
        if i == goleirodeles:
            continue
        if np.sqrt((g.jogador_inimigo[i].x - g.p_bola[0])**2 + (g.jogador_inimigo[i].y- g.p_bola[1])**2) < np.sqrt((g.jogador_inimigo[idatac].x - g.p_bola[0])**2 + (g.jogador_inimigo[idatac].y- g.p_bola[1])**2):
            idpasse = idatac
            idatac = i

    Roles.Interrole(g, g.id0, idatac)
    Roles.Marcarrole(g, g.id1, idpasse)
    Roles.Goleirorole(g, g.idgoleironosso, g.id0)
    return g


def PlayHalt(g):
    for i in range(g.num_jogadores):
        game.parada(g, i)