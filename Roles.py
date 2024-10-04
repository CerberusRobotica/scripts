import Utils
import movetoskill
import numpy as np
import VetOp
import game
import Tatica

def Pegarpasserole(g, id2, id):
    if g.preso[id2] == True:
        game.parada(g, id2)
        return
    distkicker = 1600
    distancia_bola = 0
    posicao0 = Utils.verificardestino(g, id, Utils.posicaochute(g, id, id2, (g.goldeles[0], 0), g.p_bola))
    posicaof = VetOp.normalizar(distkicker, np.array((g.p_bola[0], g.p_bola[1])) - posicao0)
    posicaof = posicaof + posicao0

    beta = []
    for i in range(int(len(Utils.removerrobo(g, id, id2))/2)):
        beta.append(g.betab)

    if movetoskill.testarcolisoes(g.p_bola[0], g.p_bola[1], np.array((g.goldeles[0] - g.p_bola[0], g.goldeles[1] - g.p_bola[1])), Utils.removerrobo(g, id, id2), beta):
        distancia_bola = 1000000000

    ang = np.arctan(distancia_bola/distkicker)
    posicaof1 = ((posicaof[0] * np.cos(ang) + posicaof[1] * np.sin(ang)),
             (-posicaof[0] * np.sin(ang) + posicaof[1] * np.cos(ang)))
    posicaof2 = ((posicaof[0] * np.cos(-ang) + posicaof[1] * np.sin(-ang)),
             (-posicaof[0] * np.sin(-ang) + posicaof[1] * np.cos(-ang)))
    if posicaof1[0]**2 + posicaof1[1]**2 > posicaof2[0]**2 + posicaof2[1]**2:
        ang = -ang

    if Tatica.SairdaareaproibidaTatica(g, id, posicaof):
        return False

    if Tatica.AlinharcomTatica(g, id2, id, posicaof, (g.goldeles[0], 0), ang):
        game.parada(g, id2)


def Atacanterole(g, id, id2):
    if Tatica.SairdaareaproibidaTatica(g, id, (g.goldeles[0], 0)):
        return False
    if Tatica.AlinharcomTatica(g, id, id2, g.p_bola,(g.goldeles[0], 0)):
        if not game.chutar(g, id):
            game.chutar(g, id)
        else:
            return True
    return False


def Interceptarrole(g, id):
    if np.sqrt((g.jogador_aliado[id].x - g.p_bola[0])**2 + (g.jogador_aliado[id].y - g.p_bola[1]**2)) < 200:
        game.parada(g, id)
        return True
    game.move(g, id, g.p_bola, 0, Utils.velocidadepdist(g, g.jogador_aliado[id].x, g.jogador_aliado[id].y, g.p_bola[0], g.p_bola[1]))
    return False

def Goleirorole(g, idgoleiro, id):
    idatac = 0
    for i in range(1, g.num_jogadores):
        if np.sqrt((g.jogador_inimigo[i].x - g.p_bola[0])**2 + (g.jogador_inimigo[i].y- g.p_bola[1])**2) < np.sqrt((g.jogador_inimigo[idatac].x - g.p_bola[0])**2 + (g.jogador_inimigo[idatac].y- g.p_bola[1])**2):
            idatac = i


    a = (g.p_bola[1] - g.jogador_inimigo[idatac].y) / (g.p_bola[0] - g.jogador_inimigo[idatac].x)
    if np.linalg.norm(g.velocidadebola) > 300:
        a = 10000000000000
        if g.velocidadebola[0] != 0:
            a = g.velocidadebola[1]/g.velocidadebola[0]
    encontro = a*g.golnosso[0] + g.p_bola[1] -a*g.p_bola[0]
    if encontro < g.golnosso[1]:
        encontro = g.golnosso[1]
    if encontro > g.golnosso[2]:
        encontro = g.golnosso[2]
    v = Utils.verificardestino(g, idgoleiro, (g.golnosso[0], encontro), 1)

    if Utils.dentrodaarea(g, g.p_bola) and np.linalg.norm(g.velocidadebola) < 200:
        if Tatica.AlinharcomTatica(idgoleiro, id, g.p_bola, (g.jogador_aliado[id].x, g.jogador_aliado[id].y), 0, 150, 1, 0):
            if not game.chutar(g, idgoleiro):
                game.chutar(g, idgoleiro)
    elif np.sqrt((g.jogador_aliado[idgoleiro].x - g.p_bola[0])**2 + (g.jogador_aliado[idgoleiro].y- g.p_bola[1])**2) < 300:
        game.chutar(g, idgoleiro)
    else:
        game.move(g, idgoleiro, (v[0], v[1]), 0, -1, 150, 1, 0)
    game.girar(g, idgoleiro, g.p_bola, 0.05)

def Interrole(g, idinter, idatac):
    if np.sqrt((g.jogador_aliado[idinter].x - g.p_bola[0]) ** 2 + (g.jogador_aliado[idinter].y - g.p_bola[1]) ** 2) < 250:
        if Tatica.AlinharcomTatica(g, idinter, idinter, g.p_bola, (g.goldeles[0], 0)):
            if game.chutar(g, idinter):
                return True
            return False
    else:
        if Tatica.AlinharcomTatica(g, idinter, idinter, g.p_bola, (g.jogador_inimigo[idatac].x, g.jogador_inimigo[idatac].y), 0,500, 1):
            game.parada(g, idinter)
            return True
        else:
            return False

def Marcarrole(g, idmarca, idpasse):
    if np.sqrt((g.jogador_aliado[idmarca].x - g.p_bola[0])**2 + (g.jogador_aliado[idmarca].y - g.p_bola[1])**2) < 250:
        if Tatica.AlinharcomTatica(g, idmarca, idmarca, g.p_bola, (g.goldeles[0], 0)):
            if game.chutar(g, idmarca):
                return True
            return False
    else:
        distancia = np.sqrt((g.jogador_inimigo[idpasse].x - g.p_bola[0])**2 + (g.jogador_inimigo[idpasse].y - g.p_bola[1])**2)/2
        if Tatica.AlinharcomTatica(g, idmarca, idmarca, (g.jogador_inimigo[idpasse].x, g.jogador_inimigo[idpasse].y), g.p_bola, 0, distancia, 1):
            game.parada(g, idmarca)
            return True
        else:
            return False

