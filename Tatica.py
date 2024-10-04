import game
import Utils
import movetoskill
import numpy as np
import VetOp

def AlinharcomTatica(g, id, id2, start, goal, ang = 0, dist = 150, ignorar = 0, considerararea = 1):
    posicao = np.array((g.jogador_aliado[id].x, g.jogador_aliado[id].y))
    v = Utils.verificardestino(g, id, Utils.posicaochute(g, id, id2, goal, start, ang, dist, considerararea), not considerararea)
    game.girar(g, id, g.p_bola, 0.05)

    alpha = Utils.velocidadepdist(g, posicao[0], posicao[1], v[0], v[1])
    if not Utils.alinhado(np.array(start) - v, (start[0] - posicao[0], start[1] - posicao[1]), 0.15):
        game.move(g, id, (v[0], v[1]), 1, -1, 200, ignorar, considerararea)
        return False
    else:
        return True


def SairdaareaproibidaTatica(g, id, goal):
    #print("saindo:")
    #print(id)
    k = Utils.verificardestino(g, id, (g.jogador_aliado[id].x, g.jogador_aliado[id].y), 0)
    if g.jogador_aliado[id].x != k[0] and  g.jogador_aliado[id].y != k[1]:
        game.move(g, id, (g.jogador_aliado[id].x - goal[0], g.jogador_aliado[id].y - goal[1]), 1, g.v_max)
        return True
    return False