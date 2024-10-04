import Utils
import movetoskill
import numpy as np
import VetOp
import game
import Tatica



def posicaochute(g, id, id2, goal, start = (1000000, 10000000), ang = 0, dist = 100, considerararea = 1):
    if start[0] == 1000000 and start[1] == 10000000:
        start = g.p_bola
    pos = removerrobo(g, id, id2)
    betav = []
    for i in range(int(len(pos) / 2)):
        betav.append(g.betab)
    v = movetoskill.skill(start[0], start[1], goal[0], goal[1], pos, betav,
                          dist)  # x0, y0, xf, yf, pos, beta, alpha

    if np.linalg.norm(v) == 0:
        return False

    v = np.array(((v[0] * np.cos(-ang) + v[1] * np.sin(-ang)),(-v[0] * np.sin(-ang) + v[1] * np.cos(-ang))))

    v = -v + np.array((start[0], start[1]))
    v = verificardestino(g, id, v, not considerararea)
    return v


def verificarcolisao(g, id, goal):
    posh = removerrobo(g, id, id)
    for i in range(int(len(posh) / 2)):
        xr = posh["xr" + str(i)]
        yr = posh["yr" + str(i)]
        if np.sqrt((goal[0] - xr) ** 2 + (goal[1] - yr) ** 2) < g.betab:
            return np.array((goal[0] - xr, goal[1] - yr)) / np.linalg.norm(
                np.array((goal[0] - xr, goal[1] - yr))) * (g.betab + 10) + np.array((xr, yr))
    return goal


def verificardestino(g, id, goal, goleiro = 0):
    pontos = []
    if len(goal) == 2:
        goal = verificarcolisao(g, id, goal)
        if not goleiro:
            if goal[0] > g.areaamarela[1] and goal[1] < g.areaamarela[2] and goal[1] > g.areaamarela[3]:
                pontos.append((g.areaamarela[1], goal[1]))
                pontos.append((goal[0], g.areaamarela[2]))
                pontos.append((goal[0], g.areaamarela[3]))

            elif goal[0] < g.areaazul[0] and goal[1] < g.areaazul[2] and goal[1] > g.areaazul[3]:
                pontos.append((g.areaazul[0], goal[1]))
                pontos.append((g.areaazul[0], goal[1]))
                pontos.append((goal[0], g.areaazul[2]))
                pontos.append((goal[0], g.areaazul[3]))
            if not pontos == []:
                ponto = pontos[0]
                for i in range(1, len(pontos)):
                   if np.sqrt((pontos[i][0] - goal[0]) ** 2 + (pontos[i][1] - goal[1]) ** 2) < np.sqrt((ponto[0] - goal[0]) ** 2 + (ponto[1] - goal[1]) ** 2):
                        ponto = pontos[i]
                goal = ponto
        if goal[0] > g.limites[0]:
            goal = (g.limites[0], goal[1])
        if goal[0] < g.limites[1]:
            goal = (g.limites[1], goal[1])
        if goal[1] > g.limites[2]:
            goal =(goal[0], g.limites[2])
        if goal[1] < g.limites[3]:
            goal = (goal[0], g.limites[3])
        return goal


def dentrodaarea(g, goal):
    if g.time == "yellow":
        if goal[0] > g.areaamarela[1] and goal[1] < g.areaamarela[2] and goal[1] > g.areaamarela[3]:
            return True
    else:
        if goal[0] < g.areaazul[0] and goal[1] < g.areaazul[2] and goal[1] > g.areaazul[3]:
            return True
    return False

def removerrobo(g, id, id2):
    id = id + 3
    id2 = id2 + 3
    j = 0
    posttemp = {"xr0": 0}
    for i in range(int(len(g.pos1) / 2)):
        if id == i or id2 == i:
            continue
        posttemp["xr" + str(j)] = g.pos1["xr" + str(i)]
        posttemp["yr" + str(j)] = g.pos1["yr" + str(i)]
        j = j + 1
    return posttemp

def getposicoes(g):
    pos = {"xr0": 0}  # cria dicionario de obstaculos
    for i in range(g.num_jogadores):  # adicionam os obstaculos no dicionario
        pos["xr" + str(i)] = g.jogador_inimigo[i].x
        pos["yr" + str(i)] = g.jogador_inimigo[i].y
    for i in range(g.num_jogadores):
        pos["xr" + str(i + g.num_jogadores)] = g.jogador_aliado[i].x
        pos["yr" + str(i + g.num_jogadores)] = g.jogador_aliado[i].y
    return pos


def getgoleiro(g, desejo):  # retorna o id do goleiro inimigo
    j = 0
    if desejo == "deles":
        for i in range(1, g.num_jogadores):
            if np.sqrt((g.jogador_inimigo[i].x - g.goldeles[0]) ** 2 + (
                    g.jogador_inimigo[i].y) ** 2) < np.sqrt(
                (g.jogador_inimigo[j].x - g.goldeles[0]) ** 2 + (g.jogador_inimigo[j].y) ** 2):
                j = i
        else:
            return j
    for i in range(1, g.num_jogadores):
        if np.sqrt((g.jogador_aliado[i].x - g.golnosso[0]) ** 2 + (
                g.jogador_aliado[i].y) ** 2) < np.sqrt(
            (g.jogador_aliado[j].x - g.golnosso[0]) ** 2 + (g.jogador_aliado[j].y) ** 2):
            j = i
    else:
        return j

def temomaisproximo(g, goal = "k"):
    if goal == "k":
        goal = g.p_bola
    distancias = []
    for i in range(g.num_jogadores):
        distancias.append(np.sqrt((g.jogador_aliado[i].x - goal[0])**2 + (g.jogador_aliado[i].y - goal[1])**2))
    for i in range(g.num_jogadores):
        distancias.append(np.sqrt((g.jogador_inimigo[i].x - goal[0])**2 + (g.jogador_inimigo[i].y - goal[1])**2))
    id = 0
    for i in range(1, len(distancias)):
        if distancias[i] < distancias[id]:
            id = i
    if id < 3:
        return "nos"
    return "eles"

def alinhado(u, v, tolerancia):
    if VetOp.angulovetores(u, v) <= tolerancia:
        return True
    return False


def velocidadepdist(g, x0, y0, xf, yf, modf = 1):
    # return np.sqrt((xf - x0) ** 2 + (yf - y0) ** 2) / 2828 * v_max
    # vel = (1 / (1 + np.exp(-(np.sqrt((xf - x0) ** 2 + (yf - y0) ** 2)/1000) + 0.3))) * v_max
    k = np.sqrt((xf - x0) ** 2 + (yf - y0) ** 2)
    print(k)
    if k < 1:
        print("AAAAAAAAAAAAAAAAAAAAAAA")
        print((x0, y0, xf, yf))
        return ((1 / (1 + np.exp(-(np.sqrt((xf - x0) ** 2 + (yf - y0) ** 2) / 1000*modf)))) - 0.3) * g.v_max
    return g.v_max

def ladodocampo(g):
    if g.p_bola[0] > 0:
        return "yellow"
    return "blue"


def calcularvelocidades(g, id=0):
    global vel
    if len(g.pos0) == 0 or g.t1 - g.t0 == 0:
        print("vish")
        return g
    vel = {"vx0": 0}
    j = 0
    for i in range(int(len(g.pos0) / 2)):
        if id == i:
            continue
        vel["vx" + str(j)] = abs((g.pos1["xr" + str(i)] - g.pos0["xr" + str(i)]) / (g.t1 - g.t0))
        vel["vy" + str(j)] = abs((g.pos1["yr" + str(i)] - g.pos0["yr" + str(i)]) / (g.t1 - g.t0))
        j = j + 1
    return g

def calculartempo(g):
    g.t0 = g.t1
    g.t1 = g.t
    return g


def calcularposicoes(g):
    if g.pos0 == {} and g.pos1 == {}:
        g.pos1 = getposicoes(g)
        g.pos0 = getposicoes(g)
    if g.pos1 != getposicoes(g):
        g.pos0 = g.pos1
        g.pos1 = getposicoes(g)
    return g

def calcularvelocidadebola(g):
    g.posbola0 = g.posbola1
    g.posbola1 = g.p_bola
    g.velocidadebola = np.array((g.posbola1[0] - g.posbola0[0], g.posbola1[1] - g.posbola0[1]))/(g.t1-g.t0)
    return g