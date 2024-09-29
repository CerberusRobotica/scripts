#! /usr/bin/env python3

import math
import random
from math import atan2

import numpy as np
import rospy
from PIL.SpiderImagePlugin import isInt
from docutils.parsers.rst.states import InterpretedRoleNotImplementedError
from gazebo_msgs.msg import *
from geometry_msgs.msg import *
from grsim_ros_bridge_msgs.msg import *
from krssg_ssl_msgs.msg import *
from pyasn1_modules.rfc6402 import id_cmc_popLinkWitnessV2
from reportlab.lib.validators import isBoolean

import movetoskill
import VetOp
import testecolisao
import globals


time = "yellow"
v_max = 1
v_min = 0.1
confiabilidade = 0.8
t0 = 0
t1 = 0
t = 0
posbola0 = [0, 0]
posbola1 =  [0, 0]
pos0 = {}
pos1 = {}
vel = {}
velocidadebola = np.array((0, 0))
preso = [False, False, False]
temptempo = 0


temptempoplay = 0
playstuck = False
play = "ataque"
playsetup = True
id0 = 1
id1 = 2
idgoleironosso = 0
playcache = "ataque"


betab = 210



bola = Pose()
p_bola = (0, 0)
cabecalho = 0

jogador_blue = {
    0: SSL_DetectionRobot(),
    1: SSL_DetectionRobot(),
    2: SSL_DetectionRobot(),
    3: SSL_DetectionRobot(),
    4: SSL_DetectionRobot()
}

jogador_yellow = {
    0: SSL_DetectionRobot(),
    1: SSL_DetectionRobot(),
    2: SSL_DetectionRobot(),
    3: SSL_DetectionRobot(),
    4: SSL_DetectionRobot()
}

dist = {
    0: (0, 0),
    1: (0, 0),
    2: (0, 0),
    3: (0, 0),
    4: (0, 0)
}

dist_mod = {
    0: 0,
    1: 0,
    2: 0,
    3: 0,
    4: 0
}

ang = {
    0: 0,
    1: 0,
    2: 0,
    3: 0,
    4: 0
}

dist_by = {  #
    0: [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)],
    1: [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)],
    2: [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)],
    3: [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)],
    4: [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)]
}

min_dist = {
    0: 0,
    1: 0,
    2: 0,
    3: 0,
    4: 0
}

min_vec_dist = {
    0: (0, 0),
    1: (0, 0),
    2: (0, 0),
    3: (0, 0),
    4: (0, 0)
}

ang_arc = {
    0: 0,
    1: 0,
    2: 0,
    3: 0,
    4: 0
}

ssl_msg = {
    0: SSL(),
    1: SSL(),
    2: SSL(),
    3: SSL(),
    4: SSL()
}


def definir_dados(dados):
    for i in range(0, len(dados.robots_blue)):
        id_robots = dados.robots_blue[i].robot_id
        if id_robots == 0:
            jogador_blue[0] = dados.robots_blue[i]
        if id_robots == 1:
            jogador_blue[1] = dados.robots_blue[i]
        if id_robots == 2:
            jogador_blue[2] = dados.robots_blue[i]
        if id_robots == 3:
            jogador_blue[3] = dados.robots_blue[i]
        if id_robots == 4:
            jogador_blue[4] = dados.robots_blue[i]

    for i in range(0, len(dados.robots_yellow)):
        id_robotsy = dados.robots_yellow[i].robot_id
        if id_robotsy == 0:
            jogador_yellow[0] = dados.robots_yellow[i]
        if id_robotsy == 1:
            jogador_yellow[1] = dados.robots_yellow[i]
        if id_robotsy == 2:
            jogador_yellow[2] = dados.robots_yellow[i]
        if id_robotsy == 3:
            jogador_yellow[3] = dados.robots_yellow[i]
        if id_robotsy == 4:
            jogador_yellow[4] = dados.robots_yellow[i]
    global t
    t = dados.t_capture
    global bola
    bola = dados.balls


def salvar_bola():
    global p_bola
    try:
        p_bola = ((bola[0].x), (bola[0].y))
        dist_angle(p_bola[0], p_bola[1])
        return (p_bola)

    except:
        pass


def verifica_area(positionX, positionY):
    if positionX > -2000 and positionX < 0:
        if positionY > -2000 and positionY < 2000:
            return True
        else:
            return False
    else:
        return False


def campo(positionX, positionY):
    if positionX > -2000 and positionX < 2000:
        if positionY > -2000 and positionY < 2000:
            return True
        else:
            return False
    else:
        return False


def defender_pos(positionX, positionY):
    if positionX > -2000 and positionX < 0:
        if positionY > -2000 and positionY < 2000:
            return True
        else:
            return False
    else:
        return False


def dist_angle(ball_x, ball_y):
    for i in range(num_jogadores):
        dist[i] = (ball_x - jogador_inimigo[i].x, ball_y - jogador_inimigo[i].y)
        dist_mod[i] = math.sqrt((ball_x - jogador_inimigo[i].x) ** 2 + (ball_y - jogador_inimigo[i].y) ** 2)
        ang[i] = math.atan2(dist[i][1], dist[i][0])


def distancia_jogadores():
    for i in range(num_jogadores):
        for j in range(num_jogadores):
            dist_by[i] = math.sqrt(
                (jogador_aliado[j].x - jogador_inimigo[i].x) ** 2 + (jogador_aliado[j].y - jogador_inimigo[i].y) ** 2)


def arc_jogadores(robotIndex):
    global dist_arc_robot
    global ang_v2, ang_v1
    global ang_min

    dist_arc_robot = []
    vect_arc_robot = []
    ang_arc_robot = []

    ang_min = 0

    for h in range(-950, 1000, 50):
        dist_arc_robot.append(math.sqrt((1900 - jogador_inimigo[robotIndex].x) ** 2 + (
                h - jogador_inimigo[robotIndex].y) ** 2))  # modulo distancia arco-robot

        vect_arc_robot.append(
            ((1900 - jogador_inimigo[robotIndex].x), (h - jogador_inimigo[robotIndex].y)))  # vector distancia arco-robot

        if (1900 - jogador_inimigo[robotIndex].x) != 0:
            ang_arc_robot.append(
                math.atan2((h - jogador_inimigo[robotIndex].y), (1900 - jogador_inimigo[robotIndex].x)))  # angulo arco-robot

        else:
            ang_arc_robot.append(0)

    min_dist[robotIndex] = min(dist_arc_robot)

    vector_1 = vect_arc_robot[len(vect_arc_robot) - 1]
    vector_2 = vect_arc_robot[0]

    ang_v1 = (math.atan2((vector_1[0] - jogador_inimigo[robotIndex].y), (vector_1[1] - jogador_inimigo[robotIndex].x)))
    ang_v2 = (math.atan2((vector_2[0] - jogador_inimigo[robotIndex].y), (vector_2[1] - jogador_inimigo[robotIndex].x)))

    if abs((h - jogador_inimigo[robotIndex].y)) < 0.02:
        ang_arc[robotIndex] = 0

    elif abs((1900 - jogador_inimigo[robotIndex].x)) < 0.02:
        ang_arc[robotIndex] = (math.pi) / 2

    else:
        ang_arc[robotIndex] = ang_arc_robot[dist_arc_robot.index(min(dist_arc_robot))]


def detectar(robotIndex):
    if campo(jogador_inimigo[robotIndex].x, jogador_inimigo[robotIndex].y) >= 1980:
        ssl_msg[robotIndex].cmd_vel.angular.z = 0.0
        ssl_msg[robotIndex].cmd_vel.linear.x = -0.8
    if campo(jogador_inimigo[robotIndex].x, jogador_inimigo[robotIndex].y) <= -1980:
        ssl_msg[robotIndex].cmd_vel.angular.z = 0.0
        ssl_msg[robotIndex].cmd_vel.linear.x = 0.8

    pub[robotIndex].publish(ssl_msg[robotIndex])


def atacar(robotIndex):  ####NAO USADA
    global cabecalho

    cabecalho = ang[robotIndex] - jogador_inimigo[robotIndex].orientation

    if cabecalho > math.pi:
        cabecalho -= 2 * math.pi

    elif cabecalho < -math.pi:
        cabecalho += 2 * math.pi

    if (-0.1) <= cabecalho <= (0.1):
        ssl_msg[robotIndex].cmd_vel.angular.z = 0.0
        ssl_msg[robotIndex].cmd_vel.linear.x = 0.5

    elif cabecalho < (-0.1):
        ssl_msg[robotIndex].cmd_vel.angular.z = -1.0
        ssl_msg[robotIndex].cmd_vel.linear.x = 0.0

    else:
        ssl_msg[robotIndex].cmd_vel.angular.z = 1.0
        ssl_msg[robotIndex].cmd_vel.linear.x = 0.0

    pub[robotIndex].publish(ssl_msg[robotIndex])


def pegar_bola(robotIndex):  #####NAO USADA
    x_diff = p_bola[0] - jogador_inimigo[robotIndex].x
    y_diff = p_bola[1] - jogador_inimigo[robotIndex].y
    euclides = math.sqrt((x_diff) ** 2 + (y_diff) ** 2)
    if euclides <= 5:
        return robot_arco(robotIndex)
    else:
        return atacar(robotIndex)


def robot_arco(RobotIndex):
    x_diff = p_bola[0] - jogador_inimigo[RobotIndex].x
    y_diff = p_bola[1] - jogador_inimigo[RobotIndex].y
    euclides = math.sqrt((x_diff) ** 2 + (y_diff) ** 2)

    arco_x = 2000
    arco_y = list(range(-1000, 1000, 50))

    diff_x = arco_x - jogador_inimigo[RobotIndex].x
    diff_y = random.choice(arco_y) - jogador_inimigo[RobotIndex].y

    gama = math.atan2(diff_y, diff_x)
    err_orientation = gama - jogador_inimigo[RobotIndex].orientation

    if -0.1 <= err_orientation <= 0.1 and euclides <= 5:
        return chutar(RobotIndex)

    else:
        pass


def KepperMovement(robotIndex):
    global keeper

    keeper = ang[robotIndex] - jogador_inimigo[robotIndex].orientation + math.pi

    if keeper > math.pi:
        keeper -= 2 * math.pi

    elif keeper < -math.pi:
        keeper += 2 * math.pi

    if abs(keeper) < 0.1:
        ssl_msg[robotIndex].cmd_vel.angular.z = 0.0
        ssl_msg[robotIndex].cmd_vel.linear.y = 1.2

    if keeper > 0.1:
        ssl_msg[robotIndex].cmd_vel.angular.z = -0.8
        ssl_msg[robotIndex].cmd_vel.linear.x = 0.0

    else:
        ssl_msg[robotIndex].cmd_vel.angular.z = 0.8
        ssl_msg[robotIndex].cmd_vel.linear.x = 0.0


def parada(robotIndex):
    ssl_msg[robotIndex].cmd_vel.angular.z = 0.0
    ssl_msg[robotIndex].cmd_vel.linear.x = 0.0
    ssl_msg[robotIndex].cmd_vel.linear.y = 0.0
    pub[robotIndex].publish(ssl_msg[robotIndex])


def defender(robotIndex):
    y = -500 if p_bola[1] < -500 else 500 if p_bola[1] > 500 else p_bola[1]
    diff_y = y - jogador_inimigo[robotIndex].y

    if abs(diff_y) > 50:
        ssl_msg[robotIndex].cmd_vel.linear.y = 1 if diff_y > 0 else -1
    else:
        ssl_msg[robotIndex].cmd_vel.linear.y = 0

    pub[robotIndex].publish(ssl_msg[robotIndex])


def defensor_bola(robotIndex):
    y = p_bola[1]
    diff_y = y - jogador_inimigo[robotIndex].y

    if abs(diff_y) > 50:
        ssl_msg[robotIndex].cmd_vel.linear.y = 1 if diff_y > 0 else -1
    else:
        ssl_msg[robotIndex].cmd_vel.linear.y = 0

    pub[robotIndex].publish(ssl_msg[robotIndex])


def manter_ang(robotIndex):
    diff_y = 0 - jogador_inimigo[robotIndex].orientation

    if abs(diff_y) >= .1:
        ssl_msg[robotIndex].cmd_vel.angular.z = .1 if diff_y > 0 else -.1
    else:
        ssl_msg[robotIndex].cmd_vel.angular.z = 0

    pub[robotIndex].publish(ssl_msg[robotIndex])


mover_gol = False


def manter_linha(robotIndex, x):
    global mover_gol
    if time == "blue":
        diff_x = x - jogador_inimigo[robotIndex].x
    else:
        diff_x = x - jogador_aliado[robotIndex].x

    if abs(diff_x) > 7:
        mover_gol = True
        ssl_msg[robotIndex].cmd_vel.linear.x = .2 if diff_x < 0 else -.2
    elif mover_gol:
        mover_gol = False
        ssl_msg[robotIndex].cmd_vel.linear.x = 0

    pub[robotIndex].publish(ssl_msg[robotIndex])


def manter_defensor(robotIndex, at_x):
    x = at_x
    diff_x = x - jogador_inimigo[robotIndex].x

    if abs(diff_x) > 7:
        ssl_msg[robotIndex].cmd_vel.linear.x = .4 if diff_x > 0 else -.4
    elif mover_gol:
        ssl_msg[robotIndex].cmd_vel.linear.x = 0


distancia_bola_goleiro = lambda index: math.sqrt(
    (p_bola[0] - jogador_inimigo[index].x) ** 2 + (p_bola[1] - jogador_inimigo[index].y) ** 2)


def girar(id, goal, tolerancia):
    # ID: Qual robo deve girar? int
    # goal: Para que ponto ele deve olhar? vetor[1]
    # tolerancia: Qual a tolerancia em radianos para o alinhamento? float
    salvar_bola()

    delta = 0
    KP_ang = 5

    if p_bola[0] - jogador_aliado[id].x == 0:
        dest = 0
    else:
        dest = np.arctan((goal[1] - jogador_aliado[id].y) / (goal[0] - jogador_aliado[id].x))
        if goal[0] - jogador_aliado[id].x < 0:
            dest += np.pi


        orientacao = jogador_aliado[id].orientation


    if orientacao < 0:
        orientacao = (2 * np.pi - abs(orientacao))
    if dest < 0:
        dest = (2 * np.pi - abs(dest))

    if not abs(orientacao - dest) <= tolerancia:
        delta = orientacao - dest

    if delta > np.pi:
        delta = -(2 * np.pi - abs(delta))
    if delta < -np.pi:
        delta = (2 * np.pi - abs(delta))

    ssl_msg[id].cmd_vel.angular.z = -KP_ang * (delta)  # /abs(delta+0.0000000000000001)
    pub[id].publish(ssl_msg[id])


def move(id, goal: tuple, evitarbola, alpha, charlie=250, ignorar=0) -> None:
    # Qual robo se movimentara? int
    # Para onde ele deve ir? vetor[1]
    # Ele deve manter distacia da bola? 0/1
    # Velocidade float
    global vel

    if alpha > v_max:
        alpha = v_max
    if alpha < v_min:
        alpha = v_min

    beta = betab  # distancia entre o robo e os obstaculos
    pos = removerrobo(id, id)
    betatemp = beta
    if ignorar:
        betatemp = 0

    betav = []
    for i in range(int(len(pos) / 2)):
        betav.append(betatemp) #+ np.sqrt(vel["vx" + str(i)]**2 + vel["vy" + str(i)]**2)) #APRIMORAR

    if evitarbola:  # se ela deve se evitada, adiciona a bola no dicionario
        i = int(len(pos) / 2)
        pos["xr" + str(i)] = p_bola[0]
        pos["yr" + str(i)] = p_bola[1]
        betav.append(charlie)

    print(goal)
    a = movetoskill.skill(jogador_aliado[id].x, jogador_aliado[id].y, goal[0], goal[1], pos, betav, alpha)
    ang = jogador_aliado[id].orientation


    if isBoolean(a):
        a = (-goal[0] + jogador_aliado[id].x, -goal[1] + jogador_aliado[id].y)
    else:
        a = ((a[0] * np.cos(ang) + a[1] * np.sin(ang)),
             (-a[0] * np.sin(ang) + a[1] * np.cos(ang)))  # torna o vetor independete da direcao olhada

    ssl_msg[id].cmd_vel.linear.x = a[0]  # envia os comandos de velocidade
    ssl_msg[id].cmd_vel.linear.y = a[1]
    pub[id].publish(ssl_msg[id])  # ?publica os comandos
    # ssl.msg[id].cmd_
    return


def ataque_jog(id: int):  #####move o jogador id ate a bola, mantendo o olhar nela
    salvar_bola()
    girar(id, (p_bola[0], p_bola[1]), 0.05)
    move(id, (p_bola[0], p_bola[1]), 0, 1)

    if jogador_aliado[id].x - p_bola[0] < 10 and jogador_aliado[id].y - p_bola[1] < 10:
        ssl_msg[id].kicker = True
        pub[id].publish(ssl_msg[id])
        ssl_msg[id].kicker = False
        pub[id].publish(ssl_msg[id])


def AlinharcomTatica(id, id2, start, goal, ang = 0, dist = 150, ignorar = 0):
    salvar_bola()

    posicao = np.array((jogador_aliado[id].x, jogador_aliado[id].y))

    v = verificardestino(id, posicaochute(id, id2, goal, start, ang, dist))

    girar(id, p_bola, 0.05)

    alpha = velocidadepdist(posicao[0], posicao[1], v[0], v[1])
    # print("tatica")
    if not alinhado(np.array(start) - v, (start[0] - posicao[0], start[1] - posicao[1]), 0.2):
        move(id, (v[0], v[1]), 1, alpha, 200, ignorar)
        return False
    else:
        return True


def alinhado(u, v, tolerancia):
    # print(VetOp.angulovetores(u, v))
    if VetOp.angulovetores(u, v) <= tolerancia:
        return True
    return False


def velocidadepdist(x0, y0, xf, yf, modf = 1):
    # return np.sqrt((xf - x0) ** 2 + (yf - y0) ** 2) / 2828 * v_max
    # vel = (1 / (1 + np.exp(-(np.sqrt((xf - x0) ** 2 + (yf - y0) ** 2)/1000) + 0.3))) * v_max
    # print(vel)
    k = np.sqrt((xf - x0) ** 2 + (yf - y0) ** 2)
    if k < 200:
        return (1 / (1 + np.exp(-(np.sqrt((xf - x0) ** 2 + (yf - y0) ** 2) / 1000*modf)))) * v_max
    return v_max


def chutar(id):
    salvar_bola()

    girar(id, (p_bola[0], p_bola[1]), 0.05)
    move(id, (p_bola[0], p_bola[1]), 0, v_max, 0, 1)

    if abs(jogador_aliado[id].x - p_bola[0]) < 150 and abs(jogador_aliado[id].y - p_bola[1]) < 150:
        ssl_msg[id].kicker = True
        pub[id].publish(ssl_msg[id])
        ssl_msg[id].kicker = False
        pub[id].publish(ssl_msg[id])
        if np.linalg.norm(velocidadebola) > 200*v_max:
            return True

    return False


def DecidirPlay():
    global temptempoplay
    global playstuck
    global playsetup
    global playcache
    global play

    if not playstuck:
        if time == "yellow":
            if p_bola[0] > 0:
                play = "defesa"
            else:
                play = "ataque"
        else:
            if p_bola[0] < 0:
                play = "defesa"
            else:
                play = "ataque"

    if play != playcache:
        playcache = play
        playsetup = True


def SelecionarPlay():
    if play == "defesa":
        PlayDefesa()
    if play == "ataque":
        PlayAtaque()
    if play == "halt":
        PlayHalt()




def PlayHalt():
    for i in range(num_jogadores):
        parada(i)

def PlayDefesa():
    if playsetup:
        idgoleironosso = getgoleiro("nosso")
        idinter = 0
        idmarca = 2
        if idgoleironosso == 0:
            idinter = 1
        if idgoleironosso == 2:
            idmarca = 1
        for i in range(num_jogadores):
            if i == idgoleironosso:
                continue
            if np.sqrt((jogador_inimigo[i].x - p_bola[0]) ** 2 + (jogador_inimigo[i].y - p_bola[1]) ** 2) < np.sqrt((jogador_inimigo[idinter].x - p_bola[0]) ** 2 + (jogador_inimigo[idinter].y - p_bola[1]) ** 2):
                idmarca = idinter
                idinter = i


    goleirodeles = getgoleiro("deles")
    idatac = 0
    idpasse = 2
    if goleirodeles == 0:
        idatac = 1
    if goleirodeles == 2:
        idpasse = 1
    for i in range(num_jogadores):
        if i == goleirodeles:
            continue
        if np.sqrt((jogador_inimigo[i].x - p_bola[0])**2 + (jogador_inimigo[i].y- p_bola[1])**2) < np.sqrt((jogador_inimigo[idatac].x - p_bola[0])**2 + (jogador_inimigo[idatac].y- p_bola[1])**2):
            idpasse = idatac
            idatac = i

    Interrole(idinter, idatac)
    Marcarrole(idmarca, idpasse)
    Goleirorole(idgoleironosso)


def Marcarrole(idmarca, idpasse):
    if np.sqrt((jogador_aliado[idmarca].x - p_bola[0])**2 + (jogador_aliado[idmarca].y - p_bola[1])**2) < 250:
        if AlinharcomTatica(idmarca, idmarca, p_bola, (goldeles[0], 0)):
            if chutar(idmarca):
                return True
            return False
    else:
        distancia = np.sqrt((jogador_inimigo[idpasse].x - p_bola[0])**2 + (jogador_inimigo[idpasse].y - p_bola[1])**2)/2
        if AlinharcomTatica(idmarca, idmarca, (jogador_inimigo[idpasse].x, jogador_inimigo[idpasse].y), p_bola, 0, distancia, 1):
            parada(idmarca)
            return True
        else:
            return False



def Interrole(idinter, idatac):
    if np.sqrt((jogador_aliado[idinter].x - p_bola[0])**2 + (jogador_aliado[idinter].y - p_bola[1])**2) < 250:
        if AlinharcomTatica(idinter, idinter, p_bola, (goldeles[0], 0)):
            if chutar(idinter):
                return True
            return False
    else:
        if AlinharcomTatica(idinter, idinter, p_bola, (jogador_inimigo[idatac].x, jogador_inimigo[idatac].y), 0, 500, 1):
            parada(idinter)
            return True
        else:
            return False


def Goleirorole(idgoleiro):
    idatac = 0
    for i in range(1, num_jogadores):
        if np.sqrt((jogador_inimigo[i].x - p_bola[0])**2 + (jogador_inimigo[i].y- p_bola[1])**2) < np.sqrt((jogador_inimigo[idatac].x - p_bola[0])**2 + (jogador_inimigo[idatac].y- p_bola[1])**2):
            idatac = i


    a = (p_bola[1] - jogador_inimigo[idatac].y) / (p_bola[0] - jogador_inimigo[idatac].x)
    if np.linalg.norm(velocidadebola) > 300:
    	a = 10000000000000
    	if velocidadebola[0] != 0:
            a = velocidadebola[1]/velocidadebola[0]
    encontro = a*golnosso[0] + p_bola[1] -a*p_bola[0]
    if encontro < golnosso[1]:
        encontro = golnosso[1]
    if encontro > golnosso[2]:
        encontro = golnosso[2]
    v = verificardestino(idgoleiro, (golnosso[0], encontro))
    if np.sqrt((jogador_aliado[idgoleiro].x - p_bola[0])**2 + (jogador_aliado[idgoleiro].y- p_bola[1])**2) < 200:
        chutar(idgoleiro)
    else:
        move(idgoleiro, (v[0], v[1]), 0, velocidadepdist(jogador_aliado[idgoleiro].x, jogador_aliado[idgoleiro].y, v[0], v[1]), 150, 1)
    girar(idgoleiro, p_bola, 0.05)




def PlayAtaque():
    global id0
    global id1
    global temptempoplay
    global play
    global playsetup
    global idgoleironosso
    if playsetup:
        idgoleironosso = getgoleiro("nosso")
        id0 = 0
        id1 = 2
        if idgoleironosso == 0:
            id0 = 1
        if idgoleironosso == 2:
            id1 = 1

        for i in range(num_jogadores):
            if i == idgoleironosso:
                continue
            if np.sqrt((jogador_inimigo[i].x - p_bola[0]) ** 2 + (jogador_inimigo[i].y - p_bola[1]) ** 2) < np.sqrt((jogador_inimigo[id0].x - p_bola[0]) ** 2 + (jogador_inimigo[id0].y - p_bola[1]) ** 2):
                id1 = id0
                id0 = i
        playsetup = False

    if not playsetup:
        Pegarpasserole(id1, id0)
        Goleirorole(idgoleironosso)
        if Atacanterole(id0, id1):
            temptempoplay = t + 0.5
            idt = id1
            id1 = id0
            id0 = idt



def Pegarpasserole(id2, id):
    distkicker = 1400
    posicao0 = verificardestino(id, posicaochute(id, id2, (goldeles[0], 0), p_bola))
    posicaof = VetOp.normalizar(distkicker, np.array((p_bola[0], p_bola[1])) - posicao0)
    posicaof = posicaof + posicao0


    ang = np.arctan(100/distkicker)
    posicaof1 = ((posicaof[0] * np.cos(ang) + posicaof[1] * np.sin(ang)),
         (-posicaof[0] * np.sin(ang) + posicaof[1] * np.cos(ang)))
    posicaof2 = ((posicaof[0] * np.cos(-ang) + posicaof[1] * np.sin(-ang)),
         (-posicaof[0] * np.sin(-ang) + posicaof[1] * np.cos(-ang)))
    posicaof = posicaof2
    if posicaof1[0]**2 + posicaof1[1]**2 > posicaof2[0]**2 + posicaof2[1]**2:
        posicaof = posicaof1


    AlinharcomTatica(id2, id, posicaof, (goldeles[0], 0))


def Atacanterole(id, id2):
    global preso
    global temptempo
    if AlinharcomTatica(id, id2, p_bola,(goldeles[0], 0)):
        if not chutar(id):
            chutar(id)
        else:
            return True
    return False


def getgoleiro(desejo):  # retorna o id do goleiro inimigo
    j = 0
    if desejo == "deles":
        for i in range(1, num_jogadores):
            if np.sqrt((jogador_inimigo[i].x - goldeles[0]) ** 2 + (
                    jogador_inimigo[i].y) ** 2) < np.sqrt(
                (jogador_inimigo[j].x - goldeles[0]) ** 2 + (jogador_inimigo[j].y) ** 2):
                j = i
        else:
            return j

    for i in range(1, num_jogadores):
        if np.sqrt((jogador_aliado[i].x - golnosso[0]) ** 2 + (
                jogador_aliado[i].y) ** 2) < np.sqrt(
            (jogador_aliado[j].x - golnosso[0]) ** 2 + (jogador_aliado[j].y) ** 2):
            j = i
    else:
        return j


def getposicoes():
    # print(id)
    pos = {"xr0": 0}  # cria dicionario de obstaculos
    for i in range(num_jogadores):  # adicionam os obstaculos no dicionario
        pos["xr" + str(i)] = jogador_inimigo[i].x
        pos["yr" + str(i)] = jogador_inimigo[i].y
    for i in range(num_jogadores):
        pos["xr" + str(i + num_jogadores)] = jogador_aliado[i].x
        pos["yr" + str(i + num_jogadores)] = jogador_aliado[i].y
    return pos


def removerrobo(id, id2):
    # print(id)
    id = id + 3
    id2 = id2 + 3
    j = 0
    posttemp = {"xr0": 0}
    for i in range(int(len(pos1) / 2)):
        if id == i or id2 == i:
            continue
        posttemp["xr" + str(j)] = pos1["xr" + str(i)]
        posttemp["yr" + str(j)] = pos1["yr" + str(i)]
        j = j + 1
    # print(posttemp)
    return posttemp

# y =a*x + y0 -a*x0
# x = (y - y0 + a*x0)/a


def verificartrajetoria(id, goal):
    v = (goal[0] - jogador_aliado[id].x, goal[1] - jogador_aliado[1])
    if a*areanossa[0] + jogador_aliado[id].y -a*jogador_aliado[id].x


def verificardestino(id, goal):
    pontos = []
    goal = verificarcolisao(id, goal)
    if goal[0] < areanossa[0] and goal[0] > areanossa[1] and goal[1] < areanossa[2] and goal[1] > areanossa[3]:
        a = 10000000
        print("colidiu na nossa")
        if goal[0] != 0:
            a = (goal[1]-jogador_aliado[id].y)/(goal[0]-jogador_aliado[id].x)

        if not a*areanossa[0] + jogador_aliado[id].y -a*jogador_aliado[id].x > areanossa[0] or a*areanossa[0] + jogador_aliado[id].y -a*jogador_aliado[id].x < areanossa[1]:
            pontos.append((areanossa[0], a*areanossa[0] + jogador_aliado[id].y -a*jogador_aliado[id].x))

        if not a*areanossa[1] + jogador_aliado[id].y -a*jogador_aliado[id].x > areanossa[0] or a*areanossa[1] + jogador_aliado[id].y -a*jogador_aliado[id].x < areanossa[1]:
            pontos.append((areanossa[1], a*areanossa[1] + jogador_aliado[id].y -a*jogador_aliado[id].x))

        if not (areanossa[2] - jogador_aliado[id].y + a*jogador_aliado[id].x)/a > areanossa[2] or (areanossa[2] - jogador_aliado[id].y + a*jogador_aliado[id].x)/a > areanossa[3]:
            pontos.append(((areanossa[2] - jogador_aliado[id].y + a*jogador_aliado[id].x)/a, areanossa[2]))
        if not (areanossa[3] - jogador_aliado[id].y + a * jogador_aliado[id].x) / a > areanossa[2] or (areanossa[3] - jogador_aliado[id].y + a * jogador_aliado[id].x) / a > areanossa[3]:
            pontos.append(((areanossa[3] - jogador_aliado[id].y + a * jogador_aliado[id].x) / a, areanossa[3]))
    elif goal[0] < areadeles[0] and goal[0] > areadeles[1] and goal[1] < areadeles[2] and goal[1] > areadeles[3]:
        a = 10000000
        print("colidiu na deles")
        if goal[0] != 0:
            a = (goal[1] - jogador_aliado[id].y) / (goal[0] - jogador_aliado[id].x)

        if not a * areadeles[0] + jogador_aliado[id].y - a * jogador_aliado[id].x > areadeles[0] or a * areadeles[0] + \
                jogador_aliado[id].y - a * jogador_aliado[id].x < areadeles[1]:
            pontos.append((areadeles[0], a * areadeles[0] + jogador_aliado[id].y - a * jogador_aliado[id].x))

        if not a * areadeles[1] + jogador_aliado[id].y - a * jogador_aliado[id].x > areadeles[0] or a * areadeles[1] + \
                jogador_aliado[id].y - a * jogador_aliado[id].x < areadeles[1]:
            pontos.append((areadeles[1], a * areadeles[1] + jogador_aliado[id].y - a * jogador_aliado[id].x))

        if not (areadeles[2] - jogador_aliado[id].y + a * jogador_aliado[id].x) / a > areadeles[2] or (
                areadeles[2] - jogador_aliado[id].y + a * jogador_aliado[id].x) / a > areadeles[3]:
            pontos.append(((areadeles[2] - jogador_aliado[id].y + a * jogador_aliado[id].x) / a, areadeles[2]))
        if not (areadeles[3] - jogador_aliado[id].y + a * jogador_aliado[id].x) / a > areadeles[2] or (
                areadeles[3] - jogador_aliado[id].y + a * jogador_aliado[id].x) / a > areadeles[3]:
            pontos.append(((areadeles[3] - jogador_aliado[id].y + a * jogador_aliado[id].x) / a, areadeles[3]))
    else:
        return goal
    ponto = pontos[0]
    for i in range(1 ,len(pontos)):
        if np.sqrt((pontos[i][0] - jogador_aliado[id].x)**2 + (pontos[i][1] - jogador_aliado[id].y)**2) < np.sqrt((ponto[0] - jogador_aliado[id].x)**2 + (ponto[1] - jogador_aliado[id].y)**2):
            ponto = pontos[i]
    return ponto


def verificarcolisao(id, goal):
    posh = removerrobo(id, id)
    for i in range(int(len(posh) / 2)):
        xr = posh["xr" + str(i)]
        yr = posh["yr" + str(i)]
        if np.sqrt((goal[0] - xr) ** 2 + (goal[1] - yr) ** 2) < betab:
            return np.array((goal[0] - xr, goal[1] - yr)) / np.linalg.norm(
                np.array((goal[0] - xr, goal[1] - yr))) * (betab + 10) + np.array((xr, yr))
    return goal


def posicaochute(id, id2, goal, start = p_bola, ang = 0, dist = 150):

    salvar_bola()
    pos = removerrobo(id, id2)
    betav = []
    for i in range(int(len(pos) / 2)):
        betav.append(betab)
    v = movetoskill.skill(start[0], start[1], goal[0], goal[1], pos, betav,
                          dist)  # x0, y0, xf, yf, pos, beta, alpha

    if np.linalg.norm(v) == 0:
        return False

    v = np.array(((v[0] * np.cos(-ang) + v[1] * np.sin(-ang)),(-v[0] * np.sin(-ang) + v[1] * np.cos(-ang))))

    v = -v + np.array((start[0], start[1]))
    v = verificardestino(id, v)
    return v


def goleiro(robotIndex):
    manter_ang(robotIndex)
    manter_linha(robotIndex, 1950)

    if verifica_area(p_bola[0], p_bola[1]):
        defender(robotIndex)

        if distancia_bola_goleiro(robotIndex) < 400:
            chutar(robotIndex)


def def_area_1(posX, posY):
    if posX > -1800 and posX < 100:
        if posY > 500 and posY < 1900:
            return True
        else:
            return False
    else:
        return False


def def_area_2(posX, posY):
    if posX > -1800 and posX < 150:
        if posY < -500 and posY > -1900:
            return True
        else:
            return False
    else:
        return False


def def_area_3(posX, posY):
    if posX > -1000 and posX < 100:
        if posY > -1000 and posY < 1000:
            return True
        else:
            return False
    else:
        return False


def defensor_tras_1(jogIndex, at_x, at_y):
    x = at_x
    y = at_y

    diferenca_x = x - jogador_inimigo[jogIndex].x
    diferenca_y = y - jogador_inimigo[jogIndex].y

    if abs(diferenca_x) > 7:
        ssl_msg[jogIndex].cmd_vel.linear.x = .2 if diferenca_x > 0 else -.2
    elif mover_gol:
        ssl_msg[jogIndex].cmd_vel.linear.x = 0

    if abs(diferenca_y) > 7:
        ssl_msg[jogIndex].cmd_vel.linear.y = .2 if diferenca_y > 0 else -.2
    elif mover_gol:
        ssl_msg[jogIndex].cmd_vel.linear.y = 0


def defensor_1(jogIndex):
    manter_ang(jogIndex)
    manter_defensor(jogIndex, -900)

    if def_area_1(p_bola[0], p_bola[1]):
        defensor_bola(jogIndex)

        if distancia_bola_goleiro(jogIndex) < 400:
            chutar(jogIndex)
    else:
        defensor_tras_1(jogIndex, -900, 1000)


def defensor_2(jogIndex):
    manter_ang(jogIndex)
    manter_defensor(jogIndex, -900)

    if def_area_2(p_bola[0], p_bola[1]):
        defensor_bola(jogIndex)

        if distancia_bola_goleiro(jogIndex) < 400:
            chutar(jogIndex)

    else:
        defensor_tras_1(jogIndex, -900, -1000)


def defensor_3(robotIndex):
    manter_ang(robotIndex)
    manter_defensor(robotIndex, -450)

    if def_area_3(p_bola[0], p_bola[1]):
        defensor_bola(robotIndex)

        if distancia_bola_goleiro(robotIndex) < 400:
            chutar(robotIndex)

    else:
        defensor_tras_1(robotIndex, -450, 0)


def goal(jogIndex):
    global or_jog_1, or_jog_2

    if (dist_mod[jogIndex] < 150):
        arc_jogadores(jogIndex)

        or_jog_1 = ang_v1 - jogador_inimigo[jogIndex].orientation
        or_jog_2 = ang_v2 - jogador_inimigo[jogIndex].orientation

        if or_jog_1 > math.pi:
            or_jog_1 -= 2 * math.pi

        elif or_jog_1 < -math.pi:
            or_jog_1 += 2 * math.pi

        if or_jog_2 > math.pi:
            or_jog_2 -= 2 * math.pi

        elif or_jog_2 < -math.pi:
            or_jog_2 += 2 * math.pi

        ssl_msg[jogIndex].cmd_vel.angular.z = -0.2
        if -or_jog_1 <= jogador_inimigo[jogIndex].orientation <= or_jog_2:
            ssl_msg[jogIndex].cmd_vel.angular.z = 0.0
            ssl_msg[jogIndex].cmd_vel.linear.x = 0.8

        elif abs(jogador_inimigo[jogIndex].orientation) < or_jog_1:
            ssl_msg[jogIndex].cmd_vel.angular.z = -0.2
            ssl_msg[jogIndex].cmd_vel.linear.x = 0.0

        elif (jogador_inimigo[jogIndex].orientation) > or_jog_2:
            ssl_msg[jogIndex].cmd_vel.angular.z = 0.0
            ssl_msg[jogIndex].cmd_vel.linear.x = 0.0

        pub[jogIndex].publish(ssl_msg[jogIndex])

    else:
        pass


def calculartempo():
    global t0
    global t1
    if t1 < t:
        t0 = t1
        t1 = t
        return True
    return False


def calcularposicoes():
    global posbola1
    global posbola0
    global pos0
    global pos1
    if pos0 == {} and pos1 == {}:
        pos1 = getposicoes()
        pos0 = getposicoes()
    if pos1 != getposicoes():
        pos0 = pos1
        pos1 = getposicoes()


def calcularvelocidadebola():
    global posbola0
    global posbola1
    global velocidadebola
    posbola0 = posbola1
    posbola1 = p_bola
    velocidadebola = np.array((posbola1[0] - posbola0[0], posbola1[1] - posbola0[1]))/(t1-t0)

def calcularvelocidades(id=0):
    global vel
    if len(pos0) == 0 or t1 - t0 == 0:
        print("vish")
        return False
    vel = {"vx0": 0}
    j = 0
    for i in range(int(len(pos0) / 2)):
        if id == i:
            continue
        vel["vx" + str(j)] = abs((pos1["xr" + str(i)] - pos0["xr" + str(i)]) / (t1 - t0))
        vel["vy" + str(j)] = abs((pos1["yr" + str(i)] - pos0["yr" + str(i)]) / (t1 - t0))
        j = j + 1
    #vel["vx" + str(int(len(vel)/2))] = abs((pos1["xr" + str(int(len(vel)/2)] - pos0["xr" + str(int(len(vel)/2)]) / (t1 - t0))
    #vel["vy" + str(int(len(vel)/2)] = abs((pos1["yr" + str(int(len(vel)/2)] - pos0["yr" + str(int(len(vel)/2)]) / (t1 - t0))
    # print(vel)


if __name__ == "__main__":
    rospy.init_node("test_ssl", anonymous=False)

    num_jogadores = 3

    sub = rospy.Subscriber("/vision", SSL_DetectionFrame, definir_dados)
    pub = {}

    if time == "yellow":
        for i in range(num_jogadores):
            topic = '/robot_yellow_{}/cmd'.format(i)
            pub[i] = rospy.Publisher(topic, SSL, queue_size=10)
            golnosso = [2000, -500, 500]  # x, y0, yf
            goldeles = [-2000, -500, 500]
            areanossa = [2000, 1000, 1000, -1000]
            areadeles = [-1000, -2000, 1000, -1000]
            jogador_aliado = jogador_yellow
            jogador_inimigo = jogador_blue

    if time == "blue":
        for i in range(num_jogadores):
            topic = '/robot_blue_{}/cmd'.format(i)
            pub[i] = rospy.Publisher(topic, SSL, queue_size=10)
            golnosso = [-2000, -500, 500]  # x, y0, yf
            goldeles = [2000, -500, 500]
            areanossa = [-1000, -2000, 1000, -1000]
            areadeles = [2000, 1000, 1000, -1000]
            jogador_aliado = jogador_blue
            jogador_inimigo = jogador_yellow

    r = rospy.Rate(120)

    while not rospy.is_shutdown():

        salvar_bola()

        if calculartempo():
            calcularvelocidades()
            calcularposicoes()
            calcularvelocidadebola()
        if jogador_aliado[0].x == 0 and jogador_aliado[0].y == 0:
            #print("iniciando")
            for i in range(3):
                detectar(i)
        else:

            """print("inicio")
            print(jogador_aliado[1].x)
            print(jogador_aliado[1].y)
            #posicoes"""
            if t - temptempoplay > 0:
                temptempoplay = t
                DecidirPlay()
            SelecionarPlay()

            # goleiro(0)
            # defensor_3(1)
            # Pegarpasse(0, 2)
