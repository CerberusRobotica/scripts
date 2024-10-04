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

import Tatica
import Roles
import Utils
import Plays
import movetoskill
import VetOp
import testecolisao
import globals


pub = {}
g = globals.Globals()

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
    global g
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
    g.t = dados.t_capture
    global bola
    bola = dados.balls


def salvar_bola():
    global p_bola
    try:
        p_bola = ((bola[0].x), (bola[0].y))
        return p_bola

    except:
        return p_bola



def move(g, id, goal: tuple, evitarbola, alpha = -1, charlie=250, ignorar=0, considerararea = 1) -> None:
    # Qual robo se movimentara? int
    # Para onde ele deve ir? vetor[1]
    # Ele deve manter distacia da bola? 0/1
    # Velocidade float
    alpham = False
    if alpha == -1:
        alpham = True

    global vel
    if considerararea:
        goal = Utils.verificardestino(g, id, goal, not considerararea)
    if alpha > g.v_max:
        alpha = g.v_max
    beta = g.betab  # distancia entre o robo e os obstaculos
    pos = Utils.removerrobo(g, id, id)
    betatemp = beta
    if ignorar:
        betatemp = 0

    betav = []
    for i in range(int(len(pos) / 2)):
        betav.append(betatemp) #+ np.sqrt(vel["vx" + str(i)]**2 + vel["vy" + str(i)]**2)) #APRIMORAR

    if evitarbola:  # se ela deve se evitada, adiciona g bola no dicionario
        i = int(len(pos) / 2)
        pos["xr" + str(i)] = g.p_bola[0]
        pos["yr" + str(i)] = g.p_bola[1]
        betav.append(charlie)
    a = movetoskill.skill(g.jogador_aliado[id].x, g.jogador_aliado[id].y, goal[0], goal[1], pos, betav, alpha)
    a = Utils.verificardestino(g, id, a + np.array((g.jogador_aliado[id].x, g.jogador_aliado[id].y)), not considerararea)
    if alpham:
        alpha = Utils.velocidadepdist(g, g.jogador_aliado[id].x, g.jogador_aliado[id].y, a[0], a[1])
        print(id)
        print(a)
    a = VetOp.normalizar(alpha , a - np.array((g.jogador_aliado[id].x, g.jogador_aliado[id].y)))
    ang = g.jogador_aliado[id].orientation
    if isBoolean(a):
        a = (-goal[0] + g.jogador_aliado[id].x, -goal[1] + g.jogador_aliado[id].y)
    else:
        a = ((a[0] * np.cos(ang) + a[1] * np.sin(ang)),
             (-a[0] * np.sin(ang) + a[1] * np.cos(ang)))  # torna o vetor independete da direcao olhada

    ssl_msg[id].cmd_vel.linear.x = a[0]  # envia os comandos de velocidade
    ssl_msg[id].cmd_vel.linear.y = a[1]
    g.pub[id].publish(ssl_msg[id])  # ?publica os comandos
    # ssl.msg[id].cmd_
    return


def chutar(g, id):
    girar(g, id, (g.p_bola[0], g.p_bola[1]), 0.05)
    move(g, id, (g.p_bola[0], g.p_bola[1]), 0, g.v_max, 0, 1, not id == Utils.getgoleiro(g, "nosso"))
    print("tachutando")
    if abs(g.jogador_aliado[id].x - g.p_bola[0]) < 150 and abs(g.jogador_aliado[id].y - g.p_bola[1]) < 150:
        ssl_msg[id].kicker = True
        g.pub[id].publish(ssl_msg[id])
        ssl_msg[id].kicker = False
        g.pub[id].publish(ssl_msg[id])
        if np.linalg.norm(g.velocidadebola) > 300*g.v_max:
            parada(g, id)
            return True

    return False



def girar(g, id, goal, tolerancia):
    # ID: Qual robo deve girar? int
    # goal: Para que ponto ele deve olhar? vetor[1]
    # tolerancia: Qual g tolerancia em radianos para o alinhamento? float
    salvar_bola()
    orientacao = 0
    delta = 0
    KP_ang = 5
    if p_bola[0] - g.jogador_aliado[id].x == 0:
        dest = 0
    else:
        dest = np.arctan((goal[1] - g.jogador_aliado[id].y) / (goal[0] - g.jogador_aliado[id].x))
        if goal[0] - g.jogador_aliado[id].x < 0:
            dest += np.pi


        orientacao = g.jogador_aliado[id].orientation


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
    g.pub[id].publish(ssl_msg[id])

def parada(g, robotIndex):
    ssl_msg[robotIndex].cmd_vel.angular.z = 0.0
    ssl_msg[robotIndex].cmd_vel.linear.x = 0.0
    ssl_msg[robotIndex].cmd_vel.linear.y = 0.0
    g.pub[robotIndex].publish(ssl_msg[robotIndex])










def DecidirPlay(g):

    if not g.playstuck:
        if Utils.temomaisproximo(g) == "nos" or Utils.ladodocampo(g) != g.time:
            g.play = "ataque"
        else:
            g.play = "defesa"

    if g.play != g.playcache:
        g.playcache = g.play
        g.playsetup = True
    return g


def SelecionarPlay(g):
    if g.play == "defesa":
        g = Plays.PlayDefesa(g)
    if g.play == "ataque":
        g = Plays.PlayAtaque(g)
    if g.play == "halt":
        g = Plays.PlayHalt(g)
    return g




























if __name__ == "__main__":
    rospy.init_node("test_ssl", anonymous=False)
    sub = rospy.Subscriber("/vision", SSL_DetectionFrame, definir_dados)
    if g.time == "yellow":
        for i in range(g.num_jogadores):
            topic = '/robot_yellow_{}/cmd'.format(i)
            g.pub[i] = rospy.Publisher(topic, SSL, queue_size=10)
            g.golnosso = [2250, -400, 400]  # x, y0, yf
            g.goldeles = [-2250, -400, 400]
            g.jogador_aliado = jogador_yellow
            g.jogador_inimigo = jogador_blue
    if g.time == "blue":
        for i in range(g.num_jogadores):
            topic = '/robot_blue_{}/cmd'.format(i)
            g.pub[i] = rospy.Publisher(topic, SSL, queue_size=10)
            g.golnosso = [-2250, -400, 400]  # x, y0, yf
            g.goldeles = [2250, -400, 400]
            g.jogador_aliado = jogador_blue
            g.jogador_inimigo = jogador_yellow


    r = rospy.Rate(120)

    while not rospy.is_shutdown():



        if g.jogador_aliado[0].x == 0:
            print("iniciando")
        else:
            if g.t > g.t1:
                g.p_bola = salvar_bola()
                g = Utils.calculartempo(g)
                g = Utils.calcularvelocidades(g)
                g = Utils.calcularposicoes(g)
                g = Utils.calcularvelocidadebola(g)

            if g.t - g.temptempoplay > 0:
                g.temptempoplay = g.t
                g.preso = [False, False, False]
                g = DecidirPlay(g)
            g = SelecionarPlay(g)

