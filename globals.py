import numpy as np

class Globals:
    #areaamarela = [2250, 1750, 675, -675] + [200, 200 , 200, -150]
    #areaazul = [-1750, -2250, 675, -675] + [-200, -200, 200, -200]
    #limites = [2250, -2250, 2000, -2000] + [-200, 200, -200, 200]

    areaamarela = [2450, 1950, 875, -875] + [200, 200 , 200, -150]
    areaazul = [-1950, -2450, 875, -875] + [-200, -200, 200, -200]
    limites = [2050, -2050, 1800, -1800] + [-200, 200, -200, 200]


    def __init__(self):
        self.pub = {}
        self.jogador_aliado = None
        self.jogador_inimigo = None
        self.num_jogadores = 3
        self.golnosso = [0, 0, 0]
        self.goldeles = [0, 0, 0]
        self.time = "yellow"
        self.v_max = 1
        self.v_min = 0.1
        self.confiabilidade = 0.8
        self.t0 = 0
        self.t1 = 0
        self.t = 0
        self.posbola0 = [0, 0]
        self.posbola1 = [0, 0]
        self.p_bola = [0, 0]
        self.pos0 = {}
        self.pos1 = {}
        self.vel = {}
        self.velocidadebola = np.array((0, 0))
        self.preso = [False, False, False]
        self.temptempo = 0

        self.temptempoplay = 0
        self.playstuck = False
        self.play = "ataque"
        self.playsetup = True
        self.idgoleironosso = 0
        self.playcache = "ataque"

        self.betab = 210





