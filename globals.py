class Globals:
    time = ""
    num_jogadores = 0
    golnosso = [0, 0, 0]

    def __init__(self):
        self.time = "yellow"
        self.num_jogadores = 3

        self.golnosso = [2000, -500, 500]  #x, y0, yf
        if self.time == "yellow":
            self.golnosso[1] = [-2000]
