class simpleVectorVelocity:
    vx = 0
    vy = 0
    vxN = 0
    vyN = 0
    def __init__(self, vx, vy):
        self.vx = vx
        self.vy = vy
    def magnitudeIs(self):
        return math.sqrt(math.pow(self.vx,2) + math.pow(self.vy,2))
    def phaseToXIs(self):
        return math.atan2(self.vy, self.vx) * 180 / math.pi

    def normalize(self):
        #newMag = self.magnitudeIs()
        self.vyN = math.sin(self.phaseToXIs() * math.pi / 180) 
        self.vxN = math.cos(self.phaseToXIs()* math.pi / 180) 
    def vxNorm(self):
        
        self.vxN = math.cos(self.phaseToXIs()* math.pi / 180) 
        return (self.vxN)
    def vyNorm(self):
        self.vyN = math.sin(self.phaseToXIs() * math.pi / 180) 
        return (self.vyN)
