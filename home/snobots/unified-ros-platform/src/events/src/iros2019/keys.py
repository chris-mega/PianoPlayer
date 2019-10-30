class Key(object):

    def __init__(self):
        self.area = -1
        self.x = -1
        self.y = -1
        self.angle = -100
        self.width = -1
        self.height = -1
        

    def update(self, x, y, width, height, area, angle):
        self.area = area
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.angle = angle


class Keys(object):
    def __init__(self):
        self.blue = Key()
        self.hand = Key()
        self.c2 = Key()
        self.g2 = Key()