class Vector3(object):
    def __init__(self, x, y, z):
        self.x = float(x) if x is not None else None
        self.y = float(y) if y is not None else None
        self.z = float(z) if z is not None else None
