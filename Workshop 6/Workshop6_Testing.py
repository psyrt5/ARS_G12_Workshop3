#If you are initialising the bottom left grid cell to 1,1 not 0,0

from math import pi, sqrt, radians , ceil

class Testing:
    def __init__(self):
        print("Expected: 5,5")
        self.to_grid(5,5,0,0,20,20,1)
        print("Expected: 10,10")
        self.to_grid(5,5,0,0,20,20,0.5)
        print("Expected: 5,5")
        self.to_grid(0,0,-5,-5,20,20,1)
        print("Expected: -5,-5")
        self.to_grid(-5,-5,0,0,20,20,1)
        print("Expected: 5,5")
        self.to_grid(5,5,0,0,10,20,1)
        print("Expected: 4.5,4.5")
        self.to_world(9,9,-5,-5,20,20,1)
    
    def to_grid(self, px, py, origin_x, origin_y, size_x, size_y, resolution):
        gx = ceil((px-origin_x)/resolution)
        gy = ceil((py-origin_y)/resolution)
        print(gx, gy)
    
    #If you are initialising the bottom left grid cell to 1,1 not 0,0
    def to_world(self, gx, gy, origin_x, origin_y, size_x, size_y, resolution):
        px = float((gx*resolution + origin_x) + (resolution/2.0))
        py = float((gy*resolution + origin_y) + (resolution/2.0))
        print (px ,py)

testing = Testing()