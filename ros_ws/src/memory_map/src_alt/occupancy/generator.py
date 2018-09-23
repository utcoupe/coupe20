#!/usr/bin/python
import os
import time
import rospy
from PIL import Image, ImageDraw

class OccupancyGenerator():
    def __init__(self, world):
        self.ImgWidth = 500 # Width of the generated images. Height will be calculated based on the map aspect ratio.
        self.WorldSize = (float(world.get("/terrain/shape/width")), float(world.get("/terrain/shape/height")))
        self.ImgSize = (self.ImgWidth, int(self.ImgWidth * (self.WorldSize[1] / self.WorldSize[0])))

    def generateLayer(self, world, layer_name, margin = 0.0):
        final_path = os.path.dirname(__file__) + "/img/" + layer_name + ".bmp"
        layers = world.get("/terrain/walls/^")
        if not layer_name in layers.Dict.keys():
            rospy.logerr("    Tried to get layer '{}''s image, but it hasn't been generated. Aborting.".format(layer_name))
            return None
        img = self.generateStaticOccupancy(layers.Dict[layer_name], margin)
        img.save(final_path)
        return final_path

    def generateStaticOccupancy(self, layer, margin = 0.0): # margin in m.
        img = Image.new("RGB", self.ImgSize, (255, 255, 255))

        draw = ImageDraw.Draw(img)

        for wall in layer.toList():
            position, shape = wall.get("position/^").toDict(), wall.get("shape/^").toDict()

            pos = self.world_to_img_pos((position["x"], position["y"]))
            if shape["type"] == "rect":
                w, h = self.world_to_img_scale(shape["width"] + margin * 2), self.world_to_img_scale(shape["height"] + margin * 2)
                draw.rectangle((pos[0] - w/2.0, pos[1] - h/2.0, pos[0] + w/2.0, pos[1] + h/2.0), fill=(0, 0, 0))
            elif shape["type"] == "circle":
                r = self.world_to_img_scale(shape["radius"] + margin)
                draw.ellipse((pos[0] - r, pos[1] - r, pos[0] + r, pos[1] + r), fill = (0, 0, 0))
            elif shape["type"] == "polygon":
                rospy.logwarn("Occupancy generator: polygon drawing not implemented")
            elif shape["type"] == "line":
                rospy.logwarn("Occupancy generator: line drawing not implemented")
            else:
                rospy.logerr("Occupancy generator could not recognize shape type '{}'.".format(shape["type"]))
        del draw
        return img

    def world_to_img_scale(self, world_coord):
        return world_coord * (self.ImgSize[0] / (self.WorldSize[0]))
    def world_to_img_pos(self, world_pos):
        return (self.world_to_img_scale(world_pos[0]), self.ImgSize[1] - self.world_to_img_scale(world_pos[1]))
