#!/usr/bin/python
import rospkg
import time
import rospy
from PIL import Image, ImageDraw

class OccupancyGenerator():
    def generateLayer(self, world, layer_name, img_width = 0, margin = 0.0):
        def generateStaticOccupancy(layers, layer_name, img_size, world_size, margin = 0.0): # margin in m.
            img = Image.new("RGB", img_size, (255, 255, 255))

            draw = ImageDraw.Draw(img)

            walls = layers.Dict[layer_name].toList()
            for i in layers.Dict[layer_name].includes:
                if i in layers.Dict.keys():
                    walls += layers.Dict[i].toList()
                else:
                    rospy.logerr("Couldn't find layer to include named '{}', skipping.".format(i))

            for wall in walls:
                position, shape = wall.get("position/^").toDict(), wall.get("shape/^").toDict()

                pos = world_to_img_pos(img_size, world_size, (position["x"], position["y"]))
                if shape["type"] == "rect":
                    w = world_to_img_scale(img_size, world_size, shape["width"]  + margin * 2)
                    h = world_to_img_scale(img_size, world_size, shape["height"] + margin * 2)
                    draw.rectangle((pos[0] - w/2.0, pos[1] - h/2.0, pos[0] + w/2.0, pos[1] + h/2.0), fill=(0, 0, 0))
                elif shape["type"] == "circle":
                    r = world_to_img_scale(img_size, world_size, shape["radius"] + margin)
                    draw.ellipse((pos[0] - r, pos[1] - r, pos[0] + r, pos[1] + r), fill = (0, 0, 0))
                elif shape["type"] == "polygon":
                    rospy.logwarn("Occupancy generator: polygon drawing not implemented")
                elif shape["type"] == "line":
                    rospy.logwarn("Occupancy generator: line drawing not implemented")
                else:
                    rospy.logerr("Occupancy generator could not recognize shape type '{}'.".format(shape["type"]))
            del draw
            return img

        def world_to_img_scale(img_size, world_size, world_coord):
            return world_coord * (img_size[0] / (world_size[0]))
        def world_to_img_pos(img_size, world_size, world_pos):
            return (world_to_img_scale(img_size, world_size, world_pos[0]), img_size[1] - world_to_img_scale(img_size, world_size, world_pos[1]))

        img_width = img_width if img_width != 0 else 500 # Default value
        world_size = (float(world.get("/terrain/shape/width")), float(world.get("/terrain/shape/height")))
        img_size = (img_width, int(img_width * (world_size[1] / world_size[0])))

        final_path = rospkg.RosPack().get_path("memory_map") + "/def/occupancy/" + layer_name + ".bmp"
        layers = world.get("/terrain/walls/^")
        if not layer_name in layers.Dict.keys():
            rospy.logerr("    Couldn't find layer '{}'. Aborting.".format(layer_name))
            return None

        img = generateStaticOccupancy(layers, layer_name, img_size, world_size, margin)
        img.save(final_path)
        return final_path
