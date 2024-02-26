#!/usr/bin/env python3
#!c:/Python35/python3.exe -u
import asyncio
import sys
import cv2
import numpy as np
import cozmo
import time
import os
from glob import glob

# needed for distance (in centimeters)
from cozmo.util import degrees, distance_mm

from find_cube import *

from PIL import ImageDraw, ImageFont

#try:
    #from PIL import ImageDraw, ImageFont
#except ImportError:
    #sys.exit('run `pip3 install --user Pillow numpy` to run this example')
#def nothing(x):
    #pass

YELLOW_LOWER = np.array([9, 115, 151])
YELLOW_UPPER = np.array([179, 215, 255])

GREEN_LOWER = np.array([0,0,0])
GREEN_UPPER = np.array([179, 255, 60])

# Define a decorator as a subclass of Annotator; displays the keypoint
class BoxAnnotator(cozmo.annotate.Annotator):

    cube = None

    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)

        if BoxAnnotator.cube is not None:

            #double size of bounding box to match size of rendered image
            BoxAnnotator.cube = np.multiply(BoxAnnotator.cube,2)

            #define and display bounding box with params:
            #msg.img_topLeft_x, msg.img_topLeft_y, msg.img_width, msg.img_height
            box = cozmo.util.ImageBox(BoxAnnotator.cube[0]-BoxAnnotator.cube[2]/2,
                                      BoxAnnotator.cube[1]-BoxAnnotator.cube[2]/2,
                                      BoxAnnotator.cube[2], BoxAnnotator.cube[2])
            cozmo.annotate.add_img_box_to_image(image, box, "green", text=None)

            BoxAnnotator.cube = None



async def run(robot: cozmo.robot.Robot):

    robot.world.image_annotator.annotation_enabled = False
    robot.world.image_annotator.add_annotator('box', BoxAnnotator)

    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = True
    robot.camera.enable_auto_exposure = True

    gain, exposure, mode = 390,3,1

    try:

        while True:
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)   #get camera image
            if event.image is not None:
                image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_BGR2RGB)

                if mode == 1:
                    robot.camera.enable_auto_exposure = True
                else:
                    robot.camera.set_manual_exposure(exposure,fixed_gain)

                #find the cube
                cube = find_cube(image, YELLOW_LOWER, YELLOW_UPPER)
                print(cube)
                BoxAnnotator.cube = cube

                ################################################################
                # Todo: Add Motion Here
                ################################################################

                # if cube
                if cube:
                    # calculate cm distance from Cozmo to cube
                    distance_to_cube_mm = cube[2]
                    print("distance_to_cube_mm: ", distance_to_cube_mm)

                    # if next to cube (within 5cm/2inch)
                    if distance_to_cube_mm < 40.0:
                        robot.say_text("Found Cube. Within 5 centimeters.").wait_for_completed()
                    else: # if far from cube
                        # go to cube ("drive to within 5cm (2 inches) of it")
                        action = robot.go_to_object(cube, distance_mm(40.0))
                        action.wait_for_completed()

                # if no cube
                else: 
                    # spin to find cube
                    robot.turn_in_place(degrees(15)).wait_for_completed()




    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)
    #cv2.destroyAllWindows()


if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)
