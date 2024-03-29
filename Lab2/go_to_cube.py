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

from find_cube import *

try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')
def nothing(x):
    pass

# YELLOW_LOWER = np.array([10, 60, 118])
# YELLOW_UPPER = np.array([80, 255, 230])

GREEN_LOWER = np.array([15, 20, 20])
GREEN_UPPER = np.array([120, 255, 175])

CLOSE_RADIUS = 60
CENTER_X = 160
DIRECTION_BOUND = 20

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
                image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2BGR)

                if mode == 1:
                    robot.camera.enable_auto_exposure = True
                else:
                    robot.camera.set_manual_exposure(exposure,gain)

                #find the cube
                cube = find_cube(image, GREEN_LOWER, GREEN_UPPER)
                print(cube)
                BoxAnnotator.cube = cube

                ################################################################
                # Todo: Add Motion Here
                ################################################################

                # if no cube
                if cube == [0, 0, 0] or cube == None:
                    # spin to find cube
                    await robot.turn_in_place(cozmo.util.degrees(30)).wait_for_completed()

                # if cube
                else: 
                    # if cube too far right
                    if cube[0] > CENTER_X + DIRECTION_BOUND:
                        # turn right slightly
                        await robot.turn_in_place(cozmo.util.degrees(-3)).wait_for_completed()
                    # if cube too far left
                    elif cube[0] < CENTER_X - DIRECTION_BOUND:
                        # turn left slightly
                        await robot.turn_in_place(cozmo.util.degrees(3)).wait_for_completed()
                    # if far from cube
                    elif cube[2] < CLOSE_RADIUS:
                        await robot.drive_straight(cozmo.util.distance_inches(1), cozmo.util.speed_mmps(50)).wait_for_completed()
                time.sleep(0.25)


    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)
    #cv2.destroyAllWindows()


if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)
