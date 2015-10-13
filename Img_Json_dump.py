#!/usr/bin/env python
import rospy
import pymongo
import mongodb_store.util as dc_util
import cv2
import cv
from cv_bridge import CvBridge
import os
import sys
import json
import re
import yaml
import numpy as np
from collections import OrderedDict

#Creating an unordered dictionary and adding it to the yaml representer

def represent_ordereddict(dumper, data):
    value = []

    for item_key, item_value in data.items():
        node_key = dumper.represent_data(item_key)
        node_value = dumper.represent_data(item_value)

        value.append((node_key, node_value))

    return yaml.nodes.MappingNode(u'tag:yaml.org,2002:map', value)

yaml.add_representer(OrderedDict, represent_ordereddict)


#To save depth images without losing values due to png

def save_depth_image(image, filename):
    cv.Save(filename, image, name="depth")

#Connecting to the mongodb database. 

mong = pymongo.MongoClient("bruxelles", 62345)
db = mong['message_store'].upper_bodies
nimg = db.find({},{"ubd_rgb.encoding":1}).count()
first_doc = True

raw_input("A total of {} images recorded. Press enter to start dumping into {}".format(nimg, os.getcwd()))
for i, entry in enumerate(db.find()):
    cls = dc_util.load_class(entry["_meta"]["stored_class"])
    message = dc_util.dictionary_to_message(entry, cls)
    
    #print(type(message))
    #print(message.robot)

    arrayPosition=0
    size = len(message.ubd_rgb)


# Dumping the rgb and depth images + adding the images in a json file

    if(size!=0):
        while(arrayPosition<=size-1):

			img_rgb = CvBridge().imgmsg_to_cv2(message.ubd_rgb[arrayPosition])
			img_d = CvBridge().imgmsg_to_cv2(message.ubd_d[arrayPosition])

			sys.stdout.write('\r{}/{} ({:.2%})'.format(i, nimg, float(i)/(nimg-1)))
			sys.stdout.flush()

			out = "{}-{}-rgb.png".format(message.header.stamp, arrayPosition) #the name of the rgb image file to be dumped
			if img_rgb.shape[0] == 480:
				out = "full_" + out
			cv2.imwrite(out, img_rgb)

			out_d = "{}-{}-d.yml".format(message.header.stamp, arrayPosition) #the name of the depth image file to be dumped
			if img_d.shape[0] == 480:
				out = "full_" + out

			#dumping image in a YAML file based on the Numpy array created from the image conversion

			depthSizeImg = img_d.shape[:2]
			ymlList = np.ravel(np.asarray(img_d))
			
			text = ("%YAML:1.0\ndepth: !!opencv-matrix\n   rows: {}\n   cols: {}\n   dt: {}\n   data: ").format(depthSizeImg[0],depthSizeImg[1], type(img_d[0][0]))
			f = open(out_d, 'a')
			f.write(text)
			yaml.dump(ymlList.tolist(), f, indent = 8)
			f.close()

			#The modified and improved Json Format

			outInJson = "{}_{}.json".format(message.header.stamp, arrayPosition) #the Json file.

			jsonDocument = json.dumps({'robot':{'position': {'x':message.robot.position.x, 'y':message.robot.position.y,'z':message.robot.position.z}, 'orientation':{'x':message.robot.orientation.x, 'y':message.robot.orientation.y,'z':message.robot.orientation.z, 'w':message.robot.orientation.w}}, 'ubd_pos': {'x': message.ubd_pos[arrayPosition].x, 'y': message.ubd_pos[arrayPosition].y, 'z': message.ubd_pos[arrayPosition].z}, 'median_depth': message.ubd.median_depth[arrayPosition]}, indent = 2)

			text_file = open(outInJson, "w")
			text_file.write(jsonDocument)
			text_file.close()

			arrayPosition+=1 #Python has no i++


