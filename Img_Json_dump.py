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

			out = "{}-{}-d.yaml".format(message.header.stamp, arrayPosition) #the name of the depth image file to be dumped
			if img_d.shape[0] == 480:
				out = "full_" + out

			#dumping image in a YAML file based on the Numpy array created from the image conversion
			
			with open(out, 'w') as f:
			yaml.dump(img_d.tolist(), f) 

			#The modified and improved Json Format

			outInJson = "{}_{}.json".format(message.header.stamp, arrayPosition) #the Json file.

			jsonDocument = json.dumps({'robot':{'position': {'x':message.robot.position.x, 'y':message.robot.position.y,'z':message.robot.position.z}, 'orientation':{'x':message.robot.orientation.x, 'y':message.robot.orientation.y,'z':message.robot.orientation.z, 'w':message.robot.orientation.w}}, 'ubd_pos': {'x': message.ubd_pos[arrayPosition].x, 'y': message.ubd_pos[arrayPosition].y, 'z': message.ubd_pos[arrayPosition].z}, 'median_depth': message.ubd.median_depth[arrayPosition]})

			text_file = open(outInJson, "w")
			text_file.write(jsonDocument)
			text_file.close()

			arrayPosition+=1 #Python has no i++

print

#db.upper_bodies.mapReduce(function(){emit(this._id,this.ubd_rgb);},function(key,values){return Array.count(ubd_rgb)},{out:"order_count"})
