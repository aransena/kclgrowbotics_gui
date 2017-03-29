#!/usr/bin/env python

import json
import rospy

from time import sleep
from arduino_ws.msg import Adc_Mega
from random import randint

from kivy.uix.boxlayout import BoxLayout


class Logic(BoxLayout):
    def __init__(self, **kwargs):
        super(Logic, self).__init__(**kwargs)
        self.settings = []
        self.running = False
        self.score = 0
        self.LED_pub = None
        self.growth = [1]*16
        self.occupancy = []
        self.prev_occupancy = []
        self.images = {0: 'images/empty.png', 1: 'images/plant1.png', 2: 'images/plant2.png', 3:'images/plant3.png'}
        self.colors = {0: -1, 1: 160, 2: 96, 3:0}


    def connect_to_ROS(self):
        rospy.init_node('interface_node', anonymous=True)
        rospy.Subscriber("shelf_state", Adc_Mega, self.shelf_callback)
        self.LED_pub = rospy.Publisher('LED_master', Adc_Mega, queue_size=10)

    def shelf_callback(self, msg):
        self.occupancy = msg.data
        msg = Adc_Mega()
        Ha = [-1]*16
        for i in range(0,len(self.occupancy)):
            state = self.occupancy[i]
            # print self.growth[i], self.growth,self.images,self.images[self.growth[i]]
            if state == 1:
                Ha[i] = self.colors[self.growth[i]]
                ind = self.growth[i]
                if ind==1:
                    self.ids.get('slot'+str(i+1)).background_color = [0,0,255,0.999]
                elif ind==2:
                    self.ids.get('slot'+str(i+1)).background_color = [0,255,0,0.999]
                elif ind==3:
                    self.ids.get('slot'+str(i+1)).background_color = [255,0,0,0.999]

                self.ids.get('im'+str(i+1)).source = self.images[ind]

            else:
                self.ids.get('slot'+str(i+1)).background_color = [255,255,0,0.999]
                self.ids.get('im'+str(i+1)).source = self.images[0]
            # for pub_it in range(0,2):
        msg.data = Ha
        self.LED_pub.publish(msg)
        score = 0
        if self.prev_occupancy:

            for n in range(0,len(self.prev_occupancy)):

                score += (self.prev_occupancy[n]-self.occupancy[n]) * self.growth[n]
                if score>0:
                    print score, n
        self.prev_occupancy = self.occupancy
        sleep(0.01)


    def update_view(self, ind, state):
        if state == 1:
            self.ids.get('slot'+str(ind)+1).background_color = [255,255,0,0.999]
            self.update_image(ind)
        else:
            self.ids.get('slot'+str(ind)+1).background_color = [0,0,255,0.999]
            self.empty_image(ind)

    def load_settings(self):
        print "LOAD SETTINGS"
        f = open("settings.txt")
        self.settings = []
        for line in f:
            self.settings.append(line.split(','))
        f.close()
        return self.settings

    def get_setting(self, name, string_val=False):
        for line in self.settings:
            if line[0] == name:
                if string_val:
                    return line[1]
                else:
                    return float(line[1])

    def setup_view(self):
        for i in range(1,17):
            self.ids.get('slot'+str(i)).background_color = [0,255,255,0.999]
            self.ids.get('im'+str(i)).source = 'images/empty.png'#'images/plant2.png'


    def start(self, fn):
        self.running = True
        self.ids.statusTxt.text = "System Running"

    def stop(self):
        for i in range(1,17):
            self.ids.get('slot'+str(i)).background_color = [0,255,0,0.95]
            self.ids.get('im'+str(i)).source = 'images/plant3.png'

    def reset(self):
        for i in range(1,17):
            self.ids.get('slot'+str(i)).background_color = [0,0,255,0.999]
            self.ids.get('im'+str(i)).source = 'images/plant1.png'


    def send_data(self, topic, data):
        self.ws.send(json.dumps({"op": "publish", "id": "GUI"+self.id_string, "topic": topic, "msg": {"data": data}}))

    def update_image(self, ind):
        if self.occupancy[ind]==1:
            self.ids.get('im'+str(ind+1)).source = self.images[self.growth[ind]]
        else:
            pass
            #self.ids.get('im'+str(ind+1)).source = self.images[0]


    def process_control(self, text, fn):
        print text
        self.ids['start'].background_color = 1.0, 0.0, 0.0, 1.0
        self.ids['stop'].background_color = 1.0, 0.0, 0.0, 1.0
        self.ids['reset'].background_color = 1.0, 0.0, 0.0, 1.0
        if "slot" in text:
            ind = int(text[4:])-1
            self.growth[ind]+=1
            if self.growth[ind] >=4:
                self.growth[ind] = 1
            print "update,", ind, self.growth[ind]
            self.update_image(ind)

        if text == "START":
            self.ids['start'].background_color = 1.0, 1.0, 0.0, 1.0
            self.start(fn)
        if text == "STOP":
            self.ids['stop'].background_color = 1.0, 1.0, 0.0, 1.0
            self.stop()
        if text == "RESET":
            self.ids['reset'].background_color = 1.0, 1.0, 0.0, 1.0
            self.reset()
        pass