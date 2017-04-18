#!/usr/bin/env python

import json
import rospy

from time import sleep,time
from arduino_ws.msg import Adc_Mega

from kivy.uix.boxlayout import BoxLayout
from random import randint

class Logic(BoxLayout):
    def __init__(self, **kwargs):
        super(Logic, self).__init__(**kwargs)
        self.growing = False
        self.settings = []
        self.running = False
        self.score = 0
        self.prev_trigger = [0]*16
        self.set_time = [0]*16
        self.age = [0]*16
        self.LED_pub = None
        self.growth = [1]*16
        self.occupancy = [1]*16
        self.prev_occupancy = []
        self.images = {0: 'images/empty.png', 1: 'images/plant1.png', 2: 'images/plant2.png', 3:'images/plant3.png'}
        self.colors = {0: -1, 1: 160, 2: 96, 3:0}


    def connect_to_ROS(self):
        rospy.init_node('interface_node', anonymous=True)
        rospy.Subscriber("shelf_state", Adc_Mega, self.shelf_callback)
        self.LED_pub = rospy.Publisher('LED_master', Adc_Mega, queue_size=10)

    def shelf_callback(self, msg):
        self.occupancy = msg.data
        rev = 15
        msg = Adc_Mega()
        Ha = [-1]*16
        for i in range(0,len(self.occupancy)):
            state = self.occupancy[rev-i]
            if state == 1:
                ind = self.growth[rev-i]
                Ha[i] = self.colors[ind]
                if ind==1:
                    self.update_background(rev-i,0,0,255,1)
                elif ind==2:
                    self.update_background(rev-i,0,255,0,1)
                elif ind==3:
                    self.update_background(rev-i,255,0,0,1)

                self.update_image(rev-i)
            else:
                self.update_background(rev-i,255,255,255,1)
                self.update_image(rev-i, empty=True)

        if self.running:
            for a,age in enumerate(self.age):
                if time()>age and age is not 0:
                    print "here"
                    self.growth[a]+=1
                    print "htt"
                    g = self.growth[a]
                    print "www"
                    prev_age = self.age[a]
                    print "there"
                    new_age = self.update_age(self,a,g)
                    self.age[a] = new_age
                    print "333"
                    print prev_age, self.age[a]

        if len(self.prev_occupancy)>0 and self.running:
            index = 0
            # prevents debounce scoring
            for slot,pslot in zip(self.occupancy,self.prev_occupancy):
                if slot > pslot:
                    self.set_time[index] = time()
                index += 1

            self.check_score()

        if self.running:
            self.ids.statusTxt.text = "Score: " + str(self.score)
        self.prev_occupancy = self.occupancy
        msg.data = Ha
        self.LED_pub.publish(msg)
        sleep(0.05)

    def check_score(self):
        scores = [0,1,5,2]
        for n,prev in enumerate(self.prev_occupancy):
            score = (prev-self.occupancy[n]) * scores[self.growth[n]]
            if score>0 and time()-self.set_time[n]>1:
                # prevents debounce scoring
                if time() - self.prev_trigger[n]>1:
                    if score is not 1:
                        self.score += score
                    else:
                        self.score -= 1
                    self.prev_trigger[n] = time()
                else:
                    self.prev_trigger[n] = time()


    def load_settings(self):
        print "LOAD SETTINGS"
        f = open("settings.txt")
        self.settings = []
        for line in f:
            self.settings.append(line.split(','))
        f.close()
        return self.settings

    def get_setting(self, name, string_val=False, list=False, float_val=True):
        for line in self.settings:
            if line[0] == name:
                if string_val:
                    return line[1]
                else:
                    if list:
                        if float_val:
                            return map(float,line[1:])
                        else:
                            return map(int,line[1:])
                    else:
                        if float_val:
                            return float(line[1])
                        else:
                            return int(line[1])

    def setup(self):
        self.ids.statusTxt.text = "Idle"
        self.growth = self.get_setting('growth', list=True, float_val=False)

        for i in range(0,16):
            self.update_background(i,255,255,0,1)
            self.update_image(i)

    def start(self):
        self.running = True
        self.ids.statusTxt.text = "Running"
        # setup plant ages
        for i,g in enumerate(self.growth):
            self.update_age(i,g)

    def update_age(self,index,growth):
        if self.growing:
            print "UPDATE", index, growth

            if growth == 1:
                timer = self.get_setting('young',list=True,float_val=False)
                self.age[index]=int(time())+randint(timer[0],timer[1]) # will mature sometime in the next 30-60 seconds
            elif growth == 2:
                timer = self.get_setting('mature',list=True,float_val=False)
                self.age[index]=int(time())+randint(timer[0],timer[1]) # will die sometime in the next 120-150 seconds
            elif growth==3:
                timer = self.get_setting('dead',list=True,float_val=False)
                self.age[index]=int(time())+randint(timer[0],timer[1]) # dead
        else:
            pass


    def stop(self):
        self.running = False
        self.ids.statusTxt.text = "Stop, Score: " + str(self.score)

    def reset(self):
        self.growth = [1]*16
        self.score = 0
        self.ids.statusTxt.text = "Reset"

    def send_data(self, topic, data):
        self.ws.send(json.dumps({"op": "publish", "id": "GUI"+self.id_string, "topic": topic, "msg": {"data": data}}))

    def update_image(self, ind, empty=False):
        if empty:
            self.ids.get('im'+str(ind+1)).source = self.images[0]

        elif self.occupancy[ind]==1:
            im_ind = self.growth[ind]
            self.ids.get('im'+str(ind+1)).source = self.images[im_ind]

    def update_background(self,ind,r,g,b,a):
        self.ids.get('slot'+str(ind+1)).background_color = [r,g,b,a]

    def process_control(self, text):
        if "slot" in text:
            ind = int(text[4:])-1
            if self.occupancy[ind] == 1:
                self.growth[ind]+=1
                if self.growth[ind] >=4:
                    self.growth[ind] = 1

                self.update_image(ind)

        else:
            print text

        if text == "START":
            self.start()
        if text == "STOP":
            self.stop()
        if text == "RESET":
            self.reset()
        pass