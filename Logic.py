#!/usr/bin/env python

import json
import rospy

from time import sleep,time
from arduino_ws.msg import Adc_Mega

from kivy.uix.boxlayout import BoxLayout


class Logic(BoxLayout):
    def __init__(self, **kwargs):
        super(Logic, self).__init__(**kwargs)
        self.settings = []
        self.running = False
        self.score = 0
        self.prev_trigger = [0]*16
        self.set_time = [0]*16
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
        msg = Adc_Mega()
        Ha = [-1]*16
        for i in range(0,len(self.occupancy)):
            state = self.occupancy[i]
            if state == 1:
                ind = self.growth[i]
                Ha[i] = self.colors[ind]
                if ind==1:
                    self.update_background(i,0,0,255,1)
                elif ind==2:
                    self.update_background(i,0,255,0,1)
                elif ind==3:
                    self.update_background(i,255,0,0,1)

                self.update_image(i)
            else:
                self.update_background(i,255,255,0,1)
                self.update_image(i, empty=True)



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
        for i in range(1,17):
            self.update_background(i-1,255,255,0,1)
            self.update_image(i-1)

    def start(self):
        self.running = True
        self.ids.statusTxt.text = "Running"

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
        self.ids['start'].background_color = 1.0, 0.0, 0.0, 1.0
        self.ids['stop'].background_color = 1.0, 0.0, 0.0, 1.0
        self.ids['reset'].background_color = 1.0, 0.0, 0.0, 1.0
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
            self.ids['start'].background_color = 1.0, 1.0, 0.0, 1.0
            self.start()
        if text == "STOP":
            self.ids['stop'].background_color = 1.0, 1.0, 0.0, 1.0
            self.stop()
        if text == "RESET":
            self.ids['reset'].background_color = 1.0, 1.0, 0.0, 1.0
            self.reset()
        pass