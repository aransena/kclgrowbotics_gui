"""Real time plotting of Microphone level using kivy
"""
# taken out of .kv file:
# #:import MeshLinePlot kivy.garden.graph.MeshLinePlot

from kivy.lang import Builder
from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.clock import Clock
from kivy.graphics import Color, Ellipse, Line, Rectangle
from kivy.config import Config
from kivy.uix.widget import Widget
from kivy.uix.label import Label
from kivy.core.window import Window
from kivy.uix.button import Button
from kivy.uix.image import Image


import os
import math
import time
import websocket
import json
import Queue
import threading
import random


Config.set('graphics', 'resizable', True)
Config.set('graphics', 'width', '1920')
Config.set('graphics', 'height', '1080')


class Logic(BoxLayout):
    def __init__(self, **kwargs):
        super(Logic, self).__init__(**kwargs)


        # Initial running parameters
        # self.info_display = InfoDisplay(size_hint=(.1, .9))
        # self.add_widget(self.info_display)

        # General parameters
        self.settings = []

    # def set_text(self, text):
    #     self.ids.filename.text = text

    # def setup_data_thread(self):
    #     self.data_thread = threading.Thread(target=self.data_stream)
    #     self.data_thread.daemon = True
    #     self.data_thread.start()

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

    def check_filepath(self, file_path):
        if file_path[-4:] == ".txt":
            file_path = file_path[:-4]
        if os.path.isfile(file_path + ".txt"):
            file_path += "_repeat"
            file_path = self.check_filepath(file_path)
        return file_path

    # def btn_pressed(self, *args):
    #     btn = args[0]
    #     print "Pressed ", btn.id
    #     btn.source = 'images/plant1.png'


    def start(self, fn):

        # self.ids.slot1.background_color = [255,255,0,0.999]
        # self.ids.im1.source = 'images/plant2.png'
        # # im=Image(source='images/plant3.jpg')
        # for i in range(0,16):
        #     ids = 'btn' + str(i)
        #     btn = Button(id = ids,
        #                  source = im,
        #                  size = self.parent.size)#, center_x = self.parent.center_x,
        #     btn.background_colour = [255,255,0,0.999]
        #     btn.bind(on_release=self.btn_pressed)
        #     self.ids.grid.add_widget(btn)
        #     print btn.id, len(self.ids.grid.children)
        #
        #
        # self.ids.grid.children[0].source = im

        for i in range(1,17):
            self.ids.get('slot'+str(i)).background_color = [255,255,0,0.999]
            self.ids.get('im'+str(i)).source = 'images/plant3.png'


                         #center_y = self.parent.center_y)
            #btn.text = 'test' + str(i)

    def stop(self):
        self.ids.slot1.background_color = [0,255,0,0.95]
        self.ids.im1.source = 'images/plant3.png'

    def send_data(self, topic, data):
        self.ws.send(json.dumps({"op": "publish", "id": "GUI"+self.id_string, "topic": topic, "msg": {"data": data}}))

    def process_control(self, text, fn):
        self.ids['start'].background_color = 1.0, 0.0, 0.0, 1.0
        self.ids['stop'].background_color = 1.0, 0.0, 0.0, 1.0
        self.ids['reset'].background_color = 1.0, 0.0, 0.0, 1.0
        self.ids['play'].background_color = 1.0, 0.0, 0.0, 1.0

        if text == "START":
            self.ids['start'].background_color = 1.0, 1.0, 0.0, 1.0
            self.start(fn)
        if text == "STOP":
            self.ids['stop'].background_color = 1.0, 1.0, 0.0, 1.0
            self.stop()
        if text == "RESET":
            self.ids['reset'].background_color = 1.0, 1.0, 0.0, 1.0
            #  self.reset()
        if text == "PLAY":
            self.ids['play'].background_color = 1.0, 1.0, 0.0, 1.0
            #  self.play(fn)
        pass


class Growbotics(App):
    def __init__(self, **kwargs):
        super(Growbotics, self).__init__(**kwargs)
        self.settings = []

    def build(self):
        self.root = Widget()
        self.root = Builder.load_file("growbotics.kv")

        return self.root

    def on_start(self):
        print "START"

        self.settings = self.root.load_settings()


    def on_stop(self):
        print "STOP"


if __name__ == "__main__":
    position = []
    Growbotics().run()
