#!/usr/bin/env python


from kivy.lang import Builder
from kivy.app import App
from kivy.config import Config
from kivy.uix.widget import Widget
from Logic import Logic

Config.set('graphics', 'resizable', True)
Config.set('graphics', 'width', '1920')
Config.set('graphics', 'height', '1080')

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
        self.root.setup_view()
        self.root.connect_to_ROS()


    def on_stop(self):
        print "STOP"


if __name__ == "__main__":
    position = []
    Growbotics().run()
