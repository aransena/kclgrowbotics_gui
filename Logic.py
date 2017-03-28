import os
import json
from kivy.uix.boxlayout import BoxLayout


class Logic(BoxLayout):
    def __init__(self, **kwargs):
        super(Logic, self).__init__(**kwargs)
        # General parameters
        self.settings = []

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

    def setup_view(self):
        for i in range(1,17):
            self.ids.get('slot'+str(i)).background_color = [255,255,0,0.999]
            self.ids.get('im'+str(i)).source = ''#'images/plant2.png'


    def start(self, fn):
        for i in range(1,17):
            self.ids.get('slot'+str(i)).background_color = [255,255,0,0.999]
            self.ids.get('im'+str(i)).source = 'images/plant2.png'


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

    def process_control(self, text, fn):
        print text
        self.ids['start'].background_color = 1.0, 0.0, 0.0, 1.0
        self.ids['stop'].background_color = 1.0, 0.0, 0.0, 1.0
        self.ids['reset'].background_color = 1.0, 0.0, 0.0, 1.0


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