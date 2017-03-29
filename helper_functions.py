#!/usr/bin/env python
import os


def check_filepath(file_path):
        if file_path[-4:] == ".txt":
            file_path = file_path[:-4]
        if os.path.isfile(file_path + ".txt"):
            file_path += "_repeat"
            file_path = check_filepath(file_path)
        return file_path
