#!/usr/bin/env python

import face_recognition
from multiprocessing import Queue
import cv2
import os
from math import sqrt
import easygui
import threading

listdir = os.listdir("./src/tmp_dataset")

print(listdir)