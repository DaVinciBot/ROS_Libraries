#!/usr/bin/env python

import face_recognition
from multiprocessing import Queue
import cv2
import os
from math import sqrt
import easygui
import threading

import rospy

def knownFaces():
    """Load a list of each person of your dataset ***** Replace Pi2/tmp_dataset by your path to your dataset"""
    known_faces = []
    list_face = os.listdir('./src/tmp_dataset')
    if(len(list_face)==0):return known_faces
    """We sort to avoid crash during analyses"""
    list_face.sort()
    for face in list_face:
        """Encode each face of the folder *****  change your path again"""
        faceLoad = face_recognition.load_image_file("./src/tmp_dataset/"+face)
        known_faces.append(face_recognition.face_encodings(faceLoad)[0])
    """Return a tab with the encoded faces"""
    return known_faces

def takeFace(face_locations,small_frame,known_faces,res):   
    """Open a message box which ask your name"""
    name = easygui.enterbox('Your name : ','Face recognition')
    """Save your face in the dataset"""
    for (top, right, bottom, left) in face_locations:  
        crop_face=cv2.flip(small_frame[top:bottom,left:right],1)
    cv2.imwrite('./src/tmp_dataset/'+name+'.jpg',crop_face)
    """Refresh the encoded faces with the new one"""
    known_faces = knownFaces()
    """Add the result tab in a queue to get it later while threading"""
    res.put(known_faces)
    return

def bigFace(small_frame):
    """Take all positions of each faces which are in the frame in a tab"""
    faces = face_recognition.face_locations(small_frame)
    max=0
    bigface = []
    j=0
    index=0
    for (top, right, bottom, left) in faces:
        """Check which diagonal is the longest""" 
        if(sqrt((right-left)*(right-left)+(top-bottom)*(top-bottom)) > max):
            max=sqrt((right-left)*(right-left)+(top-bottom)*(top-bottom))
            index=j
        j=j+1
    """Return the location of the nearest face"""
    if(faces == []):return []
    else:
        bigface.append(faces[index])
    return bigface


def choice(face_locations,small_frame,known_faces,res):
    """Open a message box Yes/No question"""
    if(easygui.ynbox('Do you want to be in the dataset ?', 'Face recognition', ('Yeah', 'Hell no !')) == True):
        takeFace(face_locations,small_frame,known_faces,res)
    else:
        """To avoid crash, it's needed to return the known_faces"""
        res.put(known_faces)
        return         
        
def fr():
    """HOG : Histogram of Oriented Gradients <- Model/Method
    Get a reference to webcam #0 (the default one)"""
    video_capture = cv2.VideoCapture(0)
    """Set the height and width of the frame"""
    ret = video_capture.set(3,1920)
    ret = video_capture.set(4,1080)
    """Queue to get result from thread"""
    res = Queue()
    """Get the tabof all encoded faces"""
    known_faces = knownFaces()
    """ Initialize some variables"""
    face_locations = []
    face_encodings = []
    face_names = []
    time = 0
    process_this_frame = True
    
    while True:      
        """ Grab a single frame of video"""      
        ret, frame = video_capture.read()
        """ Resize frame of video to 1/4 size for faster face recognition processing"""
        small_frame = cv2.resize(frame, (0,0), fx=0.25, fy=0.25)    
        """ Only process every other frame of video to save time"""
        if process_this_frame:
            """ Find the nearest face and face encodings in the current frame of video"""
            face_locations = bigFace(small_frame)
            face_encodings = face_recognition.face_encodings(small_frame, face_locations)
            face_names = []
            """Initialize the thread for the message box"""
            thread = threading.Thread(target=choice,args=(face_locations,small_frame,known_faces,res,))
            
            for face_encoding in face_encodings:
                """ See if the face is a match for the known face(s)"""
                match = face_recognition.compare_faces(known_faces,face_encoding)
                """add 1234 because of the .jpeg extension"""
                name = "Unknown1234"
                """Change the path here too"""
                list_face = os.listdir('./src/tmp_dataset')
                list_face.sort()
                """Get the name of the face if known"""
                for face in list_face:
                    for i in range(len(match)):
                        if match[i]:
                            name = list_face[i]
                """remove the .jpeg from face's name then put the text"""
                face_names.append(name[:-4])
                """If the face is unknown for a short time behind the camera, ask to be add""" 
                if(face_names[0] == "Unknown"):
                    if(time > 100):
                        if(thread.is_alive() == False):
                            thread.start()
                            known_faces = res.get()
                        time = 0
                    time=time+1
        process_this_frame = not process_this_frame
        """ Display the results"""
        for (top, right, bottom, left), name in zip(face_locations, face_names):
            """ Scale back up face locations since the frame we detected in was scaled to 1/4 size"""
            top *= 4
            right *= 4
            bottom *= 4
            left *= 4
            """ Draw a box around the face"""
            cv2.rectangle(frame, (left, top), (right, bottom), (255, 0, 0), 2)
            """ Draw a label with a name below the face"""
            cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (255, 0, 0), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
        """ Display the resulting image"""
        cv2.imshow('Video', frame)
        """ Hit 'q' on the keyboard to quit!"""
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    """ Release handle to the webcam"""
    video_capture.release()
    cv2.destroyAllWindows()     
    
"""Call the recognition"""    
fr()   
