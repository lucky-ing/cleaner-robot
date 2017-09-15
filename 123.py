import rospy
#from roslaunch import launch, node_args
#import roslaunch
import tf
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
#from dwa_local_planner.cfg import DWAPlannerConfig
#from base_local_planner.cfg import BaseLocalPlannerConfig
from sensor_msgs.msg import LaserScan, Imu
#import copy
#from kobuki_msgs.msg import BumperEvent, RobotStateEvent
#from visualization_msgs.msg import Marker
#from map_msgs.msg import *
from geometry_msgs.msg import PointStamped,Point,PoseStamped
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
from tf.msg import tfMessage
#import Queue
from Tkinter import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, TransformStamped
#from math import *
import tkMessageBox
#import rqt_robot_monitor
import numpy as np
#from numpy import random
import serial.tools.list_ports as sl
import serial
import threading
import math
import time
#import os
import cv2
import copy

error = int(1)
ok = int(0)
turndir = {'left': 0, 'right': 1}
left_click = 1
right_click = 2
FIFILINE = 4
SETLINE = 5
HORLINE = 5
CLICKLINE = 18
PASTLINE = 12
ANGLE_TIME = 5
import ctypes

class box(ctypes.Structure):
    _fields_ = [('x',ctypes.c_float),('y',ctypes.c_float),('w',ctypes.c_float),('h',ctypes.c_float)]

class detector_t(ctypes.Structure):
    _fields_ = [("probs", ctypes.POINTER(ctypes.POINTER( ctypes.c_float))), ("boxes", ctypes.POINTER(box))]

class bbox_t(ctypes.Structure):
    _fields_ = [('x', ctypes.c_int), ('y', ctypes.c_int), ('w', ctypes.c_int), ('h', ctypes.c_int),("prob", ctypes.c_float),
                ("obj_id", ctypes.c_uint)]

class ObsDetInstance(ctypes.Structure):
    _fields_ = [("camera_height", ctypes.c_float), ("theta_Init", ctypes.c_float),
                ("cameraInternalParam_fx", ctypes.c_float),("cameraInternalParam_fy", ctypes.c_float),
                ("cameraInternalParam_cx", ctypes.c_float),("cameraInternalParam_cy", ctypes.c_float),
                ("Img_width", ctypes.c_uint), ("Img_width", ctypes.c_uint),
                ("ROI",bbox_t),
                ("estimate_distance", ctypes.c_float),("estimate_objwidth", ctypes.c_float),
                ("Thing", bbox_t)]


class CarTask(object):
    def __init__(self):
        self.orix = 0
        self.oriy = 0
        self.carx = 0
        self.cary = 0
        self.aimx = 0
        self.aimy = 0
        self.direction = 0
        self.w = 0
        self.h = 0
        self.task = 0
        self.state = 0


class CarState(object):
    def __init__(self):
        self.x = 0
        self.y = 0
        self.angle = 0


class MapRobot(object):
    def __init__(self, master):
        self.master = master
        self.senFram = LabelFrame(self.master, width=80, text='SensorState')
        self.senFram.grid(row=0, column=0, rowspan=4, columnspan=5)
        self.BumLeftLabel = Label(self.senFram, width=12, text='LeftBum', bg='white')
        self.BumLeftLabel.grid(row=0, column=0, columnspan=2)
        self.BumRightLabel = Label(self.senFram, width=12, text='RightBum', bg='white')
        self.BumRightLabel.grid(row=0, column=3, columnspan=2)
        Label(self.senFram, text='Infrared0', width=12).grid(row=1, column=0)
        Label(self.senFram, text='Infrared1', width=12).grid(row=1, column=1)
        Label(self.senFram, text='Infrared2', width=12).grid(row=1, column=2)
        Label(self.senFram, text='Infrared3', width=12).grid(row=1, column=3)
        Label(self.senFram, text='Infrared4', width=12).grid(row=1, column=4)
        self.infra0 = Label(self.senFram, text='0')
        self.infra0.grid(row=2, column=0)
        self.infra1 = Label(self.senFram, text='0')
        self.infra1.grid(row=2, column=1)
        self.infra2 = Label(self.senFram, text='0')
        self.infra2.grid(row=2, column=2)
        self.infra3 = Label(self.senFram, text='0')
        self.infra3.grid(row=2, column=3)
        self.infra4 = Label(self.senFram, text='0')
        self.infra4.grid(row=2, column=4)

        self.CarFrame = LabelFrame(self.master, width=80, text='CarState')
        self.CarFrame.grid(row=4, column=0, rowspan=2, columnspan=5)
        Label(self.CarFrame, text='X', width=12).grid(row=0, column=0)
        Label(self.CarFrame, text='Y', width=12).grid(row=0, column=1)
        Label(self.CarFrame, text='ANGLE', width=12).grid(row=0, column=2)
        Label(self.CarFrame, text='ODO_LEFT', width=12).grid(row=0, column=3)
        Label(self.CarFrame, text='ODO_RIGHT', width=12).grid(row=0, column=4)
        self.xL = Label(self.CarFrame, text='0')
        self.xL.grid(row=1, column=0)
        self.yL = Label(self.CarFrame, text='0')
        self.yL.grid(row=1, column=1)
        self.angleL = Label(self.CarFrame, text='0')
        self.angleL.grid(row=1, column=2)
        self.ODOLEFTL = Label(self.CarFrame, text='0')
        self.ODOLEFTL.grid(row=1, column=3)
        self.ODORIGHTL = Label(self.CarFrame, text='0')
        self.ODORIGHTL.grid(row=1, column=4)
        self.BandRateList = ['9600', '115200', '29600']
        self.BandRateVar = StringVar(value=self.BandRateList)
        self.BandRateVar.set('115200')
        self.BandRateL = Label(self.master, text='BaudRate')
        self.BandRateL.grid(row=6, column=0)
        self.BandRateO = OptionMenu(self.master, self.BandRateVar, *self.BandRateList)
        self.BandRateO.config(width=7)
        self.BandRateO.grid(row=6, column=1, columnspan=1, )
        Label(self.master, text='SerialPort').grid(row=6, column=2)
        if len(sl.comports()) == 0:
            self.SerialPortVar = StringVar(value=(''))
            self.SerialPortVar.set('')
            self.SerialPortList = ['']
        else:
            p = len(sl.comports())
            self.SerialPortList = ['']
            for i in range(p):
                self.SerialPortList.append(sl.comports()[i][0])
            self.SerialPortList.remove('')
            self.SerialPortVar = StringVar(value=self.SerialPortList)
            self.SerialPortVar.set(sl.comports()[0][0])
        self.SerialPort = self.SerialPortVar.get()
        self.SerialNum = OptionMenu(self.master, self.SerialPortVar, *self.SerialPortList)
        self.SerialNum.config(width=7)
        self.SerialNum.grid(row=6, column=3, columnspan=1)
        self.angleopen = 0
        self.OpenSerialButton = Button(self.master, bg='white', text='OpenSerialPort', command=self.OpenSerialCom)
        self.OpenSerialButton.grid(row=6, column=4, columnspan=1)


        self.startB = Button(self.master, text='Start', command=self.start)
        self.startB.grid(row=7, column=0, columnspan=2)
        self.closeB = Button(self.master, text='Close', command=self.close)
        self.closeB.grid(row=7, column=3, columnspan=2)

        self.SerialReadContent = []
        self.serialcom = 0



        self.VIO = CarState()
        self.VIO.x = 0
        self.askok = 0
        self.search=0
        self.stepnum=1
        self.sendok = 1
        self.direction=0



        self.BumLeft_state = 0
        self.BumLeft_state = 0
        self.BumRight_state = 0
        self.Seninf0 = 1
        self.Seninf1 = 1
        self.Seninf2 = 1
        self.Seninf3 = 1
        self.Seninf4 = 1
        self.BumLeft_state1 = 0
        self.BumRight_state1 = 0

        self.lock = threading.Event()
        self.lock.set()
        self.dataok=0
        self.ShowSwitch = 0
        rospy.init_node("listener", anonymous=True)

        self.cartask = CarTask()

        self.mapweight = 300
        self.maphight = 300

        self.robot = CarState()
        self.robot.x = 0
        self.robot.y = 0
        self.robot.angle = 0
        self.mapoffsetx = -100
        self.mapoffsety = -100

        self.lib = ctypes.CDLL("./data/darknet/libdarknet.so", ctypes.RTLD_GLOBAL)
        self.lib.YOLO_Init.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
        self.lib.yolo_init_test.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
        self.lib.yolo_init_test("./data/darknet/cfg/tiny-yolo-voc.cfg", "./data/darknet/tiny-yolo-voc.weights")
        self.lib.yolo_detect_test.argtypes = [ctypes.c_char_p, ctypes.POINTER(ObsDetInstance)]  # tiny-yolo-voc
        self.lib.yolo_detect_test.restype = ctypes.c_int
        self.obsdetinstance = ObsDetInstance()
        self.obstemp=ObsDetInstance()
        self.cameraCapture = cv2.VideoCapture(1)
        self.ROI_H = 1.0 / 2.0
        self.ROI_W = 3.0 / 4.0
        self.FrameSize = (
        int(self.cameraCapture.get(cv2.CAP_PROP_FRAME_WIDTH)), int(self.cameraCapture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        self.FrameKey = threading.Event()
        self.FrameKey.clear()
        self.frame = 0
        self.poses = []
        self.fixframe = rospy.get_param('~fixed_frame', 'map')





        #self.c=threading.Thread(target=self.detect,args=())
        #self.i=threading.Thread(target=self.img_read,args=())
        #self.s = threading.Thread(target=self.pointpublish_task, args=())
        #self.q = threading.Thread(target=self.maptask, args=())
        #self.c.start()
        #self.i.start()
        #self.q.start()
        #self.s.start()

        self.mapdata = np.zeros((self.mapweight, self.maphight))
        self.road = np.zeros((self.mapweight, self.maphight))


    def detect(self):
        i=0
        ii=0
        while self.lock.isSet():
            if self.FrameKey.isSet()==True:
                cv2.imwrite('123.jpg', self.frame)
                c = self.lib.yolo_detect_test("./123.jpg", ctypes.pointer(self.obsdetinstance))
                ii += 1
                if c != 0:
                    i += 1
                    print(self.obsdetinstance.estimate_distance, self.obsdetinstance.estimate_objwidth)
                    self.obstemp.Thing.obj_id = self.obsdetinstance.Thing.obj_id
                    self.obstemp.Thing.x=self.obsdetinstance.Thing.x
                    self.obstemp.Thing.y = self.obsdetinstance.Thing.y
                    self.obstemp.Thing.w = self.obsdetinstance.Thing.w
                    self.obstemp.Thing.h = self.obsdetinstance.Thing.h
                    print(self.obstemp.Thing.obj_id, (self.obstemp.Thing.x, self.obstemp.Thing.y),
                          (self.obstemp.Thing.w, self.obstemp.Thing.h))
                else:
                    self.obstemp.Thing.obj_id=0
                print("d:", float(i) / ii)

    def img_read(self):
        global frame
        while self.lock.isSet():
            self.FrameKey.clear()
            success, self.frame = self.cameraCapture.read()
            self.FrameKey.set()
            frame = cv2.rectangle(self.frame, (int(self.FrameSize[0] * (1.0 - self.ROI_W) / 2), int(self.FrameSize[1])),
                                  (int(self.FrameSize[0] * (1.0 + self.ROI_W) / 2), int(self.FrameSize[1] * (1.0 - self.ROI_H))), (0, 255, 0), 2)
            if self.obstemp.Thing.obj_id!=0:
                frame = cv2.rectangle(frame,
                                      (self.obstemp.Thing.x,self.obstemp.Thing.y),
                                      (self.obstemp.Thing.w,self.obstemp.Thing.h), (255, 0, 0), 2)

            cv2.imshow("123", frame)
            if cv2.waitKey(30) & 0xff == ord('q'):
                break
        self.cameraCapture.release()

    def maptask(self):

        self.cc = open('display1.txt')
        #self.path = rospy.Subscriber('/imu0', Imu, self.imucallback)
        self.pathpublisher = rospy.Publisher('nav_msgs/Path', Path, queue_size=1)
        self.sst = rospy.Publisher('map', OccupancyGrid, queue_size=1)
        self.tfmm = rospy.Publisher('tf', tfMessage, queue_size=1)
        self.sstm = rospy.Publisher('map_metadata', MapMetaData, queue_size=1)
        scale = rospy.get_param('~scale', 0.007)  # 0.07
        rospy.set_param('robot_description', self.cc.read())
        self.fixframe = rospy.get_param('~fixed_frame', 'map')
        self.ww = rospy.search_param('robot_description')
        rospy.set_param('~scale', 0.0001)

        lifetime = rospy.get_param('~lifetime', 0)  # 0 is forever
        ns = rospy.get_param('~ns', 'skeleton_markers')
        id = rospy.get_param('~id', 0)
        color = rospy.get_param('~color', {'r': 0.0, 'g': 1.0, 'b': 0.0, 'a': 1.0})




        i=0
        while self.lock.isSet():
            if True:
                i+=1
                if i>=10:
                    i=0
                    self.mappublish()
                    self.transform()
                    self.pathplublish()
                    self.ShowProcess()
                time.sleep(0.1)


    def pointpublish_task(self):
        #self.imustamped=rospy.Publisher('/imu1',Imu,queue_size=1)


        self.pointstamped = rospy.Publisher('/position', PointStamped, queue_size=1)
        self.pointi = 0
        while True:
            time.sleep(0.1)
            self.pointpublish()
    def imupublish(self):
        #self.imu0+=1
        self.imu=Imu()
        self.imu.orientation=0

    def pathplublish(self):

        self.paths = Path()
        header = Header(stamp=rospy.Time.now())
        header.seq = self.pointi
        header.frame_id = self.fixframe

        self.paths.header = header

        pose_temp = PoseStamped()
        header0 = Header(stamp=rospy.Time.now())
        header0.seq = self.pointi
        header0.frame_id = self.fixframe

        pose = Pose()

        # s_pose.header=header0
        # s_pose.point.x=self.pointi/20.0
        pose.position.x = self.robot.x / 20.0
        pose.position.y = self.robot.y / 20.0
        pose.position.z = 0
        pose.orientation.w = 1
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose_temp.pose = pose
        pose_temp.header = header0
        self.poses.append(pose_temp)
        self.paths.poses = self.poses
        self.paths.header = header0
        self.pathpublisher.publish(self.paths)
    def pointpublish(self):

        self.pointi+=1
        self.points=PointStamped()
        header=Header(stamp=rospy.Time.now())
        header.seq=self.pointi
        self.points.header=header
        point=Point()
        point.x=self.robot.x
        point.y=self.robot.y
        point.z=0
        self.points.point=point
        self.pointstamped.publish(self.points)




    def OpenSerialCom(self):
        self.SerialPort = self.SerialPortVar.get()
        self.BandRate = self.BandRateVar.get()
        try:
            self.SerialFunc = serial.Serial(self.SerialPort, (self.BandRate))  # int(self.BandRate)
        except BaseException as e:
            tkMessageBox.showerror('SerialError', message=e)
        else:
            self.SerialOpen = 1

            self.OpenSerialButton['bg'] = 'red'
            self.OpenSerialButton['text'] = 'CloseSerialPort'
            self.OpenSerialButton['activebackground'] = 'red'
            self.OpenSerialButton['command'] = self.CloseSerialCom

    def CloseSerialCom(self):
        self.SerialFunc.close()
        self.SerialOpen = 0
        self.OpenSerialButton['bg'] = 'white'
        self.OpenSerialButton['text'] = 'OpenSerialPort'
        self.OpenSerialButton['activebackground'] = 'white'
        self.OpenSerialButton['command'] = self.OpenSerialCom



    def SerialRead_Task(self):
        self.SerialReadContentTemp = ''
        num = 0
        imubuff = []
        search = 0
        ID = 0x00
        robotx=0
        roboty=0
        angle=0
        Seninf0 = 0
        Seninf1 = 0
        Seninf2 = 0
        Seninf3 = 0
        Seninf4 = 0
        click=0
        Acc = [0, 0, 0]
        Gyo = [0, 0, 0]
        Mag = [0, 0, 0]
        AtdE = [0, 0, 0]
        Press = [0, 0, 0]
        self.odo_left=0
        self.odo_right=0
        odo_left=0
        odo_right=0
        i=0
        while self.lock.isSet():
            if self.SerialFunc.in_waiting:
                #time.sleep(0.001)
                #i+=1
                #if i>=100:
                #    i=0
                #    self.pointpublish()
                self.SerialReadContentTemp = self.SerialFunc.read()
                # print('recive',self.SerialReadContentTemp)
                for i in range(len(self.SerialReadContentTemp)):
                    if True:
                        if num==0 and ord(self.SerialReadContentTemp[i])==0x01:
                            num+=1
                            continue
                        if num==0 and ord(self.SerialReadContentTemp[i])==0x11:
                            self.search=1
                            continue
                        if num==0 and ord(self.SerialReadContentTemp[i])==0x03:
                            self.sendok=1
                            continue
                        if num == 1 :
                            if ord(self.SerialReadContentTemp[i])==12:
                                num+=1

                            else:
                                num=0
                            continue
                        if num == 2:
                            robotx=ord(self.SerialReadContentTemp[i])*256
                            num+=1
                            continue
                        if num == 3:
                            robotx+=ord(self.SerialReadContentTemp[i])
                            num+=1
                            continue
                        if num == 4:
                            roboty=ord(self.SerialReadContentTemp[i])*256
                            num+=1
                            continue
                        if num == 5:
                            roboty+=ord(self.SerialReadContentTemp[i])
                            num+=1
                            continue
                        if num == 6:
                            angle=ord(self.SerialReadContentTemp[i])*256
                            num+=1
                            continue
                        if num == 7:
                            angle+=ord(self.SerialReadContentTemp[i])
                            num+=1
                            continue
                        if num == 8:
                            Seninf0=ord(self.SerialReadContentTemp[i])
                            num+=1
                            continue
                        if num == 9:
                            Seninf1=ord(self.SerialReadContentTemp[i])
                            num+=1
                            continue
                        if num == 10:
                            Seninf2=ord(self.SerialReadContentTemp[i])
                            num+=1
                            continue
                        if num == 11:
                            Seninf3=ord(self.SerialReadContentTemp[i])
                            num+=1
                            continue
                        if num == 12:
                            Seninf4=ord(self.SerialReadContentTemp[i])
                            num+=1
                            continue
                        if num == 13:
                            click=ord(self.SerialReadContentTemp[i])
                            num += 1
                            continue
                        if num==14:
                            odo_left=ord(self.SerialReadContentTemp[i])
                            num += 1
                            continue
                        if num==15:
                            odo_left += ord(self.SerialReadContentTemp[i])*256
                            num += 1
                            continue
                        if num == 16:
                            odo_left += ord(self.SerialReadContentTemp[i]) * 256*256
                            num += 1
                            continue
                        if num == 17:
                            odo_left += ord(self.SerialReadContentTemp[i]) * 256* 256*256
                            num += 1
                            continue
                        if num == 18:
                            odo_right = ord(self.SerialReadContentTemp[i])
                            num += 1
                            continue
                        if num == 19:
                            odo_right += ord(self.SerialReadContentTemp[i]) * 256
                            num += 1
                            continue
                        if num == 20:
                            odo_right += ord(self.SerialReadContentTemp[i]) * 256 * 256
                            num += 1
                            continue
                        if num == 21:
                            odo_right += ord(self.SerialReadContentTemp[i]) * 256 * 256 * 256
                            num =0
                            self.odo_left=odo_left
                            self.odo_right=odo_right
                            if(robotx&0x8000):
                                self.robot.x=-(robotx&0x7fff)*0.1
                            else:
                                self.robot.x=robotx*0.1
                            if (roboty & 0x8000):
                                self.robot.y = -(roboty & 0x7fff)*0.1
                            else:
                                self.robot.y = roboty*0.1
                            if (angle & 0x8000):
                                self.robot.angle = -(angle & 0x7fff)*0.1
                            else:
                                self.robot.angle = angle*0.1
                            self.Seninf0 = Seninf0
                            self.Seninf1 = Seninf1
                            self.Seninf2 = Seninf2
                            self.Seninf3 = Seninf3
                            self.Seninf4 = Seninf4
                            if (click&0xf0)!=0:
                                self.BumLeft_state=1
                            else:
                                self.BumLeft_state=0
                            if (click&0x0f)!=0:
                                self.BumRight_state=1
                            else:
                                self.BumRight_state=0
                            self.dataok=1
                            #print("site:    ",self.robot.x,self.robot.y,self.robot.angle)








    def ShowProcess(self):
        self.infra0['text'] = str(self.Seninf0)
        self.infra1['text'] = str(self.Seninf1)
        self.infra2['text'] = str(self.Seninf2)
        self.infra3['text'] = str(self.Seninf3)
        self.infra4['text'] = str(self.Seninf4)
        self.xL['text'] = str(self.robot.x)
        self.yL['text'] = str(self.robot.y)
        self.angleL['text'] = str(self.robot.angle)
        self.ODORIGHTL['text'] = str(self.odo_right)
        self.ODOLEFTL['text'] = str(self.odo_left)

        if self.BumLeft_state == 0:
            self.BumLeftLabel['bg'] = 'white'
        else:
            self.BumLeftLabel['bg'] = 'red'

        if self.BumRight_state == 0:
            self.BumRightLabel['bg'] = 'white'
        else:
            self.BumRightLabel['bg'] = 'red'



    def start(self):
        self.p = threading.Thread(target=self.SerialRead_Task, args=())
        self.q = threading.Thread(target=self.maptask, args=())
        self.r = threading.Thread(target=self.TaskInit, args=())
        self.s=threading.Thread(target=self.pointpublish_task, args=())

        #self.c=threading.Thread(target=self.detect,args=())
        #self.i=threading.Thread(target=self.img_read,args=())
        time.sleep(1)
        while self.SerialFunc.in_waiting:
            self.SerialReadContentTemp = self.SerialFunc.read_all()
            #self.SerialReadContentTemp = self.SerialFunc.read()
            #print(self.SerialReadContentTemp)
        print('start!!!!!!!')
        self.p.start()
        self.r.start()
        self.q.start()
        self.s.start()
        #self.i.start()
        #self.c.start()
    def close(self):
        self.lock.clear()
        while self.p.isAlive() or self.r.isAlive():
            self.lock.clear()
        master.quit()



    def sendmsg(self, msg):
        print(ord(msg))
        self.SerialFunc.write(msg)

    def transform(self):
        self.tfee = tf.TransformBroadcaster()
        self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                                tf.transformations.quaternion_from_euler(0, 0,
                                                                         math.pi * ((self.robot.angle / 180.0) + 0.5)),
                                rospy.Time.now(), 'my_frame', 'map')
        self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                                tf.transformations.quaternion_from_euler(0, 0,
                                                                         math.pi * ((self.robot.angle / 180.0) + 0.5)),
                                rospy.Time.now(),
                                'wheel_right_link', 'map')  # base_link
        '''self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'wheel_left_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'base_footprint', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'base_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'camera_depth_frame', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'camera_depth_optical_frame', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'camera_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'camera_rgb_frame', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'camera_rgb_optical_frame', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'caster_back_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'caster_front_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'cliff_sensor_front_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'cliff_sensor_left_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'cliff_sensor_right_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'gyro_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'plate_bottom_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'plate_middle_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'plate_top_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'pole_bottom_0_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'pole_bottom_1_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'pole_bottom_2_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'pole_bottom_3_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'pole_bottom_4_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'pole_bottom_5_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'pole_kinect_0_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'pole_kinect_1_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'pole_middle_0_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'pole_middle_1_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'pole_middle_2_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'pole_middle_3_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'pole_top_0_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'pole_top_1_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'pole_top_2_link', 'map')
self.tfee.sendTransform((self.robot.x / 20.0, self.robot.y / 20.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0,
                                                                 math.pi * ((self.robot.angle / 180.0) + 0.5)),
                        rospy.Time.now(),
                        'pole_top_3_link', 'map')'''

    def mappublish(self):

        self.map = OccupancyGrid(data=[])
        self.header = Header(stamp=rospy.Time.now())
        self.header.frame_id = self.fixframe
        self.or_pose = Pose()
        self.or_pose.position.x = -(self.mapweight/40.0)
        self.or_pose.position.y = -(self.maphight/40.0)
        self.or_pose.orientation.w = 1
        self.map_info = MapMetaData()
        self.map_info.height = self.maphight
        self.map_info.width = self.mapweight
        self.map_info.resolution = 0.05
        self.map_info.origin = self.or_pose
        self.map.header = self.header
        self.map.info = self.map_info
        self.map.data = []
        for i in range(self.maphight):
            for j in range(self.mapweight):

                if self.mapdata[j, i] >= SETLINE:
                    self.map.data.append(0)
                elif self.mapdata[j, i] >= 4:
                    self.map.data.append(50)
                elif self.mapdata[j, i] <= -SETLINE:
                    self.map.data.append(100)
                elif self.mapdata[j, i] < -4:
                    self.map.data.append(20)
                else:
                    self.map.data.append(-1)
                if self.mapdata[j, i] > 400:
                    self.mapdata[j, i] = 400
                if self.mapdata[j, i] < -400:
                    self.mapdata[j, i] = -400
        self.sstm.publish(self.map_info)
        self.sst.publish(self.map)
        # self.transform()


    def getleft(self):
        return self.Seninf0



    def getleftleft(self):
        if self.robot.angle == 0:
            return self.mapdata[self.robot.x - 5, self.robot.y]
        if self.robot.angle == 90:
            return self.mapdata[self.robot.x, self.robot.y + 5]
        if self.robot.angle == 180:
            return self.mapdata[self.robot.x + 5, self.robot.y]
        if self.robot.angle == -90:
            return self.mapdata[self.robot.x, self.robot.y - 5]

    def getfrontfront(self):
        if self.robot.angle == 0:
            return self.mapdata[self.robot.x, self.robot.y + 5]
        if self.robot.angle == 90:
            return self.mapdata[self.robot.x + 5, self.robot.y]
        if self.robot.angle == 180:
            return self.mapdata[self.robot.x, self.robot.y - 5]
        if self.robot.angle == -90:
            return self.mapdata[self.robot.x - 5, self.robot.y]

    def getrightright(self):
        if self.robot.angle == 0:
            return self.mapdata[self.robot.x + 5, self.robot.y]
        if self.robot.angle == 90:
            return self.mapdata[self.robot.x, self.robot.y - 5]
        if self.robot.angle == 180:
            return self.mapdata[self.robot.x - 5, self.robot.y]
        if self.robot.angle == -90:
            return self.mapdata[self.robot.x, self.robot.y + 5]

    def getright(self):
        return self.Seninf4

    def getmapfront(self):
        if self.robot.angle == 0:
            if self.robot.y + 2 > self.maphight:
                return -100
            return self.mapdata[self.robot.x, self.robot.y + 4]
        if self.robot.angle == 90:
            if self.robot.x + 2 > self.mapweight:
                return -100
            return self.mapdata[self.robot.x + 4, self.robot.y]
        if self.robot.angle == 180:
            if self.robot.y - 2 < 0:
                return -100
            return self.mapdata[self.robot.x, self.robot.y - 4]
        if self.robot.angle == -90:
            if self.robot.x - 2 < 0:
                return -100
            return self.mapdata[self.robot.x - 4, self.robot.y]

    def getmapfrontok(self):
        if self.robot.angle == 0:
            if self.robot.y + 2 > self.maphight:
                return -SETLINE
            if self.mapdata[self.robot.x, self.robot.y + 5] >= SETLINE and self.mapdata[
                        self.robot.x - 1, self.robot.y + 5] >= SETLINE and self.mapdata[
                        self.robot.x + 1, self.robot.y + 5] >= SETLINE \
                    and self.mapdata[self.robot.x - 2, self.robot.y + 5] >= SETLINE and self.mapdata[
                        self.robot.x + 2, self.robot.y + 5] >= SETLINE and self.mapdata[
                self.robot.x, self.robot.y + 4] >= SETLINE and self.mapdata[
                        self.robot.x - 1, self.robot.y + 4] >= SETLINE \
                    and self.mapdata[self.robot.x + 1, self.robot.y + 4] >= SETLINE and self.mapdata[
                        self.robot.x - 3, self.robot.y + 5] >= SETLINE and self.mapdata[
                        self.robot.x + 3, self.robot.y + 5] >= SETLINE and self.mapdata[
                        self.robot.x - 2, self.robot.y + 4] >= SETLINE \
                    and self.mapdata[self.robot.x + 2, self.robot.y + 4] >= SETLINE and self.mapdata[
                        self.robot.x - 3, self.robot.y + 4] >= SETLINE \
                    and self.mapdata[self.robot.x + 3, self.robot.y + 4] >= SETLINE and self.mapdata[
                        self.robot.x - 4, self.robot.y + 5] >= SETLINE and self.mapdata[
                        self.robot.x + 4, self.robot.y + 5] >= SETLINE:
                return SETLINE
            else:
                return -SETLINE
        if self.robot.angle == 90:
            if self.robot.x + 2 > self.mapweight:
                return -SETLINE
            if self.mapdata[self.robot.x + 5, self.robot.y] >= SETLINE and self.mapdata[
                        self.robot.x + 5, self.robot.y - 1] >= SETLINE and self.mapdata[
                        self.robot.x + 5, self.robot.y + 1] >= SETLINE:
                return SETLINE
            else:
                return -SETLINE
        if self.robot.angle == 180:
            if self.robot.y - 2 < 0:
                return -SETLINE
            if self.mapdata[self.robot.x, self.robot.y - 5] >= SETLINE and self.mapdata[
                        self.robot.x + 1, self.robot.y - 5] >= SETLINE and self.mapdata[
                        self.robot.x - 1, self.robot.y - 5] >= SETLINE \
                    and self.mapdata[self.robot.x + 2, self.robot.y - 5] >= SETLINE and self.mapdata[
                        self.robot.x - 2, self.robot.y - 5] >= SETLINE and self.mapdata[
                        self.robot.x + 3, self.robot.y - 5] >= SETLINE and self.mapdata[
                        self.robot.x - 3, self.robot.y - 5] >= SETLINE and self.mapdata[
                        self.robot.x + 4, self.robot.y - 5] >= SETLINE and self.mapdata[
                        self.robot.x - 4, self.robot.y - 5] >= SETLINE and self.mapdata[
                self.robot.x, self.robot.y - 4] >= SETLINE and self.mapdata[
                        self.robot.x + 1, self.robot.y - 4] >= SETLINE \
                    and self.mapdata[self.robot.x - 1, self.robot.y - 4] >= SETLINE and self.mapdata[
                        self.robot.x + 2, self.robot.y - 4] >= SETLINE \
                    and self.mapdata[self.robot.x - 2, self.robot.y - 4] >= SETLINE and self.mapdata[
                        self.robot.x + 3, self.robot.y - 4] >= SETLINE \
                    and self.mapdata[self.robot.x - 3, self.robot.y - 4] >= SETLINE:
                return SETLINE
            else:
                return -SETLINE
        if self.robot.angle == -90:
            if self.robot.x - 2 < 0:
                return -SETLINE
            if self.mapdata[self.robot.x - 5, self.robot.y] >= SETLINE and self.mapdata[
                        self.robot.x - 5, self.robot.y - 1] >= SETLINE and self.mapdata[
                        self.robot.x - 5, self.robot.y + 1] >= SETLINE:
                return SETLINE
            else:
                return -SETLINE

    def getmapleft(self):
        if self.robot.angle == 0:
            if self.robot.x - 2 < 0:
                return -SETLINE
            return self.mapdata[self.robot.x - 4, self.robot.y]
        if self.robot.angle == 90:
            if self.robot.y + 2 > self.maphight:
                return -SETLINE
            return self.mapdata[self.robot.x, self.robot.y + 4]
        if self.robot.angle == 180:
            if self.robot.x + 2 > self.mapweight:
                return -SETLINE
            return self.mapdata[self.robot.x + 4, self.robot.y]
        if self.robot.angle == -90:
            if self.robot.y - 2 < 0:
                return -SETLINE
            return self.mapdata[self.robot.x, self.robot.y - 4]

    def getmapright(self):
        if self.robot.angle == 0:
            if self.robot.x + 2 > self.mapweight:
                return -SETLINE
            return self.mapdata[self.robot.x + 4, self.robot.y]
        if self.robot.angle == 90:
            if self.robot.y - 2 < 0:
                return -SETLINE
            return self.mapdata[self.robot.x, self.robot.y - 4]
        if self.robot.angle == 180:
            if self.robot.x - 2 < 0:
                return -SETLINE
            return self.mapdata[self.robot.x - 4, self.robot.y]
        if self.robot.angle == -90:
            if self.robot.y + 2 > self.maphight:
                return -SETLINE
            return self.mapdata[self.robot.x, self.robot.y + 4]



    def getfront(self):
        return self.Seninf2

    def TaskInit(self):
        self.TaskStart()

    def TaskStart(self):
        self.CarCore()

    def CarCore(self):
        hor_left=np.matrix([-4,2])
        hor_right = np.matrix([4, 2])
        hor_leftmid=np.matrix([-1,4])
        hor_rightmid = np.matrix([1, 4])
        hor_middle = np.matrix([0, 4])

        bum_left=np.matrix([[-3,2],[-2,2],[-2,3]])
        bum_right=np.matrix([[3,2],[2,2],[2,3]])
        cc=0
        while True:
            if self.dataok==1:
                temp=0

                for i in range(-10, 11):
                    for j in range(-10, 11):
                        if self.mapdata[int(round(self.robot.x)) + i, int(round(self.robot.y)) + j] ==0:
                            temp += 1
                if temp == 0:
                    cc += 1
                else:
                    cc = 0
                if cc > 3:
                    self.sendok=0
                    self.sendmsg('S')
                    while self.sendok==0:
                        time.sleep(0.001)
                T = np.matrix([[math.cos(self.robot.angle*math.pi/180.0),-math.sin(self.robot.angle*math.pi/180.0)],
                                   [math.sin(self.robot.angle*math.pi/180.0),math.cos(self.robot.angle*math.pi/180.0)]])
                for i in range(-3,4):
                    for j in range(-3,4):
                        if abs(i)==3 and abs(j)==3:
                            continue
                        if abs(i)<2 and abs(j)<2:
                            self.mapdata[int(round(self.robot.x+i+self.mapweight/2)),int(round(self.robot.y +j+self.maphight/2))]+=6
                        elif abs(i)<3 and abs(j)<3:
                            self.mapdata[int(round(self.robot.x + i + self.mapweight/2)), int(round(self.robot.y + j + self.maphight/2))] += 4
                        else:
                            self.mapdata[int(round(self.robot.x + i + self.mapweight/2)), int(round(self.robot.y + j + self.maphight/2))] += 2
                self.road[int(round(self.robot.x + self.mapweight/2)), int(round(self.robot.y + self.maphight/2))]=self.stepnum
                self.stepnum+=1
                if self.Seninf0>12:
                    A=np.dot(hor_left,T)
                    self.mapdata[int(round(self.robot.x + round(A[0,0]) + self.mapweight/2)), int(round(self.robot.y + round(A[0,1])  + self.maphight/2))] -= 250
                if self.Seninf1>15:
                    A=np.dot(hor_leftmid,T)
                    self.mapdata[int(round(self.robot.x + round(A[0,0]) + self.mapweight/2)), int(round(self.robot.y + round(A[0,1]) + self.maphight/2))] -= 250
                if self.Seninf2>40:
                    A=np.dot(hor_middle,T)
                    self.mapdata[int(round(self.robot.x + round(A[0,0]) + self.mapweight/2)), int(round(self.robot.y + round(A[0,1]) + self.maphight/2))] -= 250
                if self.Seninf3>15:
                    A=np.dot(hor_rightmid,T)
                    self.mapdata[int(round(self.robot.x + round(A[0,0]) + self.mapweight/2)), int(round(self.robot.y + round(A[0,1]) + self.maphight/2))] -= 250
                if self.Seninf4>12:
                    A=np.dot(hor_right,T)
                    self.mapdata[int(round(self.robot.x + round(A[0,0]) + self.mapweight/2)), int(round(self.robot.y + round(A[0,1]) + self.maphight/2))] -= 250
                if self.BumRight_state:
                    A=np.dot(bum_right,T)
                    #self.mapdata[int(self.robot.x + A[0,0] + self.mapweight/2), int(self.robot.y + A[0,1] + self.maphight/2)] -= 300
                    self.mapdata[int(round(self.robot.x + round(A[1,0] + self.mapweight/2))),int( round(self.robot.y + round(A[1,1] + self.maphight/2)))] -= 300
                    #self.mapdata[int(self.robot.x + A[2,0] + self.mapweight/2), int(self.robot.y + A[2,1] + self.maphight/2)] -= 300
                if self.BumLeft_state:
                    A=np.dot(bum_left,T)
                    #self.mapdata[int(self.robot.x + A[0,0] + self.mapweight/2), int(self.robot.y + A[0,1] + self.maphight/2)] -= 300
                    self.mapdata[int(round(self.robot.x + round(A[1,0] + self.mapweight/2))), int(round(self.robot.y + round(A[1,1] + self.maphight/2)))] -= 300
                    #self.mapdata[int(self.robot.x + A[2,0] + self.mapweight/2), int(self.robot.y + A[2,1] + self.maphight/2)] -= 300
                self.dataok=0
            if self.search==1:
                print("kaishi zhaolu!!!")
                points=self.findroad()
                self.search = 0
                print("points   ",points)
                for (x,y) in points:
                    print('go : ',x-self.mapweight/2, y-self.maphight/2)
                    self.SendPoint(x-self.mapweight/2, y-self.maphight/2)
                if len(points)>0:
                    self.sendok=0
                    self.sendmsg('F')
                    time.sleep(0.1)
                    self.sendmsg(chr(self.direction))
                    if self.sendok==0:
                        time.sleep(0.001)


    def SendSite(self,x,y):
        while self.sendok==0:
            time.sleep(0.001)
        self.sendok=0
        self.sendmsg('T')
        time.sleep(0.1)
        i = int(x)
        if i < 0:
            i = abs(i)
            p = (int(i & 0xff00) >> 8) | 0x80
        else:
            p = int(i & 0xff00) >> 8
        self.sendmsg(chr(p))
        time.sleep(0.1)
        p = int(i & 0x00ff)
        self.sendmsg(chr(p))
        time.sleep(0.1)
        i = int(y)
        if i < 0:
            i = abs(i)
            p = (int(i & 0xff00) >> 8) | 0x80
        else:
            p = int(i & 0xff00) >> 8
        self.sendmsg(chr(p))
        time.sleep(0.1)
        p = int(i & 0x00ff)
        self.sendmsg(chr(p))
        while self.sendok == 0:
            time.sleep(0.1)

    def SendPoint(self,x,y):
        self.sendok=0
        self.sendmsg('G')
        time.sleep(0.1)
        i = int(x)
        if i<0:
            i=abs(i)
            p = (int(i & 0xff00) >> 8)|0x80
        else:
            p = int(i & 0xff00) >> 8
        self.sendmsg(chr(p))
        time.sleep(0.1)
        p = int(i & 0x00ff)
        self.sendmsg(chr(p))
        time.sleep(0.1)
        i = int(y)
        if i<0:
            i=abs(i)
            p = (int(i & 0xff00) >> 8)|0x80
        else:
            p = int(i & 0xff00) >> 8
        self.sendmsg(chr(p))
        time.sleep(0.1)
        p = int(i & 0x00ff)
        self.sendmsg(chr(p))
        while self.sendok==0:
            time.sleep(0.1)
        print("ok!!!!")

    def findline(self):
        cartask=CarTask()
        cartask.w=self.mapweight
        cartask.h = self.maphight
        cartask.orix=0
        cartask.oriy=0
        print('findline')
        lines = []
        unknowplace = []
        maptemp = np.zeros((cartask.w, cartask.h), np.uint8)
        for i in range(2, cartask.w - 2):
            for j in range(2, cartask.h - 2):
                '''if self.mapdata[cartask.orix + i, cartask.oriy + j] == 0:
                    oktemp = 0
                    errortemp = 0
                    for k in range(3):
                        for g in range(3):
                            if self.mapdata[cartask.orix + i - 1 + k, cartask.oriy + j - 1 + g] <= -SETLINE:
                                errortemp = 1
                            if self.mapdata[cartask.orix + i - 1 + k, cartask.oriy + j - 1 + g] >= SETLINE:
                                oktemp += 1
                    if oktemp > 1 and errortemp == 0:
                        maptemp[i, j] = 255'''
                if self.mapdata[cartask.orix + i, cartask.oriy + j] >= SETLINE:
                    oktemp = 0
                    errortemp = 0
                    for k in range(3):
                        for g in range(3):
                            if self.mapdata[cartask.orix + i - 1 + k, cartask.oriy + j - 1 + g] <= -SETLINE:
                                errortemp = 1
                            if self.mapdata[cartask.orix + i - 1 + k, cartask.oriy + j - 1 + g] ==0:
                                oktemp += 1
                    if oktemp > 2 and errortemp == 0:
                        maptemp[i, j] = 255



        line = cv2.HoughLinesP(maptemp, 2, np.pi / 4, 5)
        line0 = []
        point = CarState()
        distance = cartask.h * cartask.h + cartask.w * cartask.w
        xmin = 10000
        ymin = 10000
        cv2.imwrite('line.jpg', maptemp)
        cv2.imwrite('mapdata.jpg', self.mapdata)
        line = np.array((line))
        print('len of line', len(np.shape(line)))
        print(line)
        if True:  # line!=None
            if len(np.shape(line)) > 0:
                # cartask0=CarTask()
                for [[y1, x1, y2, x2]] in line:
                    if abs(x1 - x2) < 3 and abs(y1 - y2) < 3:
                        continue
                    if abs(x1 - x2) + abs(y1 - y2) < 6:#4
                        continue
                    unknowplace.append((x1, y1, x2, y2))
        return unknowplace


    def findroad(self):
        points=[]
        lines = self.findline()
        (xp1, yp1, xp2, yp2) = (0, 0, 0, 0)
        distance = self.maphight * self.mapweight
        if len(lines) > 0:
            print('ok!!!!!!!!!!!!!')
            for (x1, y1, x2, y2) in lines:
                if (abs((x1 + x2) / 2 - self.robot.x) + abs((y1 + y2) / 2 - self.robot.y) < distance):
                    distance = abs((x1 + x2) / 2 - self.robot.x) + abs((y1 + y2) / 2 - self.robot.y)
                    (xp1, yp1, xp2, yp2) = (x1, y1, x2, y2)
        else:
            print("error!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            self.search = 0
            return []
        if (xp1+xp2)/2>self.robot.x:
            self.direction=2
        else:
            self.direction=1
        gh = np.zeros((self.mapweight+ 2, self.maphight + 2, 3))
        closed = np.ones((self.mapweight+ 2, self.maphight + 2))
        road = np.zeros((self.mapweight+ 2, self.maphight + 2))
        opened = np.zeros((self.mapweight+ 2, self.maphight + 2))
        maproad = np.zeros((self.mapweight+ 2, self.maphight + 2))
        x = int(round(self.robot.x + self.mapweight / 2))
        y = int(round(self.robot.y + self.maphight / 2))
        nowx = x
        nowy = y
        aimx = int(round((xp1 + xp2) / 2))
        aimy = int(round((yp1 + yp2) / 2))
        nowx1 = nowx
        nowy1 = nowy

        for i in range(1, self.mapweight- 1):
            for j in range(1, self.maphight - 1):
                if self.road[i,j]>0:
                    closed[i, j] = 0
                if self.mapdata[i,j]>SETLINE:
                    closed[i, j] = 0

        #for i in range(-3,4):
        #    for j in range(-3,4):
        #        closed[aimx+i, aimy+j] = 0
        print('aim', aimx, aimy)
        print('now', nowx, nowy)
        cv2.imwrite('closed' + '.bmp', closed * 255)
        while abs(aimx - nowx) + abs(aimy - nowy) > 2.0:
            print('now', nowx, nowy, '   closed', closed[nowx, nowy], '   mapdata', self.mapdata[nowx, nowy])
            if closed[nowx, nowy] == 1:
                print("we cannot get the right road")
                while True:
                    i=1

            closed[nowx, nowy] = 1
            distance = (self.mapweight+ self.maphight) * 2 + 1
            distance0 = (self.mapweight+ self.maphight) * 2 + 1
            x = 0
            y = 0
            niux = 0
            niuy = 0
            kaka = 0
            for i in range(-1, 2):
                for j in range(-1, 2):
                    if nowx + i < 0 or nowx + i > self.mapweight or nowy + j < 0 or nowy + j > self.maphight:
                        continue
                    if i == 0 or j == 0:
                        if closed[nowx + i, nowy + j] == 1:
                            continue
                        gh0 = 0.8 + gh[nowx, nowy, 0]
                        gh1 = abs(aimx - nowx - i) + abs(aimy - nowy - j)
                        gh2 = gh0 + gh1
                        if opened[nowx + i, nowy + j] == 0:
                            opened[nowx + i, nowy + j] = 1
                            gh[nowx + i, nowy + j, 0] = gh0
                            gh[nowx + i, nowy + j, 1] = gh1
                            gh[nowx + i, nowy + j, 2] = gh2
                            road[nowx + i, nowy + j] = i * 10 + j
                        else:
                            if gh2 < gh[nowx + i, nowy + j, 2]:
                                gh[nowx + i, nowy + j, 0] = gh0
                                gh[nowx + i, nowy + j, 1] = gh1
                                gh[nowx + i, nowy + j, 2] = gh2
                                road[nowx + i, nowy + j] = i * 10 + j
                            else:
                                continue

                        if gh[nowx + i, nowy + j, 2] < distance:
                            niux = nowx + i
                            niuy = nowy + j
                            distance = gh[nowx + i, nowy + j, 2]
                        else:
                            if gh[nowx + i, nowy + j, 2] < distance0:
                                x = nowx + i
                                y = nowy + j
                                distance0 = gh[nowx + i, nowy + j, 2]
            (x, y, distance0) = self.getminsite(gh, opened, closed, self.mapweight, self.maphight)
            if distance0 < distance:
                kaka = 0
                nowx = x
                nowy = y
            else:
                kaka = 1
                nowx = niux
                nowy = niuy

        while nowx != nowx1 or nowy != nowy1:


            if int(road[nowx, nowy]) == -10:
                nowx += 1
                maproad[nowx, nowy] = -10
            if int(road[nowx, nowy]) == 10:
                nowx -= 1
                maproad[nowx, nowy] = 10
            if int(road[nowx, nowy]) == 1:
                nowy -= 1
                maproad[nowx, nowy] = 1
            if int(road[nowx, nowy]) == -1:
                nowy += 1
                maproad[nowx, nowy] = -1
            if road[nowx, nowy] == 0:
                print('error!')
        if True:
            nowx = nowx1
            nowy = nowy1
            direction=maproad[nowx, nowy]
            while maproad[nowx, nowy] != 0:
                # print('self.maproad',maproad[nowx,nowy])
                print('move:', nowx, nowy)
                if maproad[nowx,nowy]!=direction:
                    direction = maproad[nowx, nowy]
                    points.append((nowx,nowy))
                if maproad[nowx, nowy] == -10:
                    nowx -= 1
                if maproad[nowx, nowy] == 10:
                    nowx += 1
                if maproad[nowx, nowy] == -1:
                    nowy -= 1
                if maproad[nowx, nowy] == 1:
                    nowy += 1

            points.append((nowx, nowy))
        return points

    def getminsite(self, gh, opened, closed, ww, hh):
        distance = (ww + hh) * 2
        x = 0
        y = 0
        for i in range(ww):
            for j in range(hh):
                if opened[i, j] == 1 and closed[i, j] == 0:
                    if gh[i, j, 2] < distance:
                        x = i
                        y = j
                        distance = gh[i, j, 2]
        return (x, y, distance)




if __name__ == "__main__":
    master = Tk()
    cleaner = MapRobot(master)

    mainloop()
