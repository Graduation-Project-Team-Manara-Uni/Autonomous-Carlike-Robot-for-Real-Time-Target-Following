#import libraries
import math
import time
from multiprocessing.managers import BaseManager
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit
import board
import busio
import time
import numpy as np
#define data with first row the desired parameters to save
data2save = np.empty((0,15))
data2save = np.append(data2save,[["time","Vx_desired","Vy_desired","psi_desierd","Ax_desired","volt_desierd","x","z","e1","e2","Vx","Vy","psi","omega","Ax"]],axis=0)
#define the pca component
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 1000
#assign zero to the steering angle (equal to 85 for servo command after curve fitting) and assign zero to the DC Motor
kit = ServoKit(channels=16)
kit.servo[3].angle=85
time.sleep(1)
pca.channels[1].duty_cycle = 0x0000  
pca.channels[2].duty_cycle = 0x0000  
time.sleep(2)
#convert to pwm
def pwm(vdc):
    return int(vdc*0xffff/12.0)

#robot parameters
Dr=0.082;Df=0.05;D=Dr+Df
Ds=0.205;w=0.15;
m=1.2
J=0.01
b=Dr
#control parameters
lamda1=5;lamda2=-0.7
ksw=0.1;n=10;Ld=1.5;k=1
#Dc parameters
R=8.2;kb=0.175;ka=kb
Iw=0.0933;rw=0.03
Gr=3

#sharing data from other codes
class MyManager(BaseManager):
    pass
    
class MotorsManager(BaseManager):
    pass
    

shared_motors = {'servo':85,'dc':0}
MyManager.register('get_shared_dict')
MotorsManager.register('get_shared_motors',lambda: shared_motors)
manager= MyManager(address=('',50000),authkey=b'jetson')
manager1= MyManager(address=('',50001),authkey=b'jetson')

print("Client connecting...")
manager.connect()
manager1.connect()
print("Client connected!")

shared_dict = manager.get_shared_dict()
shared_dict1 = manager1.get_shared_dict()

data = shared_dict._getvalue()
data1 = shared_dict1._getvalue()
phidot = data['omega'] # angular velocity of the robot
Aax = data['ax'] # Linear acceleration of the center of the robot on local x coordinate
vax = data['vx'] # Linear velocity of the center of the robot on local x coordinate
vax_0 = 0
psipre = data['psi'] # steering angle of the equivalent front wheel
Xpre = data1['x'] # the distance between camera and target in local x coordinate of the camera
Zpre = data1['z'] # the distance between camera and target in local y coordinate of the camera
if Zpre < 1e-2 : 
    Zpre=Ld
    Xpre=0.0
L=math.sqrt(Zpre**2+Xpre**2) # the distance between camera and targe in the coordinate of the camera

last_time = time.time()
vfxpre=k*(L-Ld)*Zpre/L # velocity desired of the robot on x local coordinate
max_speed=5 # max speed of the robot
volt=0.0
start_time = time.time()
vay = data['vy'] # Linear velocity of the center of the robot on local y coordinate

while True:
    try:
        data = shared_dict._getvalue()
        data1 = shared_dict1._getvalue()
        X = data1['x']
        Z = data1['z']
        current_time=time.time()
        dt = current_time - last_time
        last_time= current_time
        if Z < 1e-2 : 
            continue
        L=math.sqrt(Z**2+X**2)
        
        vfx=k*(L-Ld)*Z/L
        vfy=-k*(L-Ld)*X/L # velocity desired of the robot on y local coordinate
        if vfx>max_speed:
            vfy=(max_speed/vfx)*vfy
            vfx=max_speed
        if vfx<-max_speed:
            vfy=-(max_speed/vfx)*vfy
            vfx=-max_speed   
        Afx=(vfx-vfxpre)/dt # acceleration desired of the robot on x local coordinate 
        vfxpre=vfx
        Aax= data['ax'] # real acceleration of the robot on x local coordinate 
        
        phidot = data['omega']
        vay = data['vy'] # real velocity  of the robot on y local coordinate
        aa=55.85;bb=2.065;cc=85;
        psid=0.0
        if abs(Ds*vfx) < 1e-3:
            psid = 0.0
        else:
            psid=math.atan(D*vfy/(Ds*vfx)) # steering angle desired
        
        if psid == 0:
            vax=data['vx']-vax_0
        else:
            vax=(D*data["vy"])/(Ds*math.tan(psid))
        
        servo_angle = int(aa*math.tan(-bb*psid) + cc) # servo command after curve fitting
        servo_angle=max(45, min(servo_angle, 125)) # saturation for servo command
        psi = data['psi'] # real steering angle
        psidot=(psi-psipre)/dt 
        psipre=psi
        # Sliding Mode Controller
        a=(math.cos(psi)**2)*(m*(D**2)+(m*(b**2)+J)*(math.tan(psi)**2))
        f=(1/a)*(m*(b**2)+J)*math.tan(psi)*psidot*vax
        g=(1/a)*(D**2)*(math.cos(psi)**2)
        e1=vfx-(vax)
        e2=Afx-Aax
        s=lamda1*e1+e2
        sat=max(-1, min(s, 1))
        Fd=(1/g)*(lamda1*e1+Afx-f+ksw*sat+n*s)
        Vdc=(R*rw*Fd)/(Gr*ka)+kb*Gr*vax/rw
        Vsat=max(-5, min(Vdc, 5))
        Vsat=round(Vsat,1)
        if abs(servo_angle-85) < 4:
            servo_angle=85
        
        if abs(vfx) < 0.1:
            Vsat =0.0
            
        
            
        if Vsat>0:            
            kit.servo[3].angle = servo_angle
            time.sleep(0.05)
            pca.channels[1].duty_cycle = 0x0000  
            pca.channels[2].duty_cycle = pwm(Vsat) 
        else:
            kit.servo[3].angle = servo_angle - 2*(servo_angle - 85)
            time.sleep(0.05)
            pca.channels[1].duty_cycle = pwm(-Vsat) 
            pca.channels[2].duty_cycle = 0x0000   
        
        
        if Vsat == 0:
            time.sleep(0.3)
            data = shared_dict._getvalue()
            vax_0=data['vx']

        #save data to csv file
        data2save=np.append(data2save,[[current_time - start_time,vfx,vfy,psid,Afx,Vsat,X,Z,e1,e2,vax-vax_0,vay,psi,phidot,Aax]],axis=0)
   
        
        
    except (ConnectionResetError, BrokenPipeError):
        print("Connection to server lost. Exiting.")
        break
    finally:
        np.savetxt("data.csv",data2save,delimiter=',',fmt='%s')
   
