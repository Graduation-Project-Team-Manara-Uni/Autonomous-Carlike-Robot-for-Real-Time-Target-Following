# importing libraries
import pyrealsense2 as rs
import time
import math
import numpy as np
import collections
from multiprocessing.managers import BaseManager

# sharing parameters with other code
class MyManager(BaseManager):
    pass
    
shared_data = {'ax':0.0,'omega':0.0,'vx':0.0,'psi':0.0,'vy':0.0}
MyManager.register('get_shared_dict',lambda: shared_data)
manager= MyManager(address=('',50000),authkey=b'jetson')

with manager:

    print("Server starting...")
    
    server_dict = manager.get_shared_dict()
    
    while True:
        
        omega=0.0
        # average filter
        class OnlineSimpleMovingAverage:
            def __init__(self,window_size):
                if window_size <= 0:
                    raise ValueError("window size must be positive")
                self.window_size=window_size
                self.data_buffer=collections.deque(maxlen=window_size)
                self.current_sum=0.0
            
            def apply (self,new_value):
                if len(self.data_buffer) == self.window_size:
                    self.current_sum -= self.data_buffer[0]
                
                self.data_buffer.append(new_value)
                self.current_sum += new_value
                    
                if len(self.data_buffer) < self.window_size:
                    return self.current_sum/len(self.data_buffer)
                    
                return  self.current_sum/self.window_size         
                    
                    
                    
        
        
        
        # Configure pipeline for IMU streams
        pipeline = rs.pipeline()
        config = rs.config()
        data = np.empty((0,7))
        data = np.append(data,[["time","Ax","Vx","Omega","psi","accel_data","Vy"]],axis=0)

        def format_number(number,decimals):
            rounded=round(number,decimals)
            return f"{rounded:.{decimals}f}"

        # Enable accelerometer and gyroscope streams
        config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f,100)  # Accelerometer at 250 Hz
        config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)    # Gyroscope at 200 Hz
        c=30
        Ds = 0.205; D  = 0.135
        # filters parameters
        alpha1=0.0001
        alpha2=0.0001
        alpha3=0.0001
        tau1=0.5
        tau2=0.8
        tau3=0.0
        window_size1=300
        window_size2=300
        window_size3=10
        window_size4=50
        lowpass1=OnlineSimpleMovingAverage(window_size1)
        lowpass2=OnlineSimpleMovingAverage(window_size2)
        lowpass3=OnlineSimpleMovingAverage(window_size3)
        lowpass4=OnlineSimpleMovingAverage(window_size4)
        try:
            # Start streaming
            pipeline.start(config)
            
            print("IMU streaming started...\nPress Ctrl+C to stop")
            Ax_cnt = 0;Ax_avg = 0;
            om_cnt = 0;om_avg = 0;
            # calculate the average of the first 5000 value of AX and omega
            num = 5000;
            while num > 0:
                num-=1;frames = pipeline.wait_for_frames()

                for frame in frames:
                    if frame.get_profile().stream_type() == rs.stream.accel:
                        accel_data = frame.as_motion_frame().get_motion_data()
                        Ax_pr = accel_data.z
                        Ax_avg += accel_data.z;Ax_cnt += 1;
        

                    elif frame.get_profile().stream_type() == rs.stream.gyro:
                        gyro_data = frame.as_motion_frame().get_motion_data()
                        om_avg += -gyro_data.y;om_cnt += 1
                        


            Ax_avg /= Ax_cnt;
            om_avg /= om_cnt;
            Ax_pr -= Ax_avg;
            omega=0;Ax=0;
            
            
            Vax=0
            #time.sleep(2)
            start_time=time.time()
            lat=time.time()
            while True:

                frames = pipeline.wait_for_frames()
                at=time.time()
                for frame in frames:
                    
                    if frame.get_profile().stream_type() == rs.stream.accel:
                        accel_data = frame.as_motion_frame().get_motion_data()
                        
                        accel_data_lowpass1=lowpass1.apply(accel_data.z-Ax_avg)
                        accel_data_lowpass=round(accel_data_lowpass1,1)
                        

                    elif frame.get_profile().stream_type() == rs.stream.gyro:
                        gyro_data = frame.as_motion_frame().get_motion_data()
                        
                        omega1= -gyro_data.y 
                        omega=lowpass2.apply(omega1-om_avg)
                        Vay = omega*D
                Ax= (accel_data_lowpass) + Ds*(omega**2);
                Ax=int(Ax*(10**1))/(10**1)
                dt=at-lat;

                lat=at;
                Vax+=(Ax+Ax_pr)*dt/2;
                Vax_lowpass=lowpass3.apply(Vax)
                Ax_pr=Ax;
                
                if abs(Vax_lowpass)<1e-6:
                    if Vax_lowpass>=0:
                        Vax_lowpass=1e-6
                    else:	
                        Vax_lowpass=-1e-6
                
                psi=math.atan(Vay/Vax_lowpass)
                if abs(psi)>1.4:
                    psi=0
                psi=lowpass4.apply(psi)
                if c > 0:
                    c-=1
                    continue
                data=np.append(data,[[time.time()-start_time,Ax,Vax_lowpass,omega,psi,accel_data_lowpass,Vay]],axis=0)
                upd = {'ax':Ax,'omega':omega,'vx':Vax_lowpass,'psi':psi,'vy':Vay}
                server_dict.update(upd)
                c=30
                
                
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            #Cleanup
            pipeline.stop()
            
