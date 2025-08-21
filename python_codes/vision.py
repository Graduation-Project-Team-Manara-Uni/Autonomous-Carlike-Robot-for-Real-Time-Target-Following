# importing libraries
import cv2
import jetson_inference
import jetson_utils
import pyrealsense2 as rs
import numpy as np
from jetson_utils import loadImage, cudaMemcpy
import time 
from multiprocessing.managers import BaseManager
import statistics as st

#time.sleep(30)
# sharing parameters from other codes
class MyManager(BaseManager):
    pass

shared_data = {'x':0.0,'z':1.0}
MyManager.register('get_shared_dict',lambda: shared_data)
manager= MyManager(address=('',50001),authkey=b'jetson')
# function for calculating position of the target in 3D world coordinates
def calculate_position(depth, pixel_x, pixel_y, fx, fy, cx, cy):
    
    # Convert pixel coordinates and depth to 3D world coordinates
    X = (pixel_x - cx) * depth / fx
    Y = (pixel_y - cy) * depth / fy
    Z = depth  # Depth is the Z-coordinate in mm

    return X, Y, Z

with manager:

    print("Server starting...")
    
    server_dict = manager.get_shared_dict()
    
    while True:
        pixel_x, pixel_y = 640, 480  # Pixel coordinates
        fx, fy = 386.964, 386.964  # Focal lengths in pixels
        cx, cy = 326.022, 236.071  # Principal point (image center)

        # Initialize color dictionary
        colors_hash = {}
        target_class = 'person'  # Change this to your desired class ID
        depth_interval = 0.1  # 500ms between depth measurements
        
        fourcc = cv2.VideoWriter_fourcc(*'H264')
        output_file = 'output.mp4'
        fps = 30.0
        frame_size = (640, 480)
        
        out = cv2.VideoWriter(output_file, fourcc, fps, frame_size)

        # Initialize RealSense pipeline
        pipeline = rs.pipeline()
        config = rs.config()
        # Configure color stream at 640x480 @ 30fps
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        #Create align object to align depth to color
        align_to = rs.stream.color
        align = rs.align(align_to)


        # Start streaming
        profile=pipeline.start(config)

        # Get depth sensor and depth scale
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        last_depth_time = time.time()

        # Load SSD MobileNet v2 model with detection threshold 0.8
        net = jetson_inference.detectNet("ssd-mobilenet-v2", threshold=0.7)

        try:
            pre_pos_z = 0.01; pre_pos_x=0;
            pos_z = 0.01; pos_x = 0;
            start_time = 0.0 
            while True:
                # Wait for a coherent pair of frames: color
                frames = pipeline.wait_for_frames()
                
                aligned_frames = align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue
                
                # Convert RealSense frame to numpy array
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                cv2.imwrite('output_image.jpg', color_image)
                img_a = loadImage("output_image.jpg")
                img_cuda = cudaMemcpy(img_a)
                
                
                # Exit on ESC key
                if cv2.waitKey(1) & 0xFF == 27:
                    break
                
                
                
                
                # Check if it's time to measure depth
                current_time = time.time()
                measure_depth = (current_time - last_depth_time) >= depth_interval
                
                if not measure_depth:
                    out.write(color_image)
                    # Display the image with detections
                    #cv2.imshow("SSD MobileNet Detection - RealSense D435i", color_image)
                    continue
                
                detections = net.Detect(img_cuda)
                per_flag = False
                # Draw bounding boxes and labels on the numpy image
                for detection in detections:
                    x1, y1, x2, y2 = int(detection.Left), int(detection.Top), int(detection.Right), int(detection.Bottom)
                    class_desc = net.GetClassDesc(detection.ClassID)
                    
                    if class_desc == target_class:
                        
                    	#Get class-specific color
                    	#print(class_desc)
                    	#class_id = int(classes[idx])
                    	per_flag = True
                    	start_time = time.time()
                    	if class_desc not in colors_hash:
                    	    colors_hash[class_desc] = tuple(np.random.randint(0, 256, 3).tolist())
                    	color = colors_hash[class_desc]
                    
                    	cv2.rectangle(color_image, (x1, y1), (x2, y2), color, 2)
                    	# Get center of bounding box
                    	center_x = (x1 + x2) // 2
                    	center_y = (y1 + y2) // 2
                    	# Draw center point
                    	cv2.circle(color_image, (center_x, center_y), 3, (0, 255, 0), -1)
                    	if measure_depth:
                    		# Get depth at center point (in meters)
                    		depth_value = depth_frame.get_distance(center_x, center_y)
                    		#depth_box = depth_image[y1:y2, x1:x2]
                    		
                    		# Add depth text near the center
                    		depth_text = f"{depth_value:.2f}m"
                    		cv2.putText(color_image, depth_text, (center_x + 10, center_y), 
                                	cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    		position = calculate_position(depth_value, center_x, center_y, fx, fy, cx, cy)
                    		position_text=f"3D Position (mm): X={position[0]:.2f}, Y={position[1]:.2f}, Z={position[2]:.2f}"
                    		cv2.putText(color_image,position_text,(140,200),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
                    		pos_z = position[2];pos_x = position[0];
                    		last_depth_time = current_time
                    		break

                upd = {'x':pos_x,'z':pos_z}
                server_dict.update(upd)
                
                out.write(color_image)
               
                
                

        finally:
            # Stop streaming and close windows
            pipeline.stop()
            out.release()
            cv2.destroyAllWindows()



