import rospy 
from open_manipulator_msgs.msg import Coordinate 

import pyrealsense2 as rs
import numpy as np
import cv2
import math

def run_histogram_equalization(bgr_img):
    # convert from RGB color-space to YCrCb
    ycrcb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2YCrCb)

    # equalize the histogram of the Y channel
    ycrcb_img[:, :, 0] = cv2.equalizeHist(ycrcb_img[:, :, 0])

    # convert back to RGB color-space from YCrCb
    equalized_img = cv2.cvtColor(ycrcb_img, cv2.COLOR_YCrCb2BGR)
    return equalized_img

def main():
    # init ROS node 
    rospy.init_node('send_node',anonymous=False)
    pub = rospy.Publisher('xyz_topic',Coordinate,queue_size=10)
    # rate = rospy.Rate(1)
    msg = Coordinate()
    count = 1
    
    weight = 'src/darknet/yolov4-custom_best.weights'
    cfg = 'src/darknet/yolov4-custom.cfg'
    names = 'src/darknet/data/coco.names'
    
    # realsense and darknet init
    net = cv2.dnn.readNet(weight, cfg)

    classes = []

    with open(names, "r") as f:
        classes = [line.strip() for line in f.readlines()]

    layer_names = net.getLayerNames()

    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

    colors = np.array([[255,0,255]],dtype=np.float64)



    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    align_to = rs.stream.depth
    align = rs.align(align_to)
    
    try:
        while not rospy.is_shutdown():
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
    
            # *************************original video******************************* 
            
            depth_frame_origin = frames.get_depth_frame()
            color_frame_origin = frames.get_color_frame()
            if not depth_frame_origin or not color_frame_origin:
                continue

            # Convert images to numpy arrays 
            depth_image_orign = np.asanyarray(depth_frame_origin.get_data())
            color_image_orign = np.asanyarray(color_frame_origin.get_data())
            depth_colormap_orign = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_orign, alpha=0.03), cv2.COLORMAP_JET)
            
            #************************************************************************
            
            #Aligning color frame to depth frame
            aligned_frames =  align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            aligned_color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not aligned_color_frame: continue

            color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(aligned_color_frame.get_data())
            
            
            #histogram equalization
            color_image_equalized = run_histogram_equalization(color_image_orign)
            
            
            #apply darknet
            height, width, channels = color_image_equalized.shape
            
            # (81.158, 112.813, 99.577) ->  bgr mean value of training set picture. 
            # using mean substraction
            blob = cv2.dnn.blobFromImage(color_image_equalized, 0.001, (222, 222), (81.158, 112.813, 99.577),True)  
            net.setInput(blob)
            outs = net.forward(output_layers)
            
            class_ids = []
            confidences = []
            target_x = []
            target_y = []
            boxes = []
            
            for out in outs:
                    for detection in out:
                        
                        scores = detection[5:]
                        class_id = np.argmax(scores)
                        confidence = scores[class_id]
                        if confidence > 0.80:
                            # Object detected
                            center_x = int(detection[0] * width)
                            center_y = int(detection[1] * height)
                            w = int(detection[2] * width)
                            h = int(detection[3] * height)
                            # coordinate
                            x = int(center_x - w / 2)
                            y = int(center_y - h / 2)
                            boxes.append([x, y, w, h])
                            confidences.append(float(confidence))
                            class_ids.append(class_id)
                            target_x.append(center_x)
                            target_y.append(center_y)
                            
            indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
            font = cv2.FONT_HERSHEY_SIMPLEX
            
            temp_msg = [100, 100, 100, 100]
            
            for i in range(len(boxes)):
                if i in indexes:
                    x, y, w, h = boxes[i]
                    label = str(classes[class_ids[i]])
                    color = colors[0]
                    
                    depth = depth_frame.get_distance((target_x[i] * 610) // 848 + 105, (target_y[i] * 345)  // 480 + 60)
                    dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x,y], depth)                    
                    distance = round(math.sqrt(((dx)**2) + ((dy)**2) + ((dz)**2)),4)
                    # print(f'dx : {dx} dy : {dy} dz : {dz}')
                    # print(f'distance : {distance}')
                    
                    if (distance != 0) and (distance < 0.70) :
                        if temp_msg[0] > distance:
                            if dy > 0 : 
                                dy -= 0.002
                            else:
                                dy += 0.003
                            if dx > 0 :
                                dx -= 0.005
                            else:
                                dx += 0.007
                            if dz > 0 :
                                dz += 0.002   
                            
                            temp_msg = [distance, dx, dy, dz]
                            
                        cv2.rectangle(color_image_orign, (x, y), (x + w, y + h), color, 2)
                        cv2.putText(color_image_orign, label, (x, y - 30), font, 0.5, color, 1)  
                        cv2.putText(color_image_orign, str(distance), (x, y - 60), font, 0.5, (0,255,0), 1)                
                    
            if temp_msg[0] != 100:
                msg.start_time = rospy.Time.now()
                msg.msg_seq = count
                msg.x = temp_msg[1]
                msg.y = temp_msg[2]
                msg.z = temp_msg[3]
            
                rospy.loginfo("------")
                rospy.loginfo("Start Time(sec): %d", msg.start_time.secs)
                rospy.loginfo("Message Sequence: %d", msg.msg_seq)
                rospy.loginfo("X: %f", msg.x)
                rospy.loginfo("Y: %f", msg.y)
                rospy.loginfo("Z: %f", msg.z)
                
                pub.publish(msg)
                # rate.sleep()
                count +=1
                    
            images_orign = np.hstack((color_image_orign,depth_colormap_orign))

            cv2.namedWindow('Orign', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Orign', images_orign)
            cv2.waitKey(1)

    finally:

        # Stop streaming
        pipeline.stop()
    
   
       
        
if __name__== '__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass