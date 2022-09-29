# Import the required libraries
import cv2
import numpy as np
import rospy  
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist

# Distance constants
KNOWN_DISTANCE = 45  # INCHES
PERSON_WIDTH = 16  # INCHES


# Object detector constant
CONFIDENCE_THRESHOLD = 0.4
NMS_THRESHOLD = 0.3

# declare data type to be publish
global twist
twist = Twist()
Direction =0


# colors for object detected
COLORS = [
    (255, 0, 0),
    (255, 0, 255),
    (0, 255, 255),
    (255, 255, 0),
    (0, 255, 0),
    (255, 0, 0),
]
GREEN = (0, 255, 0)
BLACK = (0, 0, 0)
YELLOW =(0,255,255)
PERPEL = (255,0,255)
# defining fonts
FONTS = cv2.FONT_HERSHEY_COMPLEX

# getting class names from classes.txt file
class_names = []
with open("classes.txt", "r") as f:
    class_names = [cname.strip() for cname in f.readlines()]
#  setttng up opencv net
yoloNet = cv2.dnn.readNet("yolov4-tiny.weights", "yolov4-tiny.cfg")
# setting up CUDA backend
yoloNet.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
yoloNet.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)

# apply YOLO network in the model
model = cv2.dnn_DetectionModel(yoloNet)
model.setInputParams(size=(416, 416), scale=1 / 255, swapRB=True)

# object detector funciton /method
def object_detector(image):
    classes, scores, boxes = model.detect(image, CONFIDENCE_THRESHOLD, NMS_THRESHOLD)
    # creating empty list to add objects data
    data_list = []
    for (classid, score, box) in zip(classes, scores, boxes):
        # define color of each, object based on its class id
        color = COLORS[int(classid) % len(COLORS)]

        label = "%s : %f" % (class_names[classid[0]], score)

        # draw rectangle on and label on object
        cv2.rectangle(image, box, color, 2)
        cv2.putText(image, label, (box[0], box[1] - 14), FONTS, 0.5, color, 2)

        # getting the data
        # 1: class name  2: object width in pixels, 3: position where have to draw text(distance)
        if classid == 0:  # person class id
            # adding values the list
            # > classname, > width of object, > top-right corner > bounding box(rectangle)
            data_list.append(
                [class_names[classid[0]], box[2], (box[0], box[1] - 2), box]
            )
        elif classid == 67:
            # > classname, > width of object, > top-right corner > bounding box(rectangle)
            data_list.append(
                [class_names[classid[0]], box[2], (box[0], box[1] - 2), box],
            )
        # if you want includes more classes then you have to simply add more [elif] statements here
        # returning list containing the object data.
    return data_list

# function declaration for finding focal lenght
def focal_length_finder(measured_distance, real_width, width_in_rf):
    focal_length = (width_in_rf * measured_distance) / real_width

    return focal_length


# distance finder function
def distance_finder(focal_length, real_object_width, width_in_frmae):
    distance = (real_object_width * focal_length) / width_in_frmae
    distance = distance*0.0254
    return distance

# function for publishing the twist messages based on the robot's condition
def condition():
    
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.loginfo(twist)
    pub.publish(twist)


# reading the reference image from dir
ref_person = cv2.imread("ReferenceImages/image14.png")

# finding width of the person in the reference image
person_data = object_detector(ref_person)
person_width_in_rf = person_data[0][1]

print(
    f"Person width in pixels : {person_width_in_rf}"
)

# finding focal length
focal_person = focal_length_finder(KNOWN_DISTANCE, PERSON_WIDTH, person_width_in_rf)

#setting up the output frame and capture the image recorded by the camera 
cap = cv2.VideoCapture(0)
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height= cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
cap.set(3,int(width/3))
cap.set(4,int(height/3))
Distance_level =0
while True:
    ret, frame = cap.read()
    frame_height, frame_width, ret = frame.shape
    RightBound = frame_width-200
    Left_Bound = 200
    # setting up the initial condition for the twist messages
    twist.linear.x = 0
    twist.linear.y = 0
    twist.angular.z = 0
    
    # supply the video streamed by the camera to the object detector function
    data = object_detector(frame)
    for d in data:
        if d[0] == "person":
            distance = distance_finder(focal_person, PERSON_WIDTH, d[1])
            x, y = d[2]
            # Getting the box coordinates
            box = data[0][3]
            # extracting the, x, y, width and height from bounding box
            x, y, w, h = box
            body_width = w
            body_center_x=int(x + (w / 2))
            body_center_y=int(y + (h / 2))
            
            centroid_value = body_center_x, body_center_y
            distance = round(distance,2)
            # Drwaing Text on the screen
            Distance_level= int(distance)
            cv2.line(frame, (50,33), (130, 33), (BLACK), 15)
            cv2.putText(frame, f"Robot State", (50,35), FONTS,0.4, (YELLOW),1)

            # Direction Decider Condition
            if body_center_x<Left_Bound:
                print("Left Movement")
                # Direction of movement
                twist.linear.x = 0      
                twist.angular.z = 4        #LEFT z=+1



                
                cv2.line(frame, (50,65), (170, 65), (BLACK), 15)
                cv2.putText(frame, f"Move Left {body_center_x}", (50,70), FONTS,0.4, (YELLOW),1)

            elif body_center_x>RightBound:
                print("Right Movement")
                # Direction of movement
                
                twist.linear.x = 0      
                twist.angular.z = -4        #RIGHT z=-1
               
                
                cv2.line(frame, (50,65), (170, 65), (BLACK), 15)
                cv2.putText(frame, f"Move Right {body_center_x}", (50,70), FONTS,0.4, (GREEN),1)
            
                # cv2.line(frame, (50,65), (170, 65), (BLACK), 15)
                # cv2.putText(frame, f"Truing = False", (50,70), FONTS,0.4, (WHITE),1)

            elif distance >1.5 and distance<=5: #convert to m later 70=2 m  200=8m
                # Direction of movement
                twist.linear.x = 2
                twist.angular.z = 0


                
                cv2.line(frame, (50,55), (200, 55), (BLACK), 15)
                cv2.putText(frame, f"Forward Movement", (50,58), FONTS,0.4, (PERPEL),1)
                print("Move Forward")

            elif distance <=1:
                # Direction of movement
                twist.linear.x = -2
                twist.angular.z = 0
                
                print("Move Backward")
                cv2.line(frame, (50,55), (200, 55), (BLACK), 15)
                cv2.putText(frame, f"Backward Movement", (50,58), FONTS,0.4, (PERPEL),1)
            else:
                # Direction of movement
                twist.linear.x = 0
                twist.angular.z = 0
                cv2.line(frame, (50,55), (200, 55), (BLACK), 15)
                cv2.putText(frame, f"No Movement", (50,58), FONTS,0.4, (PERPEL),1)



        # print(centroid_value)
        cv2.circle(frame, centroid_value, 3, (0, 255, 0), -1, cv2.LINE_AA)
        cv2.circle(frame, centroid_value, 6, (255, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(
            frame,
            f"Dis: {round(distance,2)} m",
            (x + 5, y + 13),
            FONTS,
            0.48,
            GREEN,
            2,
        )
    cv2.line(frame, (Left_Bound, 80), (Left_Bound, 480-80), (YELLOW), 2)
    cv2.line(frame, (RightBound, 80),(RightBound, 480-80), (YELLOW), 2)


    
    condition()
    cv2.imshow("frame", frame)
    key = cv2.waitKey(1)
    # Quit and close the output frame 
    if key == ord("q"):
        break
    

cv2.destroyAllWindows()
cap.release()
