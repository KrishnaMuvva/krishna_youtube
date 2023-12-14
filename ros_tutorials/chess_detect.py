# Remove ROS path since it creates conflicts with opencv
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'

if ros_path in sys.path:
    sys.path.remove(ros_path)


# Import required libraries and packages
import os
import cv2
import numpy as np
import tensorflow as tf
import sys
import time
import serial
from utils import label_map_util
from utils import visualization_utils as vis_util
sys.path.append("..")


# Current directory path
Dir_Path = os.getcwd()

# Convolutional Neural Network directory
Chess_Model_Dir = 'chess_graph'


# CNN Path
Path_CNN = os.path.join(Dir_Path,Chess_Model_Dir,'chess_graph.pb')

# Total Classes
Classes = 6


# Labels path
Path_Labels = os.path.join(Dir_Path,'training','labelmap.pbtxt')



label_map = label_map_util.load_labelmap(Path_Labels)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=Classes, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# Load the SSD Network
detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

    sess = tf.Session(graph=detection_graph)



# Input tensor
image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

# Output variables
detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
num_detections = detection_graph.get_tensor_by_name('num_detections:0')

# Open video
video = cv2.VideoCapture(1)

# Set widht and height
ret = video.set(3,1280)
ret = video.set(4,720)

# Variables to write pose
start = time.time()
end = time.time()
write_stat = True
mul = 10

# Open serial monitor
arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)

# Select chess piece
piece = int(input ("Select one piece among\n 1. King \n 2. Queen \n 3. Rook \n 4. Bishop \n 5. Knight \n 6. Pawn \n :"))


# While the camera opened
while(True):

    # Read frame
    ret, frame = video.read()

    # Convert it into RGB
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Shape it according to tensor
    frame_expanded = np.expand_dims(frame_rgb, axis=0)

    # Obtain output
    (boxes, scores, classes, num) = sess.run(
        [detection_boxes, detection_scores, detection_classes, num_detections],
        feed_dict={image_tensor: frame_expanded})


    # Size of image
    size = classes.shape[1]

    # New output varibles according to selected piece
    new_boxes = np.zeros((1,size,4))
    new_scores = np.zeros((1,size))
    new_classes = np.zeros((1,size))

    # Intial image pose
    w1, h1 = 0,0

    b1sum = 0
    count = 0


    # Identify the selected piece
    for i in range(size):
        if int(classes[0,i]) == piece:

            # Write output accordingly
            [m,n,o] = frame.shape
            new_classes[0,0] = piece
            new_scores[0,0] = scores[0,i]
            new_boxes[0,0,0] = boxes[0,i,0]
            new_boxes[0,0,1] = boxes[0,i,1]
            new_boxes[0,0,2] = boxes[0,i,2]
            new_boxes[0,0,3] = boxes[0,i,3]

            # Get image pose

            w1 = (1*(new_boxes[0,0,3]+new_boxes[0,0,1]))/2
            h1 = (1*(new_boxes[0,0,2]+new_boxes[0,0,0]))/2

            w1,h1 = int(w1*n),int(h1*m)

            # Get breadth information
            b1 = (new_boxes[0,0,3]-new_boxes[0,0,1])*n
            b2 = (new_boxes[0,0,2]-new_boxes[0,0,0])*m

            b1sum += b1
            count += 1

            b1avg = b1sum / count

            #print("Values ", b1avg, b1, b2)
            #print("Values ", w1, h1, n, m)

            break
    
    a = b1*b2

    # 63, 175

    # Calculate alpha, beta, and gama
    alpha = 1/15
    beta = 1/65
    gama = 1/180

    # Camera matrix

    mat = [[alpha , 0 ,0], [0, beta, 0], [0, 0, gama]]
    mat = np.array(mat)

    #Image pose
    img_pose = [b1, w1, h1]
    img_pose = np.array(img_pose)

    img_center = [160, 640, 480]
    img_center = np.array(img_center)

    world_offset = [18, 0, 2]
    #world_offset = [0, 0, 0]
    world_offset = np.array(world_offset)

    # Image to world pose

    world_pose = np.matmul(mat, (img_center - img_pose).T)+world_offset

    #print("World pose", world_pose, img_pose)

    end = time.time()
    if (end - start) > 5:
        # write_stat = True
        pass

    Write into arduino serial port
    if write_stat:

        x = str(int(world_pose[0]*mul))
        y = str(int((world_pose[1]+7)*mul))
        z = str(int(world_pose[2]*mul))

        xyz = x+'X'+y+'Y'+z+'Z'

        print("XYZ -----------------------------> ", xyz)

        arduino.write(bytes(xyz, 'utf-8'))
        #arduino.write(bytes('@', 'utf-8'))
        write_stat = False
        start = time.time()

    # Write into image frame
    frame = cv2.circle(frame, (w1,h1), 2, (255,0,0), 2)

    vis_util.visualize_boxes_and_labels_on_image_array(
    frame,
    np.squeeze(new_boxes),
    np.squeeze(new_classes).astype(np.int32),
    np.squeeze(new_scores),
    category_index,
    use_normalized_coordinates=True,
    line_thickness=8,
    min_score_thresh=0.20)

    # Show image results

    cv2.imshow('Object detector', frame)

    if cv2.waitKey(1) == ord('q'):
        break

video.release()
cv2.destroyAllWindows()

