import pickle
import cv2
import numpy as np
import os
from PIL import Image
import math

sensorLoc_x = 86
sensorLoc_y = -120

def angle_between_points(p1, p2):
    """Calculate the angle between the horizontal line passing through p1 and the line connecting p1 and p2."""
    x1, y1 = p1
    x2, y2 = p2
    
    # Calculate the length of the line connecting p1 and p2
    length = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    
    # Calculate the angle in radians using arctangent function
    angle_radians = math.atan2(y2 - y1, x2 - x1)
    
    # Convert the angle to degrees
    angle_degrees = math.degrees(angle_radians)
    
    # Return the angle
    return angle_degrees

def make_images():
    for ts in data:
        img = np.zeros((400, 400, 3), dtype=np.uint8)
        for obj in ts.objects:
            x = obj.centerX - sensorLoc_x
            y = obj.centerY - sensorLoc_y
            add_circle(img, x + 200, y + 200, obj.length, obj.width, (255, 255, 255))
        images.append(img)

def add_past_img(past_img, img, idx, main_object):
    if idx == 0:
        return img
    past_main_obj = obj_poses[main_object.id][0][idx-1]
    old_shift_x = main_object.centerX - past_main_obj.centerX
    old_shift_y = main_object.centerY - past_main_obj.centerY
    shift_x = old_shift_x*math.cos(past_main_obj.rotation) - old_shift_y*math.sin(past_main_obj.rotation)
    shift_y = old_shift_x*math.sin(past_main_obj.rotation) + old_shift_y*math.cos(past_main_obj.rotation)
    
    diff_angle = main_object.rotation - past_main_obj.rotation
    # print("rotations", int(math.degrees(past_main_obj.rotation)), int(shift_x), int(shift_y), int( main_object.centerX - past_main_obj.centerX), int( main_object.centerY - past_main_obj.centerY), (math.cos(-past_main_obj.rotation)), (math.sin(-past_main_obj.rotation)))
    # shift then rotate
    # Define the transformation matrix
    M = np.float32([[1, 0, -shift_x], [0, 1, shift_y]])
     # Get the image shape
    (h, w) = past_img.shape[:2]
    # Apply the transformation
    past_img = cv2.warpAffine(past_img, M, (w, h))

    # Compute the rotation matrix
    M = cv2.getRotationMatrix2D((w//2, h//2), -math.degrees(diff_angle), 1.0)

    # Apply the rotation to the image
    past_img = cv2.warpAffine(past_img, M, (w, h))
    img = cv2.addWeighted(img, 1, past_img, 0.8, 0)
    return img


def add_circle(img, centerX, centerY, length, width, color):
    # Determine the radius of the circle
    radius = max(int(max(length, width) / 2), 1)
    cv2.circle(img, (int(centerX), int(centerY)), radius, color, thickness=-1)
    return img

def frame_to_image_wrt_object(data, main_object, count): # egocentric view wrt to object
    # main_object = data.objects[idx]
    main_obj_x = main_object.centerX
    main_obj_y = main_object.centerY
    main_angle = main_object.rotation
    # print("id", main_object.id, main_angle)

    if main_object.classType == "10":
        # pedestrian, blue
        obj_class = "pedestrian/"
    elif main_object.classType == "2":
        # car, red
        obj_class = "car/"
    elif main_object.classType == "3":
        # van, purple
        obj_class = "van/"
    elif main_object.classType == "4":
        # truck, orange
        obj_class = "truck/"
    elif main_object.classType == "5":
        # bus, yellow
        obj_class = "bus/"
    elif main_object.classType == "13":
        # bicycle, green
        obj_class = "bicycle/"
    else:
        # unknown, black
        obj_class = "unkown/"
    if not os.path.exists("test/"+obj_class+main_object.id):
        os.makedirs("test/" + obj_class + main_object.id)

    img = images[count]

    shift_x = sensorLoc_x - main_obj_x
    shift_y = sensorLoc_y - main_obj_y
    # Get the image shape
    (h, w) = img.shape[:2]
    M = np.float32([[1, 0, shift_x], [0, 1, shift_y]])
    # Apply the transformation
    img = cv2.warpAffine(img, M, (w, h))
    

    # Compute the rotation matrix
    M = cv2.getRotationMatrix2D((w//2, h//2), -math.degrees(main_angle), 1.0)

    # if(main_object.id == ("7567505")):
    #     print(main_angle, math.degrees(main_angle))

    # Apply the rotation to the image
    img = cv2.warpAffine(img, M, (w, h))
    
    num, past_img = past_imgs.setdefault(main_object.id, (0, img))

    img2 = add_past_img(past_img, img, num, main_object)
    past_imgs[main_object.id] = (num+1, img2)

    # add future pose
    # print("len poses", len(obj_poses[main_object.id][count:]))
    future_pose_img = np.zeros((400, 400, 3), dtype=np.uint8)
    for pose in obj_poses[main_object.id][0][count-obj_poses[main_object.id][1]:]:
        x = pose.centerX - main_obj_x
        y = pose.centerY - main_obj_y
        future_pose_img = add_circle(future_pose_img, x + 200, y + 200, pose.length, pose.width, (0, 255, 0))
    future_pose_img = cv2.warpAffine(future_pose_img, M, (w, h))
    img = cv2.addWeighted(img2, 1, future_pose_img, 1, 0)
    cv2.imwrite("test/{}/{}/{:03d}.png".format(obj_class, main_object.id, count), img)

def process_pickle():
    cnt = 0
    for ts in data:
        for obj in ts.objects:
            if obj_poses.get(obj.id) is None:
                obj_poses[obj.id] = ([],cnt)
            obj_poses[obj.id][0].append(obj)
        cnt +=1

def make_gif():
    count = 0
    for ts in data:

        # idx = 0
        for obj in ts.objects:
            if len(obj_poses[obj.id][0]) < 10:
                continue
            frame_to_image_wrt_object(ts, obj, count)
            # idx+=1
        count+=1

    path = "test"
    c = 0
    for obj_class in os.listdir(path):
        # if c > 1:
        #     break
        # if os.path.isdir(os.path.join(path, name)):
        for obj_id in os.listdir(os.path.join(path, obj_class)):
            # Do something with the directory
            # Set directory path and image format
            image_folder = "test/" + obj_class + "/" + obj_id
            image_format = 'png'
            print(image_folder)
            
            # Set duration for each frame in milliseconds
            frame_duration = 200

            # Create list of image file paths
            image_paths = sorted([os.path.join(image_folder, fn) for fn in os.listdir(image_folder) if fn.endswith('.png')])
            # Open each image and add to frames list
            frames = []
            for image_path in image_paths:
                with Image.open(image_path) as im:
                    frames.append(im.copy())
            # print(len(frames))

            # Save frames to", image_folder+"/animation.gif")
            frames[0].save(f"{image_folder}/animation.gif", format='GIF', append_images=frames[1:], save_all=True, duration=frame_duration, loop=0)
        c+=1

obj_poses = {}
past_images = {}
past_imgs = {}
images = []

# Open the pickle file in read binary mode
with open('array.pickle', 'rb') as f:
    # Load the contents of the file into a Python object
    data = pickle.load(f)

process_pickle()
# make_images()
# make_gif()