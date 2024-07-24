import numpy as np
from shapely.geometry import Point , Polygon
import cv2


'''
    Helper functions Starts
'''
def point_within(point,polygon):
    return polygon.contains(point)

def draw_poly(image,polygon_coords):
    '''
    Args:
        Image
        List of points

    Returns:
        Image

    '''

    # Convert the polygon coordinates to a NumPy array
    polygon_pts = np.array(polygon_coords, np.int32)
    polygon_pts = polygon_pts.reshape((-1, 1, 2))

    # Draw the polygon on the image
    cv2.polylines(image, [polygon_pts], isClosed=True, color=(0, 255, 0), thickness=3)

    return image

def get_depth (x, y,depth_image):
    '''
    Get the depth value at the specified pixel coordinates.
    '''
    err, depth_value = depth_image.get_value(x, y)
    if np.isfinite(depth_value):
        return depth_value
    else:
        return None
    
def get_dimension (bbox ,depth):
    box = list(map(int,((bbox.xyxy).tolist()[0])))
    x1, y1, x2, y2 = box

    width = abs(x2 - x1)
    width = depth * (width / 1066)

    height = abs(y2 - y1)
    height = depth * (height / 1066)

    return height , width

def get_pos(bbox):
    box = list(map(int,((bbox.xyxy).tolist()[0])))
    x_min, y_min, x_max, y_max = box

    return [(x_min+x_max)//2 , (y_min+ y_max) // 2]

def within_poly(bbox,polygon):

    box = list(map(int,((bbox.xyxy).tolist()[0])))
    #print(box)

    x_min, y_min, x_max, y_max = box
    corners = [
            (x_min, y_min),  # top-left
            (x_max, y_min),  # top-right
            (x_max, y_max),  # bottom-right
            (x_min, y_max)   # bottom-left
        ]
        # Check if all corners are inside the polygon

    return any(polygon.contains(Point(corner)) for corner in corners)
'''
    Helper functions Ends
'''


### Main Implementations
'''
    Main Functions
'''

def get_closest_obj(result,depth_image,obstacle_region,is_picked):

    decision_list = []

    # Obstacle Object
    for obj in result[0].boxes:
        x,y = get_pos(obj)
        depth = get_depth(x,y,depth_image)

        if obj.cls == 0 and within_poly(obj , obstacle_region):
            if depth :
                H , W = get_dimension(obj, depth)
                if (H >= 15 and H <= 25) or (W >= 15 and W <= 25): # 150 x 150
                    decision_list.append([obj,depth//10])
                    # pass
                else: # 300 x 300
                    decision_list.append([obj,depth//10]) 
            else :
                decision_list.append([obj,999])

        if obj.cls == 1 and within_poly(obj, obstacle_region):
            if depth :
                H , W = get_dimension(obj, depth)
                if (H >= 15 and H <= 25) or (W >= 15 and W <= 25): # 200 x 200
                    decision_list.append([obj,depth//10])
                    # pass
                else: # 400 x 400
                    decision_list.append([obj,depth//10]) 

            else:
                decision_list.append([obj,999])

        if obj.cls == 2 and not is_picked:
            if depth:
                decision_list.append([obj,depth//10])
            else:
                decision_list.append([obj,999])
        
        if obj.cls == 3 and is_picked:
            if depth:
                decision_list.append([obj,depth//10])
            else:
                decision_list.append([obj,999])
            
    decision_list.sort(key = lambda x : x[1])

    if len(decision_list) == 0:
        return None
    
    return decision_list[0]

def get_align_cmd(x,y):
    if x>=0 and x <= 1030:
        return 'l'
    elif x>=1130 and x <=1920:
        return 'r'
