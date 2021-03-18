import json

import cv2
import matplotlib.cm
import numpy as np
import pandas as pd
from PIL import Image

from get_depth_street_edge import get_paths
from get_depth_street_edge import undistort_image


def main(camera_file_path, sign_dic_r, sign_dic_l, config, data_r, data_l):

    # load lidar config and image info
    with open(config["paths"]["cams_lidar_json_path"], 'r') as f:
        config_data = json.load(f)

    with open(camera_file_path.replace('.png', '.json'), 'r') as f:
        image_info = json.load(f)

    camera_path = camera_file_path

    camera_image = cv2.imread(camera_path)
    camera_image = cv2.cvtColor(camera_image, cv2.COLOR_BGR2RGB)

    camera_image_undist = undistort_image(camera_image,image_info, config_data)
    camera_image_with_street = print_street_points(sign_dic_r, sign_dic_l, camera_image_undist)

    max_distance = config['max_distance']
    semantic_image_path, lidar_data_path = get_paths(camera_path)
    # load lidar data and convert lidar to panda df for better handling
    lidar_data = np.load(lidar_data_path)
    key_list = list(lidar_data.keys())
    key_list.remove('points')
    array_filled = np.array([np.asarray(lidar_data[i]) for i in key_list])
    lidar_df = pd.DataFrame(np.transpose(array_filled), columns=key_list)
    point_list = ["p0", "p1", "p2"]  # can not pass array to DataFrame
    for i, p in enumerate(point_list):
        lidar_df[p] = np.asarray(lidar_data["points"])[:, i]

    camera_image_with_lidar = map_lidar_points_onto_image(camera_image_with_street, sign_dic_r, sign_dic_l, lidar_df, max_distance)
    camera_image_with_lidar = map_wrong_lidar_points_onto_image(camera_image_with_lidar, sign_dic_r, sign_dic_l, lidar_df)

    camera_image_with_street = Image.fromarray(camera_image_with_lidar)

    sign_img = Image.open(config["paths"]["sign_path"])
    grey_sign = Image.open(config["paths"]["free_space_sign_path"]).convert("RGBA")
    blue_sign = Image.open(config["paths"]["small_angle_sign_path"]).convert("RGBA")
    camera_image_with_signs = print_sign(camera_image_with_street, sign_img,grey_sign, blue_sign, sign_dic_r)
    camera_image_with_signs = print_sign(camera_image_with_signs, sign_img,grey_sign, blue_sign,  sign_dic_l)

    counter_sign, counter_free_space, counter_minus_two, counter_out_of_image, total_signs = get_counter(sign_dic_r, sign_dic_l)


    return camera_image_with_signs, counter_sign, counter_free_space, counter_minus_two,counter_out_of_image, total_signs

def print_sign(camera_image_undist, sign_img,grey_sign, blue_sign, sign_dic):
    '''
    gets the camera image and sign image and print the signs on the camera image with the information stored in
    the sign dictionaries
    '''
    for x, y, w, h, flag in zip(sign_dic['x'], sign_dic['y'], sign_dic['w'], sign_dic['h'], sign_dic['flag']):
        if flag == -1:
            sign_img_resize = sign_img.resize((w, h))
            camera_image_undist.paste(sign_img_resize, (x, y), mask=sign_img_resize)
        elif flag == -2:
            sign_img_resize = blue_sign.resize((w, h))
            camera_image_undist.paste(sign_img_resize, (x, y), mask=sign_img_resize)
        elif flag == -3:
            pass
        else:
            sign_img_resize = grey_sign.resize((w, h))
            camera_image_undist.paste(sign_img_resize, (x, y), mask=sign_img_resize)
    return camera_image_undist

def print_street_points(sign_dic_r, sign_dic_l, camera_image):

    x_r_coordinates = sign_dic_r['x_street']
    y_r_coordinates = sign_dic_r['y_street']
    flag_r = sign_dic_r['flag']
    x_l_coordinates = sign_dic_l['x_street']
    y_l_coordinates = sign_dic_l['y_street']
    flag_l = sign_dic_l['flag']

    points_r = np.column_stack((x_r_coordinates, y_r_coordinates))
    points_l = np.column_stack((x_l_coordinates, y_l_coordinates))

    x_sign_r = sign_dic_r['x']
    y_sign_r = sign_dic_r['y']
    sign_tuple_r = np.column_stack((x_sign_r, y_sign_r))
    x_sign_l = sign_dic_l['x']
    y_sign_l = sign_dic_l['y']
    sign_tuple_l = np.column_stack((x_sign_l, y_sign_l))

    for point, flag in zip(points_r, flag_r):
        if flag == -1:
            image_with_points = cv2.circle(camera_image, tuple(point), 1, (255, 0, 0), 5)
        elif flag == -2:
            image_with_points = cv2.circle(camera_image, tuple(point), 1, (0, 0, 255), 5)
        elif flag == -3:
            image_with_points = cv2.circle(camera_image, tuple(point), 1, (255, 255, 255), 5)
        else:
            image_with_points = cv2.circle(camera_image, tuple(point), 1, (50, 50, 50), 5)
    for point, flag in zip(points_l, flag_l):
        if flag == -1:
            image_with_points = cv2.circle(camera_image, tuple(point), 1, (255, 0, 0), 5)
        elif flag ==-2:
            image_with_points = cv2.circle(camera_image, tuple(point), 1, (0, 0, 255), 5)
        elif flag == -3:
            pass
        else:
            image_with_points = cv2.circle(camera_image, tuple(point), 1, (50, 50, 50), 5)

    for L, R, flag in zip(points_r, sign_tuple_r, flag_r):
        if flag == -1:
            image_with_points = cv2.line(image_with_points, tuple(L), tuple(R), (255, 0, 0), 2)
        elif flag == -2:
            image_with_points = cv2.line(image_with_points, tuple(L), tuple(R), (0, 0, 255), 2)
        elif flag == -3:
            image_with_points = cv2.line(image_with_points, tuple(L), tuple(R), (255, 255, 255), 2)
        else:
            image_with_points = cv2.line(image_with_points, tuple(L), tuple(R), (50, 50, 50), 2)

    for L, R, flag in zip(points_l, sign_tuple_l, flag_l):
        if flag == -1:
            image_with_points = cv2.line(image_with_points, tuple(L), tuple(R), (255, 0, 0), 2)
        elif flag == -2:
            image_with_points = cv2.line(image_with_points, tuple(L), tuple(R), (0, 0, 255), 2)
        elif flag == -3:
            image_with_points = cv2.line(image_with_points, tuple(L), tuple(R), (255, 255, 255), 2)
        else:
            image_with_points = cv2.line(image_with_points, tuple(L), tuple(R), (50, 50, 50), 2)

    return image_with_points

def map_lidar_points_onto_image(image_undist, sign_dic_r, sign_dic_l, lidar, max_distance):
    """ Maps the lidar points onto the image, using a radius to make the lidar
        points bigger and opcity. The lidar points get a color according to
        their distance.
        Opacity does not work yet. """

    pixel_radius=3

    image_with_lidar = image_undist

    max_row = image_undist.shape[0] - 1
    max_col = image_undist.shape[1] - 1

    lidar_id_r = sign_dic_r['lidar_id']
    lidar_id_l = sign_dic_l['lidar_id']
    lidar_ids = lidar_id_r + lidar_id_l

    rows = []
    cols = []
    for id in lidar_ids:
        # get rows and cols (note that row and col are not int for some reason)
        rows.append(int(lidar['row'][id] + 0.5))
        cols.append(int(lidar['col'][id] + 0.5))

    rows = np.asarray(rows)
    cols = np.asarray(cols)

    n_lidar = len(rows)

    depths_r = sign_dic_r['depth']
    depths_l = sign_dic_l['depth']
    depths = depths_r + depths_l
    depths = np.asarray(depths)

    colors_rgb = get_color_for_depth(depths, max_distance)

    for i in range(n_lidar):
        row_box = np.arange(rows[i] - pixel_radius, rows[i] + pixel_radius + 1)
        row_box = np.clip(row_box, 0, max_row)
        col_box = np.arange(cols[i] - pixel_radius, cols[i] + pixel_radius + 1)
        col_box = np.clip(col_box, 0, max_col)
        image_with_lidar[row_box[0]:row_box[-1], col_box[0]:col_box[-1], :] \
            = colors_rgb[i]

    return image_with_lidar

def get_color_for_depth(depth, max_distance):
    """ Given a distance the color on the colormap will be returned.
        Using:  max_distance: 90 (just guessing a default)
                colormap: 'plasma' """

    cmap = matplotlib.cm.get_cmap('plasma')
    color = cmap(depth / max_distance)

    return color[:, 0:3] * 255

def map_wrong_lidar_points_onto_image(image_undist, sign_dic_r, sign_dic_l, lidar):
    """ Maps the lidar points onto the image, using a radius to make the lidar
        points bigger and opcity. The lidar points get a color according to
        their distance.
        Opacity does not work yet. """

    pixel_radius= 5

    image_with_lidar = image_undist

    max_row = image_undist.shape[0] - 1
    max_col = image_undist.shape[1] - 1

    lidar_id_r = []
    lidar_id_l = []
    for flag_r, flag_l in zip(sign_dic_r['flag'], sign_dic_l['flag']):
        if flag_r != -1 and flag_r != -2 and flag_r != -3:
            lidar_id_r.append(flag_r)
        if flag_l != -1 and flag_l !=-2 and flag_l != -3:
            lidar_id_l.append(flag_l)
    lidar_ids = lidar_id_r + lidar_id_l

    rows = []
    cols = []
    for id in lidar_ids:
        # get rows and cols (note that row and col are not int for some reason)
        rows.append(int(lidar['row'][id] + 0.5))
        cols.append(int(lidar['col'][id] + 0.5))

    rows = np.asarray(rows)
    cols = np.asarray(cols)

    n_lidar = len(rows)

    depths_r = sign_dic_r['depth']
    depths_l = sign_dic_l['depth']
    depths = depths_r + depths_l
    depths = np.asarray(depths)

    colors_rgb = []
    for i in range(0, n_lidar):
        colors_rgb.append([0,255,0])

    for i in range(n_lidar):
        row_box = np.arange(rows[i] - pixel_radius, rows[i] + pixel_radius + 1)
        row_box = np.clip(row_box, 0, max_row)
        col_box = np.arange(cols[i] - pixel_radius, cols[i] + pixel_radius + 1)
        col_box = np.clip(col_box, 0, max_col)
        image_with_lidar[row_box[0]:row_box[-1], col_box[0]:col_box[-1], :] \
            = colors_rgb[i]

    return image_with_lidar

def get_counter(sign_dic_r, sign_dic_l):
    counter_sign = 0
    counter_free_space = 0
    counter_minus_two = 0
    counter_out_of_image = 0
    flags = sign_dic_r['flag'] + sign_dic_l['flag']
    x_values = sign_dic_r['x'] + sign_dic_l['x']
    y_values = sign_dic_r['y'] + sign_dic_l['y']
    w_values = sign_dic_r['w'] + sign_dic_l['w']
    h_values = sign_dic_r['h'] + sign_dic_l['h']

    total_signs = len(flags)

    for x,y,w, h, flag in zip(x_values, y_values, w_values, h_values, flags):
        if flag == -3:
            counter_out_of_image += 1
        elif flag == -1:
            counter_sign +=1
        elif flag == -2:
            counter_minus_two += 1
        else:
            counter_free_space +=1

    return counter_sign, counter_free_space, counter_minus_two, counter_out_of_image, total_signs
