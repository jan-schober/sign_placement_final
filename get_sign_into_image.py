import math

import numpy as np
import pandas as pd
from PIL import Image


def print_signs_on_image(camera_file_path, config_dict, img_with_lidar_points, final_dic):

    # load the data from the get_depth script and delete all irrelevant points who are beyond the max distance
    # and which have no corresponding lidar point
    data = final_dic
    max_distance = config_dict['max_distance']
    data_r_dic, data_l_dic = delete_wrong_points(data, max_distance)

    # get the lidar and semantic path for the same scene
    semantic_image_path, lidar_data_path = get_paths(camera_file_path)

    # load the real life sign size in [m]
    sign_size = config_dict['sign_size']

    # load the free space parameters
    free_space_dic = config_dict['free_space']

    # this function returns as a dictionary with sign information for each slice
    # it checks if there is no lidar point under the sign and also calculates the sign size for each sign in px
    sign_dic_r, sign_dic_l = get_sign_dic(data_r_dic, data_l_dic, config_dict['camera_settings'], lidar_data_path,
                                          sign_size, free_space_dic)

    # we load the camera image and the sign image and print the signs on the camera image with the informations from
    # the sign dictionarys
    camera_image_undist = Image.fromarray(img_with_lidar_points)
    sign_img = Image.open(config_dict["paths"]["sign_path"])

    camera_image_with_signs = print_sign(camera_image_undist, sign_img, sign_dic_r)
    camera_image_with_signs = print_sign(camera_image_with_signs, sign_img, sign_dic_l)

    '''
    plt.imshow(camera_image_with_signs)
    plt.show()
    '''
    return data_r_dic, data_l_dic, sign_dic_r, sign_dic_l, camera_image_with_signs


def print_sign(camera_image_undist, sign_img, sign_dic):
    '''
    gets the camera image and sign image and print the signs on the camera image with the information stored in
    the sign dictionaries
    '''
    for x, y, w, h , flag in zip(sign_dic['x'], sign_dic['y'], sign_dic['w'], sign_dic['h'], sign_dic['flag']):
        if flag == -1:
            sign_img_resize = sign_img.resize((w, h))
            camera_image_undist.paste(sign_img_resize, (x, y), mask=sign_img_resize)
    return camera_image_undist


def delete_wrong_points(data, max_distance):
    '''
    gets the dictionary with the information for each piece and deletes all the -1 and points who are further away than
    the max distance
    the funtion returns a dictionary for the right and left side
    '''
    y_arr_r = []
    y_arr_l = []
    x_r_arr = []
    x_l_arr = []
    lidar_id_r_arr = []
    lidar_id_l_arr = []
    depth_value_x_r_arr = []
    depth_value_x_l_arr = []
    data_r_dic = {}
    data_l_dic = {}

    for i in range(0, len(data['depth_value_x_r'])):
        if data['depth_value_x_r'][i] <= max_distance and data['depth_value_x_r'][i] != -1:
            y_arr_r.append(data['y'][i])
            x_r_arr.append(data['x_r'][i])
            lidar_id_r_arr.append(data['lidar_id_r'][i])
            depth_value_x_r_arr.append(data['depth_value_x_r'][i])
        if data['depth_value_x_l'][i] <= max_distance and data['depth_value_x_l'][i] != -1:
            y_arr_l.append(data['y'][i])
            x_l_arr.append(data['x_l'][i])
            lidar_id_l_arr.append(data['lidar_id_l'][i])
            depth_value_x_l_arr.append(data['depth_value_x_l'][i])

    data_r_dic['x_r'] = x_r_arr
    data_r_dic['y_r'] = y_arr_r
    data_r_dic['lidar_id_r'] = lidar_id_r_arr
    data_r_dic['depth_value_x_r'] = depth_value_x_r_arr

    data_l_dic['x_l'] = x_l_arr
    data_l_dic['y_l'] = y_arr_l
    data_l_dic['lidar_id_l'] = lidar_id_l_arr
    data_l_dic['depth_value_x_l'] = depth_value_x_l_arr

    return data_r_dic, data_l_dic


def get_sign_dic(data_r_dict, data_l_dict, camera_dic, lidar_path, sign_size, free_space_dic):
    '''
    the function calculates the sign dictionary for the corresponding data from the get_depth_street_edge script
    '''

    # for calculating the offset and sign size in px we need the camera settings
    V_FOV_rad = math.radians(camera_dic['Vertical_FOV_degree'])
    H_FOV_rad = math.radians(camera_dic['Horizontal_FOV_degree'])
    V_PX = camera_dic['Vertical_Pixels']
    H_PX = camera_dic['Horizontal_Pixels']

    # load lidar data and convert lidar to panda df for better handling
    lidar_data = np.load(lidar_path)
    key_list = list(lidar_data.keys())
    key_list.remove('points')
    array_filled = np.array([np.asarray(lidar_data[i]) for i in key_list])
    lidar_df = pd.DataFrame(np.transpose(array_filled), columns=key_list)
    point_list = ["p0", "p1", "p2"]  # can not pass array to DataFrame
    for i, p in enumerate(point_list):
        lidar_df[p] = np.asarray(lidar_data["points"])[:, i]

    # we need the horizontal angle for each point for the free space check later
    # we safe the angle for each lidar point in the panda dataframe
    angle_alpha_array = []
    for p_0, p_1 in zip(lidar_df['p0'], lidar_df['p1']):
        angle_alpha = math.atan2(p_1, p_0)
        angle_alpha_array.append(angle_alpha)
    lidar_df['angle_alpha'] = angle_alpha_array

    # create empty dicitonary and arrays to store the informations
    sign_dic_r = {}
    x_coordinate_sign_r = []
    y_coordinate_sign_r = []
    sign_width_r = []
    sign_height_r = []
    sign_street_x_r = []
    sign_street_y_r = []
    sign_lidar_id_r = []
    sign_depth_r = []
    flag_r = []

    sign_dic_l = {}
    x_coordinate_sign_l = []
    y_coordinate_sign_l = []
    sign_width_l = []
    sign_height_l = []
    sign_street_x_l = []
    sign_street_y_l = []
    sign_lidar_id_l = []
    sign_depth_l = []
    flag_l = []

    # load the free space parameter
    beta_offset =  free_space_dic['beta_offset']
    gamma_offset = free_space_dic['gamma_offset']
    small_angle_rad = free_space_dic['small_angle_rad']
    z_min = free_space_dic['z_min']
    z_max  = free_space_dic['z_max']
    c_offset = free_space_dic['c_offset']
    c_depth_offset = free_space_dic['c_depth_offset']

    # we calculate for each lidar point the sign position, sign size
    # if there is no lidar point in the space under the sign
    # we do this seperate for the right and left side
    for depth_value, x_r, y_r, lidar_id in zip(data_r_dict['depth_value_x_r'], data_r_dict['x_r'], data_r_dict['y_r'],
                                               data_r_dict['lidar_id_r']):
        flag = check_free_space_right_side(lidar_id, lidar_df, beta_offset,gamma_offset,small_angle_rad,z_min,z_max,c_offset,c_depth_offset)

        number_x_px, number_y_px = calculate_offset_px(depth_value, H_PX, V_PX, H_FOV_rad, V_FOV_rad)
        sign_width_px, sign_height_px = calculate_sign_size_px(depth_value, H_PX, V_PX, H_FOV_rad, V_FOV_rad, sign_size)
        sign_width_r.append(sign_width_px)
        sign_height_r.append(sign_height_px)
        x_coordinate_sign_r.append(x_r + number_x_px)
        y_coordinate_sign_r.append(y_r - number_y_px - sign_height_px)
        sign_street_x_r.append(x_r)
        sign_street_y_r.append(y_r)
        sign_lidar_id_r.append(lidar_id)
        sign_depth_r.append(depth_value)
        flag_r.append(flag)

    for depth_value, x_l, y_l, lidar_id in zip(data_l_dict['depth_value_x_l'], data_l_dict['x_l'], data_l_dict['y_l'],
                                               data_l_dict['lidar_id_l']):
        flag = check_free_space_left_side(lidar_id, lidar_df, beta_offset,gamma_offset,small_angle_rad,z_min,z_max,c_offset, c_depth_offset)

        number_x_px, number_y_px = calculate_offset_px(depth_value, H_PX, V_PX, H_FOV_rad, V_FOV_rad)
        sign_width_px, sign_height_px = calculate_sign_size_px(depth_value, H_PX, V_PX, H_FOV_rad, V_FOV_rad, sign_size)
        sign_width_l.append(sign_width_px)
        sign_height_l.append(sign_height_px)
        x_coordinate_sign_l.append(x_l - number_x_px - sign_width_px)
        y_coordinate_sign_l.append(y_l - number_y_px - sign_height_px)
        sign_street_x_l.append(x_l)
        sign_street_y_l.append(y_l)
        sign_lidar_id_l.append(lidar_id)
        sign_depth_l.append(depth_value)
        flag_l.append(flag)

    # safe the information in a sign dicitonary for right and left side
    sign_dic_r['x'] = x_coordinate_sign_r
    sign_dic_r['y'] = y_coordinate_sign_r
    sign_dic_r['w'] = sign_width_r
    sign_dic_r['h'] = sign_height_r
    sign_dic_r['x_street'] = sign_street_x_r
    sign_dic_r['y_street'] = sign_street_y_r
    sign_dic_r['lidar_id'] = sign_lidar_id_r
    sign_dic_r['depth'] = sign_depth_r
    sign_dic_r['flag'] = flag_r

    sign_dic_l['x'] = x_coordinate_sign_l
    sign_dic_l['y'] = y_coordinate_sign_l
    sign_dic_l['w'] = sign_width_l
    sign_dic_l['h'] = sign_height_l
    sign_dic_l['x_street'] = sign_street_x_l
    sign_dic_l['y_street'] = sign_street_y_l
    sign_dic_l['lidar_id'] = sign_lidar_id_l
    sign_dic_l['depth'] = sign_depth_l
    sign_dic_l['flag'] = flag_l

    return sign_dic_r, sign_dic_l


def calculate_sign_size_px(depth_value, H_PX, V_PX, H_FOV_rad, V_FOV_rad, sign_size):
    '''
    this function calculates the sign size in px from real sign size in [m] for each depth value
    '''
    sign_width_px = int((sign_size * H_PX) / (depth_value * H_FOV_rad))
    sign_height_px = int((sign_size * V_PX) / (depth_value * V_FOV_rad))
    return sign_width_px, sign_height_px


def calculate_offset_px(depth_value, H_PX, V_PX, H_FOV_rad, V_FOV_rad):
    '''
    this function calculates the offset away from the street edge (0.5m in x-axis and 2m in y-axis)
    '''
    number_x_px = int((0.5 * H_PX) / (depth_value * H_FOV_rad))
    number_y_px = int((2 * V_PX) / (depth_value * V_FOV_rad))
    return number_x_px, number_y_px


def get_paths(camera_path):
    '''
    the function gets the camera image path and returns the corresponding lidar and semantic image path
    '''
    splited_path = camera_path.split('/')
    file_name = splited_path[-1].split('_')
    number = file_name[-1].split('.')
    subdir = '/'.join(splited_path[:-3])

    semantic_image_path = subdir + '/label/cam_front_center/' + file_name[0] + \
                          '_label_' + file_name[2] + '_' + number[0] + '.png'
    lidar_data_path = subdir + '/lidar/cam_front_center/' + file_name[0] + \
                      '_lidar_' + file_name[2] + '_' + number[0] + '.npz'

    return semantic_image_path, lidar_data_path

def check_free_space_right_side(lidar_id, lidar_df, beta_offset,gamma_offset,small_angle_rad,z_min,z_max,c_offset, c_depth_offset):
    '''
    the function checks for the lidar_id if there is lidar points where the sign should be placed
    it also checks if the angle is positiv, if yes it also returns False
    how we check the free space is explained briefly in confluence
    '''
    p_0 = lidar_df.iloc[lidar_id]['p0']
    p_1 = lidar_df.iloc[lidar_id]['p1']
    p_2 = lidar_df.iloc[lidar_id]['p2']
    beta_angle = math.atan2((p_1 - beta_offset), p_0)

    # 1,5m offset in 30m range
    if beta_angle >= -small_angle_rad:
        return -2

    gamma_angle = math.atan2((p_1 - gamma_offset), p_0)
    c_distance = math.sqrt((p_0) ** 2 + (p_1 - c_offset) ** 2)

    relevant_points = (lidar_df['angle_alpha'] <= beta_angle) & (lidar_df['angle_alpha'] >= gamma_angle)\
                      & (p_2 + z_min  <= lidar_df['p2']) & (lidar_df['p2'] <= p_2 + z_max)
    relevant_points = lidar_df[relevant_points]

    for p_0_rel, p_1_rel in zip(relevant_points['p0'], relevant_points['p1']):
        c_distance_rel = math.sqrt((p_0_rel) ** 2 + (p_1_rel) ** 2)
        if c_distance_rel <= c_distance + c_depth_offset:
            lidar_id_point = lidar_df[(lidar_df['p0'] == p_0_rel) & (lidar_df['p1'] == p_1_rel)].index
            lidar_id_point = int(lidar_id_point.values[0])
            return lidar_id_point

    return -1


def check_free_space_left_side(lidar_id, lidar_df, beta_offset,gamma_offset,small_angle_rad,z_min,z_max,c_offset, c_depth_offset):
    '''
    checks if there is no lidar point under the sign equivalent to the function above
    '''

    p_0 = lidar_df.iloc[lidar_id]['p0']
    p_1 = lidar_df.iloc[lidar_id]['p1']
    p_2 = lidar_df.iloc[lidar_id]['p2']
    beta_angle = math.atan2((p_1 + beta_offset), p_0)

    if beta_angle <= small_angle_rad:
        return -2

    gamma_angle = math.atan2((p_1 + gamma_offset), p_0)
    c_distance = math.sqrt((p_0) ** 2 + (p_1 + c_offset) ** 2)

    relevant_points = (lidar_df['angle_alpha'] >= beta_angle) & (lidar_df['angle_alpha'] <= gamma_angle) \
                      & (p_2 + z_min <= lidar_df['p2']) & (lidar_df['p2'] <= p_2 + z_max)
    relevant_points = lidar_df[relevant_points]

    for p_0_rel, p_1_rel in zip(relevant_points['p0'], relevant_points['p1']):
        c_distance_rel = math.sqrt((p_0_rel) ** 2 + (p_1_rel) ** 2)
        if c_distance_rel <= c_distance + c_depth_offset:
            lidar_id_point = lidar_df[(lidar_df['p0'] == p_0_rel) & (lidar_df['p1'] == p_1_rel)].index
            lidar_id_point = int(lidar_id_point.values[0])
            return lidar_id_point

    return -1

