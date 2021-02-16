import get_depth_street_edge
import get_sign_into_image
from config import config_dict
import glob
import debugging_skript_sign_placement
import json
import os
import csv
import cv2
import time

'''
How to:
1. Fill in the config dictionary in config.py
2. Run this script
3. Enjoy the results:)

You need following python packages installed:
    numpy
    glob
    json
    os
    csv
    cv2
    math
    pandas
    PIL
    matplotlib.cm
    
My confluence site with further information and explanations:
https://ess-confluence.fzi.de:8443/pages/viewpage.action?pageId=325287960
'''

start_time = time.time()
config_dict_used = config_dict

if config_dict_used['paths']['file_name'] == 'All':
    path_for_glob = config_dict_used['paths']['camera_image_root_path'] + config_dict_used['paths']['sub_dir']
    file_name_list = glob.glob(path_for_glob + 'camera/cam_front_center/*.png')

else:
    file_name_list = [config_dict_used['paths']['camera_image_root_path'] + config_dict_used['paths']['sub_dir'] + \
                      'camera/cam_front_center/' + config_dict_used['paths']['file_name'] + '.png']

step_size_image = config_dict_used['step_size_images']
number_files = int(round((len(file_name_list) / step_size_image) + 0.5))
file_counter = 1

if config_dict_used['debug_images']:
    sign_counter = 0
    counter_free_space = 0
    middle_counter = 0
    out_of_image_counter = 0
    total_counter = 0
    csv_columns_stats = ['total_number', 'good_signs', 'free_space', 'small_angle/wrong side', 'out of image']

for camera_file_path in file_name_list[::step_size_image]:

        final_dict, img_with_semantic_points, img_with_lidar_points = get_depth_street_edge.get_depth(camera_file_path, config_dict)
        data_r, data_l, sign_r, sign_l, image_with_signs = get_sign_into_image.print_signs_on_image(camera_file_path,config_dict,img_with_lidar_points, final_dict)

        file_name_camera = camera_file_path.split('/')
        file_name_camera = file_name_camera[-1]
        file_name_camera = file_name_camera.split('.')
        file_name_camera = file_name_camera[0]

        file_name_img_street_edge = file_name_camera + '_street_edge_result.png'
        file_name_csv = file_name_camera + '_with_sign_result.csv'
        file_name_final_dict = file_name_camera + '_result.json'
        file_name_final_img = file_name_camera + '_with_signs.png'

        export_path = config_dict_used["paths"]["result_export_path"]

        img_with_lidar_points = cv2.cvtColor(img_with_lidar_points, cv2.COLOR_RGB2BGR)
        cv2.imwrite(os.path.join(export_path, file_name_img_street_edge), img_with_lidar_points)
        image_with_signs.save(os.path.join(export_path, file_name_final_img))

        if config_dict_used['debug_images']:
            debugging_image, c_sign, c_free_space, c_middle, c_out_image, number_signs = debugging_skript_sign_placement.main(camera_file_path, sign_r, sign_l, config_dict, data_r, data_l)
            sign_counter = sign_counter + c_sign
            counter_free_space = counter_free_space + c_free_space
            middle_counter = middle_counter + c_middle
            out_of_image_counter = out_of_image_counter + c_out_image
            total_counter = total_counter + number_signs
            debug_image_path = file_name_camera + '_debug_img.png'
            debugging_image.save(os.path.join(export_path, debug_image_path))



        with open(os.path.join(export_path, file_name_final_dict), 'w') as fp:
            json.dump(final_dict, fp, indent=4)

        csv_output = zip(sign_r['depth'], sign_r['lidar_id'], sign_r['x_street'], sign_r['x'], sign_r['y_street'],
                         sign_r['y'], sign_r['w'], sign_r['h'],
                         sign_l['depth'], sign_l['lidar_id'], sign_l['x_street'], sign_l['x'], sign_l['y_street'],
                         sign_l['y'], sign_l['w'], sign_l['h'])

        csv_columns = ['depth_r', 'lidar_id_r', 'x_r_street', 'x_r_sign', 'y_r_street', 'y_r_sign', 'width', 'height',
                       'depth_l', 'lidar_id_l', 'x_l_street', 'x_l_sign', 'y_l_street', 'y_l_sign', 'width', 'height']

        with open(os.path.join(export_path, file_name_csv), 'w') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(csv_columns)
            for row in csv_output:
                writer.writerow(row)
            if config_dict_used['debug_images']:
                writer.writerow(csv_columns_stats)
                writer.writerow([number_signs, c_sign, c_free_space, c_middle, c_out_image])

        print(str(file_counter) + ' / ' + str(number_files))
        file_counter += 1



if config_dict_used['debug_images']:
    csv_columns_stats = ['total_number', 'good_signs', 'free_space', 'small_angle/wrong side', 'out of image']
    csv_output_stats = [total_counter, sign_counter, counter_free_space, middle_counter, out_of_image_counter]
    with open(os.path.join(export_path, 'stats.csv'), 'w') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(csv_columns_stats)
        writer.writerow(csv_output_stats)

print("My program took", time.time() - start_time, "to finish calculations")
