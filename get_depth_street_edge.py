import json
import math

import cv2
import matplotlib.cm
import numpy as np
import pandas as pd
from PIL import ImageColor
from typing import Tuple


def get_depth(
    camera_file_path: str,
    config: dict,
    semantic_image_path: str = None,
    lidar_data_path: str = None,
) -> Tuple[dict, np.ndarray, np.ndarray]:
    """Matches LIDAR Data to the image

    Args:
        camera_file_path (Path): An image path
        config (dict): The config dict
        semantic_image_path (Path, optional): A path to the semantic image Defaults to None, were the path is inferred from the config
        lidar_data_path (Path, optional): Path to lidar data (npg). Defaults to None,were the path is inferred from the config

    Returns:
        dict: Dictionary with LIDAR Points
        np.ndarray: Image with Points
        np.ndarray: Image with LIDAR Points
    """
    # If no semantic or lidar path given use the ones from the config
    if semantic_image_path == None or lidar_data_path == None:
        # a function that gives us the paths to the corresponding semantic and lidar data
        semantic_image_path, lidar_data_path = get_paths(camera_file_path)

    # load the semantic image and convert it into RGB format
    semantic_image = cv2.imread(semantic_image_path)
    semantic_image = cv2.cvtColor(semantic_image, cv2.COLOR_BGR2RGB)

    # load lidar config and image info
    with open(config["paths"]["cams_lidar_json_path"], "r") as f:
        config_data = json.load(f)

    with open(camera_file_path.replace(".png", ".json"), "r") as f:
        image_info = json.load(f)

    # undistort semantic image
    semantic_image_undist = undistort_image(semantic_image, image_info, config_data)

    # convert hex to bgr
    rgb_value = ImageColor.getcolor(config["hex_color"], "RGB")

    # get binary mask from semantic image with the rgb value from hex above
    mask = get_binary_mask(semantic_image_undist, rgb_value)

    # we slice the semantic image into n_slices and get a dictionary for each slice back
    # the dictionary contains an id for each slice, x-Right/Left-Coordinate, y-Coordinate and width for the street
    number_of_slices = config["n_slices"]
    H_PX = config["camera_settings"]["Horizontal_Pixels"]
    V_PX = config["camera_settings"]["Vertical_Pixels"]
    piece_dictionary = slice_picture(mask, number_of_slices, H_PX, V_PX)

    # we load the camera image and print the semantic image points on the camera image
    camera_image = cv2.imread(camera_file_path)
    camera_image = cv2.cvtColor(camera_image, cv2.COLOR_BGR2RGB)
    camera_image_undist = undistort_image(camera_image, image_info, config_data)
    image_with_points = print_points_on_image(piece_dictionary, camera_image_undist)

    # load lidar data and convert lidar to panda df for better handling
    lidar_data = np.load(lidar_data_path)
    key_list = list(lidar_data.keys())
    key_list.remove("points")
    array_filled = np.array([np.asarray(lidar_data[i]) for i in key_list])
    lidar_df = pd.DataFrame(np.transpose(array_filled), columns=key_list)
    point_list = ["p0", "p1", "p2"]  # can not pass array to DataFrame
    for i, p in enumerate(point_list):
        lidar_df[p] = np.asarray(lidar_data["points"])[:, i]

    # we load the threshold in px for the corresponding lidar points and search for the closest lidar point
    # to street edge points from the piece dicitonary
    threshold_lidar = config["threshold_lidar"]
    dictionary_with_lidar = get_lidar_points(
        piece_dictionary, lidar_df, threshold_lidar
    )

    # next step we print the lidar points on the camera image
    max_distance = config["max_distance"]
    image_with_lidar_points = map_lidar_points_onto_image(
        camera_image_undist, dictionary_with_lidar, lidar_df, max_distance
    )
    return dictionary_with_lidar, image_with_points, image_with_lidar_points


def slice_picture(mask, number_of_slices, H_PX, V_PX):
    """
    gets an mask and return a dictionary
    with the best points for each piece
    """
    piece_height = int(V_PX / number_of_slices)
    pieces_dictionary = {}
    piece_id = 0
    piece_id_arr = []
    piece_width_arr = []
    piece_x_l_arr = []
    piece_x_r_arr = []
    piece_y_arr = []
    for i in range(0, V_PX, piece_height):
        piece_id_arr.append(piece_id)
        piece_id += 1
        piece = mask[i : i + piece_height, 0 : 0 + H_PX]
        contours, hierachy = cv2.findContours(
            piece, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        if len(contours) != 0:
            x_r, x_l = search_for_best_contour(contours)
            piece_x_r_arr.append(x_r)
            piece_x_l_arr.append(x_l)
            piece_width_arr.append(x_r - x_l)
            piece_y_arr.append(i + piece_height)
        else:
            piece_x_r_arr.append(-1)
            piece_x_l_arr.append(-1)
            piece_width_arr.append(-1)
            piece_y_arr.append(-1)

    pieces_dictionary["piece_id"] = piece_id_arr
    pieces_dictionary["y"] = piece_y_arr
    pieces_dictionary["x_r"] = piece_x_r_arr
    pieces_dictionary["x_l"] = piece_x_l_arr
    pieces_dictionary["width"] = piece_width_arr

    return pieces_dictionary


def search_for_best_contour(contours):
    """ "
    get contours and search for the best contour,
    which is the most right and left point on the lower side
    of the rectangle
    """
    x_r_point_arr = []
    x_l_point_arr = []

    for contour_id in range(0, len(contours)):
        x, y, w, h = cv2.boundingRect(contours[contour_id])
        x_r_point_arr.append(x + w)
        x_l_point_arr.append(x)

    x_r = max(x_r_point_arr)
    x_l = min(x_l_point_arr)

    return x_r, x_l


def undistort_image(image, image_info, config):
    """Perform distortion correction"""
    cam_name = image_info["cam_name"]
    if cam_name in [
        "front_left",
        "front_center",
        "front_right",
        "side_left",
        "side_right",
        "rear_center",
    ]:
        # get parameters from config_cam file
        intr_mat_undist = np.asarray(config["cameras"][cam_name]["CamMatrix"])
        intr_mat_dist = np.asarray(config["cameras"][cam_name]["CamMatrixOriginal"])
        dist_parms = np.asarray(config["cameras"][cam_name]["Distortion"])
        lens = config["cameras"][cam_name]["Lens"]

        if lens == "Fisheye":
            return cv2.fisheye.undistortImage(
                image, intr_mat_dist, D=dist_parms, Knew=intr_mat_undist
            )
        elif lens == "Telecam":
            return cv2.undistort(
                image,
                intr_mat_dist,
                distCoeffs=dist_parms,
                newCameraMatrix=intr_mat_undist,
            )
        else:
            return image
    else:
        "cam_name not found -> no distortion correction"
        return image


def get_binary_mask(semantic_image_undist, color_value):
    """ "
    Returns a mask where 1 is the color_value and 0 is the rest
    """
    mask = cv2.inRange(semantic_image_undist, color_value, color_value)
    return mask


def print_points_on_image(dictionary, image):
    """
    Get the pieces dictionary and the camera
    image and print the points
    on the image and return the image
    """
    x_r_coordinates = delete_minus_one(dictionary["x_r"])
    x_l_coordinates = delete_minus_one(dictionary["x_l"])
    y_coordinates = delete_minus_one(dictionary["y"])

    points_r = np.column_stack((x_r_coordinates, y_coordinates))
    points_l = np.column_stack((x_l_coordinates, y_coordinates))

    for point in points_r:
        image_with_points = cv2.circle(image, tuple(point), 1, (255, 0, 0), 5)
    for point in points_l:
        image_with_points = cv2.circle(image, tuple(point), 1, (255, 0, 0), 5)

    for L, R in zip(points_l, points_r):
        image_with_points = cv2.line(
            image_with_points, tuple(L), tuple(R), (255, 0, 0), 2
        )

    return image_with_points


def delete_minus_one(coordinates_list):
    coordinates_list_without = [i for i in coordinates_list if i != -1]
    return coordinates_list_without


def get_lidar_points(dictionary, lidar_df, threshold):
    """
    get the piece dictionary and the lidar data
    and return lidar id and their depth if they are in the threshold
    """

    rows = lidar_df["row"]
    cols = lidar_df["col"]
    lidar_coordinates = np.column_stack((cols, rows))

    x_r_arr = dictionary["x_r"]
    x_l_arr = dictionary["x_l"]
    y_arr = dictionary["y"]

    lidar_index_arr_R, depth_value_arr_R = get_closest_lidar_point(
        x_r_arr, y_arr, lidar_coordinates, lidar_df, threshold
    )
    lidar_index_arr_L, depth_value_arr_L = get_closest_lidar_point(
        x_l_arr, y_arr, lidar_coordinates, lidar_df, threshold
    )

    dictionary["lidar_id_r"] = lidar_index_arr_R
    dictionary["lidar_id_l"] = lidar_index_arr_L
    dictionary["depth_value_x_r"] = depth_value_arr_R
    dictionary["depth_value_x_l"] = depth_value_arr_L

    return dictionary


def get_closest_lidar_point(x_arr, y_arr, lidar_coordinates, lidar_df, threshold):
    """
    gets two arrays for x and y image coordinates
    and an array with lidar coordinates
    and an dataframe with the lidar data and returns the lidar id and depth
    of the closest lidar point
    """
    lidar_index_arr = []
    depth_value = []

    for x, y in zip(x_arr, y_arr):
        if x == -1:
            lidar_index_arr.append(-1)
            depth_value.append(-1)
        else:
            coordinate = np.array((x, y))
            nearest_lidar_point = min(
                lidar_coordinates,
                key=lambda s: math.hypot(s[0] - coordinate[0], s[1] - coordinate[1]),
            )

            x_offset = abs(nearest_lidar_point[0] - coordinate[0])
            y_offset = abs(nearest_lidar_point[1] - coordinate[1])

            if x_offset <= threshold and y_offset <= threshold:
                lidar_id = lidar_df[
                    (lidar_df["col"] == nearest_lidar_point[0])
                    & (lidar_df["row"] == nearest_lidar_point[1])
                ].index
                lidar_id = lidar_id.values
                lidar_index_arr.append(int(lidar_id[0]))
                depth_value.append(float(lidar_df["depth"][lidar_id].values[0]))
            else:
                lidar_index_arr.append(-1)
                depth_value.append(-1)

    return lidar_index_arr, depth_value


def get_paths(camera_path):
    """
    gets the camera picture paths and returns the paths for the semantic image and the lidar data
    """
    splited_path = camera_path.split("/")
    file_name = splited_path[-1].split("_")
    number = file_name[-1].split(".")
    subdir = "/".join(splited_path[:-3])

    semantic_image_path = (
        subdir
        + "/label/cam_front_center/"
        + file_name[0]
        + "_label_"
        + file_name[2]
        + "_"
        + number[0]
        + ".png"
    )
    lidar_data_path = (
        subdir
        + "/lidar/cam_front_center/"
        + file_name[0]
        + "_lidar_"
        + file_name[2]
        + "_"
        + number[0]
        + ".npz"
    )

    return semantic_image_path, lidar_data_path


def map_lidar_points_onto_image(image_undist, dictionary, lidar, max_distance):
    """Maps the lidar points onto the image, using a radius to make the lidar
    points bigger and opcity. The lidar points get a color according to
    their distance.
    Opacity does not work yet."""

    pixel_radius = 3

    image_with_lidar = image_undist

    max_row = image_undist.shape[0] - 1
    max_col = image_undist.shape[1] - 1

    lidar_id_r = delete_minus_one(dictionary["lidar_id_r"])
    lidar_id_l = delete_minus_one(dictionary["lidar_id_l"])
    lidar_ids = lidar_id_r + lidar_id_l

    rows = []
    cols = []
    for id in lidar_ids:
        # get rows and cols (note that row and col are not int for some reason)
        rows.append(int(lidar["row"][id] + 0.5))
        cols.append(int(lidar["col"][id] + 0.5))

    rows = np.asarray(rows)
    cols = np.asarray(cols)

    n_lidar = len(rows)

    depths_r = delete_minus_one(dictionary["depth_value_x_r"])
    depths_l = delete_minus_one(dictionary["depth_value_x_l"])
    depths = depths_r + depths_l
    depths = np.asarray(depths)

    colors_rgb = get_color_for_depth(depths, max_distance)

    for i in range(n_lidar):
        row_box = np.arange(rows[i] - pixel_radius, rows[i] + pixel_radius + 1)
        row_box = np.clip(row_box, 0, max_row)
        col_box = np.arange(cols[i] - pixel_radius, cols[i] + pixel_radius + 1)
        col_box = np.clip(col_box, 0, max_col)
        image_with_lidar[
            row_box[0] : row_box[-1], col_box[0] : col_box[-1], :
        ] = colors_rgb[i]

    return image_with_lidar


def get_color_for_depth(depth, max_distance):
    """Given a distance the color on the colormap will be returned.
    Using:  max_distance: 90 (just guessing a default)
            colormap: 'plasma'"""

    cmap = matplotlib.cm.get_cmap("plasma")
    color = cmap(depth / max_distance)

    return color[:, 0:3] * 255
