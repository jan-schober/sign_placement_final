"""
The setup configuration to place signs on camera images.
The whole script is written for the A2D2-Dataset.

Following parameters should be set:

    camera_image_root_path:     Path to folder where the different scenes are stored
    sub_dir:                    The subdirectory of the different scenes
    file_name:                  If you want to create 1 image just write the name of the file (without .png suffix)
                                if you want to loop through all images write 'All'
    cams_lidar_json_path:       Path to the cams_lidar.json file from the A2D2-Dataset
    sign_path:                  Path to sign image you want to place in the image, example image is uploaded on my
                                confluence
    free_space_sign_path:       Path to sign for debbuging image, get pasted on image if the free space check sort it out
    small_angle_sign_path:      Path to sign for debbuging image, get pasted on image if the small angle check sort it out
    debug_image:                True --> export an extra image with all the signs which were sorted out
    step_size_image:            If you want to iterate over all images choose step_size = 1, otherwise choose an step size
    hex_color:                  Semantic color of the street in the A2D2-Dataset (#ff00ff)
    n_slices:                   The number of streetedges we search for each street side
    threshold_lidar:            Number of Px which the lidar can be away from the street edege point
    max_distance:
    sign_size:                  The sign size in [m], you can find the sign size for german signs in my confluence side
    camera settings:            Camera parameters from the front camera used in the A2D2-Dataset
    free_space:                 Settings for the free space check, further explanation on the confluence site

My confluence site with further information and explanations:
https://ess-confluence.fzi.de:8443/pages/viewpage.action?pageId=325287960
"""

config_dict = {}

# 20180810142822_camera_frontcenter_000009668
# All

config_dict = {
    'paths': {
        'camera_image_root_path': '/media/jan/TEST/camera_lidar_semantic/camera_lidar_semantic/',
        'sub_dir': '20180810_142822/',
        'file_name': '20180810142822_camera_frontcenter_000065580',
        'cams_lidar_json_path': '/media/jan/TEST/camera_lidar_semantic/cams_lidars.json',
        'result_export_path': '/media/jan/TEST/_results/final/',
        'sign_path': '/home/jan/PycharmProjects/a2d2_dataset_scripts/get_sign_into_image/50_sign.png',
        'free_space_sign_path': '/media/jan/TEST/50_sign_grey.png',
        'small_angle_sign_path': '/media/jan/TEST/50_sign_blue.png'

    },
    'debug_images': False,
    'step_size_images': 3,
    'hex_color': '#ff00ff',
    'n_slices': 60,
    'threshold_lidar': 12.5,
    'max_distance': 30,
    'sign_size': 0.6,
    'camera_settings': {
        'Vertical_FOV_degree': 38,
        'Horizontal_FOV_degree': 60,
        'Vertical_Pixels': 1208,
        'Horizontal_Pixels': 1920
    },
    'free_space': {
        'beta_offset': 0.15,
        'gamma_offset': 0.8,
        'small_angle_rad': 0.0333,
        'z_min': 0.7,
        'z_max': 2.25,
        'c_offset': 0.5,
        'c_depth_offset': 0.35
    }
}
