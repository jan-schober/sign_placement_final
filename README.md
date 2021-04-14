# How to run the Code:
1. Adapt the config dictionary in the config.py (You can find explanations for each dictionary entry direct in the file)
2. Run the main skript (Please install the required python packages with `pip -r requirements.txt`)

Examples can be found on [this confluence site](https://ess-confluence.fzi.de:8443/pages/viewpage.action?pageId=325287960)
# FAQ
* [How I calculated the sign size and positions](https://stackoverflow.com/questions/42035721/how-to-measure-object-size-in-real-world-in-terms-of-measurement-like-inches-cen)

* [Sign size in (m) for german roads from official regulations](http://www.verwaltungsvorschriften-im-internet.de/bsvwvbund_26012001_S3236420014.htm)


* [How the Free Space Check works](https://ess-confluence.fzi.de:8443/pages/viewpage.action?pageId=325287960&preview=/325287960/325287967/free_space.png)


# Explanation of the csv-export:

* depth_r, depth_l → Distance from lidar to the street edge in meters 
 
* lidar_id_r, lidar_id_l → The Id of the used lidar point

* x_r_street, y_r_street, x_l_street., y_l_street → The pixel coordinates of the street edges right and left side

* x_r_sign, y_r_sign, x_l_sign, y_l_sign → The pixel coordinates of the sign top left corner

* width, height → Width and height in Px of the sign
* flag
  * -1: correct placed sign
  * -2: sign is to central or wrong side
  * -3: sign is out of the image
  * positive integer: idar id from free space check, object is under the sign

# My config file:
```python
config_dict = {  
'paths': {  
  'camera_image_root_path': '/media/jan/TEST/camera_lidar_semantic/camera_lidar_semantic/',  
  'sub_dir': '20180925_101535/',  
  'file_name': '20180925101535_camera_frontcenter_000039182',  
  'cams_lidar_json_path': '/media/jan/TEST/camera_lidar_semantic/cams_lidars.json',  
  'result_export_path': '/media/jan/TEST/_results/final/',  
  'sign_path': '/home/jan/PycharmProjects/a2d2_dataset_scripts/get_sign_into_image/50_sign.png',  
  'free_space_sign_path': '/media/jan/TEST/50_sign_grey.png',  
  'small_angle_sign_path': '/media/jan/TEST/50_sign_blue.png'  
  
  },  
  'debug_images': True,  
  'step_size_images': 4,  
  'hex_color': '#ff00ff',  
  'n_slices': 70,  
  'threshold_lidar': 12.5,  
  'max_distance': 30,  
  'sign_size': 0.42,  
  'camera_settings': {  
        'Vertical_FOV_degree': 38,  
  'Horizontal_FOV_degree': 60,  
  'Vertical_Pixels': 1208,  
  'Horizontal_Pixels': 1920  
  },  
  'free_space':{  
        'beta_offset': 0.15,  
  'gamma_offset': 0.8,  
  'small_angle_rad': 0.0333,  
  'z_min': 0.7,  
  'z_max': 2.25,  
  'c_offset': 0.5,  
  'c_depth_offset' : 0.35  
  }  
}
```
