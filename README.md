# How to run the Code:
1.) Adapt the config dictionary in the config.py (You can find explanations for each dictionary entry direct in the file)
2.) Run the main skript (I made a list of the python packages you need for executing the script)

# How I calculated the sign size and positions:
[https://stackoverflow.com/questions/42035721/how-to-measure-object-size-in-real-world-in-terms-of-measurement-like-inches-cen](https://stackoverflow.com/questions/42035721/how-to-measure-object-size-in-real-world-in-terms-of-measurement-like-inches-cen)


# Sign size in (m) for german roads from official regulations:
[(http://www.verwaltungsvorschriften-im-internet.de/bsvwvbund_26012001_S3236420014.htm](http://www.verwaltungsvorschriften-im-internet.de/bsvwvbund_26012001_S3236420014.htm)

# How the free space check works:
[Free Space Check](https://ess-confluence.fzi.de:8443/pages/viewpage.action?pageId=325287960&preview=/325287960/325287967/free_space.png)

# You can find some example images and the sign images i used on this confluence site:
[https://ess-confluence.fzi.de:8443/pages/viewpage.action?pageId=325287960](https://ess-confluence.fzi.de:8443/pages/viewpage.action?pageId=325287960)

# My config file:
  
config_dict = {  
'paths': {  
        'camera_image_root_path': '/media/jan/TEST/camera_lidar_semantic/camera_lidar_semantic/',  
  'sub_dir': '20180925_101535/',  
  'file_name': '20180925101535_camera_frontcenter_000039182',  
  'cams_lidar_json_path': '/media/jan/TEST/camera_lidar_semantic/',  
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