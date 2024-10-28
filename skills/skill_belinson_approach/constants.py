from raya.enumerations import UI_THEME_TYPE

# Cameras
APPROACH_FACE_CAMERA = 'nav_bottom'
APPROACH_FEET_CAMERA = 'nav_top'

# Parameters used throughout the app
FACE_DETECTOR_PARAMS = {
    'name' : 'cnn_face',
    'source' : 'nav_bottom',
    'model_params' : {'scale' : 0.3, 'depth' : True}
}

FEET_DETECTOR_PARAMS = {
    'name' : 'yolov8m_feet',
    'source' : 'nav_top',
    'model_params' : {'depth' : True}
}

SCAN_FOR_FACES_ROTATION_PARAMS = {
    'angle' : 30.0,
    'angular_speed' : 5.0,
    'wait' : True
}

LIDAR_SCAN_PARAMS = {
    'lower_angle' : -10.0,
    'upper_angle' : 10.0
}

# Max state attempts
MAX_DETECTION_ATTEMPTS = 2
MAX_APPROACH_ATTEMPTS = 2

# Feedback dictionary
MSGS_DICT = {
    'DETECT_FACE' : {'success' : 1,
                     'failure' : 'No faces detected'},
    'APPROACH_FACE' : {'success' : 2,
                       'failure' : 'Could not approach face'},
    'DETECT_FEET' : {'success' : 3,
                     'failure' : 'Could not detect feet'},
    'APPROACH_FEET_CV' : {'success' : 4,
                          'failure' : 'Could not approach feet'},
    'ONLY_FACE' : {'success' : 5,
                   'failure' : 'Error'},
    'RECOMPUTE_PATH' : {'success' : 6,
                        'failure' : 'Could not recompute path'}
 }

# Navigation params
LINE_IDX = list(range(-10, 0, 1)) + list(range(0, 11, 1))
ANGLE_IDX = list(range(-40, 0, 10)) + list(range(0, 41, 10))

# Feet detection parameters
FEET_DETECTION_THRESHOLD = 0.6
FEET_SIZE_THRESHOLD = 0.04
DISTANCE_CONST = 0.53
FEET_DIST_BEFORE_STOP = 0.2
MAX_DISTANCE_OFFSET_PERCENTAGE = 0.8

# UI parameters
background_url = 'url(/assets/belinson_logo.png)'

CUSTOM_STYLE = {'title' : {'font-size' : '150px'},
                        'subtitle' : {'font-size' : '70px'},
                        'background' : {'background' : background_url,
                                        'backgroundRepeat' : 'no-repeat',
                                        'backgroundSize' : 'cover'}
                }

UI_RECOMPUTING_PATH = {
    'title' : '/',
    'show_loader' : True,
    'back_button_text' : '',
    'theme' : UI_THEME_TYPE.WHITE,
    'custom_style' : CUSTOM_STYLE
}

# Error messages
ERROR_NO_FACES_DETECTED = (1, 'No faces detected')
ERROR_COULDNT_APPROACH_FACE = (2, 'Could not navigate to the face')
ERROR_COULDNT_APPROACH_FEET = (3, 'Could not approach the feet')
