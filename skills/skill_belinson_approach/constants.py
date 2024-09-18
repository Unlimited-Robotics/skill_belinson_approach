APPROACH_FACE_CAMERA = 'nav_bottom',
APPROACH_FEET_CAMERA = 'nav_top',

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

FEET_DETECTION_THRESHOLD = 0.6
FEET_SIZE_THRESHOLD = 0.04
DISTANCE_CONST = 0.53
FEET_DIST_BEFORE_STOP = 0.2

### ERROR MESSAGES ###
ERROR_NO_FACES_DETECTED = (1, 'No faces detected')
ERROR_COULDNT_APPROACH_FACE = (2, 'Could not navigate to the face')
ERROR_COULDNT_APPROACH_FEET = (3, 'Could not approach the feet')