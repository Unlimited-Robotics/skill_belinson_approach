# Raya Imports
from raya.skills import RayaSkillHandler, RayaFSMSkill
from raya.controllers import *
from raya.controllers.navigation_controller import POSITION_UNIT, ANGLE_UNIT
from raya.exceptions import *

# Filesystem Imports
from .constants import *

# Other Imports
import math
import numpy as np
import tf_transformations
from geometry_msgs.msg import PointStamped, Point, Pose, Quaternion
from collections import deque


class SkillBelinsonApproach(RayaFSMSkill):

    ### Arguments ###
    DEFAULT_SETUP_ARGS = {'fsm_log_transitions':True,
                          'only_face' : False}
    REQUIRED_SETUP_ARGS = {'map_name'}
    DEFAULT_EXECUTE_ARGS = {'distance_to_goal' : 1.3}
    REQUIRED_EXECUTE_ARGS = {'face_angle'}

    ### Skill ###
    STATES = [
        'DETECT_FACE',
        'SCAN_FOR_DETECTIONS',
        'APPROACH_FACE',
        'DETECT_FEET',
        'APPROACH_FEET_CV',
        'APPROACH_FEET_LIDAR',
        'IDLE',
        'END',
    ]

    INITIAL_STATE = 'DETECT_FACE'

    END_STATES = [
        'END'
    ]
    
    STATES_TIMEOUTS = {}


    ### Application ###
    async def setup(self):

        # Init skill controllers
        self.log.info('Enabling skill controllers...')
        await self.enable_skill_controllers()

        # Setup variables
        self.setup_variables()

        # Init detection models
        self.log.info('Enabling face detection model...')
        self.face_detector = await self.cv.enable_model(**FACE_DETECTOR_PARAMS)
        self.log.info('Enabling feet detection model...')
        self.feet_detector = await self.cv.enable_model(**FEET_DETECTOR_PARAMS)   

        # Localize
        self.log.info(f"Localizing in map - {self.setup_args['map_name']}")
        await self.nav.set_map(
                        map_name= self.setup_args['map_name'],
                        wait_localization = True,
                        wait = True
                        )

    async def finish(self):
        pass


    # =============================== Actions =============================== #
    async def enter_DETECT_FACE(self):
        self.last_state = 'DETECT_FACE'

        # Initiate detector and give it 3 seconds to detect faces
        self.face_detector.set_img_detections_callback(
            callback = self.callback_all_faces,
            as_dict = True,
            call_without_detections = True,
            cameras_controller = self.cameras
        )
        self.face_detections = {}
        await self.sleep(3.0)


    async def enter_SCAN_FOR_DETECTIONS(self):
        self.last_state = 'SCAN_FOR_DETECTIONS'

        # Take predefined rotation params
        rotation_params = SCAN_FOR_FACES_ROTATION_PARAMS.copy()

        # Start scanning left and right to find faces
        for i in range(2):
            if self.face_detections:
                break
            sign = 1 if (i+1)%2 == 0 else 1
            rotation_params['angular_speed'] *= sign*(i+1)
            await self.motion.rotate(**rotation_params)


    async def enter_APPROACH_FACE(self):
        self.last_state = 'APPROACH_FACE'

         # Calculate navigation point to face
        current_detection = self.face_detections[0]
        face_position = current_detection['center_point_map']

        goal_predict = Pose()
        goal_predict.position = Point(x = face_position[0],
                                    y = face_position[1],
                                    z = face_position[2])
        target_angle = self.inverse_angle(self.execute_args['face_angle'])
        quat = tf_transformations.quaternion_from_euler(        
                                                0.0,
                                                0.0,
                                                np.deg2rad(target_angle)
                                                )
        goal_predict.orientation = Quaternion(x=quat[0], y=quat[1],
                                                z=quat[2], w=quat[3]
                                                )

        projected_point = self.get_projected_point(
                    detection_pose = goal_predict,
                    distance = self.execute_args['distance_to_goal']
                    ).point
        
        # Check if the path to the calculated point is available
        path_available = await self.check_path_available(
                                                projected_point.x,
                                                projected_point.y,
                                                self.execute_args['face_angle']
                                                )

        # Compute the desired coordinates to navigate to
        if path_available and not self.nav.is_navigating():
            path_available = False

            # Navigate towards the patient
            try:
                await self.navigation_sequence(
                    x = projected_point.x,
                    y = projected_point.y,
                    angle = self.execute_args['face_angle'],
                    pos_unit = POSITION_UNIT.METERS,
                    ang_unit = ANGLE_UNIT.DEGREES,
                    wait = True,
                    callback_feedback = self.cb_nav_feedback,
                    callback_finish = self.cb_nav_finish,
                    close_to_position = self.close_to_position
                )
                self.face_approach_success = True

            except Exception as e:
                self.face_approach_success = False
        else:
            self.face_approach_success = False

    async def enter_DETECT_FEET(self):
        self.last_state = 'DETECT_FEET'

        # Create a queue
        self.feet_queue = deque(maxlen=5)

        # Initiate detector and give it 3 seconds to detect feet
        self.feet_detector.set_img_detections_callback(
                callback = self.cb_stinky_feet,
                as_dict = True,
                call_without_detections = True,
                cameras_controller = self.cameras
            )
        await self.sleep(3.0)


    async def enter_APPROACH_FEET_CV(self):
        self.last_state = 'APPROACH_FEET_CV'

        # Move forwards as long as you detect feet
        while self.feet_detected:
            try:
                await self.motion.set_velocity(
                    x_velocity = 0.03,
                    y_velocity = 0.0,
                    angular_velocity = 0.0,
                    duration = 10.0,
                    enable_obstacles = False,
                    wait = True,
                    callback_feedback_async = self.async_cb_feet,
                    callback_finish_async = self.async_cbf_feet
                )

                # Stop moving if you're too close
                if self.distance_to_feet < FEET_DIST_BEFORE_STOP:
                    break

            except Exception as e:
                self.log.warn(f'linear movement failed, error: {e}')
                await self.sleep(0.1)
        

        # When the feet are no longer detected, compute the final distance
        # to move forwards
        try:
            unweighted_distance = np.mean(self.feet_queue)
            weighted_distance = self.check_final_queue(self.feet_queue)

            lidar_data = await self.get_lidar_data(**LIDAR_SCAN_PARAMS)
            
            if type(weighted_distance) is not int:
                weighted_distance = unweighted_distance

            distance_coeff = min(lidar_data) - DISTANCE_CONST

            self.log.debug(f'distance offset: {(distance_coeff - weighted_distance)/distance_coeff}')

            if abs((distance_coeff - weighted_distance)/distance_coeff) > MAX_DISTANCE_OFFSET_PERCENTAGE:
               
                self.log.warn(f'Feet detection was pretty far..')
                self.log.warn('Using lidar based distance for safety')
                self.feet_bad_detection = True
                return

            self.log.warn(f'Moving final distance - {weighted_distance}')
            await self.motion.move_linear(
                distance = weighted_distance,
                x_velocity = 0.03,
                wait = True,
                enable_obstacles = False
            )

            self.feet_approach_success = True 
                
        except Exception as e:
            self.feet_approach_success = False



    async def enter_APPROACH_FEET_LIDAR(self):
        self.last_state = 'APPROACH_FEET_LIDAR'

        # Set obstacle detection to false
        self.obstacle_detected = False

        # Create obstacle listener
        self.lidar.create_obstacle_listener(
                listener_name = 'obstacle',
                callback = self.callback_obstacle,
                lower_angle = -10,
                upper_angle = 10,
                upper_distance = DISTANCE_CONST, 
                ang_unit=ANGLE_UNIT.DEGREES,
            )
        
        while not self.obstacle_detected:
            try:
                await self.motion.set_velocity(
                    x_velocity = 0.015,
                    y_velocity = 0.0,
                    angular_velocity = 0.0,
                    duration = 10.0,
                    enable_obstacles = False,
                    wait = True,
                    callback_feedback_async = self.async_cb_lidar,
                    callback_finish_async = self.async_cbf_lidar
                )

            except Exception as e:
                self.log.warn(f'linear movement failed, error: {e}')
                await self.sleep(0.1)

    
    async def enter_IDLE(self):
        self.log.warn(f'Reverting to laste state: {self.last_state}')

    # ============================= Transitions ============================= #
    async def transition_from_DETECT_FACE(self):
        if self.face_detections:
            await self.send_feedback(
                        {'skill_success' : None,
                        'status_msg' : MSGS_DICT['DETECT_FACE']['success']})
            self.set_state('APPROACH_FACE')

        elif self.face_detection_attempts < MAX_DETECTION_ATTEMPTS:
            self.face_detection_attempts += 1
            self.set_state('SCAN_FOR_DETECTIONS')

        else:
            await self.send_feedback(
                            {'skill_success' : False,
                            'status_msg' : MSGS_DICT['DETECT_FACE']['failure']
                            }
                        )
            self.set_state('END')



    async def transition_from_SCAN_FOR_DETECTIONS(self):
        if self.face_detections:
            await self.send_feedback(
                        {'skill_success' : None,
                        'status_msg' : MSGS_DICT['DETECT_FACE']['success']})
            self.set_state('APPROACH_FACE')
        
        else:
            self.set_state('DETECT_FACE')



    async def transition_from_APPROACH_FACE(self):
        if self.face_approach_success and self.setup_args['only_face']:
            await self.send_feedback(
                        {'skill_success' : True,
                        'status_msg' : MSGS_DICT['ONLY_FACE']['success']})
            self.set_state('END')

        elif self.face_approach_success:
            await self.send_feedback(
                        {'skill_success' : None,
                        'status_msg' : MSGS_DICT['APPROACH_FACE']['success']})
            self.set_state('DETECT_FEET')
        
        elif self.face_detection_attempts < MAX_APPROACH_ATTEMPTS:
            self.close_to_position = True
            self.set_state('IDLE')

        else:
            await self.send_feedback(
                            {'skill_success' : False,
                            'status_msg' : MSGS_DICT['APPROACH_FACE']['failure']
                            }
                        )
            self.set_state('END')


    async def transition_from_DETECT_FEET(self):
        if self.feet_detected:
            await self.send_feedback(
                        {'skill_success' : None,
                        'status_msg' : MSGS_DICT['DETECT_FEET']['success']})
            self.set_state('APPROACH_FEET_CV')
        else:
            self.set_state('APPROACH_FEET_LIDAR')


    async def transition_from_APPROACH_FEET_CV(self):
        if self.feet_bad_detection:
            self.set_state('APPROACH_FEET_LIDAR')

        elif self.feet_approach_success:
            await self.send_feedback(
                    {'skill_success' : True,
                    'status_msg' : MSGS_DICT['APPROACH_FEET_CV']['success']
                    }
                )
            await self.cv.disable_all_models()
            self.set_state('END')
        else:
            await self.send_feedback(
                        {'skill_success' : False,
                        'status_msg' : MSGS_DICT['APPROACH_FEET_CV']['failure']
                        }
                    )
            self.set_state('END')


    async def transition_from_APPROACH_FEET_LIDAR(self):
        await self.send_feedback({'skill_success' : True,
                                  'status_msg' : None})
        await self.cv.disable_all_models()
        self.set_state('END')


    async def transition_from_IDLE(self):
        self.set_state(str(self.last_state))


    # =============================== Helpers =============================== #

    async def enable_skill_controllers(self):
        # Enable controllers
        self.ui: UIController = await self.get_controller('ui')
        self.log.info('UI controller - Enabled')
        self.log.info('Leds controller - Enabled')
        self.nav: NavigationController = await self.get_controller('navigation')
        self.log.info('Navigation controller - Enabled')
        self.motion: MotionController = await self.get_controller('motion')
        self.log.info('Motion controller - Enabled')
        self.cameras: CamerasController = await self.get_controller('cameras')
        self.log.info('Cameras controller - Enabled')
        self.cv: CVController = await self.get_controller('cv')
        self.log.info('CV controller - Enabled')
        self.lidar = await self.get_controller('lidar')
        self.log.info('Lidar controller - Enabled')

        # Enable cameras
        await self.cameras.enable_camera(APPROACH_FACE_CAMERA)
        await self.cameras.enable_camera(APPROACH_FEET_CAMERA)


    def setup_variables(self):
        self.face_detection_attempts = 0
        self.feet_detection_attempts = 0
        self.feet_detected = False
        self.feet_bad_detection = False
        self.close_to_position = False
        self.last_state = None


    async def get_lidar_data(self, lower_angle, upper_angle):
        # Get laser info and raw data
        laser_info = self.lidar.get_laser_info()
        raw_data = self.lidar.get_raw_data()

        # Filter data in the requested angles
        min_index = int(math.ceil( 
            (lower_angle - laser_info['angle_min']) / laser_info['angle_increment'] 
        ))
        max_index = int(math.floor(
            (upper_angle - laser_info['angle_min']) / laser_info['angle_increment'] 
        ))
        
        return raw_data[min_index:max_index] 
            

    def calculate_distance_img_center(self,
                                    detection: dict,
                                    image: np.array,
                                    only_y: bool = False
                                    ):
        
        '''
        Calculate the distance of a detection from the image center
        INPUTS
            detection - a dictionary with the detection's info
            image - the image on which the detection was obtained
            only_y - whether the distance should be calculated only on the y
                    axis or not
        
        OUTPUTS
            distance - the distance of the detection from the image's center
        '''
        # Calculate the center of the image
        image_center_x, image_center_y = image.shape[0]/2, image.shape[1]/2

        # Calculate the center of the face
        face_center_x = detection['object_center_px'][0]
        face_center_y = detection['object_center_px'][1]

        # Calculate the distance to the image center
        if only_y:
            distance = abs(face_center_y - image_center_y)
            return distance
        
        distance = math.sqrt((face_center_x - image_center_x)**2 + \
                                        (face_center_y - image_center_y)**2)
        
        return distance
    

    def inverse_angle(self,
                      angle: int
                      ):
        '''Get the inverse of an angle'''
        inverse = (angle + 180) % 360
        if inverse < 0:
            inverse += 360
        return inverse


    def get_projected_point(self,
                            detection_pose: Pose,
                            distance: float
                            ):
        '''
        Get x y coordinates to navigate to in order to be "distance" away
        from the detection
        INPUTS
            detection_pose - pose of the detection in quanternion
            distance - desired distance between the robot and the detection
        
        OUTPUTS
            projected_point - an x, y coordinate
        '''
        det_x = detection_pose.position.x
        det_y = detection_pose.position.y
        quaternion = (
            detection_pose.orientation.x,
            detection_pose.orientation.y,
            detection_pose.orientation.z,
            detection_pose.orientation.w
        )
        euler = tf_transformations.euler_from_quaternion(quaternion)
        tag_orientation = euler[2]  

        point_x = det_x + distance * math.cos(tag_orientation)
        point_y = det_y + distance * math.sin(tag_orientation)

        projected_point = PointStamped()
        projected_point.point = Point(x = point_x, y = point_y, z = 0.0)

        return projected_point     


    async def check_path_available(self,
                                   x,
                                   y,
                                   angle,
                                   pos_unit = POSITION_UNIT.METERS,
                                   ang_unit = ANGLE_UNIT.DEGREES
                                   ):
        try:
            await self.nav.navigate_to_position( 
                x = float(x), 
                y = float(y), 
                angle = float(angle), 
                pos_unit = pos_unit, 
                ang_unit = ang_unit,
                callback_feedback = self.cb_nav_feedback,
                callback_finish = self.cb_nav_finish,
                options={"behavior_tree": "compute_path"},
                wait=True,
            )
            self.log.debug(f'path {x, y, angle} available!')
            return True
        except Exception as e:
            self.log.debug(f'path {x, y, angle} not available!')
            return False    
        

    def check_final_queue(self,
                          queue: deque
                          ):
        '''
        Compute weighted average using a gaussian filter
        '''

        # Get mean and std, initiate params
        queue_mean = np.mean(queue)
        queue_std = np.std(queue)

        weights = np.exp(-0.5 * ((queue - queue_mean) / (queue_std)) ** 2)
        normalized_weights = weights / weights.sum()
        weighted_average = np.sum(queue * normalized_weights)
        
        return weighted_average
    

    async def navigation_sequence(self,
                            x,
                            y,
                            angle,
                            pos_unit,
                            ang_unit,
                            wait,
                            callback_feedback,
                            callback_finish,
                            close_to_position = False,
                            max_radius = 0.8
                            ):
        
        if not close_to_position:
            await self.nav.navigate_to_position(
                x = x,
                y = y,
                angle = angle,
                pos_unit = pos_unit,
                ang_unit = ang_unit,
                wait = wait,
                callback_feedback = callback_feedback,
                callback_finish = callback_finish
            )
        
        else:

            self.app.log.debug('/'*50)
            self.app.log.debug(f'Requested position: {x}, {y}, {angle}')

            path_available = False
            dx = 1 if pos_unit == POSITION_UNIT.PIXELS else 0.05
            dy = 1 if pos_unit == POSITION_UNIT.PIXELS else 0.05
            sign = 1

            for xi in range(5):
                for yi in range(5):

                    if path_available:
                        break

                    new_x = x + xi*sign*dx
                    new_y = x+ yi*sign*dy
                    
                    path_available = await self.check_path_available(
                                        x = new_x,
                                        y = new_y,
                                        angle = angle,
                                        pos_unit = pos_unit,
                                        ang_unit = ang_unit,
                                        callback_feedback = callback_feedback,
                                        callback_finish = callback_finish
                                        )
                    sign *= -1

            if path_available:
                await self.nav.navigate_to_position(
                x = new_x,
                y = new_y,
                angle = angle,
                pos_unit = pos_unit,
                ang_unit = ang_unit,
                wait = wait,
                callback_feedback = callback_feedback,
                callback_finish = callback_finish
            )
                
            else:
                raise RayaNoPathToGoal
        
    # =============================== Callbacks =============================== #

    def callback_all_faces(self, detections, image):
        if detections:
            self.face_detections = sorted(detections,
                key = lambda x: (self.calculate_distance_img_center(
                                    x, image,
                                    only_y = True),
                                    -x['confidence'])
                )
            self.face_detections = [self.face_detections[0]]


    def cb_nav_feedback(self, error, error_msg, distance_to_goal, speed):
        pass


    def cb_nav_finish(self, error, error_msg):
        pass


    def cb_stinky_feet(self, predictions, image):
        '''Callback used to obtain predictions'''
        self.current_image = image                        
        if predictions:

            # Sort predictions based on - 
            # 1. distance from image center
            # 2. distance from the robot
            # 3. prediction confidence
            sorted_detections = sorted(
                                predictions,
                                key = lambda x: \
                                    (self.calculate_distance_img_center(
                                                    x, image, only_y = True),
                                        x['distance'],
                                        -x['confidence']
                                        )
                                    )

            # Filter the predictions based on threshold and size
            filtered_predictions = [item for item in sorted_detections if \
                    item['confidence'] >= FEET_DETECTION_THRESHOLD \
                            and 0.01 < (item['height'] * item['width']) \
                                                        < FEET_SIZE_THRESHOLD]
            

            # Take the first 2 items from the filtered predictions. They are
            # assumed to be the feet of the patient
            if len(filtered_predictions) > 0:
                if len(filtered_predictions) > 2:
                    filtered_predictions = filtered_predictions[:2]
                self.distance_to_feet = \
                    filtered_predictions[0]['center_point'][0] - DISTANCE_CONST
                self.bbox_ymin = filtered_predictions[0]['y_min']
                self.feet_queue.append(self.distance_to_feet)
                self.feet_detected = True
            else:
                self.feet_detected = False

        else:
            self.feet_detected = False

    def callback_obstacle(self):
        self.obstacle_detected = True


    async def async_cb_feet(self, arg1, arg2, arg3, arg4):
        if not self.feet_detected:
            await self.motion.cancel_motion()


    async def async_cbf_feet(self, arg1, arg2, arg3, arg4):
        pass


    async def async_cb_lidar(self, arg1, arg2, arg3, arg4):
        if self.obstacle_detected:
            await self.motion.cancel_motion()


    async def async_cbf_lidar(self, arg1, arg2, arg3, arg4):
        pass