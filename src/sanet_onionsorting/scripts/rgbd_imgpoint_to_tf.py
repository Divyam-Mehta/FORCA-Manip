#!/usr/bin/env python3
'''
Author: Prasanth Suresh (ps32611@uga.edu)
Owner: THINC Lab @ CS UGA

Please make sure you provide credit if you are using this code.

'''
import cv2
import time
import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import image_geometry
import message_filters
from functools import partial
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from sanet_onionsorting.srv import Yolo
from sanet_onionsorting.srv import YoloDeepsort
from sanet_onionsorting.msg import OBlobs
# import tf
# from tf import TransformListener, transformations

class Camera(Node):
    def __init__(self, camera_name='kinect', rgb_topic='/kinect2/hd/image_color_rect', depth_topic='/kinect2/hd/image_depth_rect',
                 camera_info_topic='/kinect2/hd/camera_info', choice=None, debug=False, init_node=False):
        """
        @brief      A class to obtain time synchronized RGB and Depth frames from a camera topic and find the
                    3D position of the point wrt the required frame of reference.

        @param      camera_name        Just a relevant name for the camera being used.
        @param      rgb_topic          The topic that provides the rgb image information. 
        @param      depth_topic        The topic that provides the depth image information. 
        @param      camera_info_topic  The topic that provides the camera information. 
        @param      choice             If the camera used is a real or simulated camera based on commandline arg.
        @param      debug              Opens a live stream of the camera and prints transforms of mouseclick point.
        @param      init_node          Initializes a node inside the class if True
        """

        if not init_node:
            stamp = int(time.time())
            time_4 = stamp % 10000
            digits = f"{time_4:04d}"
            unique_name = f"depth_from_object_{digits}"
            super().__init__(unique_name)

            self.declare_parameters(
                namespace = '',
                parameters =[
                    ('choice', 'real'),
                    ('camera_name', 'oak'),
                    ('debug', False),
                    ('deepsort_service', True)
                ]
            )

            choice = self.get_parameter('choice').get_parameter_value().string_value
            camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
            deepsort_service = self.get_parameter('deepsort_service').get_parameter_value().bool_value
            debug = self.get_parameter('debug').get_parameter_value().bool_value

            print("\n{} {} chosen".format(choice, camera_name))

            if (choice == "real"):
                if (camera_name == "kinect"):
                    rgb_topic = '/kinect2/hd/image_color_rect'
                    depth_topic = '/kinect2/hd/image_depth_rect'
                    camera_info_topic = '/kinect2/hd/camera_info'

                elif (camera_name == "realsense"):
                    rgb_topic = '/camera/camera/color/image_raw'
                    depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
                    camera_info_topic = '/camera/camera/color/camera_info'

                elif camera_name == 'test':
                    rgb_topic = '/kinect2/color/image_raw'
                    depth_topic = '/kinect2/depth/image_raw'
                    camera_info_topic = '/kinect2/color/camera_info'

                elif (camera_name == "oak"):
                    rgb_topic = '/oak/rgb/image_raw'
                    depth_topic = '/oak/stereo/image_raw'
                    camera_info_topic = '/oak/rgb/camera_info'

                else:
                    raise ValueError("Wrong camera name.")

            elif (choice == "gazebo"):
                if (camera_name == "kinect"):
                    rgb_topic = '/kinect_V2/rgb/image_raw'
                    depth_topic = '/kinect_V2/depth/image_raw'
                    camera_info_topic = '/kinect_V2/rgb/camera_info'

                elif (camera_name == "realsense"):
                    rgb_topic = '/camera/color/image_raw'
                    depth_topic = '/camera/depth/image_raw'
                    camera_info_topic = '/camera/color/camera_info'

                else:
                    raise ValueError("Wrong camera name.")

        if init_node:
            try:
                # print("=======================Came to init node!=============================")
                stamp = int(time.time())
                time_4 = stamp % 10000
                digits = f"{time_4:04d}"
                unique_name = f"rgbd_imgpoint_to_tf_{digits}"
                super().__init__(unique_name)
                deepsort_service = False
            except Exception as e:
                print(
                    f"Exception {e} encountered.\nNode has already been initialized, doing nothing")
       
        q = 5

        self.debug = debug
        self.camera_name = camera_name
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.camera_info_topic = camera_info_topic
        self.choice = choice
        self.deepsort_service = deepsort_service

        self.poses = []
        self.rays = []
        self.OBlobs_x = []
        self.OBlobs_y = []
        self.OBlobs_z = []
        self.OBlobs_col = []
        self.colors = []
        self.is_updated = False
        self.found_objects = False
        self.latest_rgb = None
        self.latest_depth_32FC1 = None
        self.camera_info = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_buffer.clear()
        self.br = tf2_ros.TransformBroadcaster(self)
        self.lis = tf2_ros.TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()

        self.camera_model = image_geometry.PinholeCameraModel()

        self.pose3D_pub = self.create_publisher(OBlobs, 'object_location', q)

        if self.debug:

            self.sub_rgb = message_filters.Subscriber(
                self, Image, rgb_topic)

            self.sub_depth = message_filters.Subscriber(
                self, Image, depth_topic)

            self.sub_camera_info = message_filters.Subscriber(
                self, CameraInfo, camera_info_topic)

            self.marker_pub = self.create_publisher(
                Marker, 'visualization_marker', 10)

            # self.tss = message_filters.ApproximateTimeSynchronizer([self.sub_rgb, self.sub_depth, self.sub_camera_info], queue_size=15, slop=0.4)
            self.tss = message_filters.ApproximateTimeSynchronizer(
                [self.sub_rgb, self.sub_depth, self.sub_camera_info], queue_size=q, slop=0.5)
            #self.tss = message_filters.TimeSynchronizer([sub_rgb], 10)
            self.tss.registerCallback(self.debugCallback)

        else:
            self.sub_depth = message_filters.Subscriber(
                self, Image, depth_topic)
            
            self.sub_camera_info = message_filters.Subscriber(
                self, CameraInfo, camera_info_topic)
            
            self.tss = message_filters.ApproximateTimeSynchronizer(
                [self.sub_depth, self.sub_camera_info], queue_size=q, slop=0.5)

            self.tss.registerCallback(self.callback)
            
        self.get_logger().info('Camera {} initialised, {}, {}, {}'.format(
            self.camera_name, rgb_topic, depth_topic, camera_info_topic))
        
    def service_callback(self, future):
        try:
            response = future.result()
            self.save_response(response)
        except Exception as e:
            self.get_logger().warn("Service failed: %r" % (e,))

    def callback(self, depth_msg, camera_info_msg):
        """
        @brief  Callback function for the ROS Subscriber that takes a depth image, camera intrinsics and publishes 3D poses wrt required reference frame.
        """
        self.camera_model.from_camera_info(camera_info_msg)
        self.latest_depth_32FC1 = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')
        self.camera_info = camera_info_msg
        self.is_updated = True

    def timer_callback(self):
        if self.debug:
            # print("We're in debug mode!")
            if not self.is_updated:
                self.get_logger().info("Camera is not yet updated")
                time.sleep(1)
                return
            self.save_response()
        else:
            print('\nUpdating predictions...\n')
            if self.deepsort_service:
                request = YoloDeepsort.Request()
            else:
                request = Yolo.Request()  
            future = self.client.call_async(request)       
            future.add_done_callback(partial(self.service_callback))
            print('Updated:', self.is_updated, 'Found:', self.found_objects)
            if self.is_updated and self.found_objects:
                print('publisher')
                self.OblobsPublisher()

    def save_response(self, response=None):
        '''
        @brief      A method that saves the latest bbox info.

        @param      response           The response message from the classifier, containing bounding box info.
        '''

        if self.debug:
            try:
                # cv2.namedWindow("Image window", cv2.WINDOW_NORMAL)
                cv2.imshow("Image window", self.latest_rgb)
                # cv2.imshow("depth", depth_32FC1)
                cv2.setMouseCallback("Image window", self.mouse_callback)
                cv2.waitKey(1)
            except Exception:
                self.get_logger().info('Shutting down')
                rclpy.shutdown()
                cv2.destroyAllWindows()
        else:
            if len(response.centx) == 1 and response.centx[0] == -1:
                # If no objects are found in the image
                ob = OBlobs()
                ob.x = [-100]
                ob.y = [-100]
                ob.z = [-100]
                ob.color = [-100]
                self.pose3D_pub.publish(ob)
                self.found_objects = False
                time.sleep(0.1)
                print('\nNo onions found in the frame\n')
                return
            else:
                print('\nProcessing {0} onions info\n'.format(
                    len(response.centx)))
                self.xs = response.centx
                self.ys = response.centy
                self.colors = response.color
                self.found_objects = True
                return

    def debugCallback(self, rgb_msg, depth_msg, camera_info_msg):
        """
        @brief  Callback function for the ROS Subscriber that takes time synchronized 
                RGB and Depth image and publishes 3D poses wrt required reference frame.
        """

        # print "\nWe're in Callback!!\n"
        # img =  np.frombuffer(rgb_msg.data, dtype=np.uint8).reshape(rgb_msg.height, rgb_msg.width, -1).astype('float32')
        # img = img/255

        self.camera_model.from_camera_info(camera_info_msg)
        self.latest_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        self.latest_depth_32FC1 = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')
        self.camera_info = camera_info_msg
        self.is_updated = True

        # res = kinect_utils.filter_depth_noise(depth_32FC1)
        # depth_display = kinect_utils.normalize_depth_to_uint8(depth_32FC1.copy())
        # depth_display = cv2.normalize(depth_32FC1.copy(), None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        # depth_32FC1[depth_32FC1 < 0.1] = np.finfo(np.float32).max

        # if depth_32FC1.any() != 0:
        #     print "\nNot all zeros in {} depth image\n".format(np.shape(depth_32FC1))
        #     num_zeros = np.count_nonzero(depth_32FC1==0)
        #     print "\n{} zeros in {} sized array".format(num_zeros,np.shape(depth_32FC1)[0]*np.shape(depth_32FC1)[1])
        # else: print "\nAll zeros\n"

    def OblobsPublisher(self):

        # print "\nWe're in OblobsPublisher!!\n"
        if not self.debug:
            self.convertto3D()

        # print "\nLen of poses is: ",len(self.poses)

        if len(self.poses) > 0:
            self.getCam2Worldtf()
            ob = OBlobs()
            ob.x = self.OBlobs_x
            ob.y = self.OBlobs_y
            ob.z = self.OBlobs_z
            ob.color = self.OBlobs_col
            self.pose3D_pub.publish(ob)
            time.sleep(0.1)
            self.is_updated = False
            if self.debug:
                print('\nHere are the 3D locations: \n', ob)
                print('\nCamera is_updated inside callback: ', self.is_updated)
            self.reset()
            return
        
    def reset(self):
        self.poses = []
        self.rays = []
        self.OBlobs_x = []
        self.OBlobs_y = []
        self.OBlobs_z = []
        self.OBlobs_col = []
        self.tf_buffer.clear()
        self.sub_camera_info, self.latest_rgb, self.latest_depth_32FC1 = None, None, None

    def getCam2Worldtf(self):
        """
        @brief  Transforms 3D point in the camera frame to world frame 
                by listening to static transform between camera and world.
        """
        if self.debug:
            print("\n Camera frame is: ", self.get_tf_frame())
        for i in range(len(self.poses)):

            camerapoint = tf2_geometry_msgs.tf2_geometry_msgs.PoseStamped()
            camerapoint.header.frame_id = self.get_tf_frame()
            camerapoint.header.stamp = Time(seconds=0)
            camerapoint.pose.position.x = self.poses[i][0]
            camerapoint.pose.position.y = self.poses[i][1]
            camerapoint.pose.position.z = self.poses[i][2]
#            tf_buffer = tf2_ros.Buffer(
#                rospy.Duration(1200.0))  # tf buffer length
#            tf_listener = tf2_ros.TransformListener(tf_buffer)
            try:
                cam_to_root_tf = self.tf_buffer.lookup_transform("root",  # target frame
                                                                 self.get_tf_frame(),  # source frame
                                                                 # get the tf at first available time
                                                                 Time(seconds=0),   #v#v#v#v#v#v#v#v#
                                                                 Duration(seconds=1.0))  # wait for 1 second for target frame to become available
                tf_point = tf2_geometry_msgs.do_transform_pose(
                    camerapoint.pose, cam_to_root_tf)
            except tf2_ros.TransformException as e:
                self.get_logger().warn(
                    f"Could not transform from frame_id: {camerapoint.header.frame_id} to world frame: root because {e} occured")
                return

            if self.debug:
                print("\nCamerapoint: \n", camerapoint)
                print('\n3D pose wrt world: ', tf_point)

            self.OBlobs_x.append(tf_point.position.x)
            self.OBlobs_y.append(tf_point.position.y)
            self.OBlobs_z.append(tf_point.position.z)
            # Error when running in debug mode since colors comes from service (JK)
            if not self.debug:
                self.OBlobs_col.append(self.colors[i])
        return

    def get_current_rect_image(self):
        """
        @brief  Takes in a raw image

        @return Rectified image.
        """
        output_img = np.ndarray(self.get_current_raw_image().shape)
        self.camera_model.rectify_image(
            self.get_current_raw_image(), output_img)
        return output_img

    def get_tf_frame(self):
        """
        @return camera transform frame.
        """
        return self.camera_model.get_tf_frame()

    def is_ready(self):
        """
        @return status.
        """
        return self.ready_

    def get_ray(self, uv_rect):
        """
        @brief  Takes in a 2D point on RGB image.

        @return Orthogonal projection.
        """
        # NOTE: This function takes the u,v value from the image, assumes z = 1 (since unit vector outwards), then
        # uses the centx,centy and fx,fy of the camera to find x,y value of the ray. Norm of the whole thing is
        # norm = sqrt(x^2 + y^2 + z^2). Then returns (x/norm, y/norm, z/norm)
        return self.camera_model.project_pixel_to_3d_ray(self.camera_model.rectify_point(uv_rect))

    def mkmat(self, rows, cols, L):
        mat = np.matrix(L, dtype='float64')
        mat.resize((rows, cols))
        return mat

    def get_position_from_ray(self, ray, depth):
        """
        @brief      The 3D position of the object (in the camera frame) from a camera ray and depth value

        @param      ray    The ray (unit vector) from the camera centre point to the object point
        @param      depth  The norm (crow-flies) distance of the object from the camera

        @return     The 3D position of the object in the camera coordinate frame
        """
        # NOTE: What we do here is unnormalize what projectPixelTo3dRay() gives us (ray[0] = x/norm, ray[1] = y/norm and ray[2] is just 1/norm),
        # then multiply these values by actual depth to get the 3D pose value wrt camera frame of reference.
        # x = (ray[0] * depth) / ray[2]
        # y = (ray[1] * depth) / ray[2]
        # z = (ray[2] * depth) / ray[2]
        # print(f"Ray x: {x}, y: {y}, z: {z}")
        return [(i * depth) / ray[2] for i in ray]

    def generate_marker(self, stamp, frame_id, pose_3D):
        """
        @brief      Generates a marker of a given shape around the point on the image.

        @param      stamp    Header stamp of the marker message
        @param      frame_id  Header frame id
        @param      pose_3D   The 3D position of the point considered

        @return     Marker message to be published.
        """
        # marker_msg = Marker()
        # marker_msg.header.stamp = stamp
        # marker_msg.header.frame_id = frame_id
        # marker_msg.id = 0 #Marker unique ID

        # ARROW:0, CUBE:1, SPHERE:2, CYLINDER:3, LINE_STRIP:4, LINE_LIST:5, CUBE_LIST:6, SPHERE_LIST:7, POINTS:8, TEXT_VIEW_FACING:9, MESH_RESOURCE:10, TRIANGLE_LIST:11
        # marker_msg.type = 2
        # marker_msg.lifetime = 1
        # marker_msg.pose.position = pose_3D

        marker_msg = Marker()
        marker_msg.header.frame_id = frame_id
        marker_msg.type = marker_msg.SPHERE
        marker_msg.action = marker_msg.ADD
        marker_msg.scale.x = 0.2
        marker_msg.scale.y = 0.2
        marker_msg.scale.z = 0.2
        marker_msg.color.a = 1.0
        marker_msg.color.r = 1.0
        marker_msg.color.g = 1.0
        marker_msg.color.b = 0.0
        marker_msg.pose.orientation.w = 1.0
        magicval_1 = 1.7
        marker_msg.pose.position.x = pose_3D[0]
        marker_msg.pose.position.y = pose_3D[1]
        marker_msg.pose.position.z = pose_3D[2]
        marker_msg.id = 1

        return marker_msg

    def process_ray(self, uv_rect, depth):
        """
        @param      uv_rect   Rectified x,y pose from RGB Image.
        @param      depth     Corresponding depth value.

        @return     Orthogonal ray and 3D pose wrt camera.
        """
        ray = self.get_ray(uv_rect)
        pose = self.get_position_from_ray(ray, depth)
        # x = (uv_rect[0] - self.camera_model.cx())/self.camera_model.fx() * depth; # This is what projectPixelTo3dRay() does.
        # y = (uv_rect[1] - self.camera_model.cy())/self.camera_model.fy() * depth; # But assumes z = 1, since it returns a unit vector.
        # z = depth;
        # print(f"Ray: {ray}")
        # print(f"Pose: {pose}, Calculated: {x},{y},{z}")
        # rospy.sleep(100)
        return ray, pose

    def mouse_callback(self, event, x, y, flags, param):
        """
        @brief  Callback function for mouse left click. Calculates 3D
                pose for the clicked point.

        @param  event   Here the event would be a mouse left click.
        @param  x,y     The corresponding x,y values from the image.
        @param  flags   Any flags raised about the event.
        @param  param   Any params passed with it.
        """

        if self.latest_depth_32FC1 is not None:
            if self.camera_info.height != self.latest_depth_32FC1.shape[0] or self.camera_info.width != self.latest_depth_32FC1.shape[1]:
                x_ratio = self.latest_depth_32FC1.shape[1]/self.latest_rgb.shape[1]
                y_ratio = self.latest_depth_32FC1.shape[0]/self.latest_rgb.shape[0]
                x = x_ratio * x
                y = y_ratio * y

        if event == cv2.EVENT_LBUTTONDOWN:

            # clamp a number to be within a specified range
            def clamp(n, minn, maxn): return max(min(maxn, n), minn)

            # Small ROI around clicked point grows larger if no depth value found
            for bbox_width in range(20, int(self.latest_depth_32FC1.shape[0]/3), 5):
                tl_x = int(clamp(x-bbox_width/2, 0,
                                 self.latest_depth_32FC1.shape[1]))
                br_x = int(clamp(x+bbox_width/2, 0,
                                 self.latest_depth_32FC1.shape[1]))
                tl_y = int(clamp(y-bbox_width/2, 0,
                                 self.latest_depth_32FC1.shape[0]))
                br_y = int(clamp(y+bbox_width/2, 0,
                                 self.latest_depth_32FC1.shape[0]))
                # print('\n x, y, tl_x, tl_y, br_x, br_y: ',(x, y), (tl_x, tl_y, br_x, br_y))
                roi = self.latest_depth_32FC1[tl_y:br_y, tl_x:br_x]
                depth_distance = np.median(roi)

                if not np.isnan(depth_distance):
                    break

            self.ray, self.pose = self.process_ray((x, y), depth_distance)
            if self.debug:
                print('distance (crowflies) from camera to point: {:.2f}m'.format(
                    depth_distance))
                print("\n3D pose wrt camera: \n", self.pose)
            if self.choice == "real":
                ''' NOTE: The Real Kinect produces values in mm while ROS operates in m. '''
                self.poses.append(np.array(self.pose)/1000)
                self.OblobsPublisher()
            else:
                self.poses.append(self.pose)
                self.OblobsPublisher()

    def convertto3D(self):
        """
        @brief  Converts the point(s) provided as input into 3D coordinates wrt camera.
        """
        # print "\nWe're in convertto3D!!\n"
        # clamp a number to be within a specified range
        def clamp(n, minn, maxn): return max(min(maxn, n), minn)
        depth_distances = []
        # Small ROI around clicked point grows larger if no depth value found

        if self.camera_info.height != self.latest_depth_32FC1.shape[0] or self.camera_info.width != self.latest_depth_32FC1.shape[1]:
            x_ratio = self.latest_depth_32FC1.shape[1]/self.camera_info.width
            y_ratio = self.latest_depth_32FC1.shape[0]/self.camera_info.height
            self.xs = [x_ratio * x for x in self.xs]
            self.ys = [y_ratio * y for y in self.ys]

        for i in range(len(self.xs)):
            roi = []
            for bbox_width in range(5, int(self.latest_depth_32FC1.shape[0]/3), 5):
                tl_x = int(clamp(self.xs[i]-bbox_width/2,
                                 0, self.latest_depth_32FC1.shape[1]))
                br_x = int(clamp(self.xs[i]+bbox_width/2,
                                 0, self.latest_depth_32FC1.shape[1]))
                tl_y = int(clamp(self.ys[i]-bbox_width/2,
                                 0, self.latest_depth_32FC1.shape[0]))
                br_y = int(clamp(self.ys[i]+bbox_width/2,
                                 0, self.latest_depth_32FC1.shape[0]))
                roi = (self.latest_depth_32FC1[tl_y:br_y, tl_x:br_x]).copy()

                roi = np.ma.masked_equal(roi, 0)
                # rospy.sleep(100)
                if np.any(roi):
                    # print(np.max(roi), np.min(roi))
                    # depth_distances.append((np.max(roi) + np.min(roi)) / 2.0)   # This is going a little shorter when the camera is behind the robot
                    depth_distances.append(np.mean(roi))
                    # print "\nGot Depth\n"
                if self.camera_name == "realsense":
                    if not np.isnan(depth_distances).any():
                        break
                else:
                    if np.min(roi) > 0.0:
                        # print "\nNo Nan values in depth values\n"
                        break

        # print('distance (crowflies) from camera to point: {}m'.format(depth_distances))
        for i in range(len(depth_distances)):
            # print "\nTotal depths: ",len(depth_distances)
            # print "\ni = ",i
            ray, pose = self.process_ray(
                (self.xs[i], self.ys[i]), depth_distances[i])
            if self.choice == "real":
                ''' NOTE: The Real Kinect produces values in mm while ROS operates in m. '''
                self.rays.append(np.array(ray)/1000)
                self.poses.append(np.array(pose)/1000)
            else:
                if (self.camera_name == "kinect"):
                    self.rays.append(ray)
                    self.poses.append(pose)
                else:
                    self.rays.append(np.array(ray)/1000)
                    self.poses.append(np.array(pose)/1000)
        # print '\n(x,y): ',self.xs,self.ys
        # print '\n3D pose: ', self.poses

def main(args=None):
    """
    @brief  A ROS Node to transform a 2D point(s) on an image to its 3D world coordinates
            using RGB and Depth values obtained from an RGBD camera.
    """
    rclpy.init(args=args)
    # try:
    node = Camera()
    node.create_timer(0.4, node.timer_callback)

    if not node.debug:    
        if node.deepsort_service:
            node.client = node.create_client(YoloDeepsort, '/get_predictions')
        else:
            node.client = node.create_client(Yolo, '/get_predictions')
        while not node.client.wait_for_service(4.0):
            node.get_logger().info("Waiting for Service ...")

    rclpy.spin(node)
    rclpy.shutdown

    # except Exception as e:
    #     print('RGBD Failed: %r' %(e,))

    # except KeyboardInterrupt:
    #     print("  Keyboard Interrupt, shutting down")

if __name__ == '__main__':
    main()
