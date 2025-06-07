#!/usr/bin/env python3

import rclpy
import sys
from rclpy.node import Node
from pymavlink import mavutil
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
from cv2 import aruco
import numpy as np

from tf_transformations import euler_from_quaternion, quaternion_from_euler
import time
import math

class Drone(Node):
    def __init__(self):
        super().__init__('controller')
        self.get_logger().info("controller initialized...")

        self.master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.master.wait_heartbeat()

        self.set_mode()

        self.marker_setpoint = [300.0, -10.0, 0.0]  
        self.current_pos = None
        self.current_alt = None
        self.reached = False
        self.reachedX = False
        self.reachedY = False
        self.takeoff = False
        self.landed = False
        self.target_altitude = 1.0

        self.Kp = [0.03, 0.03]
        self.Ki = [0.001, 0.001]
        self.Kd = [0.005, 0.005]

        self.prev_errorX = 0.0
        self.prev_errorY = 0.0
        self.integralX = 0.0
        self.integralY = 0.0
        self.prev_time = self.get_clock().now()

        self.bridge = CvBridge()
        self.roi_size = 100  
        self.marker_centered = False
        self.marker_offset = [0, 0]


        self.subscription = self.create_subscription(
            Image,
            '/drone/gimbal_camera/image_raw',
            self.colorimagecb,
            10
        )


        self.timer1 = self.create_timer(0.1, self.pid)
        self.timer2 = self.create_timer(0.05,self.collectdata)

    # -------------------------------------------------------------------------------------------------------------------------------------------------------------

    def curr_position_callback(self, msg):
        self.current_pos = msg

    def rel_alt_callback(self, msg):
        self.current_alt = msg.data

    def colorimagecb(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            self.aruco_detect(frame)


        except Exception as e:
            self.get_logger().error(f"Error in colorimagecb: {e}")

    def set_mode(self):
        mode = self.master.mode_mapping()['GUIDED']
        self.master.set_mode(mode)

    def set_takeoff(self,target_altitude):
        # self.master.mav.set_attitude_target_send(
        #                             0,  # system time in milliseconds
        #                             self.settings.target_system,  # target system
        #                             0,            # target component
        #                             mask,         # type mask
        #                             att_target,   # quaternion attitude
        #                             radians(roll_rate),    # body roll rate
        #                             radians(pitch_rate),   # body pitch rate
        #                             radians(yaw_rate),     # body yaw rate
        #                             thrust)       # thrust
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0, 0, 0, 0, 0, 0, 0, target_altitude)
        threshold = 0.15
        if abs(abs(self.current_alt) - target_altitude)<=threshold or abs(self.current_alt) > target_altitude:
            # print(abs(abs(self.current_alt) - target_altitude))
            self.get_logger().info("reached the targeted altitude...")
            self.takeoff = True
        else:
            self.get_logger().info("wait till getting to the targeted altitude...")
            
        


    def disarm(self):
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,mavutil.mavlink.MAV_CMD_NAV_LAND,0, 0, 0, 0, 0, 0, 0, 0)
        threshold = 0.05
        if abs(self.current_alt)<=threshold:
            self.get_logger().info("LANDED...")
            self.landed = True
        else:
            self.get_logger().info("Landing...")
    

    def arm(self):
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0, 1, 0, 0, 0, 0, 0, 0)
        self.master.motors_armed_wait()
        return True
    
    def euclidean_distance(self,from_coordinates,to_coordinates):
        dx = to_coordinates[0] - from_coordinates[0]
        dy = to_coordinates[1] - from_coordinates[1]

        return math.sqrt(dx**2 + dy**2), math.atan2(dy, dx)
    
    def set_velocity(self, vx, vy, vz):
        type_mask = 0b0000111111000111
        self.master.mav.set_position_target_local_ned_send(0, self.master.target_system, self.master.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, type_mask, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)

        
    # def set_orientation(self, yaw):
    #     # yaw angle 
    #     self.master.mav.command_long_send(
    #             self.settings.target_system,  # target_system
    #             self.settings.target_component, # target_component
    #             mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
    #             0, # confirmation
    #             yaw, # param1 (angle value)
    #             1.0, # param2 (angular speed value)
    #             0, # param3
    #             0, # param4 (mode: 0->absolute / 1->relative)
    #             0, # param5
    #             0, # param6
    #             0) # param7



    # -------------------------------------------------------------------------------------------------------------------------------------------------------------

    def aruco_detect(self, frame):

        bilateral_filter = cv.bilateralFilter(frame, d=9, sigmaColor=75, sigmaSpace=75) 

        gray_image = cv.cvtColor(bilateral_filter, cv.COLOR_BGR2GRAY)

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        aruco_parameter = aruco.DetectorParameters()
        
        marker_corners, marker_IDs , reject = aruco.detectMarkers(gray_image,aruco_dict,parameters=aruco_parameter)

        height, width, _ = frame.shape
        cx_roi, cy_roi = width // 2, height // 2

        roi_half = self.roi_size // 2
        self.roi_box = [(cx_roi - roi_half, cy_roi - roi_half), (cx_roi + roi_half, cy_roi + roi_half)]
        if not self.marker_centered:
            cv.rectangle(frame, self.roi_box[0], self.roi_box[1], (0, 0, 255), 2)
        else:
            cv.rectangle(frame, self.roi_box[0], self.roi_box[1], (0, 255, 0), 2)
        self.marker_centered = False
        if self.reachedX and self.reachedY:
            if marker_IDs is not None:
                for corners, marker_id in zip(marker_corners, marker_IDs.flatten()):
                    corners = corners.reshape((4, 2)).astype(int)

                    cv.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)

                    self.center_x = int(np.mean(corners[:, 0]))
                    self.center_y = int(np.mean(corners[:, 1]))
                    cv.circle(frame, (self.center_x, self.center_y), radius=5, color=(0, 0, 255), thickness=-1)

                    
                    cv.putText(frame, f"ID: {marker_id}", (self.center_x + 10, self.center_y),
                            cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                    
                    if (self.roi_box[0][0] <= self.center_x <= self.roi_box[1][0]) and (self.roi_box[0][1] <= self.center_y <= self.roi_box[1][1]):
                        time.sleep(2)
                        if (self.roi_box[0][0] <= self.center_x <= self.roi_box[1][0]) and (self.roi_box[0][1] <= self.center_y <= self.roi_box[1][1]):
                            self.marker_centered = True
                    
                    if self.center_x >= width//2 and self.center_y <= height//2:
                        self.marker_offset = [(self.center_x - cx_roi), (self.center_y - cy_roi)]
                    elif self.center_x < width//2 and self.center_y <= height//2:
                        self.marker_offset = [(self.center_x - cx_roi), -(self.center_y - cy_roi)]
                    elif self.center_x < width//2 and self.center_y > height//2:
                        self.marker_offset = [-(self.center_x - cx_roi), -(self.center_y - cy_roi)]
                    elif self.center_x >= width//2 and self.center_y > height//2:
                        self.marker_offset = [-(self.center_x - cx_roi), (self.center_y - cy_roi)]
        cv.imshow("camera",frame)
        cv.waitKey(1)

    # -------------------------------------------------------------------------------------------------------------------------------------------------------------

    def collectdata(self):
        msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        self.current_pos = [msg.x,msg.y,msg.z]
        self.current_alt = msg.z
        # _,_,yaw = euler_from_quaternion()
        # print(self.current_pos)
        # print(f"NED Position -> X: {msg.x:.2f} m, Y: {msg.y:.2f} m, Z (alt): {msg.z:.2f} m")

    def pid(self):
        if self.current_pos is None or self.current_alt is None:
            self.get_logger().warn("waiting for position and altitude data...")
            return
        
        distance_to_goal,angle2goal = self.euclidean_distance(self.current_pos, self.marker_setpoint)
        # print(self.current_pos)
        # print(self.marker_setpoint)
        # print(f"distance_to_goal : {distance_to_goal}    ,  angle2goal : {math.degrees(angle2goal)}")

        if not self.takeoff:
            arm_state = self.arm()
            self.set_takeoff(target_altitude=1)

        if self.takeoff:
            # self.get_logger().info("moving toward the setpoint...")

            threshold = 0.05
            if not self.reachedX:
                errorX = self.marker_setpoint[0] - self.current_pos[0]
                # errorY = self.marker_setpoint[1] - self.current_pos[1]

                print(f"error in x : {errorX}")
                # print(errorY)

                current_time = self.get_clock().now()
                dt = (current_time - self.prev_time).nanoseconds / 1e9

                if dt < 1e-6:
                    dt = 1e-6

                self.integralX += errorX * dt
                derivativeX = (errorX - self.prev_errorX) / dt

                # self.integralY += errorY * dt
                # derivativeY = (errorY - self.prev_errorY) / dt

                uX = self.Kp[0] * errorX + self.Ki[0] * self.integralX + self.Kd[0] * derivativeX
                # uY = self.Kp[1] * errorY + self.Ki[1] * self.integralY + self.Kd[1] * derivativeY

                self.prev_errorX = errorX
                # self.prev_errorY = errorY

                self.prev_time = current_time

                self.set_velocity(uX, 0, 0)

                if abs(self.marker_setpoint[0] - self.current_pos[0]) <=threshold:
                    self.reachedX = True

            if self.reachedX and not self.reachedY:
                # errorX = self.marker_setpoint[0] - self.current_pos[0]
                errorY = self.marker_setpoint[1] - self.current_pos[1]

                # print(errorX)
                print(f"error in y : {errorY}")

                current_time = self.get_clock().now()
                dt = (current_time - self.prev_time).nanoseconds / 1e9

                if dt < 1e-6:
                    dt = 1e-6

                # self.integralX += errorX * dt
                # derivativeX = (errorX - self.prev_errorX) / dt

                self.integralY += errorY * dt
                derivativeY = (errorY - self.prev_errorY) / dt

                # uX = self.Kp[0] * errorX + self.Ki[0] * self.integralX + self.Kd[0] * derivativeX
                uY = self.Kp[1] * errorY + self.Ki[1] * self.integralY + self.Kd[1] * derivativeY

                # self.prev_errorX = errorX
                self.prev_errorY = errorY

                self.prev_time = current_time

                self.set_velocity(0, uY, 0)

                if abs(self.marker_setpoint[1] - self.current_pos[1]) <=threshold:
                    self.reachedY = True

            if self.reachedX and self.reachedY:
                if not self.marker_centered:
                    error_x_px = self.marker_offset[0]
                    error_y_px = self.marker_offset[1]
                    error_x = error_x_px / 100.0 
                    error_y = error_y_px / 100.0

                    ex = error_x - self.current_pos[0]
                    ey = error_y - self.current_pos[1]


                    # print("marker center mai aaya")

                    # print(error_x)
                    # print(error_y)

                    self.set_velocity(0.01*ex, 0.01*ey, 0)
                    self.get_logger().info(f"Logging error: ({error_x_px}, {error_y_px})")

                else:
                    self.set_velocity(0, 0, 0)
                    self.get_logger().info("Marker centered. Initiating landing...")
                    self.disarm()
                    if self.landed:
                        self.get_logger().info("mission completed...")
                        self.get_logger().info("exiting the process...")
                        exit(0)






            # if self.reachedX and self.reachedY:
            #     self.reached = True

            
            # if self.reached:
            #     self.disarm()
            #     if self.landed:
            #         self.get_logger().info("mission completed...")
            #         self.get_logger().info("exiting the process...")
            #         exit(0)

def main():
    rclpy.init(args=sys.argv)

    node = Drone() 

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
