#!/usr/bin/env python

'''
Video + Force + Kinematics Data Capture
Written by Zonghe Chua 10th Jan 2023

Please see the import statements to figure out which packages are needed. 
'''

import rospy
import rosbag
# import message_filters
# from sensor_msgs.msg import CompressedImage, JointState
# from geometry_msgs.msg import WrenchStamped, PoseStamped, TwistStamped
# from std_msgs.msg import Float64MultiArray
import cv2
# from cv_bridge import CvBridge
import numpy as np
import sys
import tqdm
import os
import glob

'''This are the inputs to the script

Run: ./extract_rosbag.py OUTPUT_FORMAT FILES

First argument defines the output format
Options here are:
video                   - (experimental) outputs the left camera video only (can be extended to do stereo video)
stereo                  - outputs left and right camera image plus ~30hz data (didn't guarantee syncing yet)
labels                  - outputs ~30hz data only 
maxhz_labels            - outputs all the sample data
default (anything else) - outputs the left camera image plus 30hz data

Second argument defines a rule for glob to find the bags to process. 
For example if you want to output stereo images for all bags that start with V then you would run
./extract_rosbag.py stereo 'V*.bag'

'''
output_format = sys.argv[1]
bag_names = str(sys.argv[2])
bag_list = glob.glob(bag_names)
print(bag_list)

for bagname in bag_list:

    # add the topics you would like to read
    topic_dict = {'left_image': '/camera/left/image_color/compressed',
                  'right_image': '/camera/right/image_color/compressed',
                  'pose_current': '/dvrk/PSM2/position_cartesian_current',
                  'pose_desired': '/dvrk/PSM2/position_cartesian_desired',
                  'joint_current': '/dvrk/PSM2/state_joint_current',
                  'jaw_current': '/dvrk/PSM2/state_jaw_current',
                  'joint_desired': '/dvrk/PSM2/state_joint_desired',
                  'jaw_desired': '/dvrk/PSM2/state_jaw_desired',
                  'body_twist': '/dvrk/PSM2/twist_body_current',
                  'body_wrench': '/dvrk/PSM2/wrench_body_current',
                  'space_jacobian': '/dvrk/PSM2/jacobian_spatial',
                  'force_gt': '/force_sensor'
                  }

    out_filedir = 'output_'+bagname.split(os.sep)[-1].split('.')[0]
    print('output into directory ' + out_filedir)
    outfile_full = os.getcwd()+'/'+out_filedir
    print('making ' + outfile_full)

    try:
        os.mkdir(outfile_full)
    except:
        print('directory exists')

    bag = rosbag.Bag(bagname)
    first_msg = False
    count = 0

    # create the arrays to hold the read values
    fs_data = np.zeros(3)
    tq_data = np.zeros(3)
    pose_data = np.zeros(7)
    pose_data_des = np.zeros(7)  # not in the NN models.
    fs_PSM_data = np.zeros(3)
    tq_PSM_data = np.zeros(3)
    vel_data = np.zeros(6)
    joint_pos_data = np.zeros(6)
    joint_vel_data = np.zeros(6)
    joint_torque_data = np.zeros(6)
    joint_pos_data_des = np.zeros(6)
    joint_torque_data_des = np.zeros(6)
    jaw_pos_data = 0
    jaw_vel_data = 0
    jaw_torque_data = 0
    jaw_pos_data_des = 0
    jaw_torque_data_des = 0
    jacobian = np.zeros(36)

    if output_format == 'video':
        try:
            video_out = cv2.cv.VideoWriter(
                out_filedir + '/capture.avi', cv2.cv.CV_FOURCC(*'MJPG'), 30, (960, 540))
        except:
            video_out = cv2.VideoWriter(
                out_filedir + '/capture.avi', cv2.VideoWriter_fourcc(*'MJPG'), 30, (960, 540))
        print('initialized video file output with 30hz data')
    elif output_format == 'labels':
        print('initialized 30Hz data output only')
    elif output_format == 'maxhz_labels':
        print('initialized at max hz data only')
    elif output_format == 'stereo':
        print("stereo output with 30hz data")
        try:
            os.mkdir(outfile_full+'/right_img')
        except:
            print("right img directory already exists")
    else:
        print('image output with 30hz data')

    # makes a new file
    if output_format == 'maxhz_labels':
        # write the output file
        f = open(outfile_full + '/labels_maxhz.txt', 'w')
    else:
        # write the output file
        f = open(outfile_full + '/labels_30hz.txt', 'w')
    # close the file
    f.close()
    # open in append mode
    if output_format == '200hz_labels':
        # write the output file
        f = open(outfile_full + '/labels_maxhz.txt', 'a')
    else:
        # write the output file
        f = open(outfile_full + '/labels_30hz.txt', 'a')

    # collect the message counts
    msg_count = 0
    for topic in topic_dict.values():
        # print(topic, bag.get_message_count(topic))
        msg_count += bag.get_message_count(topic)
    pbar = tqdm.tqdm(total=msg_count)

    for topic, msg, t in bag.read_messages(topics=topic_dict.values()):
        if first_msg is False:
            start_time = t.secs
            first_msg = True

        time = t.secs-start_time+round(t.nsecs/1e9, 4)
        # print(topic + ' ' + str(t.secs-start_time+round(t.nsecs/1e9,4)) + 's')

        # read in the topics
        if topic == topic_dict['force_gt']:
            fs_data[0] = msg.wrench.force.x
            fs_data[1] = msg.wrench.force.y
            fs_data[2] = msg.wrench.force.z
            tq_data[0] = msg.wrench.torque.x
            tq_data[1] = msg.wrench.torque.y
            tq_data[2] = msg.wrench.torque.z

        if topic == topic_dict['pose_current']:
            pose_data[0] = msg.pose.position.x
            pose_data[1] = msg.pose.position.y
            pose_data[2] = msg.pose.position.z
            pose_data[3] = msg.pose.orientation.x
            pose_data[4] = msg.pose.orientation.y
            pose_data[5] = msg.pose.orientation.z
            pose_data[6] = msg.pose.orientation.w

        if topic == topic_dict['pose_desired']:
            pose_data_des[0] = msg.pose.position.x
            pose_data_des[1] = msg.pose.position.y
            pose_data_des[2] = msg.pose.position.z
            pose_data_des[3] = msg.pose.orientation.x
            pose_data_des[4] = msg.pose.orientation.y
            pose_data_des[5] = msg.pose.orientation.z
            pose_data_des[6] = msg.pose.orientation.w

        if topic == topic_dict['right_image']:
            img_data = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(img_data, cv2.IMREAD_COLOR)
            if output_format == "stereo":
                im_file = out_filedir + '/right_img/img_'+str(count)+'.jpg'
                cv2.imwrite(im_file, img)

        if topic == topic_dict['left_image']:
            # when we read an image message then we dump the data.
            img_data = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(img_data, cv2.IMREAD_COLOR)

            if output_format == 'video':
                video_out.write(img)
            elif (output_format == 'labels' or output_format == 'maxhz_labels'):
                pass
            else:
                im_file = out_filedir + '/img_'+str(count)+'.jpg'
                cv2.imwrite(im_file, img)

            if (output_format != '200hz_labels') | (output_format == 'labels'):
                new_data = np.hstack((time,		# (1)
                                      fs_data,		# (3)
                                      tq_data,		# (3)
                                      pose_data,		# (7)
                                      pose_data_des,
                                      vel_data,		# (6)
                                      joint_pos_data,
                                      jaw_pos_data,		# (7)
                                      joint_vel_data,
                                      jaw_vel_data,		# (7)
                                      joint_torque_data,
                                      jaw_torque_data,  # (7)
                                      joint_pos_data_des,
                                      jaw_pos_data_des,  # (7)
                                      joint_torque_data_des,
                                      jaw_torque_data_des,  # (7)
                                      fs_PSM_data,		# (3)
                                      tq_PSM_data,		# (3)
                                      jacobian))		# (36)

                # update the data output file
                np.savetxt(f, new_data.reshape((1, -1)), delimiter=',')
                count += 1

        if topic == topic_dict['body_wrench']:
            fs_PSM_data[0] = msg.wrench.force.x
            fs_PSM_data[1] = msg.wrench.force.y
            fs_PSM_data[2] = msg.wrench.force.z
            tq_PSM_data[0] = msg.wrench.torque.x
            tq_PSM_data[1] = msg.wrench.torque.y
            tq_PSM_data[2] = msg.wrench.torque.z

        if topic == topic_dict['body_twist']:
            vel_data[0] = msg.twist.linear.x
            vel_data[1] = msg.twist.linear.y
            vel_data[2] = msg.twist.linear.z
            vel_data[3] = msg.twist.angular.x
            vel_data[4] = msg.twist.angular.y
            vel_data[5] = msg.twist.angular.z

        if topic == topic_dict['joint_current']:
            joint_pos_data = msg.position
            joint_vel_data = msg.velocity
            joint_torque_data = msg.effort

        if topic == topic_dict['jaw_current']:
            jaw_pos_data = msg.position[0]
            jaw_vel_data = msg.velocity[0]
            jaw_torque_data = msg.effort[0]

        if topic == topic_dict['joint_desired']:
            joint_pos_data_des = msg.position
            joint_torque_data_des = msg.effort

        if topic == topic_dict['jaw_desired']:
            jaw_pos_data_des = msg.position[0]
            jaw_torque_data_des = msg.effort[0]

        if topic == topic_dict['space_jacobian']:
            jacobian = np.array(msg.data)

            if output_format == 'maxhz_labels':
                new_data = np.hstack((time,		# (1)
                                      fs_data,		# (3)
                                      tq_data,		# (3)
                                      pose_data,		# (7)
                                      # (7) NOT IN NEURAL NETWORK!
                                      pose_data_des,
                                      vel_data,		# (6)
                                      joint_pos_data,
                                      jaw_pos_data,		# (7)
                                      joint_vel_data,
                                      jaw_vel_data,		# (7)
                                      joint_torque_data,
                                      jaw_torque_data,  # (7)
                                      joint_pos_data_des,
                                      jaw_pos_data_des,  # (7)
                                      joint_torque_data_des,
                                      jaw_torque_data_des,  # (7)
                                      fs_PSM_data,		# (3)
                                      tq_PSM_data,		# (3)
                                      jacobian))		# (36)

                # update the data output file
                # data = np.vstack((data,new_data))
                np.savetxt(f, new_data.reshape((1, -1)), delimiter=',')
                count += 1

        pbar.update(1)

    pbar.close()
    if output_format == 'video':
        print('closing video writer')
        video_out.release()

    bag.close()
    f.close()
