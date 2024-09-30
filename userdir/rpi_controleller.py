#!/usr/bin/env python

import os
import time
import argparse
import select
import yaml
import datetime

import paramiko
import scp

from xmlrpc.client import ServerProxy

# 参考資料
#
# paramico
# https://ashitaka-blog.com/2022-07-07-065334/
# paramico and scp
# https://qiita.com/Angelan1720/items/a962e12fa81724b57526


class RPIController:
    def __init__(self, robotname, ros_settings_path):
        """CPS Controller for raspberry pi
        Args:
            robotname (str): robotname
            ros_settings_path (str): ros settings file path
        """
        with open(ros_settings_path, 'r') as f:
            setting = yaml.safe_load(f)
        self.hostname = setting['robot_ip_addr']
        self.username = setting['username']
        self.password = setting['password']
        self.robotname = robotname
        os.environ['ROS_MASTER_URI'] = 'http://{}:11311'.format(setting['rosmaster_ip_addr'] if 'rosmaster_ip_addr' in setting else setting['robot_ip_addr'])
        os.environ['ROS_IP'] = setting['host_ip_addr']
        os.environ['ROS_HOSTNAME'] = setting['host_ip_addr']

        self.client = paramiko.SSHClient()
        self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.client.connect(hostname=self.hostname, port=22,
                            username=self.username, password=self.password)

        self.source_command = 'source /home/{}/.ros_rc && source /home/{}/catkin_ws/devel/setup.bash'.format(
            self.username, self.username)
        self.ssh_stds = {}
        
        # for supervisor
        self.sv_server = None
        self.sv_service_name = 'run_robot'

    # for supervisor
    def send_settings(self, use_actuator=True, use_sensor=True, use_camera=False, sensor_config_path=None, dynamimxel_config=None, controller_config=None, send_files=[]):
        """send configuration files
        Args:
            use_actuator (bool) : true if use actuator 
            use_sensor   (bool) : true if use sensor
            use_camera   (bool) : true if use use camera
            sensor_config_path   (str) : sensor_configuration file path
            dynamimxel_config    (str) : dynamixel_configuration file path
            controller_config    (str) : controller_configuration file path
            send_files   (list of str) : send file list
        """
        date_string = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
        dist_dir = '/home/{}/cps_settings/{}'.format(
            self.username, date_string)
        latest_dir = '/home/{}/cps_settings/latest_settings'.format(
            self.username)

        put_sensor_config_path = '{}/all_sensor.yaml'.format(dist_dir)
        put_dynamixel_config_path = '{}/config.yaml'.format(dist_dir)
        put_controller_config_path = '{}/controller_config.yaml'.format(
            dist_dir)

        load_sensor_config_path = '{}/all_sensor.yaml'.format(latest_dir)
        load_dynamixel_config_path = '{}/config.yaml'.format(latest_dir)
        load_controller_config_path = '{}/controller_config.yaml'.format(latest_dir)
        
        use_dynamixel_str = 'true' if use_actuator else 'false'
        use_sensor_str = 'true' if use_sensor else 'false'
        use_camera_str = 'true' if use_camera else 'false'

        make_shell = 'echo -e \'trap \\"trap - SIGTERM && kill -- -\$\$\\" SIGINT SIGTERM EXIT\\n{} && roslaunch /home/{}/cps_rpi/launch/run_robot.launch dynamixel_settings:={} controller_settings:={} namespace:={} sensor_config_path:={} use_dynamixel:={} use_sensor:={} use_camera:={} & \\nwait\\n\' > {}/run_robot.sh'.format(self.source_command, self.username,
                                                                                                                                                                                                                                                                        load_dynamixel_config_path, load_controller_config_path,
                                                                                                                                                                                                                                                                        self.robotname, load_sensor_config_path, use_dynamixel_str, use_sensor_str, use_camera_str, dist_dir)
        command = 'bash -lc "mkdir -p {} && rm -f {} && ln -s {} {} && {}"'.format(
            dist_dir, latest_dir, dist_dir, latest_dir, make_shell)
        self.ssh_stds["operation"] = self.client.exec_command(
            command, get_pty=True)
        time.sleep(2)
        with scp.SCPClient(self.client.get_transport()) as scpc:
            if sensor_config_path is not None:
                scpc.put(sensor_config_path, put_sensor_config_path)
            if dynamimxel_config is not None:
                scpc.put(dynamimxel_config, put_dynamixel_config_path)
            if controller_config is not None:
                scpc.put(controller_config, put_controller_config_path)
            for sendfile in send_files:
                scpc.put(sendfile, '{}/{}'.format(dist_dir, os.path.basename(sendfile)))
                
    def start_robot(self):
        """start robot's program via Supervisor
        """
        self.sv_server = ServerProxy('http://{}:{}@{}:9999/RPC2'.format(self.username, self.password, self.hostname))
        # if robot's program are alreday running, stop program before start.
        if self.sv_server.supervisor.getProcessInfo(self.sv_service_name)['statename'] == 'RUNNING':
            self.sv_server.supervisor.stopProcess(self.sv_service_name, True)    
        self.sv_server.supervisor.startProcess(self.sv_service_name, True)

    def stop_robot(self):
        """stop robot's program via Supervisor
        """
        self.sv_server.supervisor.stopProcess(self.sv_service_name, True)

    # destructor
    def __del__(self):
        self.client.close()
        # for suppervisor
        if self.sv_server is not None:
            self.stop_robot()
        time.sleep(1)
