#!/usr/bin/env python

import os
import time
import argparse
import select
import yaml

import paramiko
import scp

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
        os.environ['ROS_MASTER_URI']='http://{}:11311'.format(setting['robot_ip_addr'])
        os.environ['ROS_IP'] = setting['host_ip_addr']
        os.environ['ROS_HOSTNAME'] = setting['host_ip_addr']
        
        self.client = paramiko.SSHClient()
        self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.client.connect(hostname=self.hostname, port=22, username=self.username, password=self.password)
        
        self.sensor_stds = None
        self.dynaxmiel_stds = None
        self.source_command = 'source /home/{}/catkin_ws/devel/setup.bash'.format(self.username)
               
    def connect_sensor(self, sensor_config_path):
        """connect sensors
        Args:
            sensor_config (str): filepath to sensor configration 
        """
        put_sensor_config_path = "/tmp/all_sensor.yaml"
        
        with scp.SCPClient(self.client.get_transport()) as scpc:
            scpc.put(sensor_config_path, put_sensor_config_path)

        command = 'bash -lc "{} && roslaunch sensor_pi sensor_pi.launch config_path:={} namespace:={}"'.format(self.source_command, put_sensor_config_path, self.robotname)
        self.sensor_stds = self.client.exec_command(command, get_pty=True)
        
    def disconnect_sensor(self):
        """disconnect sensors
        """
        if self.sensor_stds is not None:
            print('\x03', file=self.sensor_stds[0], end='')
            self.sensor_stds[0].close()

    def connect_dynamixel(self, dynamimxel_config, controller_config):
        """connect dynamixels
        Args:
            dynamimxel_config (str): filepath to dynamixel configration 
            controller_config (str): filepath to controller configration 
        """
        put_dynamixel_config_path = '/tmp/config.yaml'
        put_controller_config_path = '/tmp/controller_config.yaml'

        with scp.SCPClient(self.client.get_transport()) as scpc:
            scpc.put(dynamimxel_config, put_dynamixel_config_path)
            scpc.put(controller_config, put_controller_config_path)
        
        command = 'bash -lc "{} && roslaunch dynamixel_irsl controllers.launch dynamixel_settings:={} controller_settings:={} namespace:={}"'.format(self.source_command, put_dynamixel_config_path, put_controller_config_path, self.robotname)
        self.dynaxmiel_stds = self.client.exec_command(command, get_pty=True)
    
    def discnnet_dynamixel(self):
        """disconnect dynamixels 
        """
        if self.dynaxmiel_stds is not None:
            print('\x03', file=self.dynaxmiel_stds[0], end='')
            self.dynaxmiel_stds[0].close()

    def get_stdout(self, stdout):
        if not stdout.channel.exit_status_ready():
            if stdout.channel.recv_ready():
                rl, wl, xl = select.select([stdout.channel], [], [], 0.0)
                if len(rl) > 0:
                    return stdout.channel.recv(2048).decode("utf-8")
        return ""
    
    def get_sensor_stdout(self):
        return self.get_stdout(self.sensor_stds[1])
    
    def get_dynaxmiel_stdout(self):
        return self.get_stdout(self.dynaxmiel_stds[1])


    def __del__(self):
        self.discnnet_dynamixel()
        self.disconnect_sensor()
        self.client.close()
