#!/usr/bin/env python

import os
import time
import argparse
import select
import yaml
import datetime

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
        os.environ['ROS_MASTER_URI'] = 'http://{}:11311'.format(setting['rosmaster_ip_addr'] if 'rosmaster_ip_addr' in setting else setting['robot_ip_addr'])
        os.environ['ROS_IP'] = setting['host_ip_addr']
        os.environ['ROS_HOSTNAME'] = setting['host_ip_addr']

        self.client = paramiko.SSHClient()
        self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.client.connect(hostname=self.hostname, port=22,
                            username=self.username, password=self.password)

        self.ssh_stds = {}
        self.source_command = 'source /home/{}/.ros_rc && source /home/{}/catkin_ws/devel/setup.bash'.format(
            self.username, self.username)

    def send_settings(self, sensor_config_path=None, dynamimxel_config=None, controller_config=None, use_camera=False, send_files=[]):
        date_string = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
        # dist_dir = '/tmp/{}'.format(date_string)
        # latest_dir = '/tmp/latest_settings'
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
        
        use_dynamixel_str = 'true' if dynamimxel_config is not None and controller_config is not None else 'false'
        use_sensor_str = 'true' if sensor_config_path is not None else 'false'
        use_camera_str = 'true' if use_camera else 'false'

        make_shell = 'echo -e \'trap \\"trap - SIGTERM && kill -- -\$\$\\" SIGINT SIGTERM EXIT\\n{} && roslaunch /home/{}/cps_rpi/launch/run_robot.launch dynamixel_settings:={} controller_settings:={} namespace:={} sensor_config_path:={} use_dynamixel:={} use_sensor:={} use_camera:={} & \\nwait\\n\' > {}/run_robot.sh'.format(self.source_command, self.username,
                                                                                                                                                                                                                                                                        load_dynamixel_config_path, load_controller_config_path,
                                                                                                                                                                                                                                                                        self.robotname, load_sensor_config_path, use_dynamixel_str, use_sensor_str, use_camera_str, dist_dir)
        # print(make_shell)
        command = 'bash -lc "mkdir -p {} && rm -f {} && ln -s {} {} && {}"'.format(
            dist_dir, latest_dir, dist_dir, latest_dir, make_shell)
        # print(command)
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

    def connect_sensor(self, sensor_config_path):
        """connect sensors
        Args:
            sensor_config (str): filepath to sensor configration 
        """
        put_sensor_config_path = "/tmp/all_sensor.yaml"

        with scp.SCPClient(self.client.get_transport()) as scpc:
            scpc.put(sensor_config_path, put_sensor_config_path)

        command = 'bash -lc "{} && roslaunch sensor_pi sensor_pi.launch config_path:={} namespace:={}"'.format(
            self.source_command, put_sensor_config_path, self.robotname)
        self.ssh_stds["sensor"] = self.client.exec_command(
            command, get_pty=True)

    def disconnect_sensor(self):
        """disconnect sensors
        """
        if "sensor" in self.ssh_stds:
            print('\x03', file=self.ssh_stds["sensor"][0], end='')
            self.ssh_stds["sensor"][0].close()
            _ = self.ssh_stds.pop("sensor")

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

        command = 'bash -lc "{} && roslaunch dynamixel_irsl controllers.launch dynamixel_settings:={} controller_settings:={} namespace:={}"'.format(
            self.source_command, put_dynamixel_config_path, put_controller_config_path, self.robotname)
        self.ssh_stds["dynamixel"] = self.client.exec_command(
            command, get_pty=True)

    def discnnet_dynamixel(self):
        """disconnect dynamixels 
        """
        if "dynamixel" in self.ssh_stds:
            print('\x03', file=self.ssh_stds["dynamixel"][0], end='')
            self.ssh_stds["dynamixel"][0].close()
            _ = self.ssh_stds.pop("dynamixel")

    def get_stdout(self, stdout):
        if not stdout.channel.exit_status_ready():
            if stdout.channel.recv_ready():
                rl, wl, xl = select.select([stdout.channel], [], [], 0.0)
                if len(rl) > 0:
                    return stdout.channel.recv(2048).decode("utf-8")
        return ""

    def get_sensor_stdout(self):
        return self.get_stdout(self.ssh_stds["sensor"][1])

    def get_dynaxmiel_stdout(self):
        return self.get_stdout(self.ssh_stds["dynamixel"][1])

    def __del__(self):
        # self.discnnet_dynamixel()
        # self.disconnect_sensor()
        for key, stds in self.ssh_stds.items():
            if not stds[0].channel.closed:
                print('\x03', file=stds[0], end='')
                stds[0].close()
                # print("{} is closed.".format(key))
        self.client.close()
