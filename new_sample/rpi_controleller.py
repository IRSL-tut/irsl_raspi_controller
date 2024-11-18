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
    def __init__(self, robotname, ros_settings_path=None, connection=True, **kwargs):
        """CPS Controller for raspberry pi
        Args:
            robotname (str): robotname
            ros_settings_path (str): ros settings file path
        """
        if ros_settings_path is not None:
            with open(ros_settings_path, 'r') as f:
                setting = yaml.safe_load(f)
        else:
            setting = kwargs
        self.hostname = setting['robot_ip_addr']
        self.username = setting['username'] if 'username' in setting else 'irsl'
        self.password = setting['password'] if 'password' in setting else 'irsl'
        self.robotname = robotname

        ### set at RobotInterface
        #os.environ['ROS_MASTER_URI'] = 'http://{}:{}'.format(setting['rosmaster_ip_addr'] if 'rosmaster_ip_addr' in setting else setting['robot_ip_addr'])
        #os.environ['ROS_IP']       = setting['host_ip_addr']
        #os.environ['ROS_HOSTNAME'] = setting['host_ip_addr']

        self.client = None
        if connection:
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
    def send_settings(self, use_actuator=True, use_sensor=True, use_camera=False, sensor_config=None, dynamixel_config=None, controller_config=None, send_files=[]):
        """send configuration files
        Args:
            use_actuator (bool) : true if use actuator 
            use_sensor   (bool) : true if use sensor
            use_camera   (bool) : true if use use camera
            sensor_config        (str) : sensor_configuration file path
            dynamixel_config     (str) : dynamixel_configuration file path
            controller_config    (str) : controller_configuration file path
            send_files   (list of str) : send file list
        """
        date_string = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
        dist_dir   = '/home/{}/cps_settings/{}'.format(self.username, date_string)
        latest_dir = '/home/{}/cps_settings/latest_settings'.format(self.username)

        put_sensor_config_path      = '{}/all_sensor.yaml'.format(dist_dir)
        put_dynamixel_config_path   = '{}/config.yaml'.format(dist_dir)
        put_controller_config_path  = '{}/controller_config.yaml'.format(dist_dir)
        put_run_robot_path          = '{}/run_robot.sh'.format(dist_dir)
        
        load_sensor_config_path     = '{}/all_sensor.yaml'.format(latest_dir)
        load_dynamixel_config_path  = '{}/config.yaml'.format(latest_dir)
        load_controller_config_path = '{}/controller_config.yaml'.format(latest_dir)

        use_dynamixel_str = 'true' if use_actuator else 'false'
        use_sensor_str = 'true' if use_sensor else 'false'
        use_camera_str = 'true' if use_camera else 'false'

        #make_shell = 'echo -e \'trap \\"trap - SIGTERM && kill -- -\$\$\\" SIGINT SIGTERM EXIT\\n{} && roslaunch /home/{}/cps_rpi/launch/run_robot.launch dynamixel_settings:={} controller_settings:={} namespace:={} sensor_config_path:={} use_dynamixel:={} use_sensor:={} use_camera:={} & \\nwait\\n\' > {}/run_robot.sh'.format(self.source_command, self.username, load_dynamixel_config_path, load_controller_config_path, self.robotname, load_sensor_config_path, use_dynamixel_str, use_sensor_str, use_camera_str, dist_dir)
        #command = 'bash -lc "mkdir -p {} && rm -f {} && ln -s {} {} && {}"'.format(dist_dir, latest_dir, dist_dir, latest_dir, make_shell)
        command = 'bash -lc "mkdir -p {} && rm -f {} && ln -s {} {}"'.format(dist_dir, latest_dir, dist_dir, latest_dir)
        
        shell_txt = '''
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

export ROS_IP={}
export ROS_MASTER_URI="http://${{ROS_IP}}:11311/"
export ROS_HOSTNAME=${{ROS_IP}}
source /opt/ros/noetic/setup.bash
source /home/{}/catkin_ws/devel/setup.bash

roslaunch /home/{}/cps_rpi/launch/run_robot.launch dynamixel_settings:={} controller_settings:={} namespace:={} sensor_config_path:={} use_dynamixel:={} use_sensor:={} use_camera:={} & 
wait
'''.format(self.hostname,
           self.username,
           self.username, load_dynamixel_config_path, load_controller_config_path,
           self.robotname, load_sensor_config_path, use_dynamixel_str, use_sensor_str, use_camera_str)

        with open('run_robot.sh', mode='w') as f: 
            f.write(shell_txt)
            
        self.ssh_stds["operation"] = self.client.exec_command(command, get_pty=True)
        ##
        time.sleep(2)
        with scp.SCPClient(self.client.get_transport()) as scpc:
            scpc.put('run_robot.sh', put_run_robot_path)
            if sensor_config is not None:
                scpc.put(sensor_config, put_sensor_config_path)
            if dynamixel_config is not None:
                scpc.put(dynamixel_config, put_dynamixel_config_path)
            if controller_config is not None:
                scpc.put(controller_config, put_controller_config_path)
            for sendfile in send_files:
                scpc.put(sendfile, '{}/{}'.format(dist_dir, os.path.basename(sendfile)))

    def set_supervisor_proxy(self, host=None, port=9999, user=None, password=None):
        host_ = host if host is not None else self.hostname
        user_ = user if user is not None else self.username
        pass_ = password if password is not None else self.password
        self.sv_server = ServerProxy('http://{}:{}@{}:{}/RPC2'.format(user_, pass_, host_, port))

    def start_robot(self, **kwargs):
        """start robot's program via Supervisor
        """
        if self.sv_server is None:
            self.set_supervisor_proxy(**kwargs)
        # if robot's program are alreday running, stop program before start.
        if self.sv_server.supervisor.getProcessInfo(self.sv_service_name)['statename'] == 'RUNNING':
            self.sv_server.supervisor.stopProcess(self.sv_service_name, True)
        self.sv_server.supervisor.startProcess(self.sv_service_name, True)

    def stop_robot(self, **kwargs):
        """stop robot's program via Supervisor
        """
        if self.sv_server is None:
            self.set_supervisor_proxy(**kwargs)
        self.sv_server.supervisor.stopProcess(self.sv_service_name, True)

    # destructor
    def __del__(self):
        if self.client is not None:
            self.client.close()
        # for suppervisor #
        #if self.sv_server is not None:
        #    self.stop_robot()
        time.sleep(1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='setting robot controller')
    ## settings
    parser.add_argument('-n', '--namespace', help='', default='AssembleRobot')
    parser.add_argument('-a', '--use_actuator', help='', type=bool, default=False)
    parser.add_argument('-s', '--use_sensor', help='', type=bool, default=False)
    parser.add_argument('-c', '--use_camera', help='', type=bool, default=False)
    parser.add_argument('-D', '--dynamixel_config', help='')
    parser.add_argument('-C', '--controller_config', help='')
    parser.add_argument('-S', '--sensor_config', help='')
    parser.add_argument('-I', '--robot_ip_addr', help='')
    ##
    parser.add_argument('-R', '--start_robot', type=bool, default=False)

    args = parser.parse_args()

    if args.use_actuator:
        if len(args.dynamixel_config) < 4:
            raise Exception(''.format())
        if len(args.controller_config) < 4:
            raise Exception(''.format())
    rpc = RPIController(args.namespace, robot_ip_addr=args.robot_ip_addr)
    rpc.send_settings(use_actuator = args.use_actuator,
                      use_sensor   = args.use_sensor,
                      use_camera   = args.use_camera,
                      dynamixel_config   = args.dynamixel_config,
                      controller_config  = args.controller_config,
                      sensor_config_path = args.sensor_config,
                      send_files=[])
    if args.start_robot:
        rpc.start_robot()
