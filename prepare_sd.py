#!/usr/bin/env python3
import argparse
import sys
import os

##############################################################
#
# Ask questions to prepare cloud-init file for mini pupper
#
# if ~/.mini_pupper_sd.txt exists no questions will be asked
#
##############################################################

target_environment = {}
hardware_version = ['v1', 'v2', 'v2_pro']
stack_owners = ['MangDang', 'Third Parties']
stack_names_mangdang = ['Stanford', 'ROS1', 'ROS2']
stack_scripts_mangdang = ['bsp_stanford_web_controller/setup.sh', 'bsp_ros1/setup.sh', 'bsp_ros2/setup.sh']
stack_names_thirdparties = ['ROS1 Jupyter Notebook', 'ESP IDF']
stack_scripts_thirdparties = ['ros1_jupyter/setup.sh', 'esp_idf/setup.sh']


def write_cache(cache_file):
    with open(cache_file, 'w') as fh:
        for key in target_environment.keys():
            fh.write("%s: %s\n" % (key, target_environment[key]))


def ask_user(prompt, var_name):
    input_text = input("%s: " % prompt)
    target_environment[var_name] = input_text


def ask_questions():
    if not 'wifi_ssid' in target_environment.keys():
        ask_user("Your WiFi SSID", 'wifi_ssid')
    if not 'wifi_password' in target_environment.keys():
        ask_user("Your WiFi password", 'wifi_password')
    if not 'ubuntu_password' in target_environment.keys():
        ask_user("Mini Pupper user password", 'ubuntu_password')
    if not 'hardware_version' in target_environment.keys():
        print("Which Mini Pupper Hardware do you want to install:\n")
        for i in range(len(hardware_version)):
            print("%s: %s" % (i + 1, hardware_version[i]))
        ask_user("Please enter number", 'hardware_version')
    if not 'stack_owner' in target_environment.keys():
        print("Which stack do you want to install?\nSelect Owner:\n")
        for i in range(len(stack_owners)):
            print("%s: %s" % (i + 1, stack_owners[i]))
        ask_user("Please enter number", 'stack_owner')
    if not 'stack' in target_environment.keys():
        if target_environment['stack_owner'] == '1':
            stack_names = stack_names_mangdang
        if target_environment['stack_owner'] == '2':
            stack_names = stack_names_thirdparties
        print("Select stack:\n")
        for i in range(len(stack_names)):
            print("%s: %s" % (i + 1, stack_names[i]))
        ask_user("Please enter number", 'stack')
    if not 'sd_path' in target_environment.keys():
        ask_user("Full path to SD card", 'sd_path')


# Detect OS
target_environment['this_os'] = sys.platform

parser = argparse.ArgumentParser(description='Prepare SD card for Mini Pupper')
parser.add_argument('-c', '--cache',
                    action='store_true',
                    help='Cache my responses')
args = parser.parse_args()

conf_file = os.path.join(os.path.expanduser("~"), '.mini_pupper_sd.txt')
if os.path.exists(conf_file):
    with open(conf_file, 'r') as fh:
        lines = fh.readlines()
        for line in lines:
            if ':' in line:
                arr = line.strip().split(':')
                key = arr[0]
                if len(arr) == 2:
                    value = arr[1].strip()
                else:
                    value = ''.join(s for s in arr[1:]).strip()
                if value.startswith('"') and value.endswith('"'):
                    value = value[1:-1]
                if len(value) > 0:
                    target_environment[key] = value
ask_questions()
if target_environment['stack_owner'] == '1':
    stack_scripts = stack_scripts_mangdang
if target_environment['stack_owner'] == '2':
    stack_scripts = stack_scripts_thirdparties
target_environment['script'] = "%s_%s" % (hardware_version[int(target_environment['hardware_version']) - 1],
                                          stack_scripts[int(target_environment['stack']) - 1])
if args.cache:
    write_cache(conf_file)

network_conf_file = os.path.join(target_environment['sd_path'], 'network-config')
if not os.path.exists(network_conf_file):
    sys.exit("Invalid path to SD card or SD card not mounted\n")

# Check Answers
script_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), target_environment['script'])
if not os.path.isfile(script_file):
    print("This Stack is not supported\n")
    sys.exit()

network_conf = """version: 2
ethernets:
  eth0:
    dhcp4: true
    optional: true
wifis:
  wlan0:
    dhcp4: true
    optional: true
    access-points:
      %s:
        password: "%s"
"""
with open(network_conf_file, 'w') as fh:
    fh.write(network_conf % (target_environment['wifi_ssid'], target_environment['wifi_password']))

user_data_file = os.path.join(target_environment['sd_path'], 'user-data')
user_data = """#cloud-config
ssh_pwauth: True
chpasswd:
  expire: false
  list:
  - ubuntu:%s
packages:
- git
runcmd:
- [ su, ubuntu, -c, "git clone https://github.com/mangdangroboticsclub/mini_pupper.git /home/ubuntu/mini_pupper" ]
- [ su, ubuntu, -c, "/home/ubuntu/mini_pupper/%s %s '%s' 2> /home/ubuntu/.setup_err.log > /home/ubuntu/.setup_out.log" ]
- [ reboot ]
"""

user_data_focal = """#cloud-config
ssh_pwauth: True
chpasswd:
  expire: false
  list:
  - ubuntu:%s
write_files:
- path: /var/lib/mini_pupper/setup.sh
  content: |
    #!/bin/bash
    /usr/bin/git clone https://github.com/mangdangroboticsclub/mini_pupper.git /home/ubuntu/mini_pupper
    /home/ubuntu/mini_pupper/%s %s '%s' 2> /home/ubuntu/.setup_err.log > /home/ubuntu/.setup_out.log
    reboot
  permissions: '0755'
  owner: root:root
- path: /etc/rc.local
  content: |
    #!/bin/bash
    /var/lib/mini_pupper/setup.sh
  permissions: '0755'
  owner: root:root
- path: /lib/systemd/system/rc-local.service
  content: |
    [Unit]
    Description=/etc/rc.local Compatibility
    Documentation=man:systemd-rc-local-generator(8)
    ConditionFileIsExecutable=/etc/rc.local
    After=network.target

    [Service]
    Type=forking
    ExecStart=/etc/rc.local
    TimeoutSec=0
    RemainAfterExit=no
    GuessMainPID=no
    User=ubuntu

    [Install]
    WantedBy=multi-user.target
    Alias=rc.local.service
  permissions: '0644'
  owner: root:root
runcmd:
- [ systemctl, enable, rc-local ]
- [ /usr/bin/sleep, 60 ]
- [ reboot ]
"""

# for now do not start automatically
user_data_focal = """#cloud-config
ssh_pwauth: True
chpasswd:
  expire: false
  list:
  - ubuntu:%s
write_files:
- path: /var/lib/mini_pupper/setup.sh
  content: |
    #!/bin/bash
    /usr/bin/git clone https://github.com/mangdangroboticsclub/mini_pupper.git /home/ubuntu/mini_pupper
    /home/ubuntu/mini_pupper/%s %s '%s' 2> /home/ubuntu/.setup_err.log > /home/ubuntu/.setup_out.log
    reboot
  permissions: '0755'
  owner: root:root
"""


# Use different cloud-init configuration for Ubuntu Focal (20.04)
if target_environment['stack_owner'] == '1' and target_environment['stack'] == '2':
    user_data = user_data_focal
if target_environment['stack_owner'] == '2' and target_environment['stack'] == '1':
    user_data = user_data_focal

with open(user_data_file, 'w') as fh:
    fh.write(user_data % (target_environment['ubuntu_password'],
                          target_environment['script'],
                          target_environment['wifi_ssid'],
                          target_environment['wifi_password']))
