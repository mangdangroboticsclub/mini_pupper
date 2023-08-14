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
hardware_version = ['v1', 'v2']
stack_names = ['Stanford', 'ROS']
stacks = {}
stacks['v1_Stanford'] = {}
stacks['v2_Stanford'] = {}
stacks['v1_ROS'] = {}
stacks['v2_ROS'] = {}
stacks['v1_Stanford']['repos'] = [" https://github.com/mangdangroboticsclub/mini_pupper_bsp.git ~/mini_pupper_bsp",
                                  " https://github.com/mangdangroboticsclub/StanfordQuadruped.git ~/StanfordQuadruped",
                                  " https://github.com/mangdangroboticsclub/mini_pupper_web_controller.git ~/mini_pupper_web_controller"]
stacks['v1_Stanford']['scripts'] = ["~/mini_pupper_web_controller/setup.sh v1",
                                    "sudo reboot"]
stacks['v2_Stanford']['repos'] = [" https://github.com/mangdangroboticsclub/mini_pupper_2_bsp.git ~/mini_pupper_bsp",
                                  " https://github.com/mangdangroboticsclub/StanfordQuadruped.git ~/StanfordQuadruped",
                                  " https://github.com/mangdangroboticsclub/mini_pupper_web_controller.git ~/mini_pupper_web_controller"]
stacks['v2_Stanford']['scripts'] = ["~/mini_pupper_web_controller/setup.sh v2",
                                    "sudo reboot"]
stacks['v1_ROS']['repos'] = [" https://github.com/mangdangroboticsclub/mini_pupper_bsp.git ~/mini_pupper_bsp",
                             "https://github.com/mangdangroboticsclub/mini_pupper_ros_bsp.git ~/mini_pupper_ros_bsp"]
stacks['v1_ROS']['scripts'] = ["~/mini_pupper_ros_bsp/setup.sh v1",
                               "sudo reboot"]
stacks['v2_ROS']['repos'] = [" https://github.com/mangdangroboticsclub/mini_pupper_2_bsp.git ~/mini_pupper_bsp",
                             "https://github.com/mangdangroboticsclub/mini_pupper_ros_bsp.git ~/mini_pupper_ros_bsp"]
stacks['v2_ROS']['scripts'] = ["~/mini_pupper_ros_bsp/setup.sh v2",
                               "sudo reboot"]


def write_cache(cache_file):
    with open(cache_file, 'w') as fh:
        for key in target_environment.keys():
            fh.write("%s: %s\n" % (key, target_environment[key]))


def ask_user(prompt, var_name):
    input_text = input("%s: " % prompt)
    target_environment[var_name] = input_text


def ask_questions():
    if 'wifi_ssid' not in target_environment.keys():
        ask_user("Your WiFi SSID", 'wifi_ssid')
    if 'wifi_password' not in target_environment.keys():
        ask_user("Your WiFi password", 'wifi_password')
    if 'ubuntu_password' not in target_environment.keys():
        ask_user("Mini Pupper user password", 'ubuntu_password')
    if 'hardware_version' not in target_environment.keys():
        print("Which Mini Pupper Hardware do you want to install:\n")
        for i in range(len(hardware_version)):
            print("%s: %s" % (i + 1, hardware_version[i]))
        ask_user("Please enter number", 'hardware_version')
    if 'stack' not in target_environment.keys():
        print("Select stack:\n")
        for i in range(len(stack_names)):
            print("%s: %s" % (i + 1, stack_names[i]))
        ask_user("Please enter number", 'stack')
    if 'sd_path' not in target_environment.keys():
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
target_environment['selected_stack'] = "%s_%s" % (hardware_version[int(target_environment['hardware_version']) - 1],
                                                  stack_names[int(target_environment['stack']) - 1])
if args.cache:
    write_cache(conf_file)

network_conf_file = os.path.join(target_environment['sd_path'], 'network-config')
if not os.path.exists(network_conf_file):
    sys.exit("Invalid path to SD card or SD card not mounted\n")

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
package_upgrade: true
package_reboot_if_required: true
ssh_pwauth: True
chpasswd:
  expire: false
  list:
  - ubuntu:%s
packages:
- git
runcmd:
%s
%s
"""
repos = ''
for repo in stacks[target_environment['selected_stack']]['repos']:
    repos += '- [ su, ubuntu, -c, "git clone %s" ]\n' % repo
scripts = ''
for script in stacks[target_environment['selected_stack']]['scripts']:
    scripts += '- [ su, ubuntu, -c, "%s 2>> /home/ubuntu/.setup_err.log >> /home/ubuntu/.setup_out.log" ]\n' % script

with open(user_data_file, 'w') as fh:
    fh.write(user_data % (target_environment['ubuntu_password'],
                          repos[:-1],
                          scripts[:-1]))

print("Flashed cloud-config successfully")
