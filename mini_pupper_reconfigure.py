#!/usr/bin/env python3
import argparse
import sys
import os

##############################################################
#
# Ask questions to prepare cloud-init file for mini pupper reconfiguration
#
# if ~/.mp_ros_bsp_sd.txt exists no questions will be asked
#
##############################################################

target_environment = {}


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
        ask_user("Mini pupper user password", 'ubuntu_password')
    if 'sd_path' not in target_environment.keys():
        ask_user("Full path to SD card", 'sd_path')


# Detect OS
target_environment['this_os'] = sys.platform

parser = argparse.ArgumentParser(description='Prepare SD card for Mini Pupper')
parser.add_argument('-c', '--cache',
                    action='store_true',
                    help='Cache my responses')
args = parser.parse_args()

conf_file = os.path.join(os.path.expanduser("~"), '.mini_pupper_reconfigure.txt')
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
ssh_pwauth: True
chpasswd:
  expire: false
  list:
  - ubuntu:%s
"""

with open(user_data_file, 'w') as fh:
    fh.write(user_data % (target_environment['ubuntu_password']))

reconfigure_file = os.path.join(target_environment['sd_path'], 'reconfigure')
with open(reconfigure_file, 'w') as fh:
    fh.write('reconfigure requested')
