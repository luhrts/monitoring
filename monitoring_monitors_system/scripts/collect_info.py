#!/usr/bin/python2.7

import re
import os
import subprocess as sp
from subprocess import PIPE

def collect_ros_info():
    print("Get ROS infos:")
    print("\tGet ROS nodes.")
    f = open('rosnode.info', 'w+')
    p = sp.Popen(['rosnode', 'list', '-a'], stdout=PIPE, stderr=PIPE)
    out, err = p.communicate()
    f.write("---- Output ----\n" + out + "\n\n---- Error ----\n" + err)
    f.close()

    print("\tGet ROS params.")
    f = open('rosparam_complete.info', 'w+')
    p = sp.Popen(['rosparam', 'get', '/'], stdout=PIPE, stderr=PIPE)
    out, err = p.communicate()
    f.write("---- Output ----\n" + out + "\n\n---- Error ----\n" + err)
    f.close()

    print("\tGet ROS services.")
    f = open('rosservice.info', 'w+')
    p = sp.Popen(['rosservice', 'list', '-n'], stdout=PIPE, stderr=PIPE)
    out, err = p.communicate()
    f.write("---- Output ----\n" + out + "\n\n---- Error ----\n" + err)
    f.close()

    print("\tGet ROS-WTF.")
    f = open('roswtf.info', 'w+')
    p = sp.Popen(['roswtf'], stdout=PIPE, stderr=PIPE)
    out, err = p.communicate()
    f.write("---- Output ----\n" + out + "\n\n---- Error ----\n" + err)
    f.close()

    print("\tGet ROS packages.")
    f = open('rospackage.info', 'w+')
    p = sp.Popen(['rospack', 'list'], stdout=PIPE, stderr=PIPE)
    out, err = p.communicate()
    f.write("---- Output ----\n" + out + "\n\n---- Error ----\n" + err)
    f.close()

def collect_hw_info():
    print("Get Hardware infos:")
    print("\tGet installed hardware.")
    f = open('hardware.info', 'w+')
    p = sp.Popen(['sudo', 'lshw'], stdout=PIPE, stderr=PIPE)
    out, err = p.communicate()
    f.write("---- Output ----\n" + out + "\n\n---- Error ----\n" + err)
    f.close()

    print("\tGet memory info.")
    f = open('memory.info', 'w+')
    p = sp.Popen(['sudo', 'dmidecode', '--type', 'memory'], stdout=PIPE, stderr=PIPE)
    out, err = p.communicate()
    f.write("---- Output ----\n" + out + "\n\n---- Error ----\n" + err)
    f.close()

    print("\tGet cpu details.")
    f = open('cpu.info', 'w+')
    p = sp.Popen(['lscpu'], stdout=PIPE, stderr=PIPE)
    out, err = p.communicate()
    f.write("---- Output ----\n" + out + "\n\n---- Error ----\n" + err)
    f.close()

    print("\tGet volume usage.")
    f = open('volumes_usage.info', 'w+')
    p = sp.Popen(['df', '-ha'], stdout=PIPE, stderr=PIPE)
    out, err = p.communicate()
    f.write("---- Output ----\n" + out + "\n\n---- Error ----\n" + err)
    f.close()

    print("\tGet volume structure.")
    f = open('volumes.info', 'w+')
    p = sp.Popen(['lsblk'], stdout=PIPE, stderr=PIPE)
    out, err = p.communicate()
    f.write("---- Output ----\n" + out + "\n\n---- Error ----\n" + err)
    f.close()


def collect_network_info():
    print("Get network infos:")
    print("\tGet host-file contents.")
    f = open('host_file.info', 'w+')
    p = sp.Popen(['cat', '/etc/hosts'], stdout=PIPE, stderr=PIPE)
    out, err = p.communicate()
    f.write("---- Output ----\n" + out + "\n\n---- Error ----\n" + err)
    f.close()

    print("\tGet interfaces.")
    f = open('interfaces.info', 'w+')
    p = sp.Popen(['ifconfig'], stdout=PIPE, stderr=PIPE)
    out, err = p.communicate()
    f.write("---- Output ----\n" + out + "\n\n---- Error ----\n" + err)
    f.close()

    print("\tGet network neighbors.")
    f = open('network_neighbors.info', 'w+')
    p = sp.Popen(['ip', 'neigh'], stdout=PIPE, stderr=PIPE)
    out, err = p.communicate()
    f.write("---- Output ----\n" + out + "\n\n---- Error ----\n" + err)
    f.close()

    print("\tScan Network:")
    p = sp.Popen(['ip', 'address'], stdout=PIPE, stderr=PIPE)
    out, err = p.communicate()

    split = re.split('(\A|\n)\d: ', out)
    interfaces = []
    for e in split:
        if len(e) > 1:
            iface = e.split(':', 1)[0]
            if not iface == 'lo':
                interfaces.append(iface)

    iface_dict = {}
    for iface in interfaces:
        p = sp.Popen(['ip', '-4', 'addr', 'show', iface], stdout=PIPE, stderr=PIPE)
        out, err = p.communicate()

        m = re.search('(?<=inet\s)\d+(\.\d+){3}/\d{1,2}', out)
        if m:
            iface_dict.update({iface: m.group(0)})

    for k in iface_dict:
        print("\t\t" + k + " (" + iface_dict[k] + ")")
        f = open('nmap_' + k + '.info', 'w+')
        p = sp.Popen(['nmap', '-sn', iface_dict[k]], stdout=PIPE, stderr=PIPE)
        out, err = p.communicate()
        f.write("---- Output ----\n" + out + "\n\n---- Error ----\n" + err)
        f.close()


def collect_software_info():
    print("Get software infos:")
    print("\tGet OS info.")
    f = open('system.info', 'w+')
    p = sp.Popen(['uname', '-a'], stdout=PIPE, stderr=PIPE)
    out, err = p.communicate()
    f.write("---- Output ----\n" + out + "\n\n---- Error ----\n" + err)
    f.close()

    print("\tGet running processes.")
    f = open('running_processes.info', 'w+')
    p0 = sp.Popen(['ps', 'aux'], stdout=PIPE)
    p = sp.Popen(['less'], stdin=p0.stdout, stdout=PIPE, stderr=PIPE)
    p0.stdout.close()
    out, err = p.communicate()
    f.write("---- Output ----\n" + out + "\n\n---- Error ----\n" + err)
    f.close()

    print("\tGet apt installed software.")
    f = open('apt_installed_pkgs.info', 'w+')
    p = sp.Popen(['apt', 'list', '--installed'], stdout=PIPE, stderr=PIPE)
    out, err = p.communicate()
    f.write("---- Output ----\n" + out + "\n\n---- Error ----\n" + err)
    f.close()

    print("\tGet dpkg software.")
    f = open('dpkg_installed_packages.info', 'w+')
    p = sp.Popen(['dpkg', '-l'], stdout=PIPE, stderr=PIPE)
    out, err = p.communicate()
    f.write("---- Output ----\n" + out + "\n\n---- Error ----\n" + err)
    f.close()

    print("\tGet ntp configuration..")
    f = open('ntp.info', 'w+')
    p = sp.Popen(['cat', '/etc/ntp.conf'], stdout=PIPE, stderr=PIPE)
    out, err = p.communicate()
    f.write("---- Output ----\n" + out + "\n\n---- Error ----\n" + err)
    f.close()


def main():
    collect_hw_info()
    collect_software_info()
    collect_ros_info()
    collect_network_info()


if __name__ == "__main__":
    main()