#!/usr/bin/env python
import subprocess

def get_pid_list():
    temp = subprocess.check_output('ps ax | grep /devel/bin/unibw/bin', shell=True).split('\n')
    pids, names = [], []
    for val in temp:
        if val.find('grep') == -1 and val:
            pid = val.split(' ')[0]
            if not pid:
                try:
                    pid = val.split(' ')[1]
                except Exception as e:
                    pass
                    # we need this except cause ps ax aslo finds the pid of the grep command itself
            name = val.split("/devel/bin/unibw/bin")[1].split(" ")[0]
            if pid:
                pids.append(pid)
                names.append(name)
    return [names, pids]
