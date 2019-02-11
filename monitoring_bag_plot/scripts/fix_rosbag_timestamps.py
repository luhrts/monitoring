#!/usr/bin/env python
import rospy
import rosbag
import sys


def parse_sys_args():
    if len(sys.argv) > 2:
        path = sys.argv[1]
        name = sys.argv[2]
        spath = sys.argv[3]
        return path, name, spath
    else:
        print "required 3 args"
        print "1 Arg:   /Path/for/loading"
        print "2 Arg:   Name_of_bag"
        print "3 Arg: path/to/safe"
        return None, None, None



def main():
    lpath, name, spath = parse_sys_args()
    if not lpath or not name or not spath:
        return

    with rosbag.Bag(spath+name.replace('.bag','')+'fixed.bag', 'w') as outbag:
        for topic, msg, t in rosbag.Bag(lpath+name).read_messages():
            stamp = None
            if msg._has_header:
                stamp = msg.header.stamp

            if topic == 'statistics':
                outbag.write(topic, msg, msg.window_stop)
            elif topic == 'tf':
                outbag.write(topic, msg, msg.transforms[0].header.stamp)
            elif stamp:
                outbag.write(topic, msg, stamp)
            else:
                outbag.write(topic, msg, t)

if __name__=='__main__':
    main()
