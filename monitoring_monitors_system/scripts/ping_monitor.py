#!/usr/bin/env python

"""
    A pure python ping implementation using raw socket.


    Note that ICMP messages can only be sent from processes running as root.


    Derived from ping.c distributed in Linux's netkit. That code is
    copyright (c) 1989 by The Regents of the University of California.
    That code is in turn derived from code written by Mike Muuss of the
    US Army Ballistic Research Laboratory in December, 1983 and
    placed in the public domain. They have my thanks.

    Bugs are naturally mine. I'd be glad to hear about them. There are
    certainly word - size dependenceies here.

    Copyright (c) Matthew Dixon Cowles, <http://www.visi.com/~mdc/>.
    Distributable under the terms of the GNU General Public License
    version 2. Provided with no warranties of any sort.

    Original Version from Matthew Dixon Cowles:
      -> ftp://ftp.visi.com/users/mdc/ping.py

    Rewrite by Jens Diemer:
      -> http://www.python-forum.de/post-69122.html#69122


    Revision history
    ~~~~~~~~~~~~~~~~

    March 11, 2010
    changes by Samuel Stauffer:
    - replaced time.clock with default_timer which is set to
      time.clock on windows and time.time on other systems.

    May 30, 2007
    little rewrite by Jens Diemer:
     -  change socket asterisk import to a normal import
     -  replace time.time() with time.clock()
     -  delete "return None" (or change to "return" only)
     -  in checksum() rename "str" to "source_string"

    November 22, 1997
    Initial hack. Doesn't do much, but rather than try to guess
    what features I (or others) will want in the future, I've only
    put in what I need now.

    December 16, 1997
    For some reason, the checksum bytes are in the wrong order when
    this is run under Solaris 2.X for SPARC but it works right under
    Linux x86. Since I don't know just what's wrong, I'll swap the
    bytes always and then do an htons().

    December 4, 2000
    Changed the struct.pack() calls to pack the checksum and ID as
    unsigned. My thanks to Jerome Poincheval for the fix.

    Januari 27, 2015
    Changed receive response to not accept ICMP request messages.
    It was possible to receive the very request that was sent.

    Last commit info:
    ~~~~~~~~~~~~~~~~~
    $LastChangedDate: $
    $Rev: $
    $Author: $
"""


import os, sys, socket, struct, select, time, datetime

default_timer = time.time

# From /usr/include/linux/icmp.h; your milage may vary.
ICMP_ECHO_REQUEST = 8 # Seems to be the same on Solaris.
ICMP_ECHO_RESPONSE = 0
ICMP_TIMESTAMP_REQUEST = 13
ICMP_TIMSTAMP_RESPONSE = 14

sequence = 0

def checksum(source_string):
    """
    I'm not too confident that this is right but testing seems
    to suggest that it gives the same answers as in_cksum in ping.c
    """
    sum = 0
    countTo = (len(source_string)/2)*2
    count = 0
    while count<countTo:
        thisVal = ord(source_string[count + 1])*256 + ord(source_string[count])
        sum = sum + thisVal
        sum = sum & 0xffffffff # Necessary?
        count = count + 2

    if countTo<len(source_string):
        sum = sum + ord(source_string[len(source_string) - 1])
        sum = sum & 0xffffffff # Necessary?

    sum = (sum >> 16)  +  (sum & 0xffff)
    sum = sum + (sum >> 16)
    answer = ~sum
    answer = answer & 0xffff

    # Swap bytes. Bugger me if I know why.
    answer = answer >> 8 | (answer << 8 & 0xff00)

    return answer

def receive_one_timestamp(my_socket, ID, timeout):
    """
    receive the ping from the socket.
    """

    rtt = -1

    timeLeft = timeout
    while True:
        startedSelect = time.time()
        whatReady = select.select([my_socket], [], [], timeLeft)
        howLongInSelect = (time.time() - startedSelect)
        if whatReady[0] == []: # Timeout
            return

        lt = datetime.datetime.utcnow()
        timeRecvClient  = ((lt.hour * 60 + lt.minute) * 60 + lt.second) * 1000 + lt.microsecond / 1000
        recPacket, addr = my_socket.recvfrom(1024)
        icmpHeader = recPacket[20:28]
        type, code, checksum, packetID, sequence = struct.unpack(
            "bbHHh", icmpHeader
        )
#        print "type: "+str(type)
#        print "code: "+str(code)
#        print "checksum: "+str(hex(checksum))
#        print "packetID: "+str(packetID)
#        print "sequence: "+str(sequence)

        # Filters out the echo request itself.
        # This can be tested by pinging 127.0.0.1
        # You'll see your own request
        if type == ICMP_TIMSTAMP_RESPONSE and packetID == ID:

#            print "len recv: "+str(len(recPacket))
            timeSentClient, timeRecvServer, timeSentServer = struct.unpack("!iii", recPacket[28:])

#            print "sent_client: "+str(timeSentClient)
#            print "recv_server: "+str(timeRecvServer)
#            print "sent_server: "+str(timeSentServer)
#            print "recv_client: "+str(timeRecvClient)

            A = timeRecvServer - timeSentClient
            B = timeRecvClient - timeSentServer
#            print A
#            print B
            offset = (A - B)/2.0
            return offset
        else:
            print "received icmp type: "+str(type)

        timeLeft = timeLeft - howLongInSelect
        if timeLeft <= 0:
            return


def send_one_timestamp(my_socket, dest_addr, ID):
    """
    Send one ping to the given >dest_addr<.
    """
    global sequence
    dest_addr  =  socket.gethostbyname(dest_addr)

    # Header is type (8), code (8), checksum (16), id (16), sequence (16)
    my_checksum = 0
    sequence += 1

    # Make a dummy heder with a 0 checksum.
    header = struct.pack("bbHHh", ICMP_TIMESTAMP_REQUEST, 0, my_checksum, ID, sequence)
    lt = datetime.datetime.utcnow()


    timestamp  = ((lt.hour * 60 + lt.minute) * 60 + lt.second) * 1000 + lt.microsecond / 1000
#    print " "
#    print "send_ts: "+str(timestamp)
    data = struct.pack("!lll", timestamp, 0, 0)

    # Calculate the checksum on the data and the dummy header.
    my_checksum = checksum(header + data)

    # Now that we have the right checksum, we put that in. It's just easier
    # to make up a new header than to stuff it into the dummy.
    header = struct.pack(
        "bbHHh", ICMP_TIMESTAMP_REQUEST, 0, socket.htons(my_checksum), ID, sequence
    )
    packet = header + data
    my_socket.sendto(packet, (dest_addr, 1)) # Don't know about the 1


def receive_one_ping(my_socket, ID, timeout):
    """
    receive the ping from the socket.
    """

    rtt = -1

    timeLeft = timeout
    while True:
        startedSelect = default_timer()
        whatReady = select.select([my_socket], [], [], timeLeft)
        howLongInSelect = (default_timer() - startedSelect)
        if whatReady[0] == []: # Timeout
            return

        timeReceived = default_timer()
        recPacket, addr = my_socket.recvfrom(1024)
        icmpHeader = recPacket[20:28]
        type, code, checksum, packetID, sequence = struct.unpack(
            "bbHHh", icmpHeader
        )
        # Filters out the echo request itself.
        # This can be tested by pinging 127.0.0.1
        # You'll see your own request
        if type != 8 and packetID == ID:
            bytesInDouble = struct.calcsize("d")
            timeSent = struct.unpack("d", recPacket[28:28 + bytesInDouble])[0]
            rtt = timeReceived - timeSent
            return rtt
        else:
            print "received icmp type: "+str(type)

        timeLeft = timeLeft - howLongInSelect
        if timeLeft <= 0:
            return


def send_one_ping(my_socket, dest_addr, ID):
    """
    Send one ping to the given >dest_addr<.
    """
    dest_addr  =  socket.gethostbyname(dest_addr)

    # Header is type (8), code (8), checksum (16), id (16), sequence (16)
    my_checksum = 0

    # Make a dummy heder with a 0 checksum.
    header = struct.pack("bbHHh", ICMP_ECHO_REQUEST, 0, my_checksum, ID, 1)
    bytesInDouble = struct.calcsize("d")
    data = (192 - bytesInDouble) * "Q"
    data = struct.pack("d", default_timer()) + data

    # Calculate the checksum on the data and the dummy header.
    my_checksum = checksum(header + data)

    # Now that we have the right checksum, we put that in. It's just easier
    # to make up a new header than to stuff it into the dummy.
    header = struct.pack(
        "bbHHh", ICMP_ECHO_REQUEST, 0, socket.htons(my_checksum), ID, 1
    )
    packet = header + data
    my_socket.sendto(packet, (dest_addr, 1)) # Don't know about the 1


def do_one(dest_addr, timeout):
    """
    Returns either the delay (in seconds) or none on timeout.
    """
    icmp = socket.getprotobyname("icmp")
    try:
        my_socket = socket.socket(socket.AF_INET, socket.SOCK_RAW, icmp)
        SO_TIMESTAMPNS = 35
        #s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(3))
        my_socket.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)

    except socket.error, (errno, msg):
        if errno == 1:
            # Operation not permitted
            msg = msg + (
                " - Note that ICMP messages can only be sent from processes"
                " running as root."
            )
            raise socket.error(msg)
        raise # raise the original error

    my_ID = os.getpid() & 0xFFFF

    send_one_ping(my_socket, dest_addr, my_ID)
    rtt = receive_one_ping(my_socket, my_ID, timeout)

    send_one_timestamp(my_socket, dest_addr, my_ID)
    offset = receive_one_timestamp(my_socket, my_ID, timeout)

    my_socket.close()
    return rtt, offset


def verbose_ping(dest_addr, timeout = 2, count = 4):
    """
    Send >count< ping to >dest_addr< with the given >timeout< and display
    the result.
    """
    for i in xrange(count):
        print "ping %s..." % dest_addr,
        try:
            rtt, offset  =  do_one(dest_addr, timeout)
        except socket.gaierror, e:
            print "failed. (socket error: '%s')" % e[1]
            break

        if rtt  ==  None:
            print "failed. (timeout within %ssec.)" % timeout
        else:
            rtt  =  rtt * 1000
            print "RTT: %f ms" % (rtt),

        if offset  ==  None:
            print "failed. (timeout within %ssec.)" % timeout
        else:
            print "Offset: %f ms" % (offset)

    print


if __name__ == '__main__':
    #verbose_ping("130.75.137.10")
    #verbose_ping("130.75.137.127")

    verbose_ping("google.de")
