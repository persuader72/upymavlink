#!/usr/bin/python
# -*- coding: utf-8 -*-

# minified mavlink python utility functions
# Copyright by Stefano Pagnottelli
# Based on mavlink.py By Andrew Tridgell
# Released under GNU GPL version 3 or later

import time
import serial
import socket
import errno

import dialect

LINK_SERIAL = 'serial'
LINK_TCP = 'tcp'
LINK_UDP = 'udp'

def set_close_on_exec(fd):
    try:
        import fcntl
        flags = fcntl.fcntl(fd, fcntl.F_GETFD)
        flags |= fcntl.FD_CLOEXEC
        fcntl.fcntl(fd, fcntl.F_SETFD, flags)
    except Exception:
        pass


class PeriodicEvent(object):
    '''a class for fixed frequency events'''
    def __init__(self, frequency):
        self.frequency = float(frequency)
        self.last_time = time.time()

    def force(self):
        '''force immediate triggering'''
        self.last_time = 0

    def trigger(self):
        '''return True if we should trigger now'''
        tnow = time.time()
        if self.last_time + (1.0/self.frequency) <= tnow:
            self.last_time = tnow
            return True
        return False


class Mavlink(object):
    def __init__(self, device, baudrate=115200, linktype=None, options={}):
        self.link_type = linktype if linktype is not None else LINK_SERIAL

        self.server = False
        self.addr = None
        self.port = None
        self.fd = None

        self.target_system = 0
        self.target_component = 0

        if self.link_type == LINK_SERIAL:
            self.port = serial.Serial(device, 1200, timeout=0, dsrdtr=False, rtscts=False, xonxoff=False)
            set_close_on_exec(self.port.fileno())
            self.port.setBaudrate(baudrate)
        elif self.link_type == LINK_TCP:
            a = device.split(':')
            self.port = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.addr = (a[0], int(a[1]))
            self.port.connect(self.addr)
            self.port.setblocking(0)
            set_close_on_exec(self.port.fileno())
            self.port.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)
        elif self.link_type == LINK_UDP:
            self.server = options.get('server', True)
            broadcast = options.get('broadcast', False)
            a = device.split(':')
            self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.addr = (a[0], int(a[1]))
            if self.server:
                self.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.port.bind((a[0], int(a[1])))
            else:
                if broadcast:
                    self.port.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            set_close_on_exec(self.port.fileno())
            self.port.setblocking(0)

        self.fd = self.port.fileno()

        self.mav_count = 0
        self.mav_loss = 0
        self.mav_last_seq = -1

        self.callback = None
        self.callback_args = None
        self.callback_kwargs = None

        self.mav = dialect.MAVLink(self)
        self.mav.set_callback(self.master_callback)

    def set_callback(self, callback, *args, **kwargs):
        self.callback = callback
        self.callback_args = args
        self.callback_kwargs = kwargs

    def recv(self, n=None):
        if n is None:
            n = self.mav.bytes_needed()
        if self.fd is None:
            waiting = self.port.inWaiting()
            if waiting < n:
                n = waiting
        ret = self.port.read(n)
        if len(ret) == 0:
            time.sleep(0.01)
        return ret

    def recv_tcp(self, n=None):
        if n is None:
            n = self.mav.bytes_needed()
        try:
            data = self.port.recv(n)
        except socket.error as e:
            if e.errno in [errno.EAGAIN, errno.EWOULDBLOCK]:
                return ""
            raise
        return data

    def recv_udp(self, n=None):
        try:
            data, self.addr = self.port.recvfrom(300)
        except socket.error as e:
            if e.errno in [ errno.EAGAIN, errno.EWOULDBLOCK, errno.ECONNREFUSED]:
                return ""
            raise
        return data


    def write(self, buf):
        if self.link_type == LINK_SERIAL:
            self.port.write(buf)
        elif self.link_type == LINK_TCP:
            self.write_tcp(buf)
        elif self.link_type == LINK_UDP:
            self.write_udp(buf)

    def write_tcp(self, buf):
        try:
            self.port.send(buf)
        except socket.error:
            pass

    def write_udp(self, buf):
        try:
            if self.addr:
                self.port.sendto(buf, self.addr)
        except socket.error:
            pass


    def mav_recv(self):
        try:
            if self.link_type == LINK_SERIAL:
                s = self.recv(16*1024)
            elif self.link_type == LINK_TCP:
                s = self.recv_tcp(16*1024)
            elif self.link_type == LINK_UDP:
                s = self.recv_udp(16*1024)
        except Exception:
            time.sleep(0.1)
            return
        if len(s) == 0:
            time.sleep(0.1)
            return

        try:
            msgs = self.mav.parse_buffer(s)
        except dialect.MAVError, e:
            msgs = None
            print 'MAVError' + str(e)

        if msgs:
            for msg in msgs:
                #if getattr(msg, '_timestamp', None) is None:
                    #self.post_message(msg)
                if msg.get_type() == "BAD_DATA":
                    pass

    def post_message(self, msg):
        """ default post message call
        :param msg: MAVLink just received message
        :type  msg: MAVLink_message
        """

        if '_posted' in msg.__dict__:
            return
        msg._posted = True
        msg._timestamp = time.time()

        type = msg.get_type()
        if type == 'HEARTBEAT':
            self.target_system = msg.get_srcSystem()
            self.target_component = msg.get_srcComponent()

        last_seq = self.mav_last_seq
        seq = (last_seq+1) % 256
        seq2 = msg.get_seq()

        if seq != seq2 and last_seq != -1:
            diff = (seq2 - seq) % 256
            self.mav_loss += diff

        self.mav_last_seq = seq2
        self.mav_count += 1

        pass

    def master_callback(self, m):
        """ process mavlink message m on master, sending any messages to recipients
        :param m: MAVLink just received message
        :type  m: MAVLink_message
        """

        if getattr(m, '_timestamp', None) is None:
            self.post_message(m)

        if self.callback:
            self.callback(m, *self.callback_args, **self.callback_kwargs)

    def request_data_stream(self,stream, rate):
        self.mav.request_data_stream_send(self.target_system, self.target_component,stream, rate, 1 if rate > 0 else 0)


