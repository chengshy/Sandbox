#!/usr/bin/env python
import socket
import select
import struct
import threading

class UdpReader(threading.Thread):
    def __init__(self, mcast_group = '239.255.42.99', mcast_port = 1511):
        threading.Thread.__init__(self)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((mcast_group, mcast_port))
        mreq = struct.pack("4sl", socket.inet_aton(mcast_group), socket.INADDR_ANY)
        self._sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        self._sock.setblocking(0)

    def run(self):
        while 1:
            readable, writable, errored = select.select([self._sock], [], [], 0.1)
            if self._sock in errored:
                # Something bad has happened
                print('Error reading from socket')
            if not self._sock in readable:
                # No data from optitrack yet; just keep looping
                continue

            packet_str = self._sock.recv(1000000)
            self.handle_packet(packet_str)

    def handle_packet(self, packet_str):
        byte_i = 0
        (i_message, n_data_bytes) = struct.unpack('HH', packet_str[byte_i:byte_i+4])
        byte_i += 4
        print('i_message: ', i_message, '  n_date_bytes: ', n_data_bytes)
        if i_message == 7: # frame of mocap data packet
            (frame_number, n_marker_sets) = struct.unpack('ii', packet_str[byte_i:byte_i+8])
            byte_i += 8

            marker_ids = []
            for marker_set_i in range(n_marker_sets):
                 # Name as C string (ends in 0 byte)
                 name_length = packet_str[byte_i:].find('\00')
                 name = packet_str[byte_i:byte_i+name_length]
                 marker_ids.append(name)
                 byte_i += name_length
                 byte_i += 1

                 (n_markers,) = struct.unpack('i', packet_str[byte_i:byte_i+4])
                 byte_i += 4

                 for marker_i in range(n_markers):
                     (x, y, z) = struct.unpack('fff', packet_str[byte_i:byte_i+12])
                     byte_i += 12

            (n_unidentified_markers,) = struct.unpack('i', packet_str[byte_i:byte_i+4])
            byte_i += 4
            print('%d unidentified markers' % (n_unidentified_markers,))
            for marker_i in range(n_unidentified_markers):
                (x, y, z) = struct.unpack('fff', packet_str[byte_i:byte_i+12])
                byte_i += 12

            (n_rigid_bodies,) = struct.unpack('i', packet_str[byte_i:byte_i+4])
            byte_i += 4
            print('%d rigid bodies' % (n_rigid_bodies,))
            for body_i in range(n_rigid_bodies):
                (idnum, x, y, z, qx, qy, qz, qw,) = struct.unpack('ifffffff', packet_str[byte_i:byte_i+32])
                rigid_body_name = marker_ids[body_i]
                byte_i += 32

                (n_markers,) = struct.unpack('i', packet_str[byte_i:byte_i+4])
                if n_markers > 100 or n_markers < 0:
                    print('Bad number of packets (%d), probably a parse error' % n_markers)
                    return
                byte_i += 4
                # Skip marker positions
                byte_i += n_markers * 3 * 4 # x, y, z floats for each marker
                # Skip marker ids
                this_body_marker_ids = []
                for marker_i in range(n_markers):
                    (marker_idnum,) = struct.unpack('i', packet_str[byte_i:byte_i+4])
                    this_body_marker_ids.append(marker_idnum)
                    byte_i += 4
                # Skip marker sizes
                byte_i += n_markers * 4
                # Skip mean marker error
                (mean_marker_error,) = struct.unpack('f', packet_str[byte_i:byte_i+4])
                byte_i += 4

                # No idea why, but there appear to be two extra bytes at the end of each rigid body.
                # this seems to be now (can't find it anywhere in example code) -jbinney
                (valid_flag,) = struct.unpack('h', packet_str[byte_i:byte_i+2])
                print('valid_flag: ', valid_flag)
                byte_i += 2

                # Normalize quaternion
                qlen = (qx*qx + qy*qy + qz*qz + qw*qw)**0.5
                (qx, qy, qz, qw) = (qx / qlen, qy / qlen, qz / qlen, qw / qlen)

if __name__ == '__main__':
    udp_reader = UdpReader()
    udp_reader.run()

