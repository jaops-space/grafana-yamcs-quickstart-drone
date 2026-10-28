#!/usr/bin/env python3

import binascii
import io
import socket
import sys
import argparse
import struct

from struct import unpack_from
from threading import Lock, Thread
from time import sleep

from drone_model import ARMED, EMERGENCY, EXTENSION_SIZE, NAMES, STANDBY, DroneModel

parser = argparse.ArgumentParser(description='Yamcs Simulator')
parser.add_argument('--testdata', type=str, default='testdata.ccsds', help='simulated testdata.ccsds data')

# telemetry
parser.add_argument('--tm_host',    type=str, default='127.0.0.1', help='TM host')
parser.add_argument('--tm_port',    type=int, default=10015,       help='TM port')
parser.add_argument('-r', '--rate', type=int, default=1,           help='TM playback rate. 1 = 1Hz, 10 = 10Hz, etc.')

# telecommand
parser.add_argument('--tc_host', type=str, default='0.0.0.0', help='TC bind address')
parser.add_argument('--tc_port', type=int, default=10025 ,      help='TC port')

args = vars(parser.parse_args())

# test data
TEST_DATA = args['testdata']

# telemetry
TM_SEND_ADDRESS = args['tm_host']
TM_SEND_PORT    = args['tm_port']
RATE            = args['rate']

# telecommand
TC_RECEIVE_ADDRESS = args['tc_host']
TC_RECEIVE_PORT    = args['tc_port']

LEGACY_PACKET_SIZE = 123
RESULT_ACCEPTED, RESULT_REJECTED, RESULT_FAILED = 1, 2, 3

def send_tm(simulator):
    tm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    with io.open(TEST_DATA, 'rb') as f:
        simulator.tm_counter = 1
        header = bytearray(6)
        while f.readinto(header) == 6:
            (len,) = unpack_from('>H', header, 4)

            packet = bytearray(len + 7)
            f.seek(-6, io.SEEK_CUR)
            f.readinto(packet)

            simulator.apply_command_state(packet)
            # Compose may start the simulator just before the Yamcs service has
            # joined DNS. Keep the current packet and retry instead of letting
            # the telemetry thread die during startup.
            while True:
                try:
                    tm_socket.sendto(packet, (TM_SEND_ADDRESS, TM_SEND_PORT))
                    break
                except OSError:
                    sleep(1)
            simulator.tm_counter += 1

            sleep(1 / simulator.rate)


def receive_tc(simulator):
    tc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tc_socket.bind((TC_RECEIVE_ADDRESS, TC_RECEIVE_PORT ))
    while True:
        data, _ = tc_socket.recvfrom(4096)
        simulator.apply_command(data)


class Simulator():
    """Plays back the spacecraft part of the recording and runs the drone live.

    The drone part of each packet comes from drone_model, stepped once per packet,
    so commands change what the vehicle actually does instead of overwriting bytes.
    """

    def __init__(self, rate):
        self.tm_counter = 0
        self.tc_counter = 0
        self.tm_thread = None
        self.tc_thread = None
        self.last_tc = None
        self.rate = rate
        self.state_lock = Lock()
        self.drone = DroneModel()
        self.last_command_id = 0
        self.last_command_result = 0

    def start(self):
        self.tm_thread = Thread(target=send_tm, args=(self,))
        self.tm_thread.daemon = True
        self.tm_thread.start()
        self.tc_thread = Thread(target=receive_tc, args=(self,))
        self.tc_thread.daemon = True
        self.tc_thread.start()

    def print_status(self):
        cmdhex = None
        if self.last_tc:
            cmdhex = binascii.hexlify(self.last_tc).decode('ascii')
        return 'Sent: {} packets. Received: {} commands. State: {}. Last command: {}'.format(
            self.tm_counter, self.tc_counter, NAMES[self.drone.state], cmdhex)

    def apply_command(self, data):
        """Map the drone MDB commands onto state machine transition requests."""
        with self.state_lock:
            self.last_tc = data
            self.tc_counter += 1
            self.last_command_result = RESULT_FAILED
            if len(data) < 8:
                return
            command_id = struct.unpack_from('>H', data, 6)[0]
            self.last_command_id = command_id
            drone = self.drone
            try:
                if command_id == 10:                                # Arm
                    accepted = drone.request(ARMED)
                elif command_id == 11:                              # Disarm
                    accepted = drone.request(STANDBY)
                elif command_id == 12 and len(data) >= 9:           # SetFlightMode
                    accepted = drone.request(data[8])
                elif command_id == 13 and len(data) >= 12:          # SetTargetAltitude
                    drone.target_alt = struct.unpack_from('>f', data, 8)[0]
                    accepted = True
                elif command_id == 14 and len(data) >= 12:          # MotorTest, on the ground only
                    motor, throttle, duration = struct.unpack_from('>BBH', data, 8)
                    accepted = drone.state in (STANDBY, ARMED) and 1 <= motor <= 4 and 0 <= throttle <= 100
                    if accepted:
                        drone.motor_test = (motor - 1, throttle, drone.mission_t + duration)
                elif command_id == 16 and len(data) >= 17:          # ConfigureGeofence
                    struct.unpack_from('>HffB', data, 8)            # validated by Yamcs; acknowledged only
                    accepted = True
                elif command_id == 17:                              # EmergencyLand
                    accepted = drone.request(EMERGENCY)
                else:
                    return
                self.last_command_result = RESULT_ACCEPTED if accepted else RESULT_REJECTED
            except (IndexError, struct.error):
                self.last_command_result = RESULT_FAILED

    def apply_command_state(self, packet):
        """Step the drone one packet period and write its telemetry into the packet."""
        if len(packet) < LEGACY_PACKET_SIZE + EXTENSION_SIZE:
            return
        with self.state_lock:
            self.drone.step(1.0 / self.rate)
            packet[LEGACY_PACKET_SIZE:LEGACY_PACKET_SIZE + EXTENSION_SIZE] = self.drone.pack(
                last_command=(self.last_command_id, self.last_command_result))


if __name__ == '__main__':
    simulator = Simulator(RATE)
    simulator.start()
    sys.stdout.write('Using playback rate of ' + str(RATE) + 'Hz, ');
    sys.stdout.write('TM host=' + str(TM_SEND_ADDRESS) + ', TM port=' + str(TM_SEND_PORT) + ', ');
    sys.stdout.write('TC host=' + str(TC_RECEIVE_ADDRESS) + ', TC port=' + str(TC_RECEIVE_PORT) + '\r\n');
    try:
        prev_status = None
        while True:
            status = simulator.print_status()
            if status != prev_status:
                sys.stdout.write('\r')
                sys.stdout.write(status)
                sys.stdout.flush()
                prev_status = status
            sleep(0.5)
    except KeyboardInterrupt:
        sys.stdout.write('\n')
        sys.stdout.flush()
