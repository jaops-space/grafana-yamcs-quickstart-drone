#!/usr/bin/env python3

import binascii
import io
import socket
import sys
import argparse
import struct

from struct import unpack_from
from threading import Lock, Thread
from time import monotonic, sleep

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
DRONE_MODE_OFFSET = LEGACY_PACKET_SIZE + 4
DRONE_ARMED_OFFSET = LEGACY_PACKET_SIZE + 5
DRONE_FAILSAFE_OFFSET = LEGACY_PACKET_SIZE + 6
DRONE_MOTORS_OFFSET = LEGACY_PACKET_SIZE + 52
DRONE_POSITION_OFFSET = LEGACY_PACKET_SIZE + 108
DRONE_ALTITUDE_OFFSET = LEGACY_PACKET_SIZE + 116
DRONE_SPEED_OFFSET = LEGACY_PACKET_SIZE + 120
DRONE_PHASE_OFFSET = LEGACY_PACKET_SIZE + 170
DRONE_LAST_COMMAND_OFFSET = LEGACY_PACKET_SIZE + 186
DRONE_TARGET_ALTITUDE_OFFSET = LEGACY_PACKET_SIZE + 189
DRONE_PACKET_SIZE = LEGACY_PACKET_SIZE + 193

RESULT_ACCEPTED, RESULT_REJECTED, RESULT_FAILED = 1, 2, 3
OVERRIDE_HOLD_S = 45        # commanded modes hand back to the recording after this
DESCENT_RATE = 2.5          # m/s during an emergency landing
AIRBORNE_M = 0.5
IDLE_RPM, IDLE_THROTTLE = 960, 8

MODE_LABELS = {
    0: 'STANDBY',
    1: 'ARMED',
    2: 'TAKEOFF',
    3: 'SURVEY',
    4: 'RETURN_TO_HOME',
    5: 'LANDING',
    6: 'CHARGING',
    7: 'EMERGENCY',
}


def write_fixed_string(packet, offset, value, size=16):
    encoded = value.encode('utf-8')[:size - 1]
    packet[offset:offset + size] = encoded + bytes(size - len(encoded))

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

    def __init__(self, rate):
        self.tm_counter = 0
        self.tc_counter = 0
        self.tm_thread = None
        self.tc_thread = None
        self.last_tc = None
        self.rate = rate
        self.state_lock = Lock()
        self.mode_override = None
        self.armed_override = None
        self.override_until = 0.0
        self.altitude = 0.0         # last altitude sent, used to refuse unsafe commands
        self.position = (0.0, 0.0)  # last latitude and longitude sent
        self.emergency = None       # {'alt', 'start', 'lat', 'lon', 'landed'} after EmergencyLand
        self.target_altitude = 40.0
        self.last_command_id = 0
        self.last_command_result = 0
        self.motor_test = None

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
        return 'Sent: {} packets. Received: {} commands. Last command: {}'.format(
                         self.tm_counter, self.tc_counter, cmdhex)

    def hold(self, mode, armed, seconds=OVERRIDE_HOLD_S):
        self.mode_override, self.armed_override = mode, armed
        self.override_until = monotonic() + seconds

    def apply_command(self, data):
        """Apply the small command protocol defined in the drone MDB."""
        with self.state_lock:
            self.last_tc = data
            self.tc_counter += 1
            self.last_command_result = RESULT_FAILED
            if len(data) < 8:
                return

            command_id = struct.unpack_from('>H', data, 6)[0]
            self.last_command_id = command_id
            airborne = self.altitude > AIRBORNE_M
            try:
                if command_id == 10:  # Arm
                    if not airborne:
                        self.hold(1, True)
                elif command_id == 11:  # Disarm
                    if airborne:
                        # Never cut the motors in flight.
                        self.last_command_result = RESULT_REJECTED
                        return
                    self.hold(0, False)
                    self.motor_test = None
                elif command_id == 12 and len(data) >= 9:  # SetFlightMode
                    self.hold(data[8], True)
                elif command_id == 13 and len(data) >= 12:  # SetTargetAltitude
                    self.target_altitude = struct.unpack_from('>f', data, 8)[0]
                elif command_id == 14 and len(data) >= 12:  # MotorTest
                    motor, throttle, duration = struct.unpack_from('>BBH', data, 8)
                    if airborne or not 1 <= motor <= 4 or not 0 <= throttle <= 100:
                        self.last_command_result = RESULT_REJECTED
                        return
                    self.motor_test = (motor - 1, throttle, monotonic() + duration)
                elif command_id == 16 and len(data) >= 17:  # ConfigureGeofence
                    # Arguments are validated and encoded by Yamcs. The demo stores
                    # acknowledgement in telemetry; a real vehicle would persist them.
                    struct.unpack_from('>HffB', data, 8)
                elif command_id == 17:  # EmergencyLand
                    if airborne:
                        self.emergency = {'alt': self.altitude, 'start': monotonic(), 'landed': False,
                                          'lat': self.position[0], 'lon': self.position[1]}
                        self.hold(5, True, seconds=3600)
                else:
                    return
                self.last_command_result = RESULT_ACCEPTED
            except (IndexError, struct.error):
                self.last_command_result = RESULT_FAILED

    @staticmethod
    def set_motors(packet, rpm, throttle, status):
        for motor in range(4):
            offset = DRONE_MOTORS_OFFSET + motor * 8
            temperature = struct.unpack_from('>h', packet, offset + 4)[0]
            current = round(throttle * 0.22 / 0.01) if rpm else 0
            struct.pack_into('>HHhBB', packet, offset, rpm, current, temperature, throttle, status)

    def apply_command_state(self, packet):
        if len(packet) < DRONE_PACKET_SIZE:
            return
        with self.state_lock:
            now = monotonic()
            recorded_alt = struct.unpack_from('>f', packet, DRONE_ALTITUDE_OFFSET)[0]
            recorded_armed = bool(packet[DRONE_ARMED_OFFSET])
            if self.emergency is not None:
                em = self.emergency
                # Descend vertically from where the command was received, then stay
                # landed and disarmed until the recording is back on the pad too.
                altitude = 0.0 if em['landed'] else max(0.0, em['alt'] - DESCENT_RATE * (now - em['start']))
                struct.pack_into('>ffff', packet, DRONE_POSITION_OFFSET, em['lat'], em['lon'], altitude, 0.0)
                if altitude == 0.0 and not em['landed']:
                    em['landed'] = True
                    self.hold(0, False, seconds=3600)
                if em['landed']:
                    self.set_motors(packet, 0, 0, 0)
                    if recorded_alt <= AIRBORNE_M and not recorded_armed:
                        self.emergency = None
                        self.mode_override = self.armed_override = None
            elif self.mode_override is not None and now >= self.override_until:
                self.mode_override = self.armed_override = None

            if self.mode_override is not None:
                packet[DRONE_MODE_OFFSET] = self.mode_override
                write_fixed_string(packet, DRONE_PHASE_OFFSET, MODE_LABELS.get(self.mode_override, 'COMMAND_MODE'))
            if self.armed_override is not None:
                packet[DRONE_ARMED_OFFSET] = int(self.armed_override)
                if recorded_alt <= AIRBORNE_M and self.emergency is None:
                    # On the pad the motors follow the commanded arm state.
                    if self.armed_override:
                        self.set_motors(packet, IDLE_RPM, IDLE_THROTTLE, 1)
                    else:
                        self.set_motors(packet, 0, 0, 0)
            lat, lon, self.altitude = struct.unpack_from('>fff', packet, DRONE_POSITION_OFFSET)
            self.position = (lat, lon)

            if self.motor_test is not None:
                motor, throttle, stop_time = self.motor_test
                if monotonic() < stop_time:
                    offset = DRONE_MOTORS_OFFSET + motor * 8
                    struct.pack_into('>HHhBB', packet, offset, throttle * 122, throttle * 18, 320, throttle, 1)
                    write_fixed_string(packet, DRONE_PHASE_OFFSET, 'MOTOR_TEST')
                else:
                    self.motor_test = None

            struct.pack_into('>H', packet, DRONE_LAST_COMMAND_OFFSET, self.last_command_id)
            packet[DRONE_LAST_COMMAND_OFFSET + 2] = self.last_command_result
            struct.pack_into('>f', packet, DRONE_TARGET_ALTITUDE_OFFSET, self.target_altitude)


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
