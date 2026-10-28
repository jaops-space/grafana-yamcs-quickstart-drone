"""Drone flight state machine and physics, shared by the simulator and the recording generator.

States and the only transitions allowed between them:

    CHARGING <-> STANDBY <-> ARMED -> TAKEOFF
                    ^                    |
                    |                    v
    EMERGENCY <- LANDING <- RETURN <- SURVEY
        |
        +-> STANDBY        EMERGENCY is entered from any flight state.

The mission advances on its own when a state's exit condition is met (target
altitude reached, home reached, touchdown, battery charged). Commands may only
request a transition the table allows; anything else is rejected. Position,
altitude and attitude are integrated each step, so the vehicle never jumps.
"""

from __future__ import annotations

import math
import struct

STANDBY, ARMED, TAKEOFF, SURVEY, RETURN, LANDING, CHARGING, EMERGENCY = range(8)
NAMES = ["STANDBY", "ARMED", "TAKEOFF", "SURVEY", "RETURN_TO_HOME", "LANDING", "CHARGING", "EMERGENCY"]
FLIGHT = {TAKEOFF, SURVEY, RETURN, LANDING}

TRANSITIONS = {
    STANDBY: {ARMED, CHARGING},
    ARMED: {STANDBY, TAKEOFF},
    TAKEOFF: {SURVEY, EMERGENCY},
    SURVEY: {RETURN, EMERGENCY},
    RETURN: {LANDING, EMERGENCY},
    LANDING: {STANDBY, EMERGENCY},
    EMERGENCY: {STANDBY},
    CHARGING: {STANDBY},
}

HOME_LAT, HOME_LON = 48.8566, 2.3522
CIRCLE_R = 80.0              # survey circle through home, centre 80 m north
SURVEY_LAPS = 1.5
CRUISE_SPEED = 6.3           # m/s
RETURN_SPEED = 5.0
CLIMB_RATE, DESCENT_RATE, EMERGENCY_RATE = 1.8, 1.2, 2.5
SOC_FULL, SOC_LAUNCH = 96.0, 90.0
AMBIENT_C, G = 27.0, 9.81


def fixed_string(value: str, size: int = 16) -> bytes:
    encoded = value.encode("utf-8")[: size - 1]
    return encoded + bytes(size - len(encoded))


class DroneModel:
    def __init__(self) -> None:
        self.state = STANDBY
        self.in_state = 0.0               # seconds since the last transition
        self.mission_t = 0.0
        self.north = self.east = self.alt = 0.0
        self.vn = self.ve = self.vz = 0.0
        self.heading = 0.0
        self.roll = self.pitch = 0.0
        self.prev_roll = self.prev_pitch = self.prev_heading = 0.0
        self.soc = SOC_FULL
        self.motor_temp = [AMBIENT_C] * 4
        self.battery_temp = AMBIENT_C
        self.target_alt = 40.0
        self.survey_angle = math.pi       # angle on the circle; pi is home
        self.motor3 = 1                   # 1 running, 2 degraded, 3 fault
        self.failsafe_until = -1.0
        self.motor_test = None            # (motor index, throttle, end time)
        self.motor_current = [0.0] * 4
        self.missions = 0                 # completed takeoffs; every second one has the fault
        self.throttle = 0.0

    # ------------------------------------------------------------ transitions
    def can_enter(self, state: int) -> bool:
        """Allowed by the table and physically possible right now: LANDING -> STANDBY
        and EMERGENCY -> STANDBY are touchdown transitions, never mid-air."""
        if state not in TRANSITIONS[self.state]:
            return False
        if state == STANDBY and self.alt > 0.05:
            return False
        return True

    def request(self, state: int) -> bool:
        """Command entry point: only allowed transitions succeed."""
        if not self.can_enter(state):
            return False
        self._enter(state)
        return True

    def _enter(self, state: int) -> None:
        if state == SURVEY and self.state == TAKEOFF:
            self.survey_angle = math.pi
        if state == TAKEOFF:
            self.missions += 1
        if state == STANDBY:
            self.motor3 = 1               # maintenance after every landing
        self.state, self.in_state = state, 0.0

    @property
    def armed(self) -> bool:
        return self.state not in (STANDBY, CHARGING)

    @property
    def failsafe(self) -> bool:
        return self.mission_t < self.failsafe_until

    # ----------------------------------------------------------------- physics
    def step(self, dt: float = 1.0) -> None:
        self.mission_t += dt
        self.in_state += dt
        s = self.state

        if s == STANDBY:
            if self.in_state >= 5 and self.soc < SOC_LAUNCH:
                self._enter(CHARGING)
            elif self.in_state >= 25:
                self._enter(ARMED)
        elif s == ARMED and self.in_state >= 7:
            self._enter(TAKEOFF)
        elif s == CHARGING:
            self.soc = min(SOC_FULL, self.soc + 1.6 * dt)
            if self.soc >= SOC_FULL:
                self._enter(STANDBY)

        # Horizontal target velocity for the state.
        tvn = tve = 0.0
        if self.state == SURVEY:
            cx = CIRCLE_R
            px, py = cx + CIRCLE_R * math.cos(self.survey_angle), CIRCLE_R * math.sin(self.survey_angle)
            if math.hypot(px - self.north, py - self.east) < 6:
                self.survey_angle += CRUISE_SPEED / CIRCLE_R * dt
                px, py = cx + CIRCLE_R * math.cos(self.survey_angle), CIRCLE_R * math.sin(self.survey_angle)
            tvn, tve = self._towards(px, py, CRUISE_SPEED)
            progress = (self.survey_angle - math.pi) / (2 * math.pi)
            faulty_mission = self.missions % 2 == 0
            if faulty_mission and progress >= 0.9 and self.motor3 == 1:
                self.motor3 = 2                                  # bearing starts to wear
            if faulty_mission and progress >= 1.15 and self.motor3 == 2:
                self.motor3 = 3                                  # fault: failsafe brings it home
                self.failsafe_until = self.mission_t + 15
                self._enter(RETURN)
            elif progress >= SURVEY_LAPS:
                self._enter(RETURN)
        elif self.state == RETURN:
            tvn, tve = self._towards(0.0, 0.0, RETURN_SPEED)
            if math.hypot(self.north, self.east) < 1.0:
                self._enter(LANDING)
        if self.motor3 == 3 and not self.failsafe:
            self.motor3 = 2

        # Smooth horizontal response (about 2 s time constant).
        k = min(1.0, dt / 2.0)
        self.vn += (tvn - self.vn) * k
        self.ve += (tve - self.ve) * k
        self.north += self.vn * dt
        self.east += self.ve * dt

        # Vertical.
        if self.state == TAKEOFF:
            self.vz = min(CLIMB_RATE, max(0.4, (self.target_alt - self.alt) * 0.5))
            if self.alt >= self.target_alt - 0.3:
                self._enter(SURVEY)
        elif self.state in (SURVEY, RETURN):
            goal = self.target_alt if self.state == SURVEY else max(5.0, self.target_alt - 5.0)
            goal += 1.2 * math.sin(self.mission_t / 9) if self.state == SURVEY else 0.0
            self.vz = max(-1.0, min(1.0, (goal - self.alt) * 0.4))
        elif self.state == LANDING:
            self.vz = -min(DESCENT_RATE, max(0.3, self.alt * 0.25))
        elif self.state == EMERGENCY:
            self.vz = -EMERGENCY_RATE
        else:
            self.vz = 0.0
        self.alt = max(0.0, self.alt + self.vz * dt)
        if self.state in (LANDING, EMERGENCY) and self.alt == 0.0:
            self.vn = self.ve = 0.0
            self._enter(STANDBY)

        self._attitude(dt)
        self._power(dt)

    def _towards(self, n: float, e: float, speed: float) -> tuple[float, float]:
        dn, de = n - self.north, e - self.east
        dist = math.hypot(dn, de)
        if dist < 1e-6:
            return 0.0, 0.0
        v = min(speed, dist * 0.6)
        return dn / dist * v, de / dist * v

    def _attitude(self, dt: float) -> None:
        self.prev_roll, self.prev_pitch, self.prev_heading = self.roll, self.pitch, self.heading
        speed = math.hypot(self.vn, self.ve)
        if speed > 0.5:
            self.heading = math.degrees(math.atan2(self.ve, self.vn)) % 360
        turn = (((self.heading - self.prev_heading + 180) % 360) - 180) / dt
        airborne = self.alt > 0.05
        self.roll = math.degrees(math.atan(speed * math.radians(turn) / G)) if airborne else 0.0
        self.pitch = -1.1 * speed if airborne else 0.0        # nose down to fly forward
        if airborne:
            self.roll += 0.6 * math.sin(self.mission_t / 2.3)
            self.pitch += 0.5 * math.sin(self.mission_t / 3.1)

    def _power(self, dt: float) -> None:
        speed = math.hypot(self.vn, self.ve)
        if self.state in (STANDBY, CHARGING):
            thr = 0.0
        elif self.state == ARMED:
            thr = 8.0
        else:
            thr = max(10.0, min(95.0, 48.0 + 6.0 * self.vz + 0.9 * speed))
        self.throttle = thr
        currents = []
        for m in range(4):
            cur = (thr * 0.22 + 0.15 * math.sin(self.mission_t / 4 + m)) if thr > 0 else 0.0
            if m == 2 and thr > 0 and self.motor3 >= 2:
                cur += 6.0 if self.motor3 == 2 else 9.5
            currents.append(max(0.0, cur))
        if self.motor_test and self.state in (STANDBY, ARMED):
            m, t_thr, end = self.motor_test
            if self.mission_t < end:
                currents[m] = t_thr * 0.22
            else:
                self.motor_test = None
        self.motor_current = currents
        pack_current = -18.0 if self.state == CHARGING else sum(currents) + (1.6 if self.armed else 0.8)
        self.pack_current = pack_current
        if self.state != CHARGING:
            self.soc = max(0.0, self.soc - pack_current * 0.0048 * dt)
        for m in range(4):
            self.motor_temp[m] += (AMBIENT_C + currents[m] * 1.9 - self.motor_temp[m]) * min(1.0, dt / 25.0)
        self.battery_temp += (AMBIENT_C + max(pack_current, 0) * 0.16 - self.battery_temp) * min(1.0, dt / 60.0)

    # --------------------------------------------------------------- telemetry
    def pack(self, target_altitude: float | None = None, last_command: tuple[int, int] = (0, 0)) -> bytes:
        """The 193-byte drone extension, in the layout the /drone MDB expects."""
        soc = max(0.0, min(100.0, self.soc))
        pack_current = self.pack_current if hasattr(self, "pack_current") else 0.0
        pack_voltage = 25.6 + soc * 0.078 - (0.012 * pack_current if pack_current > 0 else -0.25)
        label = "FAILSAFE_RETURN" if self.failsafe else NAMES[self.state]

        out = bytearray()
        out += struct.pack(">IBBB16s", int(self.mission_t), self.state, int(self.armed), int(self.failsafe),
                           fixed_string("JAOPS-DRONE-01"))
        v_adc, i_adc, t_adc = round(pack_voltage / 0.01), round(pack_current / 0.01), round(self.battery_temp / 0.1)
        out += struct.pack(">HHhhBhh", v_adc, v_adc, i_adc, i_adc, round(soc), t_adc, t_adc)
        cell_offsets = (-0.018, 0.006, 0.012, 0.0, -0.008, 0.009, -0.004, 0.003)
        out += struct.pack(">8H", *(round((pack_voltage / 8 + d) / 0.001) for d in cell_offsets))

        for m in range(4):
            thr = self.throttle + (m - 1.5) * 0.8 if self.throttle > 0 else 0.0
            rpm = round(thr * 120 + 30 * math.sin(self.mission_t / 3 + m)) if thr > 0 else 0
            status = 1 if thr > 0 else 0
            if self.motor_test and self.motor_test[0] == m and self.state in (STANDBY, ARMED):
                thr, rpm, status = self.motor_test[1], self.motor_test[1] * 120, 1
            if m == 2 and thr > 0 and self.motor3 >= 2:
                status = self.motor3
                rpm = round(rpm * (0.82 if self.motor3 == 2 else 0.7))
            out += struct.pack(">HHhBB", max(0, rpm), round(self.motor_current[m] / 0.01),
                               round(self.motor_temp[m] / 0.1), round(max(0, min(100, thr))), status)

        yaw_rate = ((self.heading - self.prev_heading + 180) % 360) - 180
        out += struct.pack(">6f", self.roll, self.pitch, self.heading,
                           self.roll - self.prev_roll, self.pitch - self.prev_pitch, yaw_rate)
        lat = HOME_LAT + self.north / 111_320.0
        lon = HOME_LON + self.east / (111_320.0 * math.cos(math.radians(HOME_LAT)))
        out += struct.pack(">4f", lat, lon, self.alt, math.hypot(self.vn, self.ve))

        flying = self.state in FLIGHT or self.state == EMERGENCY
        rails = (pack_voltage, 12.0 - (0.15 if flying else 0), 5.0 - (0.04 if flying else 0))
        branches = (sum(self.motor_current), 1.8 if self.armed else 0.4, 0.9 if self.armed else 0.3, 0.6)
        out += struct.pack(">3H", *(round(v / 0.001) for v in rails))
        out += struct.pack(">4H", *(round(min(v, 65.0) / 0.001) for v in branches))
        out += struct.pack(">4B", int(self.armed), 1, 1, 1)
        out += struct.pack(">4B", 0, 1 if (self.throttle > 0 and self.motor3 == 3) else 0, 0, 0)

        vibration = []
        for m in range(4):
            base = 0.05 + (self.throttle / 100) * 0.42 if self.throttle > 0 else 0.02
            if m == 2 and self.throttle > 0 and self.motor3 >= 2:
                base += 0.6 if self.motor3 == 2 else 1.1
            vibration.extend(base * scale for scale in (1.0, 0.55, 0.28))
        out += struct.pack(">12H", *(round(v / 0.001) for v in vibration))
        out += fixed_string(label)
        out += struct.pack(">HBf", last_command[0], last_command[1],
                           self.target_alt if target_altitude is None else target_altitude)
        return bytes(out)


EXTENSION_SIZE = len(DroneModel().pack())
