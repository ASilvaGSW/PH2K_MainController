"""
Hose Jig v2 (Servos-only)

- EN: Controller for servos-only hose jig firmware (hose_jig_v2.ino).
- ES: Controlador para firmware del hose jig solo servos (hose_jig_v2.ino).
- JA: サーボのみ版ホース治具のファームウェア制御（hose_jig_v2.ino）。

Supported CAN commands (firmware v2):
- 0x01 Reset microcontroller
- 0x02 Ping
- 0x05 Move servos to open position
- 0x06 Move servos to close position
- 0x08 Read servo movement counter
- 0x09 Reset servo movement counter
- 0x0A Move servos to absolute position
- 0x10 Update SERVO_OPEN_ANGLE
- 0x11 Update SERVO_CLOSE_ANGLE
- 0x14 Read SERVO_OPEN_ANGLE
- 0x15 Read SERVO_CLOSE_ANGLE
- 0x20 Move Hose Holder servos
- 0xFF Power off - move servos to home

Notes:
- EN: This class is separate from hose_jig.py (v1 with actuator/MCP2515).
- ES: Esta clase es separada de hose_jig.py (v1 con actuador/MCP2515).
- JA: このクラスは hose_jig.py（アクチュエータ/MCP2515 ありの v1）とは別です。
"""

from typing import Optional


class HoseJigV2:
    def __init__(self, canbus, canbus_id=0x0CA):
        """
        EN: Initialize controller with CAN bus instance and target CAN ID.
        ES: Inicializa el controlador con instancia de CAN y CAN ID objetivo.
        JA: CAN バスと対象 CAN ID を指定して初期化します。
        """
        self.canbus = canbus
        self.canbus_id = canbus_id

    # 0x01: Reset microcontroller
    def reset_microcontroller(self):
        """
        EN: Reset microcontroller.
        ES: Reinicia el microcontrolador.
        JA: マイコンをリセットします。
        """
        status, _ = self.canbus.send_message(self.canbus_id, [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    # 0x02: Ping
    def ping(self):
        """
        EN: Ping device to verify communication.
        ES: Ping al dispositivo para verificar comunicación.
        JA: 通信確認のためピン送信します。
        """
        status, _ = self.canbus.send_message(self.canbus_id, [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    # 0x05: Move servos to open
    def gripper_open(self, jig: Optional[int] = None):
        """
        EN: Move left/center/right servos to SERVO_OPEN_ANGLE.
        ES: Mueve servos a SERVO_OPEN_ANGLE.
        JA: サーボを SERVO_OPEN_ANGLE に動かします。
        """
        # EN: jig None->both, 1->jig1, 2->jig2
        # ES: jig None->ambos, 1->jig1, 2->jig2
        # JA: jig None->両方、1->治具1、2->治具2
        j = 0 if jig is None else int(jig)
        status, _ = self.canbus.send_message(self.canbus_id, [0x05, j, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    # 0x06: Move servos to close
    def gripper_close(self, jig: Optional[int] = None):
        """
        EN: Move left/center/right servos to SERVO_CLOSE_ANGLE.
        ES: Mueve servos a SERVO_CLOSE_ANGLE.
        JA: サーボを SERVO_CLOSE_ANGLE に動かします。
        """
        j = 0 if jig is None else int(jig)
        status, _ = self.canbus.send_message(self.canbus_id, [0x06, j, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    # 0x08: Read servo movement counter
    def read_servo_move_counter(self, jig: Optional[int] = None):
        """
        EN: Read the servo movement counter from firmware.
        ES: Lee el contador de movimientos de servos del firmware.
        JA: ファームウェアからサーボの移動回数カウンタを読み取ります。

        Returns: (status, counter) — counter is int or None if unavailable.
        """
        j = 0 if jig is None else int(jig)
        status, reply = self.canbus.send_message(self.canbus_id, [0x08, j, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        counter = None
        if reply and len(reply) >= 5 and reply[0] == 0x08 and reply[1] == 0x01:
            # reply: [0x08, 0x01, jigSel, high, low, ...]
            counter = (reply[3] << 8) | reply[4]
        return status, counter

    # 0x09: Reset servo movement counter
    def reset_servo_move_counter(self, jig: Optional[int] = None):
        """
        EN: Reset the servo movement counter.
        ES: Reinicia el contador de movimientos de servos.
        JA: サーボ移動カウンタをリセットします。
        """
        j = 0 if jig is None else int(jig)
        status, _ = self.canbus.send_message(self.canbus_id, [0x09, j, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    # 0x0A: Move servos to absolute position
    def move_servos_absolute(self, position, jig: Optional[int] = None):
        """
        EN: Move all servos to an absolute position (0–180 typical).
        ES: Mueve todos los servos a posición absoluta (0–180 típico).
        JA: 全サーボを絶対位置へ移動（一般的に 0–180）。
        """
        pos = max(0, min(180, int(position)))
        j = 0 if jig is None else int(jig)
        status, _ = self.canbus.send_message(self.canbus_id, [0x1A, j, pos, 0x00, 0x00, 0x00, 0x00, 0x00],wait_for_reply=False)
        return status

    # 0x20: Move Hose Holder servos
    def move_hose_holder(self, angle, selector: Optional[int] = None):
        """
        EN: Move Hose Holder servos (Pins 21, 22).
        ES: Mueve los servos del Hose Holder (Pines 21, 22).
        JA: ホースホルダーのサーボ（ピン21、22）を動かします。

        :param angle: Target angle (0-180)
        :param selector: 0=Both (default), 1=Servo1(21), 2=Servo2(22)
        """
        sel = 0 if selector is None else int(selector)
        ang = max(0, min(180, int(angle)))
        status, _ = self.canbus.send_message(self.canbus_id, [0x20, sel, ang, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    def close_stamper_hose_jig(self):
        self.move_servos_absolute(92, 1)

    def close_taping_hose_jig(self):
        self.move_servos_absolute(92, 2)


    def open_stamper_hose_jig(self):
        self.move_servos_absolute(7, 1)

    def open_taping_hose_jig(self):
        self.move_servos_absolute(7, 2)

    # 0x10: Update SERVO_OPEN_ANGLE
    def set_open_angle(self, value, jig: Optional[int] = None):
        """
        EN: Update SERVO_OPEN_ANGLE in EEPROM (uint16, 0–180 typical).
        ES: Actualiza SERVO_OPEN_ANGLE en EEPROM (uint16, 0–180 típico).
        JA: EEPROM の SERVO_OPEN_ANGLE を更新（uint16、一般的に 0–180）。
        """
        val = max(0, min(180, int(value)))
        high = (val >> 8) & 0xFF
        low = val & 0xFF
        j = 0 if jig is None else int(jig)
        status, _ = self.canbus.send_message(self.canbus_id, [0x10, high, low, j, 0x00, 0x00, 0x00, 0x00])
        return status

    # 0x11: Update SERVO_CLOSE_ANGLE
    def set_close_angle(self, value, jig: Optional[int] = None):
        """
        EN: Update SERVO_CLOSE_ANGLE in EEPROM (uint16, 0–180 typical).
        ES: Actualiza SERVO_CLOSE_ANGLE en EEPROM (uint16, 0–180 típico).
        JA: EEPROM の SERVO_CLOSE_ANGLE を更新（uint16、一般的に 0–180）。
        """
        val = max(0, min(180, int(value)))
        high = (val >> 8) & 0xFF
        low = val & 0xFF
        j = 0 if jig is None else int(jig)
        status, _ = self.canbus.send_message(self.canbus_id, [0x11, high, low, j, 0x00, 0x00, 0x00, 0x00])
        return status

    # 0x14: Read SERVO_OPEN_ANGLE
    def read_open_angle(self, jig: Optional[int] = None):
        """
        EN: Read SERVO_OPEN_ANGLE (uint16) from firmware.
        ES: Lee SERVO_OPEN_ANGLE (uint16) del firmware.
        JA: ファームウェアから SERVO_OPEN_ANGLE（uint16）を読み取ります。

        Returns: (status, value) — value is int or None if unavailable.
        """
        j = 0 if jig is None else int(jig)
        status, reply = self.canbus.send_message(self.canbus_id, [0x14, j, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        value = None
        if reply and len(reply) >= 5 and reply[0] == 0x14 and reply[1] == 0x01:
            # reply: [0x14, 0x01, jigSel, high, low, ...]
            value = (reply[3] << 8) | reply[4]
        return status, value

    # 0x15: Read SERVO_CLOSE_ANGLE
    def read_close_angle(self, jig: Optional[int] = None):
        """
        EN: Read SERVO_CLOSE_ANGLE (uint16) from firmware.
        ES: Lee SERVO_CLOSE_ANGLE (uint16) del firmware.
        JA: ファームウェアから SERVO_CLOSE_ANGLE（uint16）を読み取ります。

        Returns: (status, value) — value is int or None if unavailable.
        """
        j = 0 if jig is None else int(jig)
        status, reply = self.canbus.send_message(self.canbus_id, [0x15, j, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        value = None
        if reply and len(reply) >= 5 and reply[0] == 0x15 and reply[1] == 0x01:
            # reply: [0x15, 0x01, jigSel, high, low, ...]
            value = (reply[3] << 8) | reply[4]
        return status, value

    # 0xFF: Power off - move servos to home
    def power_off_home(self):
        """
        EN: Power off routine, moves servos to home position.
        ES: Rutina de apagado, mueve servos a posición home.
        JA: 電源オフのルーチンでサーボをホーム位置へ移動します。
        """
        status, _ = self.canbus.send_message(self.canbus_id, [0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    # Convenience aliases
    def open(self):
        return self.gripper_open()

    def close(self):
        return self.gripper_close()

    def move_absolute(self, position):
        return self.move_servos_absolute(position)