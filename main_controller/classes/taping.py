import time
# TAPING v2 0x00A
# CAN Command Cases (input/output summary) / Resumen de comandos CAN / CANコマンド概要:
# 0x01: Reset microcontroller | Reinicio del microcontrolador | マイコンをリセット
#   IN: None | OUT: [0x01, 0x01, ...]
# 0x02: Heartbeat | Latido / Ping | ハートビート
#   IN: None | OUT: [0x02, 0x01, ...]
# 0x03: Feeder run ~3s, then stop at 1500us | Alimentador ~3s y luego stop a 1500us | フィーダ約3秒動作後1500usで停止
#   IN: None | OUT: [0x03, 0x01, ...]
# 0x04: Wrapper CCW 1200us for ~1s, then 1500us | Envolvedora CCW 1200us ~1s y luego 1500us | ラッパー反時計1200us約1秒後1500us
#   IN: None | OUT: [0x04, 0x01, ...]
# 0x05: Wrapper CW 1650us for ~1s, then 1500us | Envolvedora CW 1650us ~1s y luego 1500us | ラッパー時計1650us約1秒後1500us
#   IN: None | OUT: [0x05, 0x01, ...]
# 0x06: Cutter sequence (move → wait → move → wait → home) | Secuencia del cutter | カッターシーケンス
#   IN: None | OUT: [0x06, 0x01, ...]
# 0x07: Wrapper CW 1650us for ~3s, then 1500us | Envolvedora CW 1650us ~3s y luego 1500us | ラッパー時計1650us約3秒後1500us
#   IN: None | OUT: [0x07, 0x01, ...]
# 0x08: Holder to 1850us | Soporte a 1850us | ホルダー1850usへ
#   IN: None | OUT: [0x08, 0x01, ...]
# 0x09: Holder to 1650us | Soporte a 1650us | ホルダー1650usへ
#   IN: None | OUT: [0x09, 0x01, ...]
# 0x0A: Gripper close (1850us) | Gripper cerrar (1850us) | グリッパー閉じる(1850us)
#   IN: None | OUT: [0x0A, 0x01, ...]
# 0x0B: Gripper open (2000us) | Gripper abrir (2000us) | グリッパー開く(2000us)
#   IN: None | OUT: [0x0B, 0x01, ...]
# 0x0C: Elevator up (gradual to 1550us) | Elevador subir (rampa a 1550us) | エレベーター上昇(段階的に1550us)
#   IN: None | OUT: [0x0C, 0x01, ...]
# 0x0D: Elevator home (gradual to 700us) | Elevador home (rampa a 700us) | エレベーター原点(段階的に700us)
#   IN: None | OUT: [0x0D, 0x01, ...]
# 0x10: Move selected servo to microseconds (500–2500) | Mover servo seleccionado a microsegundos (500–2500) | 選択サーボをマイクロ秒へ移動（500–2500）
#   IN: [0x10, servoId, high(us), low(us), 0,0,0,0] — servoId: 1 feeder, 2 wrapper, 3 cutter, 4 holder, 5 gripper, 6 elevator
#   OUT: [0x10, 0x01, servoId, high(us), low(us), ...] (OK) or [0x10, 0x02, servoId, ...] (FAIL)
# Nota/Note/注意: Las secuencias 0x10–0x12 están disponibles en el firmware taping.ino actual.

class Taping:
    def __init__(self, canbus, canbus_id):
        self.canbus = canbus
        self.canbus_id = canbus_id

    # 0x01: Reset microcontroller
    def reset_microcontroller(self):
        """Reset the taping microcontroller"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x01] + [0x00]*7)
        return status

    # 0x02: Send Heartbeat
    def send_heartbeat(self):
        """Send heartbeat to check taping device status"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x02] + [0x00]*7)
        return status

    # 0x03: Execute step1 - Feeder
    def step1_feeder(self):
        """Feeder run ~3s, then stop at 1500us / Alimentador ~3s, luego stop a 1500us / フィーダ約3秒動作後1500usで停止"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x03] + [0x00]*7,False)
        return status

    # 0x04: Execute step2 - Wrapper CCW
    def step2_wrapper_ccw(self):
        """Wrapper CCW 1200us ~1s, luego 1500us / ラッパー反時計1200us約1秒後1500us"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x04] + [0x00]*7,False)
        return status

    # 0x05: Execute step3 - Wrapper CW
    def step3_wrapper_cw(self):
        """Wrapper CW 1650us ~1s, luego 1500us / ラッパー時計1650us約1秒後1500us"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x05] + [0x00]*7,False)
        return status

    # 0x06: Execute step4 - Cutter
    def step4_cutter(self):
        """Cutter sequence (move → wait → move → wait → home) / Secuencia del cutter / カッターシーケンス"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x06] + [0x00]*7,False)
        return status

    # 0x07: Execute step5 - Wrapper 3 CW
    def step5_wrapper_3_cw(self):
        """Wrapper CW 1650us ~3s, luego 1500us / ラッパー時計1650us約3秒後1500us"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x07] + [0x00]*7,False)
        return status

    # 0x08: Execute step6 - Holder Hold
    def step6_holder_hold(self):
        """Holder a 1850us / ホルダー1850usへ"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x18] + [0x00]*7,False)
        return status

    # 0x09: Execute step7 - Holder Home Position
    def step7_holder_home_position(self):
        """Holder a 1650us (home) / ホルダー1650us(ホーム)"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x09] + [0x00]*7,False)
        return status

    # 0x0A: Execute step8 - Gripper Close
    def step8_gripper_close(self):
        """Cerrar gripper (1850us) / グリッパーを閉じる(1850us)"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0A] + [0x00]*7,False)
        return status

    # 0x0B: Execute step9 - Gripper Open
    def step9_gripper_open(self):
        """Abrir gripper (2000us) / グリッパーを開く(2000us)"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0B] + [0x00]*7,False)
        return status

    # 0x0C: Execute step10 - Elevator Up
    def step10_elevator_up(self):
        """Subir elevador (rampa a 1550us) / エレベーター上昇(段階的に1550us)"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0C] + [0x00]*7,False)
        return status

    # 0x0D: Execute step11 - Elevator Home/Down
    def step11_elevator_down(self):
        """Bajar elevador (rampa a 700us, home) / エレベーター下降(段階的に700us)"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0D] + [0x00]*7,False)
        return status

    # 0x0E: Execute step12 - Sync Move
    def step12_sync_move(self):
        """Sincronizar movimiento de todos los servos / すべてのサーボを同期的に動かす"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0E] + [0x00]*7,False)
        return status

    # 0x10: FullCycle sequence
    def execute_fullcycle(self):
        """Ejecutar secuencia FullCycle / フルサイクルのシーケンスを実行"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x10] + [0x00]*7, False)
        return status

    # 0x11: Forward sequence
    def execute_forward_sequence(self):
        """Ejecutar secuencia Adelante / 前進シーケンスを実行"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x11] + [0x00]*7, False)
        return status

    # 0x12: Backward sequence
    def execute_backward_sequence(self):
        """Ejecutar secuencia Atrás / 後退シーケンスを実行"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x12] + [0x00]*7, False)
        return status

    def checkAlignment(self):
        """Check if the tape is aligned with the holder / ホルダーと磁気センサーが一致しているか確認"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x15] + [0x00]*7, False)
        return status

    def completeCycle(self,wait=5):
        self.execute_forward_sequence()
        time.sleep(wait)
        self.execute_backward_sequence()
        return True

    # Convenience methods for common operations
    def feed_tape(self):
        """Feed tape - alias/alias/別名: step1_feeder"""
        return self.step1_feeder()

    def cut_tape(self):
        """Cut tape - alias/alias/別名: step4_cutter"""
        return self.step4_cutter()

    def wrapper_ccw(self):
        """Wrapper CCW - alias/alias/別名: step2_wrapper_ccw"""
        return self.step2_wrapper_ccw()

    def wrapper_cw(self):
        """Wrapper CW - alias/alias/別名: step3_wrapper_cw"""
        return self.step3_wrapper_cw()

    def wrapper_cw_3s(self):
        """Wrapper CW 3s - alias/alias/別名: step5_wrapper_3_cw"""
        return self.step5_wrapper_3_cw()

    def hold_tape(self):
        """Hold tape - alias/alias/別名: step6_holder_hold"""
        return self.step6_holder_hold()

    def release_tape(self):
        """Release tape - alias/alias/別名: step7_holder_home_position"""
        return self.step7_holder_home_position()

    def close_gripper(self):
        """Close gripper - alias for step8_gripper_close"""
        return self.step8_gripper_close()

    def open_gripper(self):
        """Open gripper - alias for step9_gripper_open"""
        return self.step9_gripper_open()

    def elevator_up(self):
        """Move elevator up - alias/alias/別名: step10_elevator_up"""
        return self.step10_elevator_up()

    def elevator_down(self):
        """Move elevator down (home) - alias/alias/別名: step11_elevator_down"""
        return self.step11_elevator_down()

    # High-level operations
    def home_all(self):
        """Move all components to home/safe positions | Llevar todo a home/seguro | すべてを原点/安全位置へ"""
        # Holder to home, elevator to home, gripper open
        self.step7_holder_home_position()
        self.step11_elevator_down()
        self.step9_gripper_open()
        return True

    def prepare_for_hose(self):
        """Prepare taping station for hose insertion"""
        self.elevator_down()
        self.open_gripper()
        return True

    def complete_taping_cycle(self):
        """Execute a minimal taping cycle (feeder → wrapper CW → cutter)"""
        self.step1_feeder()
        self.step3_wrapper_cw()
        self.step4_cutter()
        return True

    def power_off(self):
        """Power off - move all components to safe positions"""
        return self.home_all()

   