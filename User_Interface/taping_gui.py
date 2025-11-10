import os
import sys
import platform
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox

# Ajuste de ruta e importación de Canbus/Taping como en cycle_parameter_tester.py
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'main_controller'))
try:
    if platform.system() == 'Linux':
        from classes.canbus_jetson import Canbus
        _impl_msg = "Usando implementación de Canbus para Jetson Orin Nano"
    else:
        from classes.canbus import Canbus
        _impl_msg = "Usando implementación de Canbus para Windows"
    from classes.taping import Taping
    _import_error = None
except Exception as e:
    # Permitir abrir la GUI sin romper, pero avisar en la UI
    Canbus = None
    Taping = None
    _import_error = e
    _impl_msg = None


LANG_MAP = {
    "English": {
        "title": "Taping Control",
        "can_id": "CAN ID",
        "bitrate": "Bitrate",
        "device_list": "List Devices",
        "connect": "Connect",
        "connected": "Connected",
        "disconnected": "Disconnected",
        "heartbeat": "Send Heartbeat",
        "step1": "Step1 Feeder",
        "step2": "Step2 Wrapper CCW",
        "step3": "Step3 Wrapper CW",
        "step4": "Step4 Cutter",
        "step5": "Step5 Wrapper 3 CW",
        "step6": "Step6 Holder Hold",
        "step7": "Step7 Holder Home",
        "step8": "Step8 Gripper Close",
        "step9": "Step9 Gripper Open",
        "step10": "Step10 Elevator Up",
        "step11": "Step11 Elevator Down",
        "step12": "Step12 Sync Move",
        "seq_fullcycle": "FullCycle",
        "seq_forward": "Forward Sequence",
        "seq_backward": "Backward Sequence",
        "complete_cycle": "Complete Cycle (Auto)",
        "logs": "Logs",
        "import_error": "Import error: ",
    },
    "Español": {
        "title": "Control de Taping",
        "can_id": "ID CAN",
        "bitrate": "Velocidad",
        "device_list": "Listar dispositivos",
        "connect": "Conectar",
        "connected": "Conectado",
        "disconnected": "Desconectado",
        "heartbeat": "Enviar Heartbeat",
        "step1": "Paso1 Alimentador",
        "step2": "Paso2 Wrapper CCW",
        "step3": "Paso3 Wrapper CW",
        "step4": "Paso4 Cortador",
        "step5": "Paso5 Wrapper 3 CW",
        "step6": "Paso6 Holder Sujetar",
        "step7": "Paso7 Holder Origen",
        "step8": "Paso8 Gripper Cerrar",
        "step9": "Paso9 Gripper Abrir",
        "step10": "Paso10 Elevador Arriba",
        "step11": "Paso11 Elevador Abajo",
        "step12": "Paso12 Movimiento Sincrónico",
        "seq_fullcycle": "Ciclo Completo",
        "seq_forward": "Secuencia Adelante",
        "seq_backward": "Secuencia Atrás",
        "complete_cycle": "Ciclo Completo (Auto)",
        "logs": "Registros",
        "import_error": "Error de importación: ",
    },
    "日本語": {
        "title": "テーピング制御",
        "can_id": "CAN ID",
        "bitrate": "ビットレート",
        "device_list": "デバイス一覧",
        "connect": "接続",
        "connected": "接続済み",
        "disconnected": "未接続",
        "heartbeat": "ハートビート送信",
        "step1": "ステップ1 フィーダー",
        "step2": "ステップ2 ラッパー反時計",
        "step3": "ステップ3 ラッパー時計",
        "step4": "ステップ4 カッター",
        "step5": "ステップ5 ラッパー3時計",
        "step6": "ステップ6 ホルダー保持",
        "step7": "ステップ7 ホルダー原点",
        "step8": "ステップ8 グリッパー閉じる",
        "step9": "ステップ9 グリッパー開く",
        "step10": "ステップ10 昇降 上",
        "step11": "ステップ11 昇降 下",
        "step12": "ステップ12 同期移動",
        "seq_fullcycle": "フルサイクル",
        "seq_forward": "前進シーケンス",
        "seq_backward": "後退シーケンス",
        "complete_cycle": "完全サイクル（自動）",
        "logs": "ログ",
        "import_error": "インポートエラー: ",
    },
}


class TapingGUI:
    def __init__(self, root):
        self.root = root
        self.lang = "Español"
        self.t = LANG_MAP[self.lang]
        self.canbus = None
        self.taping = None

        self.root.title(self.t["title"])
        self.root.geometry("820x620")

        # Layout principal
        self._build_language_bar()
        self._build_connection_bar()
        self._build_controls_grid()
        self._build_logs()

        # Import warnings
        if Canbus is None or Taping is None:
            self._log(self.t["import_error"] + str(_import_error))
        elif _impl_msg:
            # Informar qué implementación se está usando (consistencia con cycle_parameter_tester)
            self._log(_impl_msg)

    def _build_language_bar(self):
        frm = ttk.Frame(self.root)
        frm.pack(fill=tk.X, padx=10, pady=8)

        ttk.Label(frm, text="Language").pack(side=tk.LEFT)
        self.lang_var = tk.StringVar(value=self.lang)
        cb = ttk.Combobox(frm, textvariable=self.lang_var, values=list(LANG_MAP.keys()), state="readonly", width=12)
        cb.pack(side=tk.LEFT, padx=8)
        cb.bind("<<ComboboxSelected>>", self._on_language_change)

    def _build_connection_bar(self):
        frm = ttk.LabelFrame(self.root, text="CAN")
        frm.pack(fill=tk.X, padx=10, pady=8)

        ttk.Label(frm, text=self.t["can_id"]).grid(row=0, column=0, padx=6, pady=6, sticky="w")
        self.can_id_var = tk.StringVar(value="0x00A")
        ttk.Entry(frm, textvariable=self.can_id_var, width=10).grid(row=0, column=1, padx=6, pady=6, sticky="w")

        ttk.Label(frm, text=self.t["bitrate"]).grid(row=0, column=2, padx=6, pady=6, sticky="w")
        self.bitrate_var = tk.StringVar(value="125000")
        ttk.Entry(frm, textvariable=self.bitrate_var, width=10, state="readonly").grid(row=0, column=3, padx=6, pady=6, sticky="w")

        self.status_var = tk.StringVar(value=self.t["disconnected"]) 
        ttk.Label(frm, textvariable=self.status_var, foreground="#006400").grid(row=0, column=4, padx=6, pady=6, sticky="w")

        ttk.Button(frm, text=self.t["device_list"], command=self._list_devices).grid(row=1, column=0, padx=6, pady=6, sticky="w")
        ttk.Button(frm, text=self.t["connect"], command=self._connect).grid(row=1, column=1, padx=6, pady=6, sticky="w")

    def _build_controls_grid(self):
        frm = ttk.LabelFrame(self.root, text=self.t["title"]) 
        frm.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)

        # Botones en rejilla 3x4
        buttons = [
            ("heartbeat", self._send_heartbeat),
            ("step1", self._step1),
            ("step2", self._step2),
            ("step3", self._step3),
            ("step4", self._step4),
            ("step5", self._step5),
            ("step6", self._step6),
            ("step7", self._step7),
            ("step8", self._step8),
            ("step9", self._step9),
            ("step10", self._step10),
            ("step11", self._step11),
            ("step12", self._step12),
            ("seq_fullcycle", self._seq_fullcycle),
            ("seq_forward", self._seq_forward),
            ("seq_backward", self._seq_backward),
            ("complete_cycle", self._complete_cycle),
        ]

        for idx, (key, handler) in enumerate(buttons):
            r = idx // 3
            c = idx % 3
            ttk.Button(frm, text=self.t[key], command=handler, width=26).grid(row=r, column=c, padx=6, pady=6, sticky="ew")

        for i in range(3):
            frm.columnconfigure(i, weight=1)

    def _build_logs(self):
        frm = ttk.LabelFrame(self.root, text=self.t["logs"]) 
        frm.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)
        self.log_text = tk.Text(frm, height=12)
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def _on_language_change(self, event=None):
        self.lang = self.lang_var.get()
        self.t = LANG_MAP[self.lang]
        self.root.title(self.t["title"]) 
        # Re-render textos
        for w in self.root.winfo_children():
            w.destroy()
        self._build_language_bar()
        self._build_connection_bar()
        self._build_controls_grid()
        self._build_logs()

    def _list_devices(self):
        if Canbus is None:
            self._log(self.t["import_error"] + str(_import_error))
            return
        cb = Canbus()
        devices = cb.device_list()
        self._log(f"Devices: {devices}")

    def _connect(self):
        if Canbus is None or Taping is None:
            self._log(self.t["import_error"] + str(_import_error))
            return
        # Bitrate fijo a 125000 (125 kbps)
        bitrate = 125000
        try:
            self.canbus = Canbus(bitrate=bitrate)
            self.canbus.start_canbus()
            can_id_str = self.can_id_var.get().strip()
            can_id = int(can_id_str, 0)  # Permite 0xNNN o decimal
            self.taping = Taping(canbus=self.canbus, canbus_id=can_id)
            self.status_var.set(self.t["connected"]) 
            self._log(f"Connected. CAN ID={hex(can_id)}, bitrate={bitrate}")
            # Test heartbeat
            self._send_heartbeat()
        except Exception as e:
            self.status_var.set(self.t["disconnected"]) 
            self._log(f"Connect error: {e}")

    def _ensure_connected(self):
        if not self.taping or not self.canbus:
            self._log("Not connected")
            return False
        return True

    def _send_heartbeat(self):
        if not self._ensure_connected():
            return
        try:
            res = self.taping.send_heartbeat()
            self._log(f"Heartbeat -> {res}")
        except Exception as e:
            self._log(f"Heartbeat error: {e}")

    def _step1(self):
        if not self._ensure_connected():
            return
        try:
            res = self.taping.step1_feeder()        
            self._log(f"Step1 -> {res}")
        except Exception as e:
            self._log(f"Step1 error: {e}")

    def _step2(self):
        if not self._ensure_connected():
            return
        try:
            res = self.taping.step2_wrapper_ccw()
            self._log(f"Step2 -> {res}")
        except Exception as e:
            self._log(f"Step2 error: {e}")

    def _step3(self):
        if not self._ensure_connected():
            return
        try:
            res = self.taping.step3_wrapper_cw()
            self._log(f"Step3 -> {res}")
        except Exception as e:
            self._log(f"Step3 error: {e}")

    def _step4(self):
        if not self._ensure_connected():
            return
        try:
            res = self.taping.step4_cutter()
            self._log(f"Step4 -> {res}")
        except Exception as e:
            self._log(f"Step4 error: {e}")

    def _step5(self):
        if not self._ensure_connected():
            return
        try:
            res = self.taping.step5_wrapper_3_cw()
            self._log(f"Step5 -> {res}")
        except Exception as e:
            self._log(f"Step5 error: {e}")

    def _step6(self):
        if not self._ensure_connected():
            return
        try:
            res = self.taping.step6_holder_hold()
            self._log(f"Step6 -> {res}")
        except Exception as e:
            self._log(f"Step6 error: {e}")

    def _step7(self):
        if not self._ensure_connected():
            return
        try:
            res = self.taping.step7_holder_home_position()
            self._log(f"Step7 -> {res}")
        except Exception as e:
            self._log(f"Step7 error: {e}")

    def _step8(self):
        if not self._ensure_connected():
            return
        try:
            res = self.taping.step8_gripper_close()
            self._log(f"Step8 -> {res}")
        except Exception as e:
            self._log(f"Step8 error: {e}")

    def _step9(self):
        if not self._ensure_connected():
            return
        try:
            res = self.taping.step9_gripper_open()
            self._log(f"Step9 -> {res}")
        except Exception as e:
            self._log(f"Step9 error: {e}")

    def _step10(self):
        if not self._ensure_connected():
            return
        try:
            res = self.taping.step10_elevator_up()
            self._log(f"Step10 -> {res}")
        except Exception as e:
            self._log(f"Step10 error: {e}")

    def _step11(self):
        if not self._ensure_connected():
            return
        try:
            res = self.taping.step11_elevator_down()
            self._log(f"Step11 -> {res}")
        except Exception as e:
            self._log(f"Step11 error: {e}")

    def _step12(self):
        if not self._ensure_connected():
            return
        try:
            res = self.taping.step12_sync_move()
            self._log(f"Step12 -> {res}")
        except Exception as e:
            self._log(f"Step12 error: {e}")

    def _seq_fullcycle(self):
        if not self._ensure_connected():
            return
        try:
            res = self.taping.execute_fullcycle()
            self._log(f"FullCycle -> {res}")
        except Exception as e:
            self._log(f"FullCycle error: {e}")

    def _seq_forward(self):
        if not self._ensure_connected():
            return
        try:
            res = self.taping.execute_forward_sequence()
            self._log(f"Forward Sequence -> {res}")
        except Exception as e:
            self._log(f"Forward Sequence error: {e}")

    def _seq_backward(self):
        if not self._ensure_connected():
            return
        try:
            res = self.taping.execute_backward_sequence()
            self._log(f"Backward Sequence -> {res}")
        except Exception as e:
            self._log(f"Backward Sequence error: {e}")

    def _complete_cycle(self):
        if not self._ensure_connected():
            return
        try:
            res = self.taping.completeCycle()
            self._log(f"Complete Cycle -> {res}")
        except Exception as e:
            self._log(f"Complete Cycle error: {e}")

    def _log(self, msg: str):
        self.log_text.insert(tk.END, msg + "\n")
        self.log_text.see(tk.END)


def main():
    root = tk.Tk()
    style = ttk.Style()
    try:
        style.theme_use("clam")
    except Exception:
        pass
    app = TapingGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()