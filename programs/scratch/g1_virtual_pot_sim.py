#!/usr/bin/env python3

import json
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk


APP_TITLE = "G1 Virtual Pot Simulator"
DEFAULT_PROJECT_SUFFIX = ".g1config.json"
NUM_MOTORS = 35
POS_STOP_F = 2.146e9
VEL_STOP_F = 16000.0
COMMAND_DT_SEC = 0.02
CMD_TOPIC = "rt/lowcmd"
STATE_TOPIC = "rt/lowstate"


@dataclass
class BindingRecord:
    name: str
    packet_index: int
    joint_index: int
    kp: float
    kd: float
    scale: float = 1.0
    offset: float = 0.0
    min_q: float | None = None
    max_q: float | None = None
    average_window_size: int = 1


class VirtualPotSimulatorApp:
    def __init__(self, root: tk.Tk, project_path: Path):
        self.root = root
        self.project_path = project_path
        self.root.title(f"{APP_TITLE} - {project_path.name}")
        self.root.geometry("1220x840")

        self.project = json.loads(project_path.read_text(encoding="utf-8"))
        self.bindings = [BindingRecord(**item) for item in self.project.get("bindings", [])]
        if not self.bindings:
            raise RuntimeError("Project file contains no bindings")

        self.slider_vars: list[tk.DoubleVar] = []
        self.slider_values: list[float] = []
        self.value_labels: list[tk.StringVar] = []
        self.sim_process: subprocess.Popen | None = None
        self.publisher_thread: threading.Thread | None = None
        self.publish_mode: str | None = None
        self.stop_event = threading.Event()
        self.status_var = tk.StringVar(value="Ready")
        self.sim_status_var = tk.StringVar(value="Simulator: not started")
        self.publish_sim_button_var = tk.StringVar(value="Start Publishing To Sim")
        self.publish_robot_button_var = tk.StringVar(value="Start Publishing To Robot")
        self.arm_button_var = tk.StringVar(value="Arm Robot Controls")
        self.arm_status_var = tk.StringVar(value="Robot control state: DISARMED")
        self.robot_armed = False

        self.python_cmd_var = tk.StringVar(value=sys.executable)
        self.sim_script_var = tk.StringVar(
            value=str(Path(__file__).resolve().parents[1] / "unitree_mujoco" / "simulate_python" / "unitree_mujoco.py")
        )
        self.sim_domain_var = tk.StringVar(value="1")
        self.sim_interface_var = tk.StringVar(value="lo")
        self.robot_domain_var = tk.StringVar(value="0")
        self.robot_interface_var = tk.StringVar(value="")

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        top = ttk.Frame(self.root, padding=10)
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(5, weight=1)

        ttk.Label(top, text=f"Project: {self.project_path.name}").grid(
            row=0, column=0, columnspan=2, sticky="w"
        )
        ttk.Label(top, text="Python").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(top, textvariable=self.python_cmd_var, width=36).grid(
            row=1, column=1, sticky="ew", padx=(6, 12), pady=(8, 0)
        )
        ttk.Label(top, text="Sim Script").grid(row=1, column=2, sticky="w", pady=(8, 0))
        ttk.Entry(top, textvariable=self.sim_script_var, width=56).grid(
            row=1, column=3, columnspan=3, sticky="ew", padx=(6, 12), pady=(8, 0)
        )
        ttk.Label(top, text="Sim Domain").grid(row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(top, textvariable=self.sim_domain_var, width=8).grid(
            row=2, column=1, sticky="w", padx=(6, 12), pady=(8, 0)
        )
        ttk.Label(top, text="Sim Interface").grid(row=2, column=2, sticky="w", pady=(8, 0))
        ttk.Entry(top, textvariable=self.sim_interface_var, width=10).grid(
            row=2, column=3, sticky="w", padx=(6, 12), pady=(8, 0)
        )
        ttk.Button(top, text="Launch MuJoCo Simulator", command=self.launch_mujoco).grid(
            row=2, column=4, padx=4, pady=(8, 0)
        )
        ttk.Button(top, textvariable=self.publish_sim_button_var, command=self.toggle_sim_publisher).grid(
            row=2, column=5, sticky="w", padx=4, pady=(8, 0)
        )
        ttk.Label(top, text="Robot Domain").grid(row=3, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(top, textvariable=self.robot_domain_var, width=8).grid(
            row=3, column=1, sticky="w", padx=(6, 12), pady=(8, 0)
        )
        ttk.Label(top, text="Robot NIC").grid(row=3, column=2, sticky="w", pady=(8, 0))
        ttk.Entry(top, textvariable=self.robot_interface_var, width=14).grid(
            row=3, column=3, sticky="w", padx=(6, 12), pady=(8, 0)
        )
        ttk.Button(
            top,
            textvariable=self.publish_robot_button_var,
            command=self.toggle_robot_publisher,
        ).grid(row=3, column=5, sticky="w", padx=4, pady=(8, 0))
        ttk.Button(
            top,
            textvariable=self.arm_button_var,
            command=self.toggle_robot_arm,
        ).grid(row=3, column=4, padx=4, pady=(8, 0))

        body = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        body.grid(row=1, column=0, sticky="nsew")
        body.columnconfigure(0, weight=1)
        body.rowconfigure(0, weight=1)

        canvas = tk.Canvas(body, highlightthickness=0)
        canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(body, orient="vertical", command=canvas.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        canvas.configure(yscrollcommand=scrollbar.set)

        slider_container = ttk.Frame(canvas)
        slider_container.columnconfigure(1, weight=1)
        canvas.create_window((0, 0), window=slider_container, anchor="nw")
        slider_container.bind(
            "<Configure>", lambda _event: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        for row, binding in enumerate(self.bindings):
            value_var = tk.StringVar(value="raw=0.000 target=0.000")
            slider_var = tk.DoubleVar(value=0.0)
            slider_var.trace_add("write", lambda *_args, idx=row: self._update_value_label(idx))
            self.slider_vars.append(slider_var)
            self.slider_values.append(0.0)
            self.value_labels.append(value_var)

            ttk.Label(slider_container, text=binding.name, width=18).grid(
                row=row, column=0, sticky="w", padx=(0, 8), pady=6
            )
            scale = ttk.Scale(
                slider_container,
                from_=-1.5,
                to=1.5,
                orient=tk.HORIZONTAL,
                variable=slider_var,
            )
            scale.grid(row=row, column=1, sticky="ew", padx=(0, 8), pady=6)
            ttk.Label(slider_container, textvariable=value_var, width=30).grid(
                row=row, column=2, sticky="w", padx=(0, 8), pady=6
            )
            ttk.Button(
                slider_container,
                text="Zero",
                command=lambda idx=row: self._set_slider(idx, 0.0),
            ).grid(row=row, column=3, padx=4, pady=6)

        buttons = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        buttons.grid(row=2, column=0, sticky="ew")
        ttk.Button(buttons, text="Zero All", command=self.zero_all).pack(side=tk.LEFT)
        ttk.Button(buttons, text="Center To Offsets", command=self.center_to_offsets).pack(
            side=tk.LEFT, padx=6
        )
        ttk.Label(buttons, textvariable=self.sim_status_var).pack(side=tk.LEFT, padx=20)
        ttk.Label(buttons, textvariable=self.arm_status_var).pack(side=tk.LEFT, padx=20)
        ttk.Label(buttons, textvariable=self.status_var).pack(side=tk.RIGHT)

        for index in range(len(self.bindings)):
            self._update_value_label(index)

    def _set_status(self, message: str) -> None:
        self.status_var.set(message)

    def _apply_binding_transform(self, raw_value: float, binding: BindingRecord) -> float:
        mapped = raw_value * binding.scale + binding.offset
        if binding.min_q is not None:
            mapped = max(binding.min_q, mapped)
        if binding.max_q is not None:
            mapped = min(binding.max_q, mapped)
        return mapped

    def _update_value_label(self, index: int) -> None:
        raw_value = self.slider_vars[index].get()
        self.slider_values[index] = raw_value
        mapped = self._apply_binding_transform(raw_value, self.bindings[index])
        self.value_labels[index].set(f"raw={raw_value:+.3f} target={mapped:+.3f}")

    def _set_slider(self, index: int, value: float) -> None:
        self.slider_vars[index].set(value)
        self._update_value_label(index)

    def zero_all(self) -> None:
        for index in range(len(self.slider_vars)):
            self._set_slider(index, 0.0)
        self._set_status("All virtual pots set to zero")

    def center_to_offsets(self) -> None:
        for index, binding in enumerate(self.bindings):
            if binding.scale == 0.0:
                self._set_slider(index, 0.0)
            else:
                self._set_slider(index, -binding.offset / binding.scale)
        self._set_status("Virtual pots adjusted so target equals offset")

    def launch_mujoco(self) -> None:
        if self.sim_process is not None and self.sim_process.poll() is None:
            messagebox.showinfo(APP_TITLE, "MuJoCo simulator is already running.")
            return

        sim_script = Path(self.sim_script_var.get()).expanduser()
        if not sim_script.exists():
            messagebox.showerror(APP_TITLE, f"Simulator script not found:\n{sim_script}")
            return

        cmd = [self.python_cmd_var.get(), str(sim_script)]
        try:
            self.sim_process = subprocess.Popen(cmd, cwd=str(sim_script.parent))
        except OSError as exc:
            messagebox.showerror(APP_TITLE, f"Failed to launch simulator:\n{exc}")
            return

        self.sim_status_var.set(f"Simulator: running ({self.sim_process.pid})")
        self._set_status("Launched MuJoCo simulator")

    def toggle_sim_publisher(self) -> None:
        self._toggle_publisher(mode="sim")

    def toggle_robot_publisher(self) -> None:
        self._toggle_publisher(mode="robot")

    def toggle_robot_arm(self) -> None:
        if self.robot_armed:
            if self.publish_mode == "robot":
                messagebox.showinfo(
                    APP_TITLE,
                    "Stop robot publishing before disarming robot controls.",
                )
                return
            self.robot_armed = False
            self.arm_button_var.set("Arm Robot Controls")
            self.arm_status_var.set("Robot control state: DISARMED")
            self._set_status("Robot controls disarmed")
            return

        invalid_bindings = [
            binding.name for binding in self.bindings if binding.joint_index < 0 or binding.joint_index >= NUM_MOTORS
        ]
        if invalid_bindings:
            messagebox.showerror(
                APP_TITLE,
                "Robot controls cannot be armed until all bindings have valid joint indices.\n\n"
                f"Invalid bindings: {', '.join(invalid_bindings[:8])}"
                + (" ..." if len(invalid_bindings) > 8 else ""),
            )
            return

        confirmed = messagebox.askyesno(
            APP_TITLE,
            "Arming enables the 'Start Publishing To Robot' path.\n"
            "Only arm if the robot is supported, clear, and ready for controlled testing.\n\n"
            "Arm robot controls?",
            icon=messagebox.WARNING,
        )
        if not confirmed:
            return

        self.robot_armed = True
        self.arm_button_var.set("Disarm Robot Controls")
        self.arm_status_var.set("Robot control state: ARMED")
        self._set_status("Robot controls armed")

    def _toggle_publisher(self, mode: str) -> None:
        if self.publisher_thread is not None and self.publisher_thread.is_alive():
            if self.publish_mode != mode:
                messagebox.showinfo(
                    APP_TITLE,
                    f"{self.publish_mode.capitalize()} publisher is already running. Stop it first.",
                )
                return
            self.stop_event.set()
            self._set_publish_buttons(None)
            self._set_status(f"Stopping {mode} publisher...")
            return

        try:
            if mode == "sim":
                domain_id = int(self.sim_domain_var.get())
                interface_name = self.sim_interface_var.get().strip()
            else:
                domain_id = int(self.robot_domain_var.get())
                interface_name = self.robot_interface_var.get().strip()
        except ValueError:
            messagebox.showerror(APP_TITLE, f"{mode.capitalize()} DDS domain must be an integer")
            return

        if not interface_name:
            messagebox.showerror(APP_TITLE, f"{mode.capitalize()} interface/NIC is required")
            return

        if mode == "robot":
            if not self.robot_armed:
                messagebox.showerror(
                    APP_TITLE,
                    "Robot controls are disarmed. Click 'Arm Robot Controls' first.",
                )
                return
            confirmed = messagebox.askyesno(
                APP_TITLE,
                "This will publish live commands to the real robot DDS interface.\n"
                "Continue only if the robot is safe, supported, and you have an immediate stop method.\n\n"
                f"Domain: {domain_id}\nNIC: {interface_name}\n\n"
                "Start robot publishing?",
                icon=messagebox.WARNING,
            )
            if not confirmed:
                return

        self.stop_event.clear()
        self.publish_mode = mode
        self.publisher_thread = threading.Thread(
            target=self._publisher_loop,
            args=(domain_id, interface_name),
            daemon=True,
        )
        self.publisher_thread.start()
        self._set_publish_buttons(mode)
        self._set_status(f"Started {mode} publisher")

    def _set_publish_buttons(self, active_mode: str | None) -> None:
        self.publish_mode = active_mode
        self.publish_sim_button_var.set(
            "Stop Publishing To Sim" if active_mode == "sim" else "Start Publishing To Sim"
        )
        self.publish_robot_button_var.set(
            "Stop Publishing To Robot" if active_mode == "robot" else "Start Publishing To Robot"
        )

    def _publisher_loop(self, domain_id: int, interface_name: str) -> None:
        try:
            root_dir = Path(__file__).resolve().parents[1]
            sdk_python_dir = root_dir / "unitree_sdk2_python"
            if str(sdk_python_dir) not in sys.path:
                sys.path.insert(0, str(sdk_python_dir))

            from unitree_sdk2py.core.channel import (
                ChannelFactoryInitialize,
                ChannelPublisher,
                ChannelSubscriber,
            )
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as HG_LowCmd
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as HG_LowState
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import MotorCmd_ as HG_MotorCmd
            from unitree_sdk2py.utils.crc import CRC

            ChannelFactoryInitialize(domain_id, interface_name)
            publisher = ChannelPublisher(CMD_TOPIC, HG_LowCmd)
            publisher.Init()
            subscriber = ChannelSubscriber(STATE_TOPIC, HG_LowState)
            subscriber.Init()
            crc = CRC()

            low_cmd = HG_LowCmd(
                mode_pr=0,
                mode_machine=0,
                motor_cmd=[
                    HG_MotorCmd(
                        mode=1,
                        q=POS_STOP_F,
                        dq=VEL_STOP_F,
                        tau=0.0,
                        kp=0.0,
                        kd=0.0,
                        reserve=0,
                    )
                    for _ in range(NUM_MOTORS)
                ],
                reserve=[0, 0, 0, 0],
                crc=0,
            )

            while not self.stop_event.is_set():
                state_msg = subscriber.Read()
                if state_msg is not None:
                    low_cmd.mode_machine = state_msg.mode_machine

                slider_values = list(self.slider_values)
                for binding_index, binding in enumerate(self.bindings):
                    if binding.joint_index < 0 or binding.joint_index >= NUM_MOTORS:
                        continue
                    target = self._apply_binding_transform(slider_values[binding_index], binding)
                    motor_cmd = low_cmd.motor_cmd[binding.joint_index]
                    motor_cmd.mode = 1
                    motor_cmd.q = target
                    motor_cmd.dq = 0.0
                    motor_cmd.kp = binding.kp
                    motor_cmd.kd = binding.kd
                    motor_cmd.tau = 0.0

                low_cmd.crc = crc.Crc(low_cmd)
                publisher.Write(low_cmd)
                time.sleep(COMMAND_DT_SEC)

        except Exception as exc:  # noqa: BLE001
            self.root.after(
                0,
                lambda: messagebox.showerror(
                    APP_TITLE,
                    f"Virtual-pot publisher failed:\n{exc}",
                ),
            )
        finally:
            self.stop_event.set()
            self.root.after(0, self._publisher_stopped)

    def _publisher_stopped(self) -> None:
        last_mode = self.publish_mode
        self._set_publish_buttons(None)
        if last_mode is None:
            self._set_status("Virtual-pot publisher stopped")
        else:
            self._set_status(f"{last_mode.capitalize()} publisher stopped")

    def _on_close(self) -> None:
        self.stop_event.set()
        if self.sim_process is not None and self.sim_process.poll() is None:
            self.sim_process.terminate()
        self.root.destroy()


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if len(args) != 1:
        raise SystemExit(
            f"Usage: {Path(__file__).name} <project{DEFAULT_PROJECT_SUFFIX}>"
        )

    project_path = Path(args[0]).resolve()
    if not project_path.exists():
        raise SystemExit(f"Project file not found: {project_path}")

    root = tk.Tk()
    app = VirtualPotSimulatorApp(root, project_path)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
