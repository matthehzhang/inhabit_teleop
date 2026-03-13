#!/usr/bin/env python3
"""
G1 Unified Studio — node-based joint config editor + integrated pot simulator.
Dark-mode UI built on customtkinter. Replaces the separate g1_joint_config_ui.py
and g1_virtual_pot_sim.py apps with a single tabbed interface.

Usage:
    python g1_unified_studio.py                     # new empty project
    python g1_unified_studio.py <project.g1config.json>  # open existing
"""

import json
import subprocess
import sys
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional
import tkinter as tk
from tkinter import filedialog, messagebox

try:
    import customtkinter as ctk
except ImportError:
    raise SystemExit("customtkinter is required: pip install customtkinter")

# ═══════════════════════════════════════════════════════════════════════════════
# Constants
# ═══════════════════════════════════════════════════════════════════════════════

APP_TITLE = "G1 Unified Studio"
DEFAULT_IMPORT_MODULE = "g1_bridge_lib_20ch"
DEFAULT_PACKET_VALUE_COUNT = 20
DEFAULT_PROJECT_SUFFIX = ".g1config.json"
NUM_MOTORS = 35
POS_STOP_F = 2.146e9
VEL_STOP_F = 16000.0
COMMAND_DT_SEC = 0.02
CMD_TOPIC = "rt/lowcmd"
STATE_TOPIC = "rt/lowstate"

COLORS = {
    "bg":           "#1a1a2e",
    "node_fill":    "#16213e",
    "node_sel":     "#0f3460",
    "node_border":  "#2a2a4a",
    "sel_border":   "#00d4ff",
    "text":         "#e0e0e0",
    "text_dim":     "#8888aa",
    "accent":       "#00d4ff",
    "frame":        "#0f3460",
    "entry_bg":     "#16213e",
    "error":        "#ff4444",
    "grid_line":    "#1e1e3a",
}

NODE_W = 200
NODE_H = 110
NODES_PER_ROW = 4
NODE_GAP_X = 40
NODE_GAP_Y = 50
LAYOUT_MARGIN = 60

# ═══════════════════════════════════════════════════════════════════════════════
# Data model
# ═══════════════════════════════════════════════════════════════════════════════


@dataclass
class JointBindingRecord:
    name: str
    packet_index: int
    joint_index: int
    kp: float
    kd: float
    scale: float = 1.0
    offset: float = 0.0
    min_q: Optional[float] = None
    max_q: Optional[float] = None
    average_window_size: int = 1


def default_bindings(count: int = 17) -> list[JointBindingRecord]:
    return [
        JointBindingRecord(
            name=f"control_{i:02d}",
            packet_index=i,
            joint_index=-1,
            kp=20.0,
            kd=1.0,
        )
        for i in range(count)
    ]


def auto_layout_positions(count: int) -> list[tuple[float, float]]:
    positions = []
    for i in range(count):
        x = (i % NODES_PER_ROW) * (NODE_W + NODE_GAP_X) + LAYOUT_MARGIN
        y = (i // NODES_PER_ROW) * (NODE_H + NODE_GAP_Y) + LAYOUT_MARGIN
        positions.append((float(x), float(y)))
    return positions


# ═══════════════════════════════════════════════════════════════════════════════
# AppState — shared observable state
# ═══════════════════════════════════════════════════════════════════════════════


class AppState:
    def __init__(self):
        self.project_path: Optional[Path] = None
        self.config_name: str = "17-pot control"
        self.import_module: str = DEFAULT_IMPORT_MODULE
        self.packet_value_count: int = DEFAULT_PACKET_VALUE_COUNT
        self.bindings: list[JointBindingRecord] = default_bindings()
        self.node_positions: dict[str, tuple[float, float]] = {}
        self.is_dirty: bool = False

        self._on_bindings_changed: list = []
        self._on_dirty_changed: list = []

    def register_bindings_changed(self, cb) -> None:
        self._on_bindings_changed.append(cb)

    def register_dirty_changed(self, cb) -> None:
        self._on_dirty_changed.append(cb)

    def notify_bindings_changed(self) -> None:
        for cb in self._on_bindings_changed:
            cb()

    def mark_dirty(self) -> None:
        if not self.is_dirty:
            self.is_dirty = True
            for cb in self._on_dirty_changed:
                cb()

    def mark_clean(self) -> None:
        self.is_dirty = False
        for cb in self._on_dirty_changed:
            cb()

    def get_node_pos(self, name: str, default_x: float, default_y: float) -> tuple[float, float]:
        return self.node_positions.get(name, (default_x, default_y))

    def set_node_pos(self, name: str, x: float, y: float) -> None:
        self.node_positions[name] = (x, y)

    def ensure_positions(self) -> None:
        """Assign auto-layout positions to any bindings without saved positions."""
        defaults = auto_layout_positions(len(self.bindings))
        for i, b in enumerate(self.bindings):
            if b.name not in self.node_positions:
                dx, dy = defaults[i] if i < len(defaults) else (LAYOUT_MARGIN, LAYOUT_MARGIN + i * (NODE_H + NODE_GAP_Y))
                self.node_positions[b.name] = (dx, dy)


# ═══════════════════════════════════════════════════════════════════════════════
# ProjectIO — file operations
# ═══════════════════════════════════════════════════════════════════════════════


class ProjectIO:
    def __init__(self, state: AppState):
        self._state = state
        self._after_load_callbacks: list = []

    def on_after_load(self, cb) -> None:
        self._after_load_callbacks.append(cb)

    def _fire_after_load(self) -> None:
        for cb in self._after_load_callbacks:
            cb()

    def new_project(self, parent) -> None:
        if not self._confirm_discard(parent):
            return
        self._state.project_path = None
        self._state.config_name = "17-pot control"
        self._state.import_module = DEFAULT_IMPORT_MODULE
        self._state.packet_value_count = DEFAULT_PACKET_VALUE_COUNT
        self._state.bindings = default_bindings()
        self._state.node_positions = {}
        self._state.ensure_positions()
        self._state.mark_clean()
        self._state.notify_bindings_changed()
        self._fire_after_load()

    def open_project(self, parent) -> bool:
        if not self._confirm_discard(parent):
            return False
        chosen = filedialog.askopenfilename(
            title="Open Joint Config Project",
            filetypes=[("G1 Config JSON", f"*{DEFAULT_PROJECT_SUFFIX}"), ("JSON", "*.json")],
            initialdir=str(Path(__file__).resolve().parent),
        )
        if not chosen:
            return False
        return self._load_file(Path(chosen))

    def _load_file(self, path: Path) -> bool:
        payload = json.loads(path.read_text(encoding="utf-8"))
        bindings = []
        for item in payload.get("bindings", []):
            # Strip unknown keys for forward compat
            known = {f.name for f in JointBindingRecord.__dataclass_fields__.values()}
            filtered = {k: v for k, v in item.items() if k in known}
            bindings.append(JointBindingRecord(**filtered))
        if not bindings:
            bindings = default_bindings()

        self._state.project_path = path
        self._state.config_name = payload.get("config_name", "17-pot control")
        self._state.import_module = payload.get("import_module", DEFAULT_IMPORT_MODULE)
        self._state.packet_value_count = payload.get("packet_value_count", DEFAULT_PACKET_VALUE_COUNT)
        self._state.bindings = bindings
        self._state.node_positions = {
            k: tuple(v) for k, v in payload.get("node_positions", {}).items()
        }
        self._state.ensure_positions()
        self._state.mark_clean()
        self._state.notify_bindings_changed()
        self._fire_after_load()
        return True

    def save_project(self, parent, force_dialog: bool = False) -> bool:
        path = self._state.project_path
        if path is None or force_dialog:
            chosen = filedialog.asksaveasfilename(
                title="Save Joint Config Project",
                defaultextension=DEFAULT_PROJECT_SUFFIX,
                filetypes=[("G1 Config JSON", f"*{DEFAULT_PROJECT_SUFFIX}"), ("JSON", "*.json")],
                initialdir=str(Path(__file__).resolve().parent),
            )
            if not chosen:
                return False
            path = Path(chosen)

        payload = {
            "version": 1,
            "config_name": self._state.config_name,
            "import_module": self._state.import_module,
            "packet_value_count": self._state.packet_value_count,
            "bindings": [asdict(b) for b in self._state.bindings],
            "node_positions": {k: list(v) for k, v in self._state.node_positions.items()},
        }
        path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        self._state.project_path = path
        self._state.mark_clean()
        self._fire_after_load()
        return True

    def validate_for_export(self) -> None:
        s = self._state
        if s.packet_value_count < 1:
            raise ValueError("Packet value count must be >= 1")
        if not s.config_name.strip():
            raise ValueError("Config name cannot be empty")
        if not s.import_module.strip():
            raise ValueError("Import module cannot be empty")
        if not s.bindings:
            raise ValueError("At least one binding is required")
        for b in s.bindings:
            if b.packet_index < 0 or b.packet_index >= s.packet_value_count:
                raise ValueError(f"{b.name}: packet_index {b.packet_index} out of range 0..{s.packet_value_count - 1}")
            if b.joint_index < 0:
                raise ValueError(f"{b.name}: joint_index must be replaced with a real motor index before export")
            if b.min_q is not None and b.max_q is not None and b.min_q > b.max_q:
                raise ValueError(f"{b.name}: min_q must be <= max_q")

    def render_python_config(self) -> str:
        s = self._state
        lines = [
            f"from {s.import_module} import BridgeConfig, JointBinding",
            "",
            "",
            "BRIDGE_CONFIG = BridgeConfig(",
            f"    name={repr(s.config_name)},",
            f"    packet_value_count={s.packet_value_count},",
            "    joint_bindings=(",
        ]
        for b in s.bindings:
            lines.extend([
                "        JointBinding(",
                f"            name={repr(b.name)},",
                f"            packet_index={b.packet_index},",
                f"            joint_index={b.joint_index},",
                f"            kp={b.kp},",
                f"            kd={b.kd},",
                f"            scale={b.scale},",
                f"            offset={b.offset},",
                f"            min_q={repr(b.min_q)},",
                f"            max_q={repr(b.max_q)},",
                f"            average_window_size={b.average_window_size},",
                "        ),",
            ])
        lines.extend(["    ),", ")", ""])
        return "\n".join(lines)

    def export_python(self, parent) -> None:
        try:
            self.validate_for_export()
        except ValueError as exc:
            messagebox.showerror(APP_TITLE, str(exc))
            return

        initial_name = "g1_generated_bridge_config.py"
        if self._state.project_path is not None:
            stem = self._state.project_path.name
            if stem.endswith(DEFAULT_PROJECT_SUFFIX):
                stem = stem[: -len(DEFAULT_PROJECT_SUFFIX)]
            else:
                stem = self._state.project_path.stem
            initial_name = f"{stem}_export.py"

        chosen = filedialog.asksaveasfilename(
            title="Export Bridge Python Config",
            defaultextension=".py",
            filetypes=[("Python", "*.py")],
            initialdir=str(Path(__file__).resolve().parent),
            initialfile=initial_name,
        )
        if not chosen:
            return
        Path(chosen).write_text(self.render_python_config(), encoding="utf-8")
        messagebox.showinfo(APP_TITLE, f"Exported:\n{chosen}")

    def _confirm_discard(self, parent) -> bool:
        if not self._state.is_dirty:
            return True
        return messagebox.askyesno(APP_TITLE, "Discard unsaved changes?", parent=parent)


# ═══════════════════════════════════════════════════════════════════════════════
# NodeCanvas — zoomable/pannable node editor canvas
# ═══════════════════════════════════════════════════════════════════════════════


class NodeCanvas(tk.Canvas):
    def __init__(self, parent, state: AppState, on_select, **kwargs):
        super().__init__(parent, bg=COLORS["bg"], highlightthickness=0, **kwargs)
        self._state = state
        self._on_select = on_select  # callback(index | None)

        self._scale = 1.0
        self._offset_x = 0.0
        self._offset_y = 0.0
        self._selected: Optional[int] = None
        self._drag_index: Optional[int] = None
        self._drag_start_world = (0.0, 0.0)
        self._drag_start_mouse = (0, 0)
        self._pan_start_mouse = (0, 0)
        self._pan_start_offset = (0.0, 0.0)
        self._is_panning = False

        self._bind_events()
        state.register_bindings_changed(self.redraw)

    # ── coordinate transforms ──

    def _w2c(self, wx: float, wy: float) -> tuple[float, float]:
        return wx * self._scale + self._offset_x, wy * self._scale + self._offset_y

    def _c2w(self, cx: float, cy: float) -> tuple[float, float]:
        return (cx - self._offset_x) / self._scale, (cy - self._offset_y) / self._scale

    # ── event bindings ──

    def _bind_events(self) -> None:
        # Zoom
        self.bind("<Button-4>", self._on_scroll_up)
        self.bind("<Button-5>", self._on_scroll_down)
        self.bind("<MouseWheel>", self._on_mousewheel)

        # Pan: middle-click or ctrl+left
        self.bind("<ButtonPress-2>", self._pan_start)
        self.bind("<B2-Motion>", self._pan_motion)
        self.bind("<ButtonRelease-2>", self._pan_end)
        self.bind("<Control-ButtonPress-1>", self._pan_start)
        self.bind("<Control-B1-Motion>", self._pan_motion)
        self.bind("<Control-ButtonRelease-1>", self._pan_end)

        # Node interact
        self.bind("<ButtonPress-1>", self._on_press)
        self.bind("<B1-Motion>", self._on_drag)
        self.bind("<ButtonRelease-1>", self._on_release)
        self.bind("<Double-Button-1>", self._on_double)
        self.bind("<Button-3>", self._on_right_click)

    # ── zoom ──

    def _zoom(self, factor: float, cx: float, cy: float) -> None:
        new_scale = max(0.15, min(5.0, self._scale * factor))
        real_factor = new_scale / self._scale
        self._offset_x = cx - (cx - self._offset_x) * real_factor
        self._offset_y = cy - (cy - self._offset_y) * real_factor
        self._scale = new_scale
        self.redraw()

    def _on_scroll_up(self, e):
        self._zoom(1.12, e.x, e.y)

    def _on_scroll_down(self, e):
        self._zoom(1 / 1.12, e.x, e.y)

    def _on_mousewheel(self, e):
        factor = 1.12 if e.delta > 0 else 1 / 1.12
        self._zoom(factor, e.x, e.y)

    # ── pan ──

    def _pan_start(self, e):
        self._is_panning = True
        self._pan_start_mouse = (e.x, e.y)
        self._pan_start_offset = (self._offset_x, self._offset_y)

    def _pan_motion(self, e):
        if not self._is_panning:
            return
        self._offset_x = self._pan_start_offset[0] + (e.x - self._pan_start_mouse[0])
        self._offset_y = self._pan_start_offset[1] + (e.y - self._pan_start_mouse[1])
        self.redraw()

    def _pan_end(self, _e):
        self._is_panning = False

    # ── hit testing ──

    def _hit_test(self, cx: int, cy: int) -> Optional[int]:
        wx, wy = self._c2w(cx, cy)
        for i in reversed(range(len(self._state.bindings))):
            b = self._state.bindings[i]
            px, py = self._state.get_node_pos(b.name, 0, 0)
            if px <= wx <= px + NODE_W and py <= wy <= py + NODE_H:
                return i
        return None

    # ── node interaction ──

    def _on_press(self, e):
        if self._is_panning:
            return
        idx = self._hit_test(e.x, e.y)
        if idx is None:
            self.select(None)
            return
        self.select(idx)
        self._drag_index = idx
        b = self._state.bindings[idx]
        px, py = self._state.get_node_pos(b.name, 0, 0)
        self._drag_start_world = (px, py)
        self._drag_start_mouse = (e.x, e.y)

    def _on_drag(self, e):
        if self._drag_index is None or self._is_panning:
            return
        dx = (e.x - self._drag_start_mouse[0]) / self._scale
        dy = (e.y - self._drag_start_mouse[1]) / self._scale
        b = self._state.bindings[self._drag_index]
        self._state.set_node_pos(
            b.name,
            self._drag_start_world[0] + dx,
            self._drag_start_world[1] + dy,
        )
        self.redraw()

    def _on_release(self, _e):
        if self._drag_index is not None:
            self._state.mark_dirty()
        self._drag_index = None
        self._is_panning = False  # clear in case Ctrl was released before mouse

    def _on_double(self, e):
        idx = self._hit_test(e.x, e.y)
        if idx is not None:
            self.select(idx)

    def _on_right_click(self, e):
        idx = self._hit_test(e.x, e.y)
        wx, wy = self._c2w(e.x, e.y)
        menu = tk.Menu(self, tearoff=0, bg=COLORS["frame"], fg=COLORS["text"],
                       activebackground=COLORS["accent"], activeforeground="#000000",
                       font=("Segoe UI", 10))
        menu.add_command(label="Add Node Here", command=lambda: self._cmd_add(wx, wy))
        if idx is not None:
            menu.add_command(label="Duplicate", command=lambda: self._cmd_dup(idx))
            menu.add_separator()
            menu.add_command(label="Delete", command=lambda: self._cmd_delete(idx))
        menu.tk_popup(e.x_root, e.y_root)

    def _unique_name(self, base: str) -> str:
        existing = {b.name for b in self._state.bindings}
        if base not in existing:
            return base
        i = 1
        while f"{base}_{i}" in existing:
            i += 1
        return f"{base}_{i}"

    def _cmd_add(self, wx: float, wy: float) -> None:
        n = len(self._state.bindings)
        name = self._unique_name(f"control_{n:02d}")
        b = JointBindingRecord(
            name=name, packet_index=n, joint_index=-1, kp=20.0, kd=1.0)
        self._state.bindings.append(b)
        self._state.set_node_pos(b.name, wx, wy)
        self._state.mark_dirty()
        self._state.notify_bindings_changed()
        self.select(len(self._state.bindings) - 1)

    def _cmd_dup(self, idx: int) -> None:
        src = self._state.bindings[idx]
        b = JointBindingRecord(**asdict(src))
        b.name = self._unique_name(f"{src.name}_copy")
        sx, sy = self._state.get_node_pos(src.name, 0, 0)
        self._state.bindings.insert(idx + 1, b)
        self._state.set_node_pos(b.name, sx + 30, sy + 30)
        self._state.mark_dirty()
        self._state.notify_bindings_changed()
        self.select(idx + 1)

    def _cmd_delete(self, idx: int) -> None:
        b = self._state.bindings.pop(idx)
        self._state.node_positions.pop(b.name, None)
        self._state.mark_dirty()
        if self._selected == idx:
            self.select(None)
        elif self._selected is not None and self._selected > idx:
            self._selected -= 1
        self._state.notify_bindings_changed()

    # ── selection ──

    def select(self, idx: Optional[int]) -> None:
        self._selected = idx
        self.redraw()
        self._on_select(idx)

    # ── rendering ──

    def redraw(self) -> None:
        self.delete("all")
        self._draw_grid()
        for i in range(len(self._state.bindings)):
            self._draw_node(i)

    def _draw_grid(self) -> None:
        w = self.winfo_width() or 1500
        h = self.winfo_height() or 900
        # Compute grid spacing in world coords — aim for ~80px screen spacing
        spacing = max(50, int(80 / self._scale / 10) * 10)
        x0, y0 = self._c2w(0, 0)
        x1, y1 = self._c2w(w, h)
        gx = int(x0 / spacing) * spacing
        while gx < x1:
            cx, _ = self._w2c(gx, 0)
            self.create_line(cx, 0, cx, h, fill=COLORS["grid_line"], width=1)
            gx += spacing
        gy = int(y0 / spacing) * spacing
        while gy < y1:
            _, cy = self._w2c(0, gy)
            self.create_line(0, cy, w, cy, fill=COLORS["grid_line"], width=1)
            gy += spacing

    def _draw_node(self, idx: int) -> None:
        b = self._state.bindings[idx]
        px, py = self._state.get_node_pos(b.name, 0, 0)
        x0, y0 = self._w2c(px, py)
        x1, y1 = self._w2c(px + NODE_W, py + NODE_H)
        sel = idx == self._selected

        # Glow for selected
        if sel:
            self.create_rectangle(x0 - 3, y0 - 3, x1 + 3, y1 + 3,
                                  outline=COLORS["sel_border"], fill="", width=2)

        fill = COLORS["node_sel"] if sel else COLORS["node_fill"]
        border = COLORS["sel_border"] if sel else COLORS["node_border"]
        self.create_rectangle(x0, y0, x1, y1, fill=fill, outline=border,
                              width=2 if sel else 1)

        # Scale font sizes with zoom
        fs = max(7, min(14, int(10 * self._scale)))
        fs_hdr = max(8, min(16, int(12 * self._scale)))
        tx = x0 + (x1 - x0) * 0.06

        self.create_text(tx, y0 + (y1 - y0) * 0.18, text=b.name, anchor="w",
                         fill=COLORS["text"], font=("Consolas", fs_hdr, "bold"))
        self.create_text(tx, y0 + (y1 - y0) * 0.42,
                         text=f"pkt[{b.packet_index}] -> jnt[{b.joint_index}]",
                         anchor="w", fill=COLORS["text_dim"], font=("Consolas", fs))
        self.create_text(tx, y0 + (y1 - y0) * 0.62,
                         text=f"kp={b.kp}  kd={b.kd}",
                         anchor="w", fill=COLORS["text_dim"], font=("Consolas", fs))

        if b.scale != 1.0 or b.offset != 0.0:
            self.create_text(tx, y0 + (y1 - y0) * 0.80,
                             text=f"s={b.scale} o={b.offset}",
                             anchor="w", fill=COLORS["text_dim"], font=("Consolas", max(6, fs - 1)))

        if b.joint_index < 0:
            self.create_text(x0 + (x1 - x0) * 0.5, y0 + (y1 - y0) * 0.92,
                             text="! unassigned", anchor="center",
                             fill=COLORS["error"], font=("Consolas", max(7, fs)))


# ═══════════════════════════════════════════════════════════════════════════════
# PropertyPanel — side editor for the selected node
# ═══════════════════════════════════════════════════════════════════════════════


class PropertyPanel(ctk.CTkFrame):
    FIELDS = [
        ("name", "Name"),
        ("packet_index", "Packet Idx"),
        ("joint_index", "Joint Idx"),
        ("kp", "Kp"),
        ("kd", "Kd"),
        ("scale", "Scale"),
        ("offset", "Offset"),
        ("min_q", "Min q"),
        ("max_q", "Max q"),
        ("average_window_size", "Avg Window"),
    ]

    def __init__(self, parent, state: AppState, on_apply, **kwargs):
        super().__init__(parent, fg_color=COLORS["frame"], width=280, **kwargs)
        self._state = state
        self._on_apply = on_apply
        self._selected: Optional[int] = None
        self._vars: dict[str, tk.StringVar] = {}
        self._build()

    def _build(self) -> None:
        ctk.CTkLabel(self, text="Properties", font=ctk.CTkFont(size=15, weight="bold"),
                     text_color=COLORS["accent"]).pack(pady=(14, 10), padx=14, anchor="w")

        form = ctk.CTkFrame(self, fg_color="transparent")
        form.pack(fill="x", padx=14)
        form.columnconfigure(1, weight=1)

        for row, (key, label) in enumerate(self.FIELDS):
            var = tk.StringVar()
            self._vars[key] = var
            ctk.CTkLabel(form, text=label, text_color=COLORS["text_dim"],
                         font=ctk.CTkFont(size=12)).grid(
                row=row, column=0, sticky="w", pady=3, padx=(0, 8))
            ctk.CTkEntry(form, textvariable=var, fg_color=COLORS["entry_bg"],
                         text_color=COLORS["text"], border_color=COLORS["node_border"],
                         height=28, font=ctk.CTkFont(size=12)).grid(
                row=row, column=1, sticky="ew", pady=3)

        btns = ctk.CTkFrame(self, fg_color="transparent")
        btns.pack(fill="x", padx=14, pady=(14, 6))
        ctk.CTkButton(btns, text="Apply", width=80, fg_color=COLORS["accent"],
                       text_color="#000000", hover_color="#33e0ff",
                       command=self.apply).pack(side="left", padx=(0, 6))
        ctk.CTkButton(btns, text="Revert", width=80, fg_color=COLORS["node_fill"],
                       text_color=COLORS["text_dim"], hover_color=COLORS["node_border"],
                       command=self.revert).pack(side="left")

    def load(self, idx: int) -> None:
        self._selected = idx
        b = self._state.bindings[idx]
        self._vars["name"].set(b.name)
        self._vars["packet_index"].set(str(b.packet_index))
        self._vars["joint_index"].set(str(b.joint_index))
        self._vars["kp"].set(str(b.kp))
        self._vars["kd"].set(str(b.kd))
        self._vars["scale"].set(str(b.scale))
        self._vars["offset"].set(str(b.offset))
        self._vars["min_q"].set("" if b.min_q is None else str(b.min_q))
        self._vars["max_q"].set("" if b.max_q is None else str(b.max_q))
        self._vars["average_window_size"].set(str(b.average_window_size))

    def revert(self) -> None:
        if self._selected is not None:
            self.load(self._selected)

    def apply(self) -> bool:
        if self._selected is None:
            return True
        try:
            updated = self._parse()
        except ValueError as exc:
            messagebox.showerror(APP_TITLE, str(exc))
            return False
        old = self._state.bindings[self._selected]
        # Move position key if name changed
        if updated.name != old.name and old.name in self._state.node_positions:
            self._state.node_positions[updated.name] = self._state.node_positions.pop(old.name)
        self._state.bindings[self._selected] = updated
        self._state.mark_dirty()
        self._state.notify_bindings_changed()
        self._on_apply(self._selected)
        return True

    def _parse_optional_float(self, val: str, label: str) -> Optional[float]:
        if val.strip() == "":
            return None
        try:
            return float(val)
        except ValueError:
            raise ValueError(f"{label} must be blank or a number")

    def _parse(self) -> JointBindingRecord:
        try:
            pkt = int(self._vars["packet_index"].get())
            jnt = int(self._vars["joint_index"].get())
            kp = float(self._vars["kp"].get())
            kd = float(self._vars["kd"].get())
            scale = float(self._vars["scale"].get())
            offset = float(self._vars["offset"].get())
            avg = int(self._vars["average_window_size"].get())
        except ValueError:
            raise ValueError("Numeric fields must be valid numbers")
        name = self._vars["name"].get().strip()
        if not name:
            raise ValueError("Name cannot be empty")
        if avg < 1:
            raise ValueError("Avg window must be >= 1")
        mn = self._parse_optional_float(self._vars["min_q"].get(), "Min q")
        mx = self._parse_optional_float(self._vars["max_q"].get(), "Max q")
        if mn is not None and mx is not None and mn > mx:
            raise ValueError("Min q must be <= Max q")
        return JointBindingRecord(
            name=name, packet_index=pkt, joint_index=jnt,
            kp=kp, kd=kd, scale=scale, offset=offset,
            min_q=mn, max_q=mx, average_window_size=avg,
        )


# ═══════════════════════════════════════════════════════════════════════════════
# ConfigTab — node canvas + property panel
# ═══════════════════════════════════════════════════════════════════════════════


class ConfigTab(ctk.CTkFrame):
    def __init__(self, parent, state: AppState, status_var: tk.StringVar, **kwargs):
        super().__init__(parent, fg_color=COLORS["bg"], **kwargs)
        self._state = state
        self._status = status_var
        self._build()

    def _build(self) -> None:
        # Action bar
        bar = ctk.CTkFrame(self, fg_color=COLORS["frame"], height=40)
        bar.pack(fill="x", pady=(0, 2))
        for text, cmd in [
            ("17-Pot Template", self._reset_template),
            ("Add Node", self._add_node),
            ("Auto Layout", self._auto_layout),
        ]:
            ctk.CTkButton(bar, text=text, width=110, height=30,
                           fg_color=COLORS["node_fill"], text_color=COLORS["accent"],
                           hover_color=COLORS["node_border"],
                           command=cmd).pack(side="left", padx=4, pady=5)

        # Content: canvas + property panel
        content = ctk.CTkFrame(self, fg_color="transparent")
        content.pack(fill="both", expand=True)
        content.columnconfigure(0, weight=1)
        content.rowconfigure(0, weight=1)

        self._canvas = NodeCanvas(content, self._state, on_select=self._on_select)
        self._canvas.grid(row=0, column=0, sticky="nsew")

        self._props = PropertyPanel(content, self._state, on_apply=self._on_apply)
        # Hidden initially — shown by _on_select

    def _on_select(self, idx: Optional[int]) -> None:
        if idx is None:
            self._props.grid_forget()
        else:
            self._props.load(idx)
            self._props.grid(row=0, column=1, sticky="ns", padx=(2, 0))

    def _on_apply(self, idx: int) -> None:
        self._canvas.select(idx)
        self._status.set(f"Updated '{self._state.bindings[idx].name}'")

    def _reset_template(self) -> None:
        self._state.bindings = default_bindings()
        self._state.node_positions = {}
        self._state.ensure_positions()
        self._state.mark_dirty()
        self._state.notify_bindings_changed()
        self._canvas.select(None)
        self._status.set("Loaded 17-pot template")

    def _add_node(self) -> None:
        # Place at center of current view
        w = self._canvas.winfo_width() or 800
        h = self._canvas.winfo_height() or 600
        wx, wy = self._canvas._c2w(w / 2, h / 2)
        self._canvas._cmd_add(wx, wy)
        self._status.set("Added node")

    def _auto_layout(self) -> None:
        positions = auto_layout_positions(len(self._state.bindings))
        for i, b in enumerate(self._state.bindings):
            if i < len(positions):
                self._state.set_node_pos(b.name, *positions[i])
        self._state.mark_dirty()
        self._canvas.redraw()
        self._status.set("Auto-arranged nodes")

    def force_redraw(self) -> None:
        self._canvas.redraw()


# ═══════════════════════════════════════════════════════════════════════════════
# PotSimTab — sliders + DDS publishing
# ═══════════════════════════════════════════════════════════════════════════════


class PotSimTab(ctk.CTkFrame):
    def __init__(self, parent, state: AppState, root: ctk.CTk, status_var: tk.StringVar, **kwargs):
        super().__init__(parent, fg_color=COLORS["bg"], **kwargs)
        self._state = state
        self._app_root = root  # avoid shadowing tkinter Widget._root()
        self._status = status_var

        self._slider_vars: list[tk.DoubleVar] = []
        self._slider_values: list[float] = []
        self._value_labels: list[tk.StringVar] = []
        self._lock = threading.Lock()

        self._sim_process: Optional[subprocess.Popen] = None
        self._publisher_thread: Optional[threading.Thread] = None
        self._publish_mode: Optional[str] = None
        self._stop_event = threading.Event()
        self._robot_armed = False

        self._python_cmd = tk.StringVar(value=sys.executable)
        self._sim_script = tk.StringVar(
            value=str(Path(__file__).resolve().parents[1]
                      / "unitree_mujoco" / "simulate_python" / "unitree_mujoco.py"))
        self._sim_domain = tk.StringVar(value="1")
        self._sim_iface = tk.StringVar(value="lo")
        self._robot_domain = tk.StringVar(value="0")
        self._robot_iface = tk.StringVar(value="")
        self._sim_status = tk.StringVar(value="Sim: not started")
        self._arm_status = tk.StringVar(value="DISARMED")
        self._pub_sim_text = tk.StringVar(value="Start Sim Publisher")
        self._pub_robot_text = tk.StringVar(value="Start Robot Publisher")
        self._arm_btn_text = tk.StringVar(value="Arm Robot")

        self._build()
        state.register_bindings_changed(self._rebuild_sliders)

    def _build(self) -> None:
        self._build_controls()
        self._slider_frame = ctk.CTkScrollableFrame(self, fg_color=COLORS["bg"])
        self._slider_frame.pack(fill="both", expand=True, padx=8, pady=4)
        self._slider_frame.columnconfigure(1, weight=1)
        self._build_bottom_bar()
        self._rebuild_sliders()

    def _build_controls(self) -> None:
        ctl = ctk.CTkFrame(self, fg_color=COLORS["frame"])
        ctl.pack(fill="x", padx=0, pady=(0, 2))

        # Row 1: sim settings
        r1 = ctk.CTkFrame(ctl, fg_color="transparent")
        r1.pack(fill="x", padx=8, pady=(6, 2))
        for label, var, w in [
            ("Python:", self._python_cmd, 200),
            ("Sim Script:", self._sim_script, 300),
        ]:
            ctk.CTkLabel(r1, text=label, text_color=COLORS["text_dim"]).pack(side="left", padx=(0, 4))
            ctk.CTkEntry(r1, textvariable=var, width=w, fg_color=COLORS["entry_bg"],
                         text_color=COLORS["text"]).pack(side="left", padx=(0, 10))
        ctk.CTkButton(r1, text="Launch MuJoCo", width=120, fg_color=COLORS["node_fill"],
                       text_color=COLORS["accent"], hover_color=COLORS["node_border"],
                       command=self._launch_mujoco).pack(side="left", padx=4)

        # Row 2: sim domain + publisher
        r2 = ctk.CTkFrame(ctl, fg_color="transparent")
        r2.pack(fill="x", padx=8, pady=2)
        for label, var, w in [
            ("Sim Domain:", self._sim_domain, 50),
            ("Sim NIC:", self._sim_iface, 80),
        ]:
            ctk.CTkLabel(r2, text=label, text_color=COLORS["text_dim"]).pack(side="left", padx=(0, 4))
            ctk.CTkEntry(r2, textvariable=var, width=w, fg_color=COLORS["entry_bg"],
                         text_color=COLORS["text"]).pack(side="left", padx=(0, 10))
        ctk.CTkButton(r2, textvariable=self._pub_sim_text, width=160,
                       fg_color=COLORS["node_fill"], text_color=COLORS["accent"],
                       hover_color=COLORS["node_border"],
                       command=self._toggle_sim).pack(side="left", padx=4)

        # Row 3: robot domain + publisher + arm
        r3 = ctk.CTkFrame(ctl, fg_color="transparent")
        r3.pack(fill="x", padx=8, pady=(2, 6))
        for label, var, w in [
            ("Robot Domain:", self._robot_domain, 50),
            ("Robot NIC:", self._robot_iface, 100),
        ]:
            ctk.CTkLabel(r3, text=label, text_color=COLORS["text_dim"]).pack(side="left", padx=(0, 4))
            ctk.CTkEntry(r3, textvariable=var, width=w, fg_color=COLORS["entry_bg"],
                         text_color=COLORS["text"]).pack(side="left", padx=(0, 10))
        ctk.CTkButton(r3, textvariable=self._arm_btn_text, width=100,
                       fg_color=COLORS["node_fill"], text_color=COLORS["accent"],
                       hover_color=COLORS["node_border"],
                       command=self._toggle_arm).pack(side="left", padx=4)
        ctk.CTkButton(r3, textvariable=self._pub_robot_text, width=180,
                       fg_color=COLORS["node_fill"], text_color=COLORS["accent"],
                       hover_color=COLORS["node_border"],
                       command=self._toggle_robot).pack(side="left", padx=4)

    def _build_bottom_bar(self) -> None:
        bar = ctk.CTkFrame(self, fg_color=COLORS["frame"], height=32)
        bar.pack(fill="x", padx=0, pady=(2, 0))
        ctk.CTkButton(bar, text="Zero All", width=80, height=28,
                       fg_color=COLORS["node_fill"], text_color=COLORS["accent"],
                       hover_color=COLORS["node_border"],
                       command=self._zero_all).pack(side="left", padx=4, pady=2)
        ctk.CTkButton(bar, text="Center To Offsets", width=130, height=28,
                       fg_color=COLORS["node_fill"], text_color=COLORS["accent"],
                       hover_color=COLORS["node_border"],
                       command=self._center_offsets).pack(side="left", padx=4, pady=2)
        ctk.CTkLabel(bar, textvariable=self._sim_status,
                     text_color=COLORS["text_dim"]).pack(side="left", padx=16)
        ctk.CTkLabel(bar, textvariable=self._arm_status,
                     text_color=COLORS["text_dim"]).pack(side="left", padx=8)

    # ── slider management ──

    def _rebuild_sliders(self) -> None:
        if self._publisher_thread and self._publisher_thread.is_alive():
            self._stop_event.set()
            self._status.set("Publisher stopped: bindings changed")
            self._set_pub_buttons(None)

        for child in self._slider_frame.winfo_children():
            child.destroy()

        self._slider_vars = []
        self._slider_values = []
        self._value_labels = []

        for row, b in enumerate(self._state.bindings):
            sv = tk.DoubleVar(value=0.0)
            lv = tk.StringVar(value="raw=+0.000 tgt=+0.000")
            self._slider_vars.append(sv)
            self._slider_values.append(0.0)
            self._value_labels.append(lv)

            ctk.CTkLabel(self._slider_frame, text=b.name, width=140, anchor="w",
                         text_color=COLORS["text"],
                         font=ctk.CTkFont(family="Consolas", size=12)).grid(
                row=row, column=0, sticky="w", padx=(0, 8), pady=4)

            slider = ctk.CTkSlider(self._slider_frame, from_=-1.5, to=1.5,
                                    variable=sv, width=300,
                                    fg_color=COLORS["node_border"],
                                    progress_color=COLORS["frame"],
                                    button_color=COLORS["accent"],
                                    button_hover_color="#33e0ff")
            slider.grid(row=row, column=1, sticky="ew", padx=(0, 8), pady=4)
            sv.trace_add("write", lambda *_a, i=row: self._on_slider(i))

            ctk.CTkLabel(self._slider_frame, textvariable=lv, width=220, anchor="w",
                         text_color=COLORS["text_dim"],
                         font=ctk.CTkFont(family="Consolas", size=11)).grid(
                row=row, column=2, sticky="w", padx=(0, 4), pady=4)

            ctk.CTkButton(self._slider_frame, text="0", width=32, height=26,
                           fg_color=COLORS["node_fill"], text_color=COLORS["text_dim"],
                           hover_color=COLORS["node_border"],
                           command=lambda i=row: self._set_slider(i, 0.0)).grid(
                row=row, column=3, padx=2, pady=4)

    def _on_slider(self, idx: int) -> None:
        raw = self._slider_vars[idx].get()
        with self._lock:
            self._slider_values[idx] = raw
        b = self._state.bindings[idx]
        tgt = self._apply_transform(raw, b)
        self._value_labels[idx].set(f"raw={raw:+.3f} tgt={tgt:+.3f}")

    def _set_slider(self, idx: int, val: float) -> None:
        self._slider_vars[idx].set(val)

    @staticmethod
    def _apply_transform(raw: float, b: JointBindingRecord) -> float:
        mapped = raw * b.scale + b.offset
        if b.min_q is not None:
            mapped = max(b.min_q, mapped)
        if b.max_q is not None:
            mapped = min(b.max_q, mapped)
        return mapped

    def _zero_all(self) -> None:
        for i in range(len(self._slider_vars)):
            self._set_slider(i, 0.0)
        self._status.set("All sliders zeroed")

    def _center_offsets(self) -> None:
        for i, b in enumerate(self._state.bindings):
            val = 0.0 if b.scale == 0.0 else -b.offset / b.scale
            self._set_slider(i, val)
        self._status.set("Sliders centered to offsets")

    # ── MuJoCo launch ──

    def _launch_mujoco(self) -> None:
        if self._sim_process is not None and self._sim_process.poll() is None:
            messagebox.showinfo(APP_TITLE, "MuJoCo simulator is already running.")
            return
        script = Path(self._sim_script.get()).expanduser()
        if not script.exists():
            messagebox.showerror(APP_TITLE, f"Script not found:\n{script}")
            return
        try:
            self._sim_process = subprocess.Popen(
                [self._python_cmd.get(), str(script)], cwd=str(script.parent))
        except OSError as exc:
            messagebox.showerror(APP_TITLE, f"Launch failed:\n{exc}")
            return
        self._sim_status.set(f"Sim: running ({self._sim_process.pid})")
        self._status.set("Launched MuJoCo")

    # ── arm/disarm ──

    def _toggle_arm(self) -> None:
        if self._robot_armed:
            if self._publish_mode == "robot":
                messagebox.showinfo(APP_TITLE, "Stop robot publishing before disarming.")
                return
            self._robot_armed = False
            self._arm_btn_text.set("Arm Robot")
            self._arm_status.set("DISARMED")
            self._status.set("Robot controls disarmed")
            return

        bad = [b.name for b in self._state.bindings
               if b.joint_index < 0 or b.joint_index >= NUM_MOTORS]
        if bad:
            messagebox.showerror(APP_TITLE,
                "Cannot arm: invalid joint indices.\n\n"
                + ", ".join(bad[:8]) + (" ..." if len(bad) > 8 else ""))
            return

        if not messagebox.askyesno(APP_TITLE,
                "Arm robot controls?\nOnly arm if the robot is supported, clear, and ready.",
                icon=messagebox.WARNING):
            return
        self._robot_armed = True
        self._arm_btn_text.set("Disarm Robot")
        self._arm_status.set("ARMED")
        self._status.set("Robot controls armed")

    # ── publishers ──

    def _toggle_sim(self) -> None:
        self._toggle_publisher("sim")

    def _toggle_robot(self) -> None:
        self._toggle_publisher("robot")

    def _toggle_publisher(self, mode: str) -> None:
        if self._publisher_thread and self._publisher_thread.is_alive():
            if self._publish_mode != mode:
                messagebox.showinfo(APP_TITLE,
                    f"{self._publish_mode} publisher is running. Stop it first.")
                return
            self._stop_event.set()
            self._status.set(f"Stopping {mode} publisher...")
            return

        try:
            if mode == "sim":
                domain = int(self._sim_domain.get())
                iface = self._sim_iface.get().strip()
            else:
                domain = int(self._robot_domain.get())
                iface = self._robot_iface.get().strip()
        except ValueError:
            messagebox.showerror(APP_TITLE, "Domain must be an integer")
            return

        if not iface:
            messagebox.showerror(APP_TITLE, f"{mode.capitalize()} NIC is required")
            return

        if mode == "robot":
            if not self._robot_armed:
                messagebox.showerror(APP_TITLE, "Robot controls are disarmed. Arm first.")
                return
            if not messagebox.askyesno(APP_TITLE,
                    f"Publish live commands to robot?\n\nDomain: {domain}\nNIC: {iface}",
                    icon=messagebox.WARNING):
                return

        # Ensure old thread is fully stopped before starting a new one
        if self._publisher_thread is not None:
            self._stop_event.set()
            self._publisher_thread.join(timeout=2.0)

        self._stop_event.clear()
        self._publish_mode = mode
        self._publisher_thread = threading.Thread(
            target=self._publisher_loop, args=(domain, iface), daemon=True)
        self._publisher_thread.start()
        self._set_pub_buttons(mode)
        self._status.set(f"Started {mode} publisher")

    def _set_pub_buttons(self, active: Optional[str]) -> None:
        self._publish_mode = active
        self._pub_sim_text.set("Stop Sim Publisher" if active == "sim" else "Start Sim Publisher")
        self._pub_robot_text.set("Stop Robot Publisher" if active == "robot" else "Start Robot Publisher")

    def _publisher_loop(self, domain_id: int, iface: str) -> None:
        bindings = list(self._state.bindings)
        try:
            root_dir = Path(__file__).resolve().parents[1]
            sdk_dir = root_dir / "unitree_sdk2_python"
            if str(sdk_dir) not in sys.path:
                sys.path.insert(0, str(sdk_dir))

            from unitree_sdk2py.core.channel import (
                ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber)
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as HG_LowCmd
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as HG_LowState
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import MotorCmd_ as HG_MotorCmd
            from unitree_sdk2py.utils.crc import CRC

            ChannelFactoryInitialize(domain_id, iface)
            pub = ChannelPublisher(CMD_TOPIC, HG_LowCmd)
            pub.Init()
            sub = ChannelSubscriber(STATE_TOPIC, HG_LowState)
            sub.Init()
            crc = CRC()

            low_cmd = HG_LowCmd(
                mode_pr=0, mode_machine=0,
                motor_cmd=[HG_MotorCmd(mode=1, q=POS_STOP_F, dq=VEL_STOP_F,
                                        tau=0.0, kp=0.0, kd=0.0, reserve=0)
                           for _ in range(NUM_MOTORS)],
                reserve=[0, 0, 0, 0], crc=0)

            while not self._stop_event.is_set():
                msg = sub.Read()
                if msg is not None:
                    low_cmd.mode_machine = msg.mode_machine

                with self._lock:
                    vals = list(self._slider_values)

                for bi, b in enumerate(bindings):
                    if b.joint_index < 0 or b.joint_index >= NUM_MOTORS:
                        continue
                    if bi >= len(vals):
                        continue
                    tgt = self._apply_transform(vals[bi], b)
                    mc = low_cmd.motor_cmd[b.joint_index]
                    mc.mode = 1
                    mc.q = tgt
                    mc.dq = 0.0
                    mc.kp = b.kp
                    mc.kd = b.kd
                    mc.tau = 0.0

                low_cmd.crc = crc.Crc(low_cmd)
                pub.Write(low_cmd)
                time.sleep(COMMAND_DT_SEC)

        except Exception as exc:
            self._app_root.after(0, lambda: messagebox.showerror(APP_TITLE,
                f"Publisher failed:\n{exc}"))
        finally:
            self._stop_event.set()
            self._app_root.after(0, self._publisher_stopped)

    def _publisher_stopped(self) -> None:
        mode = self._publish_mode
        self._set_pub_buttons(None)
        self._status.set(f"{(mode or 'Publisher').capitalize()} stopped")

    def stop_all(self) -> None:
        self._stop_event.set()
        if self._sim_process and self._sim_process.poll() is None:
            self._sim_process.terminate()


# ═══════════════════════════════════════════════════════════════════════════════
# Toolbar — top bar with project operations
# ═══════════════════════════════════════════════════════════════════════════════


class Toolbar(ctk.CTkFrame):
    def __init__(self, parent, state: AppState, io: ProjectIO,
                 on_title_change, **kwargs):
        super().__init__(parent, fg_color=COLORS["frame"], height=46, **kwargs)
        self._state = state
        self._io = io
        self._on_title = on_title_change
        self.pack_propagate(False)

        self._name_var = tk.StringVar(value=state.config_name)
        self._mod_var = tk.StringVar(value=state.import_module)
        self._pkt_var = tk.StringVar(value=str(state.packet_value_count))
        self._suppress_sync = False

        self._build()
        self._bind_sync()
        state.register_dirty_changed(on_title_change)

    def _build(self) -> None:
        for text, cmd in [
            ("New", lambda: self._io.new_project(self)),
            ("Open", lambda: self._io.open_project(self)),
            ("Save", lambda: self._io.save_project(self)),
            ("Export Py", lambda: self._io.export_python(self)),
        ]:
            ctk.CTkButton(self, text=text, width=80, height=30,
                           fg_color=COLORS["node_fill"], text_color=COLORS["accent"],
                           border_color=COLORS["node_border"], border_width=1,
                           hover_color=COLORS["node_border"],
                           command=cmd).pack(side="left", padx=(6, 0), pady=8)

        ctk.CTkLabel(self, text="  Config:", text_color=COLORS["text_dim"]).pack(side="left")
        ctk.CTkEntry(self, textvariable=self._name_var, width=150,
                     fg_color=COLORS["entry_bg"], text_color=COLORS["text"],
                     height=28).pack(side="left", padx=(4, 8))

        ctk.CTkLabel(self, text="Module:", text_color=COLORS["text_dim"]).pack(side="left")
        ctk.CTkEntry(self, textvariable=self._mod_var, width=160,
                     fg_color=COLORS["entry_bg"], text_color=COLORS["text"],
                     height=28).pack(side="left", padx=(4, 8))

        ctk.CTkLabel(self, text="Pkt:", text_color=COLORS["text_dim"]).pack(side="left")
        ctk.CTkEntry(self, textvariable=self._pkt_var, width=50,
                     fg_color=COLORS["entry_bg"], text_color=COLORS["text"],
                     height=28).pack(side="left", padx=(4, 0))

    def _bind_sync(self) -> None:
        def sync_name(*_):
            if self._suppress_sync:
                return
            self._state.config_name = self._name_var.get()
            self._state.mark_dirty()
        def sync_mod(*_):
            if self._suppress_sync:
                return
            self._state.import_module = self._mod_var.get()
            self._state.mark_dirty()
        def sync_pkt(*_):
            if self._suppress_sync:
                return
            try:
                self._state.packet_value_count = int(self._pkt_var.get())
            except ValueError:
                pass
            self._state.mark_dirty()
        self._name_var.trace_add("write", sync_name)
        self._mod_var.trace_add("write", sync_mod)
        self._pkt_var.trace_add("write", sync_pkt)

    def reload_from_state(self) -> None:
        self._suppress_sync = True
        try:
            self._name_var.set(self._state.config_name)
            self._mod_var.set(self._state.import_module)
            self._pkt_var.set(str(self._state.packet_value_count))
        finally:
            self._suppress_sync = False
        self._on_title()


# ═══════════════════════════════════════════════════════════════════════════════
# UnifiedStudioApp — root window
# ═══════════════════════════════════════════════════════════════════════════════


class UnifiedStudioApp:
    def __init__(self, initial_project: Optional[Path] = None):
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("dark-blue")

        self._root = ctk.CTk()
        self._root.title(APP_TITLE)
        self._root.geometry("1500x900")
        self._root.configure(fg_color=COLORS["bg"])

        self._state = AppState()
        self._io = ProjectIO(self._state)

        # Status bar (bottom)
        self._status = tk.StringVar(value="Ready")
        status_bar = ctk.CTkLabel(self._root, textvariable=self._status,
                                  anchor="w", fg_color=COLORS["frame"],
                                  text_color=COLORS["text_dim"], height=26)
        status_bar.pack(fill="x", side="bottom")

        # Toolbar (top)
        self._toolbar = Toolbar(self._root, self._state, self._io,
                                on_title_change=self._update_title)
        self._toolbar.pack(fill="x", side="top")

        # Tabs
        self._tabs = ctk.CTkTabview(
            self._root, fg_color=COLORS["bg"],
            segmented_button_fg_color=COLORS["frame"],
            segmented_button_selected_color=COLORS["accent"],
            segmented_button_selected_hover_color="#33e0ff",
            segmented_button_unselected_color=COLORS["frame"],
            text_color=COLORS["text"],
        )
        self._tabs.pack(fill="both", expand=True, padx=0, pady=0)

        self._tabs.add("Node Config")
        self._tabs.add("Pot Simulator")

        self._config_tab = ConfigTab(self._tabs.tab("Node Config"),
                                     self._state, self._status)
        self._config_tab.pack(fill="both", expand=True)

        self._pot_tab = PotSimTab(self._tabs.tab("Pot Simulator"),
                                  self._state, self._root, self._status)
        self._pot_tab.pack(fill="both", expand=True)

        # Wire after-load callback
        self._io.on_after_load(self._on_project_loaded)

        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

        # Load initial project or set up defaults
        if initial_project and initial_project.exists():
            self._io._load_file(initial_project)
        else:
            self._state.ensure_positions()
            self._state.notify_bindings_changed()

        self._update_title()

    def _update_title(self) -> None:
        dirty = "*" if self._state.is_dirty else ""
        name = self._state.project_path.name if self._state.project_path else "untitled"
        self._root.title(f"{APP_TITLE} - {name}{dirty}")

    def _on_project_loaded(self) -> None:
        self._toolbar.reload_from_state()
        self._update_title()

    def _on_close(self) -> None:
        if self._state.is_dirty:
            if not messagebox.askyesno(APP_TITLE, "Quit without saving?"):
                return
        self._pot_tab.stop_all()
        self._root.destroy()

    def run(self) -> None:
        self._root.mainloop()


# ═══════════════════════════════════════════════════════════════════════════════
# Entry point
# ═══════════════════════════════════════════════════════════════════════════════


def main() -> int:
    project = None
    if len(sys.argv) > 1:
        project = Path(sys.argv[1]).resolve()
        if not project.exists():
            raise SystemExit(f"File not found: {project}")
    app = UnifiedStudioApp(initial_project=project)
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
