#!/usr/bin/env python3

import json
import subprocess
import sys
from dataclasses import dataclass, asdict
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk


APP_TITLE = "G1 Joint Config Editor"
DEFAULT_IMPORT_MODULE = "g1_bridge_lib_20ch"
DEFAULT_PACKET_VALUE_COUNT = 20
DEFAULT_PROJECT_SUFFIX = ".g1config.json"
PYTHON_EXPORT_SUFFIX = ".py"
SIMULATOR_SCRIPT_NAME = "g1_virtual_pot_sim.py"


@dataclass
class JointBindingRecord:
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


def default_bindings(count: int = 17) -> list[JointBindingRecord]:
    return [
        JointBindingRecord(
            name=f"control_{index:02d}",
            packet_index=index,
            joint_index=-1,
            kp=20.0,
            kd=1.0,
        )
        for index in range(count)
    ]


class ConfigEditorApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title(APP_TITLE)
        self.root.geometry("1420x780")

        self.project_path: Path | None = None
        self.bindings: list[JointBindingRecord] = default_bindings()
        self._selected_index: int | None = None
        self._is_dirty = False
        self._suspend_dirty_tracking = False

        self.config_name_var = tk.StringVar(value="17-pot control")
        self.import_module_var = tk.StringVar(value=DEFAULT_IMPORT_MODULE)
        self.packet_value_count_var = tk.StringVar(value=str(DEFAULT_PACKET_VALUE_COUNT))
        self.status_var = tk.StringVar(value="Ready")

        self.editor_vars = {
            "name": tk.StringVar(),
            "packet_index": tk.StringVar(),
            "joint_index": tk.StringVar(),
            "kp": tk.StringVar(),
            "kd": tk.StringVar(),
            "scale": tk.StringVar(),
            "offset": tk.StringVar(),
            "min_q": tk.StringVar(),
            "max_q": tk.StringVar(),
            "average_window_size": tk.StringVar(),
        }

        self._build_ui()
        self._refresh_table()
        self._select_index(0)
        self._bind_dirty_tracking()

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        toolbar = ttk.Frame(self.root, padding=10)
        toolbar.grid(row=0, column=0, sticky="ew")
        for index in range(10):
            toolbar.columnconfigure(index, weight=0)
        toolbar.columnconfigure(11, weight=1)

        ttk.Label(toolbar, text="Config Name").grid(row=0, column=0, sticky="w", padx=(0, 6))
        ttk.Entry(toolbar, textvariable=self.config_name_var, width=22).grid(
            row=0, column=1, sticky="ew", padx=(0, 12)
        )
        ttk.Label(toolbar, text="Import Module").grid(row=0, column=2, sticky="w", padx=(0, 6))
        ttk.Entry(toolbar, textvariable=self.import_module_var, width=20).grid(
            row=0, column=3, sticky="ew", padx=(0, 12)
        )
        ttk.Label(toolbar, text="Packet Values").grid(row=0, column=4, sticky="w", padx=(0, 6))
        ttk.Entry(toolbar, textvariable=self.packet_value_count_var, width=8).grid(
            row=0, column=5, sticky="w", padx=(0, 16)
        )

        ttk.Button(toolbar, text="New", command=self.new_project).grid(row=0, column=6, padx=4)
        ttk.Button(toolbar, text="Open JSON", command=self.open_project).grid(row=0, column=7, padx=4)
        ttk.Button(toolbar, text="Save JSON", command=self.save_project).grid(row=0, column=8, padx=4)
        ttk.Button(toolbar, text="Export Python", command=self.export_python).grid(
            row=0, column=9, padx=4
        )
        ttk.Button(toolbar, text="Virtual Pots", command=self.open_virtual_pot_sim).grid(
            row=0, column=10, padx=4
        )

        content = ttk.Panedwindow(self.root, orient=tk.HORIZONTAL)
        content.grid(row=1, column=0, sticky="nsew", padx=10, pady=(0, 10))

        list_frame = ttk.Frame(content, padding=8)
        editor_frame = ttk.Frame(content, padding=10)
        content.add(list_frame, weight=3)
        content.add(editor_frame, weight=2)

        self._build_list_panel(list_frame)
        self._build_editor_panel(editor_frame)

        status = ttk.Label(self.root, textvariable=self.status_var, anchor="w", padding=(10, 4))
        status.grid(row=2, column=0, sticky="ew")

    def _build_list_panel(self, parent: ttk.Frame) -> None:
        parent.rowconfigure(1, weight=1)
        parent.columnconfigure(0, weight=1)

        actions = ttk.Frame(parent)
        actions.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        ttk.Button(actions, text="17-Pot Template", command=self.reset_17_template).pack(
            side=tk.LEFT, padx=(0, 4)
        )
        ttk.Button(actions, text="Add", command=self.add_binding).pack(side=tk.LEFT, padx=4)
        ttk.Button(actions, text="Duplicate", command=self.duplicate_binding).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Button(actions, text="Remove", command=self.remove_binding).pack(side=tk.LEFT, padx=4)
        ttk.Button(actions, text="Up", command=lambda: self.move_binding(-1)).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Button(actions, text="Down", command=lambda: self.move_binding(1)).pack(
            side=tk.LEFT, padx=4
        )

        columns = (
            "name",
            "packet_index",
            "joint_index",
            "kp",
            "kd",
            "scale",
            "offset",
            "min_q",
            "max_q",
            "average_window_size",
        )
        self.tree = ttk.Treeview(parent, columns=columns, show="headings", selectmode="browse")
        headings = {
            "name": "Name",
            "packet_index": "Packet",
            "joint_index": "Joint",
            "kp": "Kp",
            "kd": "Kd",
            "scale": "Scale",
            "offset": "Offset",
            "min_q": "Min q",
            "max_q": "Max q",
            "average_window_size": "Avg",
        }
        widths = {
            "name": 170,
            "packet_index": 70,
            "joint_index": 70,
            "kp": 70,
            "kd": 70,
            "scale": 80,
            "offset": 80,
            "min_q": 80,
            "max_q": 80,
            "average_window_size": 60,
        }
        for column in columns:
            self.tree.heading(column, text=headings[column])
            self.tree.column(column, width=widths[column], anchor="center")

        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=scrollbar.set)
        self.tree.grid(row=1, column=0, sticky="nsew")
        scrollbar.grid(row=1, column=1, sticky="ns")
        self.tree.bind("<<TreeviewSelect>>", self._on_tree_select)

    def _build_editor_panel(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(1, weight=1)

        ttk.Label(parent, text="Binding Editor", font=("", 12, "bold")).grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 12)
        )

        fields = [
            ("name", "Name"),
            ("packet_index", "Packet Index"),
            ("joint_index", "Joint Index"),
            ("kp", "Kp"),
            ("kd", "Kd"),
            ("scale", "Scale"),
            ("offset", "Offset"),
            ("min_q", "Min q"),
            ("max_q", "Max q"),
            ("average_window_size", "Average Window"),
        ]

        for row, (key, label) in enumerate(fields, start=1):
            ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=4, padx=(0, 10))
            ttk.Entry(parent, textvariable=self.editor_vars[key]).grid(
                row=row, column=1, sticky="ew", pady=4
            )

        button_row = ttk.Frame(parent)
        button_row.grid(row=len(fields) + 1, column=0, columnspan=2, sticky="ew", pady=(14, 0))
        ttk.Button(button_row, text="Apply Changes", command=self.apply_current_edits).pack(
            side=tk.LEFT
        )
        ttk.Button(button_row, text="Revert Row", command=self._reload_selected_into_editor).pack(
            side=tk.LEFT, padx=6
        )

        notes = (
            "Use negative scale to invert a potentiometer.\n"
            "Offset shifts the neutral point.\n"
            "Min q / Max q are optional clamp limits in radians.\n"
            "Save JSON for editing; export Python for the bridge."
        )
        ttk.Label(parent, text=notes, justify="left").grid(
            row=len(fields) + 2, column=0, columnspan=2, sticky="w", pady=(18, 0)
        )

    def _bind_dirty_tracking(self) -> None:
        for variable in (
            self.config_name_var,
            self.import_module_var,
            self.packet_value_count_var,
            *self.editor_vars.values(),
        ):
            variable.trace_add("write", self._mark_dirty)

    def _mark_dirty(self, *_args) -> None:
        if self._suspend_dirty_tracking:
            return
        self._is_dirty = True
        self._update_title()

    def _update_title(self) -> None:
        suffix = "*" if self._is_dirty else ""
        project = self.project_path.name if self.project_path else "untitled"
        self.root.title(f"{APP_TITLE} - {project}{suffix}")

    def _set_status(self, message: str) -> None:
        self.status_var.set(message)

    def _refresh_table(self) -> None:
        selected = self._selected_index
        self.tree.delete(*self.tree.get_children())
        for index, binding in enumerate(self.bindings):
            self.tree.insert(
                "",
                "end",
                iid=str(index),
                values=(
                    binding.name,
                    binding.packet_index,
                    binding.joint_index,
                    binding.kp,
                    binding.kd,
                    binding.scale,
                    binding.offset,
                    "" if binding.min_q is None else binding.min_q,
                    "" if binding.max_q is None else binding.max_q,
                    binding.average_window_size,
                ),
            )

        if self.bindings:
            self._select_index(0 if selected is None else min(selected, len(self.bindings) - 1))
        else:
            self._selected_index = None
            for variable in self.editor_vars.values():
                variable.set("")

    def _select_index(self, index: int) -> None:
        self._selected_index = index
        iid = str(index)
        self.tree.selection_set(iid)
        self.tree.focus(iid)
        self.tree.see(iid)
        self._reload_selected_into_editor()

    def _on_tree_select(self, _event=None) -> None:
        selected = self.tree.selection()
        if not selected:
            return
        self._selected_index = int(selected[0])
        self._reload_selected_into_editor()

    def _reload_selected_into_editor(self) -> None:
        if self._selected_index is None:
            return
        binding = self.bindings[self._selected_index]
        self._suspend_dirty_tracking = True
        try:
            self.editor_vars["name"].set(binding.name)
            self.editor_vars["packet_index"].set(str(binding.packet_index))
            self.editor_vars["joint_index"].set(str(binding.joint_index))
            self.editor_vars["kp"].set(str(binding.kp))
            self.editor_vars["kd"].set(str(binding.kd))
            self.editor_vars["scale"].set(str(binding.scale))
            self.editor_vars["offset"].set(str(binding.offset))
            self.editor_vars["min_q"].set("" if binding.min_q is None else str(binding.min_q))
            self.editor_vars["max_q"].set("" if binding.max_q is None else str(binding.max_q))
            self.editor_vars["average_window_size"].set(str(binding.average_window_size))
        finally:
            self._suspend_dirty_tracking = False

    def _parse_optional_float(self, value: str, field_name: str) -> float | None:
        if value.strip() == "":
            return None
        try:
            return float(value)
        except ValueError as exc:
            raise ValueError(f"{field_name} must be blank or a number") from exc

    def _binding_from_editor(self) -> JointBindingRecord:
        try:
            packet_index = int(self.editor_vars["packet_index"].get())
            joint_index = int(self.editor_vars["joint_index"].get())
            kp = float(self.editor_vars["kp"].get())
            kd = float(self.editor_vars["kd"].get())
            scale = float(self.editor_vars["scale"].get())
            offset = float(self.editor_vars["offset"].get())
            average_window_size = int(self.editor_vars["average_window_size"].get())
        except ValueError as exc:
            raise ValueError("Packet index, joint index, gains, scale, offset, and average window must be numeric") from exc

        name = self.editor_vars["name"].get().strip()
        if not name:
            raise ValueError("Name cannot be empty")
        if average_window_size < 1:
            raise ValueError("Average window must be >= 1")

        min_q = self._parse_optional_float(self.editor_vars["min_q"].get(), "Min q")
        max_q = self._parse_optional_float(self.editor_vars["max_q"].get(), "Max q")
        if min_q is not None and max_q is not None and min_q > max_q:
            raise ValueError("Min q must be <= Max q")

        return JointBindingRecord(
            name=name,
            packet_index=packet_index,
            joint_index=joint_index,
            kp=kp,
            kd=kd,
            scale=scale,
            offset=offset,
            min_q=min_q,
            max_q=max_q,
            average_window_size=average_window_size,
        )

    def apply_current_edits(self, *, show_success: bool = True) -> bool:
        if self._selected_index is None:
            return True
        try:
            binding = self._binding_from_editor()
        except ValueError as exc:
            messagebox.showerror(APP_TITLE, str(exc))
            return False

        self.bindings[self._selected_index] = binding
        self._refresh_table()
        self._select_index(self._selected_index)
        if show_success:
            self._set_status(f"Updated binding '{binding.name}'")
        return True

    def _confirm_discard(self) -> bool:
        if not self._is_dirty:
            return True
        return messagebox.askyesno(APP_TITLE, "Discard unsaved changes?")

    def new_project(self) -> None:
        if not self._confirm_discard():
            return
        self.project_path = None
        self.config_name_var.set("17-pot control")
        self.import_module_var.set(DEFAULT_IMPORT_MODULE)
        self.packet_value_count_var.set(str(DEFAULT_PACKET_VALUE_COUNT))
        self.bindings = default_bindings()
        self._is_dirty = False
        self._refresh_table()
        self._set_status("Started new project")
        self._update_title()

    def reset_17_template(self) -> None:
        self.bindings = default_bindings()
        self._refresh_table()
        self._set_status("Loaded 17-pot template rows")

    def add_binding(self) -> None:
        index = len(self.bindings)
        self.bindings.append(
            JointBindingRecord(
                name=f"control_{index:02d}",
                packet_index=index,
                joint_index=-1,
                kp=20.0,
                kd=1.0,
            )
        )
        self._refresh_table()
        self._select_index(len(self.bindings) - 1)
        self._set_status("Added binding")

    def duplicate_binding(self) -> None:
        if self._selected_index is None:
            return
        source = self.bindings[self._selected_index]
        copy = JointBindingRecord(**asdict(source))
        copy.name = f"{source.name}_copy"
        self.bindings.insert(self._selected_index + 1, copy)
        self._refresh_table()
        self._select_index(self._selected_index + 1)
        self._set_status(f"Duplicated '{source.name}'")

    def remove_binding(self) -> None:
        if self._selected_index is None or not self.bindings:
            return
        removed = self.bindings.pop(self._selected_index)
        self._refresh_table()
        self._set_status(f"Removed '{removed.name}'")

    def move_binding(self, direction: int) -> None:
        if self._selected_index is None:
            return
        new_index = self._selected_index + direction
        if new_index < 0 or new_index >= len(self.bindings):
            return
        self.bindings[self._selected_index], self.bindings[new_index] = (
            self.bindings[new_index],
            self.bindings[self._selected_index],
        )
        self._refresh_table()
        self._select_index(new_index)
        self._set_status("Reordered binding list")

    def _project_dict(self) -> dict:
        return {
            "version": 1,
            "config_name": self.config_name_var.get().strip(),
            "import_module": self.import_module_var.get().strip(),
            "packet_value_count": int(self.packet_value_count_var.get()),
            "bindings": [asdict(binding) for binding in self.bindings],
        }

    def save_project(self) -> None:
        if not self.apply_current_edits(show_success=False):
            return
        try:
            payload = self._project_dict()
        except ValueError:
            messagebox.showerror(APP_TITLE, "Packet value count must be an integer")
            return

        path = self.project_path
        if path is None:
            chosen = filedialog.asksaveasfilename(
                title="Save Joint Config Project",
                defaultextension=DEFAULT_PROJECT_SUFFIX,
                filetypes=[("G1 Config JSON", f"*{DEFAULT_PROJECT_SUFFIX}"), ("JSON", "*.json")],
                initialdir=str(Path(__file__).resolve().parent),
            )
            if not chosen:
                return
            path = Path(chosen)

        path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        self.project_path = path
        self._is_dirty = False
        self._update_title()
        self._set_status(f"Saved project to {path.name}")

    def open_project(self) -> None:
        if not self._confirm_discard():
            return
        chosen = filedialog.askopenfilename(
            title="Open Joint Config Project",
            filetypes=[("G1 Config JSON", f"*{DEFAULT_PROJECT_SUFFIX}"), ("JSON", "*.json")],
            initialdir=str(Path(__file__).resolve().parent),
        )
        if not chosen:
            return

        path = Path(chosen)
        payload = json.loads(path.read_text(encoding="utf-8"))
        bindings = [JointBindingRecord(**item) for item in payload.get("bindings", [])]
        if not bindings:
            bindings = default_bindings()

        self.project_path = path
        self.config_name_var.set(payload.get("config_name", "17-pot control"))
        self.import_module_var.set(payload.get("import_module", DEFAULT_IMPORT_MODULE))
        self.packet_value_count_var.set(str(payload.get("packet_value_count", DEFAULT_PACKET_VALUE_COUNT)))
        self.bindings = bindings
        self._is_dirty = False
        self._refresh_table()
        self._set_status(f"Opened project {path.name}")
        self._update_title()

    def open_virtual_pot_sim(self) -> None:
        if not self.apply_current_edits(show_success=False):
            return
        if self.project_path is None:
            should_save = messagebox.askyesno(
                APP_TITLE,
                "The virtual pot simulator uses the JSON project file.\nSave this project now?",
            )
            if not should_save:
                return
            self.save_project()
            if self.project_path is None:
                return
        else:
            self.save_project()

        sim_script = Path(__file__).resolve().parent / SIMULATOR_SCRIPT_NAME
        cmd = [sys.executable, str(sim_script), str(self.project_path)]
        try:
            subprocess.Popen(cmd, cwd=str(sim_script.parent))
        except OSError as exc:
            messagebox.showerror(APP_TITLE, f"Failed to launch virtual pot simulator:\n{exc}")
            return
        self._set_status("Opened virtual pot simulator")

    def _validate_for_export(self) -> None:
        try:
            packet_value_count = int(self.packet_value_count_var.get())
        except ValueError as exc:
            raise ValueError("Packet value count must be an integer") from exc
        if packet_value_count < 1:
            raise ValueError("Packet value count must be >= 1")

        config_name = self.config_name_var.get().strip()
        if not config_name:
            raise ValueError("Config name cannot be empty")

        import_module = self.import_module_var.get().strip()
        if not import_module:
            raise ValueError("Import module cannot be empty")

        if not self.bindings:
            raise ValueError("At least one binding is required")

        for binding in self.bindings:
            if binding.packet_index < 0 or binding.packet_index >= packet_value_count:
                raise ValueError(
                    f"{binding.name}: packet_index {binding.packet_index} is outside 0..{packet_value_count - 1}"
                )
            if binding.joint_index < 0:
                raise ValueError(
                    f"{binding.name}: joint_index must be replaced with a real motor index before export"
                )
            if binding.min_q is not None and binding.max_q is not None and binding.min_q > binding.max_q:
                raise ValueError(f"{binding.name}: min_q must be <= max_q")

    def _python_literal(self, value) -> str:
        if value is None:
            return "None"
        if isinstance(value, str):
            return repr(value)
        return repr(value)

    def _render_python_config(self) -> str:
        import_module = self.import_module_var.get().strip()
        config_name = self.config_name_var.get().strip()
        packet_value_count = int(self.packet_value_count_var.get())

        lines = [
            f"from {import_module} import BridgeConfig, JointBinding",
            "",
            "",
            "BRIDGE_CONFIG = BridgeConfig(",
            f"    name={self._python_literal(config_name)},",
            f"    packet_value_count={packet_value_count},",
            "    joint_bindings=(",
        ]

        for binding in self.bindings:
            lines.extend(
                [
                    "        JointBinding(",
                    f"            name={self._python_literal(binding.name)},",
                    f"            packet_index={binding.packet_index},",
                    f"            joint_index={binding.joint_index},",
                    f"            kp={binding.kp},",
                    f"            kd={binding.kd},",
                    f"            scale={binding.scale},",
                    f"            offset={binding.offset},",
                    f"            min_q={self._python_literal(binding.min_q)},",
                    f"            max_q={self._python_literal(binding.max_q)},",
                    f"            average_window_size={binding.average_window_size},",
                    "        ),",
                ]
            )

        lines.extend(
            [
                "    ),",
                ")",
                "",
            ]
        )
        return "\n".join(lines)

    def export_python(self) -> None:
        if not self.apply_current_edits(show_success=False):
            return
        try:
            self._validate_for_export()
        except ValueError as exc:
            messagebox.showerror(APP_TITLE, str(exc))
            return

        initial_name = "g1_generated_bridge_config.py"
        if self.project_path is not None:
            stem = self.project_path.name
            if stem.endswith(DEFAULT_PROJECT_SUFFIX):
                stem = stem[: -len(DEFAULT_PROJECT_SUFFIX)]
            else:
                stem = self.project_path.stem
            initial_name = f"{stem}_export.py"

        chosen = filedialog.asksaveasfilename(
            title="Export Bridge Python Config",
            defaultextension=PYTHON_EXPORT_SUFFIX,
            filetypes=[("Python", "*.py")],
            initialdir=str(Path(__file__).resolve().parent),
            initialfile=initial_name,
        )
        if not chosen:
            return

        path = Path(chosen)
        path.write_text(self._render_python_config(), encoding="utf-8")
        self._set_status(f"Exported Python config to {path.name}")
        messagebox.showinfo(APP_TITLE, f"Exported bridge config:\n{path}")


def main() -> int:
    root = tk.Tk()
    app = ConfigEditorApp(root)
    app._update_title()
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
