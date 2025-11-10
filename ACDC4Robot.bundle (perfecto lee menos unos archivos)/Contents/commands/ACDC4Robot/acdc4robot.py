import adsk.core
import adsk.fusion
import traceback
import os
import sys

# === FIX PATH ===
# Asegura que "core" y "commands" (en Contents) sean importables como paquetes top-level
this_dir = os.path.dirname(os.path.abspath(__file__))
contents_dir = os.path.abspath(os.path.join(this_dir, "..", ".."))
if contents_dir not in sys.path:
    sys.path.insert(0, contents_dir)

from core import robot  # usa core/robot.py


_app = adsk.core.Application.get()
_ui = _app.userInterface if _app else None


def build_ui(cmd: adsk.core.Command):
    """Define los inputs del comando."""
    try:
        inputs = cmd.commandInputs

        inputs.addStringValueInput(
            "robotName",
            "Robot name",
            "acdc_robot"
        )

        inputs.addStringValueInput(
            "outputFolder",
            "URDF base output folder",
            "C:/ACDC4Robot_URDF"
        )

        inputs.addBoolValueInput(
            "useAssemblyJoints",
            "Use existing joints (not used in this version)",
            True,
            "",
            True
        )

    except:
        msg = "ACDC4Robot.build_ui() failed:\n{}".format(traceback.format_exc())
        _safe_msg(msg)


def on_input_changed(args: adsk.core.InputChangedEventArgs):
    """Solo debug opcional."""
    try:
        changed = args.input
        if _ui:
            _ui.messageBox(
                f"[DEBUG][acdc4robot.on_input_changed] Input cambiado: {changed.id}"
            )
    except:
        msg = "ACDC4Robot.on_input_changed() failed:\n{}".format(traceback.format_exc())
        _safe_msg(msg)


def execute(args: adsk.core.CommandEventArgs):
    """
    Lógica principal:
    - Lee parámetros del panel.
    - Verifica diseño activo.
    - Llama a robot.export_robot().
    """
    try:
        cmd = args.command
        inputs = cmd.commandInputs

        robot_name = _get_str(inputs, "robotName", "acdc_robot")
        output_folder = _get_str(inputs, "outputFolder", "C:/ACDC4Robot_URDF")
        use_joints = _get_bool(inputs, "useAssemblyJoints", True)  # hoy no se usa

        design = adsk.fusion.Design.cast(_app.activeProduct)
        if not design:
            raise RuntimeError("No active Fusion 360 design found.")

        root_comp = design.rootComponent
        if not root_comp:
            raise RuntimeError("No root component found.")

        # Crear carpeta base si no existe
        if output_folder and not os.path.exists(output_folder):
            os.makedirs(output_folder, exist_ok=True)

        # Permitir dejar vacío -> usa Desktop dentro de robot.export_robot
        base_dir = output_folder if output_folder.strip() else None

        _log(
            f"[ACDC4Robot] execute() -> export_robot("
            f"name='{robot_name}', base_output_dir='{base_dir or 'Desktop'}', "
            f"use_joints={use_joints})"
        )

        robot.export_robot(
            robot_name=robot_name,
            base_output_dir=base_dir
        )

        if _ui:
            _ui.messageBox(
                "[ACDC4Robot] Exportación URDF + STL completada.\n"
                "Revisa la carpeta configurada.",
                "ACDC4Robot"
            )

    except:
        msg = "ACDC4Robot.execute() failed:\n{}".format(traceback.format_exc())
        _safe_msg(msg)


# ================= HELPERS =================

def _get_str(inputs: adsk.core.CommandInputs, input_id: str, default: str) -> str:
    try:
        inp = inputs.itemById(input_id)
        if not inp:
            return default
        val = getattr(inp, "value", None)
        if isinstance(val, str) and val.strip():
            return val.strip()
        text = getattr(inp, "text", None)
        if isinstance(text, str) and text.strip():
            return text.strip()
        return default
    except:
        _safe_msg("ACDC4Robot._get_str() failed:\n{}".format(traceback.format_exc()))
        return default


def _get_bool(inputs: adsk.core.CommandInputs, input_id: str, default: bool) -> bool:
    try:
        inp = inputs.itemById(input_id)
        if not inp:
            return default
        val = getattr(inp, "value", None)
        if isinstance(val, bool):
            return val
        return default
    except:
        _safe_msg("ACDC4Robot._get_bool() failed:\n{}".format(traceback.format_exc()))
        return default


def _log(msg: str):
    try:
        print(msg)
    except:
        pass
    if _app:
        try:
            _app.log(msg, adsk.core.LogLevels.InfoLogLevel,
                     adsk.core.LogTypes.ConsoleLogType)
        except:
            pass


def _safe_msg(msg: str):
    try:
        print(msg)
    except:
        pass
    if _app:
        try:
            _app.log(msg, adsk.core.LogLevels.ErrorLogLevel,
                     adsk.core.LogTypes.ConsoleLogType)
        except:
            pass
    if _ui:
        try:
            _ui.messageBox(msg, "ACDC4Robot")
        except:
            pass
