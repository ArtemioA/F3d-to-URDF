import adsk.core
import traceback
import os
import sys

# === AÑADIR PATH MANUAL PARA QUE ENCUENTRE "core" ===
# Esto permite importar core/futil.py aunque Fusion cambie el working dir
this_dir = os.path.dirname(os.path.abspath(__file__))
contents_dir = os.path.abspath(os.path.join(this_dir, "..", ".."))
if contents_dir not in sys.path:
    sys.path.insert(0, contents_dir)

from core import futil
from commands.ACDC4Robot import acdc4robot

# =======================================================
# Identificadores del comando
# =======================================================
CMD_ID = 'ACDC4Robot_URDF_Exporter'
CMD_NAME = 'Export to URDF (ACDC4Robot)'
CMD_DESCRIPTION = 'Export the current Fusion 360 model/assembly to URDF format.'

# Workspace y panel donde aparecerá el botón
WORKSPACE_ID = 'FusionSolidEnvironment'
PANEL_ID = 'SolidScriptsAddinsPanel'   # Panel de "Complementos / Add-ins"
CMD_RESOURCES = ''  # ícono opcional

_app = adsk.core.Application.get()
_ui = _app.userInterface if _app else None

# Mantiene vivos los handlers
local_handlers = []


def start():
    """Registra el comando y lo agrega al panel de Complementos."""
    try:
        if not _ui:
            return

        cmd_def = _ui.commandDefinitions.itemById(CMD_ID)
        if not cmd_def:
            cmd_def = _ui.commandDefinitions.addButtonDefinition(
                CMD_ID, CMD_NAME, CMD_DESCRIPTION, CMD_RESOURCES
            )

        # Registrar creación
        futil.add_handler(cmd_def.commandCreated, command_created, local_handlers)

        # Insertar en el panel de complementos
        workspace = _ui.workspaces.itemById(WORKSPACE_ID)
        if not workspace:
            raise RuntimeError(f'Workspace "{WORKSPACE_ID}" not found.')

        panel = workspace.toolbarPanels.itemById(PANEL_ID)
        if not panel:
            raise RuntimeError(f'Panel "{PANEL_ID}" not found.')

        control = panel.controls.itemById(CMD_ID)
        if not control:
            control = panel.controls.addCommand(cmd_def)
            control.isPromoted = True
            control.isPromotedByDefault = True

        # Mensaje opcional de debug
        # _ui.messageBox("[DEBUG][entry.start] Comando agregado al panel Complementos.")

    except:
        msg = 'ACDC4Robot.start() failed:\n{}'.format(traceback.format_exc())
        if _ui:
            _ui.messageBox(msg)
        else:
            print(msg)


def stop():
    """Elimina el comando y limpia referencias."""
    try:
        if not _ui:
            return

        workspace = _ui.workspaces.itemById(WORKSPACE_ID)
        if workspace:
            panel = workspace.toolbarPanels.itemById(PANEL_ID)
            if panel:
                control = panel.controls.itemById(CMD_ID)
                if control:
                    control.deleteMe()

        cmd_def = _ui.commandDefinitions.itemById(CMD_ID)
        if cmd_def:
            cmd_def.deleteMe()

        # _ui.messageBox("[DEBUG][entry.stop] Comando eliminado correctamente.")

    except:
        msg = 'ACDC4Robot.stop() failed:\n{}'.format(traceback.format_exc())
        if _ui:
            _ui.messageBox(msg)
        else:
            print(msg)


def command_created(args: adsk.core.CommandCreatedEventArgs):
    """Se llama cuando el usuario hace click en el botón."""
    try:
        cmd = args.command

        if _ui:
            _ui.messageBox("[DEBUG][entry.command_created] Inicializando comando...")

        # Registrar handlers
        futil.add_handler(cmd.execute, command_execute, local_handlers)
        futil.add_handler(cmd.executePreview, command_execute_preview, local_handlers)
        futil.add_handler(cmd.inputChanged, command_input_changed, local_handlers)
        futil.add_handler(cmd.destroy, command_destroy, local_handlers)

        # Construir la UI del comando
        acdc4robot.build_ui(cmd)

        if _ui:
            _ui.messageBox("[DEBUG][entry.command_created] UI creada y handlers activos.")

    except:
        msg = 'ACDC4Robot.command_created() failed:\n{}'.format(traceback.format_exc())
        if _ui:
            _ui.messageBox(msg)
        else:
            print(msg)


def command_execute(args: adsk.core.CommandEventArgs):
    """Ejecuta la lógica principal (exportación URDF)."""
    try:
        if _ui:
            _ui.messageBox("[DEBUG][entry.command_execute] Ejecutando exportación URDF...")
        acdc4robot.execute(args)
    except:
        msg = 'ACDC4Robot.command_execute() failed:\n{}'.format(traceback.format_exc())
        if _ui:
            _ui.messageBox(msg)
        else:
            print(msg)


def command_execute_preview(args: adsk.core.CommandEventArgs):
    """Preview opcional."""
    try:
        # _ui.messageBox("[DEBUG][entry.command_execute_preview] Preview activado (sin acción).")
        pass
    except:
        msg = 'ACDC4Robot.command_execute_preview() failed:\n{}'.format(traceback.format_exc())
        if _ui:
            _ui.messageBox(msg)
        else:
            print(msg)


def command_input_changed(args: adsk.core.InputChangedEventArgs):
    """Propaga cambios a acdc4robot."""
    try:
        acdc4robot.on_input_changed(args)
    except:
        msg = 'ACDC4Robot.command_input_changed() failed:\n{}'.format(traceback.format_exc())
        if _ui:
            _ui.messageBox(msg)
        else:
            print(msg)


def command_destroy(args: adsk.core.CommandEventArgs):
    """Limpia handlers al cerrar el comando."""
    try:
        global local_handlers
        local_handlers = []
        if _ui:
            _ui.messageBox("[DEBUG][entry.command_destroy] Handlers limpiados.")
    except:
        msg = 'ACDC4Robot.command_destroy() failed:\n{}'.format(traceback.format_exc())
        if _ui:
            _ui.messageBox(msg)
        else:
            print(msg)
