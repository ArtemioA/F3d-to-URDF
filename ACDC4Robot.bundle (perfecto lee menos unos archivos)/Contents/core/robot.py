import adsk.core
import adsk.fusion
import os
import traceback


def _get_app_ui():
    app = adsk.core.Application.get()
    ui = app.userInterface if app else None
    return app, ui


def _sanitize(name: str) -> str:
    """Limpia nombres para URDF y archivos."""
    if not name:
        return "link"
    bad = '<>:"/\\|?* '
    for c in bad:
        name = name.replace(c, '_')
    return name


class RobotExporter:
    def __init__(self, robot_name: str, base_output_dir: str = None):
        self.app, self.ui = _get_app_ui()
        if not self.app:
            raise RuntimeError("No se pudo obtener la aplicación de Fusion 360.")

        product = self.app.activeProduct
        self.design = adsk.fusion.Design.cast(product)
        if not self.design:
            raise RuntimeError("El producto activo no es un diseño de Fusion 360.")

        self.robot_name = _sanitize(robot_name) or "acdc_robot"

        # Por defecto Desktop (igual que tu script original)
        if base_output_dir is None:
            home = os.path.expanduser("~")
            base_output_dir = os.path.join(home, "Desktop")

        # Carpeta final: <base>/<robot_name>_urdf/
        self.output_dir = os.path.join(base_output_dir, f"{self.robot_name}_urdf")
        self.meshes_dir = os.path.join(self.output_dir, "meshes")
        os.makedirs(self.meshes_dir, exist_ok=True)

        # Cada link ahora guarda también su "target" de export (occurrence o body)
        # links: {name, mesh, origin, target}
        self.links = []
        self.joints = []

    # =========================================================
    # API principal
    # =========================================================
    def export_all(self):
        try:
            self._log(f"[ACDC4Robot] Exportando robot '{self.robot_name}'...")
            self._build_links_and_joints()
            self._export_meshes()
            self._write_urdf()
            self._log(f"[ACDC4Robot] Export completado en:\n{self.output_dir}")
        except:
            err = traceback.format_exc()
            self._log("[ACDC4Robot] Error en RobotExporter.export_all():\n" + err)
            if self.ui:
                self.ui.messageBox(
                    "Error en RobotExporter.export_all():\n\n" + err
                )
            raise

    # =========================================================
    # Construcción de links/joints
    # =========================================================
    def _build_links_and_joints(self):
        root = self.design.rootComponent

        base_link_name = "base_link"
        # base_link virtual sin malla
        self.links = [{
            "name": base_link_name,
            "mesh": None,
            "origin": (0, 0, 0),
            "target": None,
        }]
        self.joints = []

        idx = 0

        # ---------- 1) Cuerpos sueltos en el root ("Cuerpos") ----------
        for body in root.bRepBodies:
            try:
                if not body or not body.isSolid:
                    continue
            except:
                continue

            link_name = _sanitize(f"root_{idx}_{body.name}")
            mesh_name = f"{link_name}.stl"

            self.links.append({
                "name": link_name,
                "mesh": mesh_name,
                "origin": (0, 0, 0),
                "target": body,          # clave: exportamos este body directo
            })

            self.joints.append({
                "name": f"joint_{link_name}",
                "parent": base_link_name,
                "child": link_name,
                "type": "fixed",
                "origin": (0, 0, 0),
                "axis": (0, 0, 1),
            })

            idx += 1

        # ---------- 2) Todas las occurrences con cuerpos sólidos ----------
        all_occs = root.allOccurrences
        for occ in all_occs:
            comp = occ.component
            if not comp:
                continue

            has_body = any(b.isSolid for b in comp.bRepBodies if b)
            if not has_body:
                continue

            link_name = _sanitize(f"link_{idx}_{occ.name}")
            mesh_name = f"{link_name}.stl"

            self.links.append({
                "name": link_name,
                "mesh": mesh_name,
                "origin": (0, 0, 0),
                "target": occ,           # aquí usamos la occurrence completa
            })

            self.joints.append({
                "name": f"joint_{link_name}",
                "parent": base_link_name,
                "child": link_name,
                "type": "fixed",
                "origin": (0, 0, 0),
                "axis": (0, 0, 1),
            })

            idx += 1

        # Si sólo quedó base_link, le damos una malla del root como fallback
        if len(self.links) == 1:
            self.links[0]["mesh"] = f"{base_link_name}.stl"
            self.links[0]["target"] = root

    # =========================================================
    # Export de mallas STL
    # =========================================================
    def _export_meshes(self):
        export_mgr = self.design.exportManager

        for link in self.links:
            mesh_name = link.get("mesh")
            target = link.get("target")

            if not mesh_name or target is None:
                continue  # base_link u otros sin malla

            stl_path = os.path.join(self.meshes_dir, mesh_name)

            try:
                stl_opts = export_mgr.createSTLExportOptions(target, stl_path)

                # Refinamiento alto (soporta APIs nuevas y viejas)
                try:
                    stl_opts.meshRefinement = adsk.fusion.MeshRefinementOptions.High
                except:
                    try:
                        stl_opts.meshRefinement = (
                            adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
                        )
                    except:
                        pass

                if hasattr(stl_opts, "isBinaryFormat"):
                    stl_opts.isBinaryFormat = True

                ok = export_mgr.execute(stl_opts)
                if ok:
                    self._log(f"[ACDC4Robot] STL generado: {stl_path}")
                else:
                    self._log(
                        f"[ACDC4Robot] ERROR ejecutando export STL para {link['name']}"
                    )
            except:
                self._log(
                    f"[ACDC4Robot] Error exportando STL para link {link['name']}"
                )
                self._log(traceback.format_exc())

    # =========================================================
    # Generación del archivo URDF (estructura sencilla)
    # =========================================================
    def _write_urdf(self):
        urdf_path = os.path.join(self.output_dir, f"{self.robot_name}.urdf")

        lines = []
        lines.append(f'<robot name="{self.robot_name}">')

        # LINKS
        for link in self.links:
            name = link["name"]
            mesh = link.get("mesh")
            lines.append(f'  <link name="{name}">')
            if mesh:
                lines.append('    <visual>')
                lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
                lines.append(
                    f'      <geometry><mesh filename="meshes/{mesh}" '
                    f'scale="0.001 0.001 0.001"/></geometry>'
                )
                lines.append('    </visual>')
                lines.append('    <collision>')
                lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
                lines.append(
                    f'      <geometry><mesh filename="meshes/{mesh}" '
                    f'scale="0.001 0.001 0.001"/></geometry>'
                )
                lines.append('    </collision>')
            lines.append('    <inertial>')
            lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
            lines.append('      <mass value="1.0"/>')
            lines.append(
                '      <inertia ixx="1" ixy="0" ixz="0" '
                'iyy="1" iyz="0" izz="1"/>'
            )
            lines.append('    </inertial>')
            lines.append('  </link>')

        # JOINTS
        for j in self.joints:
            lines.append(f'  <joint name="{j["name"]}" type="{j["type"]}">')
            ox, oy, oz = j["origin"]
            ax, ay, az = j["axis"]
            lines.append(f'    <origin xyz="{ox} {oy} {oz}" rpy="0 0 0"/>')
            lines.append(f'    <parent link="{j["parent"]}"/>')
            lines.append(f'    <child link="{j["child"]}"/>')
            lines.append(f'    <axis xyz="{ax} {ay} {az}"/>')
            lines.append('  </joint>')

        lines.append('</robot>')

        with open(urdf_path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))

        self._log(f"[ACDC4Robot] URDF generado: {urdf_path}")

    # =========================================================
    # Logs
    # =========================================================
    def _log(self, msg: str):
        try:
            print(msg)
        except:
            pass
        # Si quieres popups, descomenta:
        # if self.ui:
        #     self.ui.messageBox(msg, "ACDC4Robot",
        #                        adsk.core.MessageBoxButtonTypes.OKButtonType)


def export_robot(robot_name: str, base_output_dir: str = None, *args, **kwargs):
    """
    Función helper llamada desde acdc4robot.py.
    Compatible con export_robot(name, base_dir, ...).
    """
    if robot_name is None and len(args) > 0:
        robot_name = args[0]
    if base_output_dir is None and len(args) > 1:
        base_output_dir = args[1]

    exporter = RobotExporter(robot_name, base_output_dir)
    exporter.export_all()

