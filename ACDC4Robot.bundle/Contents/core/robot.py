import adsk.core
import adsk.fusion
import os
import math
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


def _matrix_to_xyz_rpy(m: adsk.core.Matrix3D, design) -> tuple:
    """Convierte Matrix3D de Fusion a (xyz, rpy) en metros."""
    p = m.translation
    try:
        scale = design.unitsManager.convert(1.0, "cm", "m")
    except Exception:
        scale = 0.01

    x, y, z = p.x * scale, p.y * scale, p.z * scale

    r11, r12, r13 = m.getCell(0, 0), m.getCell(0, 1), m.getCell(0, 2)
    r21, r22, r23 = m.getCell(1, 0), m.getCell(1, 1), m.getCell(1, 2)
    r31, r32, r33 = m.getCell(2, 0), m.getCell(2, 1), m.getCell(2, 2)

    # roll (X), pitch (Y), yaw (Z)
    if abs(r31) < 1.0:
        pitch = math.asin(-r31)
        roll = math.atan2(r32, r33)
        yaw = math.atan2(r21, r11)
    else:
        pitch = math.pi / 2 if r31 <= -1.0 else -math.pi / 2
        roll = 0.0
        yaw = math.atan2(-r12, r22)

    return (x, y, z), (roll, pitch, yaw)


class RobotExporter:
    """
    Exporta todos los sólidos como STL y genera un URDF con joints funcionales:

    - Un link por occurrence con sólidos.
    - Joints desde Fusion (joints + asBuiltJoints).
    - Revolute/Slider con <limit>; sin límites -> continuous o rango amplio.
    - Links sin joint -> fixed al base_link.
    """

    def __init__(self, robot_name: str, base_output_dir: str = None):
        self.app, self.ui = _get_app_ui()
        if not self.app:
            raise RuntimeError("No se pudo obtener la aplicación de Fusion 360.")

        product = self.app.activeProduct
        self.design = adsk.fusion.Design.cast(product)
        if not self.design:
            raise RuntimeError("El producto activo no es un diseño de Fusion 360.")

        self.robot_name = _sanitize(robot_name) or "acdc_robot"

        if base_output_dir is None:
            home = os.path.expanduser("~")
            base_output_dir = os.path.join(home, "Desktop")

        self.output_dir = os.path.join(base_output_dir, f"{self.robot_name}_urdf")
        self.meshes_dir = os.path.join(self.output_dir, "meshes")
        os.makedirs(self.meshes_dir, exist_ok=True)

        self.links = []          # [{name, mesh, occ}]
        self.joints = []         # [{name,parent,child,type,origin_xyz,origin_rpy,axis,limit}]
        self.occ_to_link = {}    # occ_key(str) -> link_name
        self.base_link_name = None

    # ========================= API =========================
    def export_all(self):
        try:
            self._log(f"[ACDC4Robot] Exportando robot '{self.robot_name}'...")
            self._build_links_and_joints()
            self._export_meshes()
            self._write_urdf()
            self._log(f"[ACDC4Robot] Export completado en:\n{self.output_dir}")
        except Exception:
            if self.ui:
                self.ui.messageBox("Error en RobotExporter.export_all():\n\n" + traceback.format_exc())
            raise

    # ==================== Helpers Occ keys =================
    def _occ_key(self, occ) -> str:
        """Clave hashable estable para una occurrence."""
        if occ is None:
            return ""
        try:
            token = getattr(occ, "entityToken", None)
            if token:
                return token
        except:
            pass
        try:
            fp = getattr(occ, "fullPathName", None)
            if fp:
                return fp
        except:
            pass
        try:
            return f"{occ.name}_{id(occ)}"
        except:
            return str(id(occ))

    # ============ Build links & joints from Fusion =========
    def _build_links_and_joints(self):
        root = self.design.rootComponent

        # 1) Links por occurrence con sólidos
        idx = 0
        for occ in root.allOccurrences:
            comp = occ.component
            has_bodies = False
            try:
                if comp and any(b.isSolid for b in comp.bRepBodies):
                    has_bodies = True
            except:
                pass
            if not has_bodies:
                continue

            link_name = _sanitize(f"link_{idx}_{occ.name}")
            mesh_name = f"{link_name}.stl"
            idx += 1

            self.links.append({
                "name": link_name,
                "mesh": mesh_name,
                "occ": occ,
            })

            key = self._occ_key(occ)
            if key:
                self.occ_to_link[key] = link_name

        if not self.links:
            raise RuntimeError("[ACDC4Robot] No se encontraron occurrences con sólidos.")

        # 2) base_link (grounded si existe)
        self.base_link_name = self._choose_base_link()
        self._log(f"[ACDC4Robot] base_link seleccionado: {self.base_link_name}")

        # 3) Joints desde Fusion
        self._create_joints_from_fusion()

        # 4) Links sueltos -> fixed al base_link
        attached_children = {j["child"] for j in self.joints}
        for link in self.links:
            name = link["name"]
            if name == self.base_link_name:
                continue
            if name not in attached_children:
                occ = link.get("occ")
                try:
                    if occ and hasattr(occ, "transform2"):
                        xyz, rpy = _matrix_to_xyz_rpy(occ.transform2, self.design)
                    else:
                        xyz, rpy = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
                except:
                    xyz, rpy = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

                j_name = f"auto_fixed_{name}"
                self.joints.append({
                    "name": j_name,
                    "parent": self.base_link_name,
                    "child": name,
                    "type": "fixed",
                    "origin_xyz": xyz,
                    "origin_rpy": rpy,
                    "axis": (0.0, 0.0, 1.0),
                    "limit": None,
                })
                self._log(f"[ACDC4Robot] Joint fijo auto: {self.base_link_name} -> {name}")

    def _choose_base_link(self) -> str:
        # grounded primero
        for link in self.links:
            occ = link.get("occ")
            try:
                if hasattr(occ, "isGrounded") and occ.isGrounded:
                    return link["name"]
            except:
                pass
        return self.links[0]["name"]

    def _create_joints_from_fusion(self):
        root = self.design.rootComponent
        all_joints = []

        try:
            for j in root.joints:
                all_joints.append(j)
        except:
            pass
        try:
            for j in root.asBuiltJoints:
                all_joints.append(j)
        except:
            pass

        if not all_joints:
            self._log("[ACDC4Robot] No Fusion joints encontrados.")
            return

        try:
            from adsk.fusion import JointTypes
        except Exception:
            JointTypes = None

        used_children = set()

        for j in all_joints:
            try:
                occ1 = getattr(j, "occurrenceOne", None)
                occ2 = getattr(j, "occurrenceTwo", None)

                k1 = self._occ_key(occ1)
                k2 = self._occ_key(occ2)

                l1 = self.occ_to_link.get(k1)
                l2 = self.occ_to_link.get(k2)
                if not l1 or not l2:
                    continue

                parent, child = self._pick_parent_child(l1, l2, used_children)
                used_children.add(child)

                jtype, axis, limit = self._map_joint_type_axis_limit(j, JointTypes)
                ox, oy, oz, rr, pp, yy = self._get_joint_origin_from_fusion(j)

                j_name = _sanitize(j.name) if j.name else f"joint_{parent}_to_{child}"

                self.joints.append({
                    "name": j_name,
                    "parent": parent,
                    "child": child,
                    "type": jtype,
                    "origin_xyz": (ox, oy, oz),
                    "origin_rpy": (rr, pp, yy),
                    "axis": axis,
                    "limit": limit,   # None or (lower, upper)
                })

                self._log(f"[ACDC4Robot] Joint Fusion -> URDF: {j_name} ({jtype}) {parent}->{child} limit={limit}")
            except Exception:
                self._log("[ACDC4Robot] Error procesando un joint de Fusion:")
                self._log(traceback.format_exc())

    def _pick_parent_child(self, l1: str, l2: str, used_children: set):
        if l1 == self.base_link_name:
            return l1, l2
        if l2 == self.base_link_name:
            return l2, l1
        if l1 in used_children and l2 not in used_children:
            return l2, l1
        if l2 in used_children and l1 not in used_children:
            return l1, l2
        return (l1, l2) if l1 < l2 else (l2, l1)

    def _map_joint_type_axis_limit(self, j, JointTypes):
        """
        Devuelve (jtype, axis, limit):
        - jtype: 'fixed', 'revolute', 'continuous', 'prismatic'
        - axis: (x,y,z)
        - limit: None o (lower, upper) en rad o m
        """
        jtype = "fixed"
        axis = (0.0, 0.0, 1.0)
        limit = None

        motion = None
        try:
            motion = j.jointMotion
        except:
            pass

        jt = None
        try:
            if motion:
                jt = motion.jointType
        except:
            jt = None

        # Tipo base
        try:
            if JointTypes and jt is not None:
                if jt == JointTypes.RigidJointType:
                    jtype = "fixed"
                elif jt == JointTypes.RevoluteJointType:
                    jtype = "revolute"
                elif jt == JointTypes.SliderJointType:
                    jtype = "prismatic"
                else:
                    jtype = "fixed"
        except:
            jtype = "fixed"

        # Eje
        try:
            if motion and hasattr(motion, "rotationAxisVector"):
                av = motion.rotationAxisVector
                axis = (av.x, av.y, av.z)
            else:
                geo = getattr(j, "geometry", None)
                if geo and hasattr(geo, "primaryAxisVector"):
                    av = geo.primaryAxisVector
                    axis = (av.x, av.y, av.z)
        except:
            pass

        # Normalizar eje si tiene sentido
        try:
            length = math.sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)
            if length > 1e-6:
                axis = (axis[0]/length, axis[1]/length, axis[2]/length)
            else:
                axis = (0.0, 0.0, 1.0)
        except:
            axis = (0.0, 0.0, 1.0)

        # Límites desde Fusion si existen
        if jtype == "revolute":
            try:
                rl = getattr(motion, "rotationLimits", None)
                if rl:
                    lo = None
                    hi = None
                    if getattr(rl, "isMinimumValueEnabled", False):
                        lo = rl.minimumValue
                    if getattr(rl, "isMaximumValueEnabled", False):
                        hi = rl.maximumValue
                    if lo is not None and hi is not None and hi > lo:
                        limit = (lo, hi)
            except:
                pass

            # Si no hay límites -> continuous (ideal p/ ruedas)
            if limit is None:
                jtype = "continuous"
                # continuous en URDF no lleva lower/upper, solo effort/velocity

        elif jtype == "prismatic":
            try:
                sl = getattr(motion, "slideLimits", None)
                if sl:
                    lo = None
                    hi = None
                    if getattr(sl, "isMinimumValueEnabled", False):
                        lo = sl.minimumValue
                    if getattr(sl, "isMaximumValueEnabled", False):
                        hi = sl.maximumValue
                    if lo is not None and hi is not None and hi > lo:
                        limit = (lo, hi)
            except:
                pass

            # Si no hay límites, damos algo pequeño para que el viewer pueda mover
            if limit is None:
                limit = (-0.1, 0.1)

        return jtype, axis, limit

    def _get_joint_origin_from_fusion(self, j):
        """Origen (xyz,rpy) del joint. Si falla, 0."""
        try:
            geo = getattr(j, "geometryOrOriginTwo", None)
            if geo and hasattr(geo, "transform"):
                m = geo.transform
                (ox, oy, oz), (rr, pp, yy) = _matrix_to_xyz_rpy(m, self.design)
                return ox, oy, oz, rr, pp, yy
        except:
            pass
        return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

    # ===================== STL export ======================
    def _export_meshes(self):
        export_mgr = self.design.exportManager
        root = self.design.rootComponent

        for link in self.links:
            mesh_name = link.get("mesh")
            if not mesh_name:
                continue

            stl_path = os.path.join(self.meshes_dir, mesh_name)
            occ = link.get("occ")
            export_target = occ if occ else root

            try:
                stl_opts = export_mgr.createSTLExportOptions(export_target, stl_path)
                try:
                    stl_opts.meshRefinement = adsk.fusion.MeshRefinementOptions.High
                except:
                    try:
                        stl_opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
                    except:
                        pass
                if hasattr(stl_opts, "isBinaryFormat"):
                    stl_opts.isBinaryFormat = True

                export_mgr.execute(stl_opts)
                self._log(f"[ACDC4Robot] STL generado: {stl_path}")
            except Exception:
                self._log(f"[ACDC4Robot] Error exportando STL para link '{link.get('name','?')}'")
                self._log(traceback.format_exc())

    # ===================== URDF writer =====================
    def _write_urdf(self):
        urdf_path = os.path.join(self.output_dir, f"{self.robot_name}.urdf")

        lines = []
        lines.append(f'<robot name="{self.robot_name}">')

        # Links
        for link in self.links:
            name = link["name"]
            mesh = link.get("mesh")

            lines.append(f'  <link name="{name}">')

            if mesh:
                lines.append('    <visual>')
                lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
                lines.append(f'      <geometry><mesh filename="meshes/{mesh}" scale="0.001 0.001 0.001"/></geometry>')
                lines.append('    </visual>')

                lines.append('    <collision>')
                lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
                lines.append(f'      <geometry><mesh filename="meshes/{mesh}" scale="0.001 0.001 0.001"/></geometry>')
                lines.append('    </collision>')

            # Dummy inertial
            lines.append('    <inertial>')
            lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
            lines.append('      <mass value="1.0"/>')
            lines.append('      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>')
            lines.append('    </inertial>')

            lines.append('  </link>')

        # Joints
        for j in self.joints:
            name = j["name"]
            parent = j["parent"]
            child = j["child"]
            jtype = j["type"]
            ox, oy, oz = j["origin_xyz"]
            rr, pp, yy = j["origin_rpy"]
            ax, ay, az = j["axis"]
            limit = j.get("limit")

            lines.append(f'  <joint name="{name}" type="{jtype}">')
            lines.append(f'    <origin xyz="{ox} {oy} {oz}" rpy="{rr} {pp} {yy}"/>')
            lines.append(f'    <parent link="{parent}"/>')
            lines.append(f'    <child link="{child}"/>')
            lines.append(f'    <axis xyz="{ax} {ay} {az}"/>')

            # Limits: crucial for interactividad
            if jtype == "revolute":
                # si vino con limit, úsalo; si no, amplio
                lo, hi = limit if limit else (-6.28318, 6.28318)
                lines.append(f'    <limit lower="{lo}" upper="{hi}" effort="10" velocity="10"/>')
            elif jtype == "continuous":
                # continuous: sin lower/upper
                lines.append('    <limit effort="10" velocity="10"/>')
            elif jtype == "prismatic":
                lo, hi = limit if limit else (-0.1, 0.1)
                lines.append(f'    <limit lower="{lo}" upper="{hi}" effort="10" velocity="10"/>')

            lines.append('  </joint>')

        lines.append('</robot>')

        with open(urdf_path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))

        self._log(f"[ACDC4Robot] URDF generado: {urdf_path}")

    # ======================= Logs ==========================
    def _log(self, msg: str):
        try:
            print(msg)
        except:
            pass
        # Para debug hardcore:
        # if self.ui:
        #     self.ui.messageBox(str(msg), "ACDC4Robot Debug")


def export_robot(robot_name: str, base_output_dir: str = None, *args, **kwargs):
    """Helper llamado desde acdc4robot.py."""
    if robot_name is None and len(args) > 0:
        robot_name = args[0]
    if base_output_dir is None and len(args) > 1:
        base_output_dir = args[1]

    exporter = RobotExporter(robot_name, base_output_dir)
    exporter.export_all()
