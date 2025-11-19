# runtime URDF inertial patcher to avoid massless/zero-inertia links
from pathlib import Path
import xml.etree.ElementTree as ET

def _as_float(s):
    try:
        return float(s)
    except Exception:
        return None

def fix_urdf_inertials(urdf_path: Path, min_mass=1e-2, min_inertia=1e-4, links_to_fix=None):
    """
    Ensure specified links have positive mass and diagonal inertia.
    """
    urdf_path = Path(urdf_path)
    original = urdf_path.read_text(encoding="utf-8")
    tree = ET.ElementTree()
    tree.parse(urdf_path)
    root = tree.getroot()

    for link in root.findall("link"):
        name = link.get("name", "")
        if links_to_fix and name not in links_to_fix:
            continue

        inertial = link.find("inertial")
        if inertial is None:
            continue

        mass_node = inertial.find("mass")
        inertia_node = inertial.find("inertia")

        if mass_node is not None:
            m = _as_float(mass_node.get("value", ""))
            if m is None or m <= 0.0:
                mass_node.set("value", f"{min_mass}")

        if inertia_node is not None:
            ixx = _as_float(inertia_node.get("ixx", "0"))
            iyy = _as_float(inertia_node.get("iyy", "0"))
            izz = _as_float(inertia_node.get("izz", "0"))
            if (ixx is None or ixx <= 0.0) or (iyy is None or iyy <= 0.0) or (izz is None or izz <= 0.0):
                inertia_node.set("ixx", f"{min_inertia}")
                inertia_node.set("iyy", f"{min_inertia}")
                inertia_node.set("izz", f"{min_inertia}")
            inertia_node.set("ixy", "0")
            inertia_node.set("ixz", "0")
            inertia_node.set("iyz", "0")

    bak = urdf_path.with_suffix(urdf_path.suffix + ".bak")
    if not bak.exists():
        bak.write_text(original, encoding="utf-8")
    tree.write(urdf_path, encoding="utf-8", xml_declaration=True)
    return True