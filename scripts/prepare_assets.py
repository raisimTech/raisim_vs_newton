"""Prepare licensed public SDK models using Python's standard library and the SDK API."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET


def add(a, b):
    return [x + y for x, y in zip(a, b)]


def sub(a, b):
    return [x - y for x, y in zip(a, b)]


def mv(matrix, vector):
    return [sum(x * y for x, y in zip(row, vector)) for row in matrix]


def rotation(q):
    w, x, y, z = q
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]


def multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return [w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v]


def prepare(paths, output):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    sdk = Path(paths["sdk"])
    sources = {m: sdk / "rsc" / m for m in ("anymal", "strandbeest")}
    anymal = sources["anymal"] / "urdf" / "anymal.urdf"
    tree = ET.fromstring(re.sub(r"<\?xml[^>]*\?>", "", anymal.read_text()))
    for link in tree.findall("link"):
        for visual in list(link.findall("visual")):
            link.remove(visual)
    if tree.findall(".//collision/geometry/mesh"):
        raise ValueError("This benchmark expects primitive ANYmal collisions")
    ET.ElementTree(tree).write(output / "anymal.urdf", encoding="unicode")
    source = sources["strandbeest"] / "strandbeest.urdf"
    robot = ET.parse(source).getroot()
    links = robot.findall("link")
    names = output / "strandbeest-links.txt"
    names.write_text("\n".join(x.attrib["name"] for x in links) + "\n")
    environment = os.environ.copy()
    if os.name == "nt":
        prefix = Path(paths["sdk_prefix"])
        environment["PATH"] = os.pathsep.join(str(prefix / p) for p in ("bin", "lib", "bin/openusd", "lib/openusd")) + os.pathsep + environment.get("PATH", "")
    process = subprocess.run([paths["exporter"], str(source), str(names)], text=True,
                             capture_output=True, env=environment)
    if process.returncode:
        raise RuntimeError("Public SDK model export failed; check your activation key.\n" +
                           process.stdout + process.stderr)
    frames = json.loads(next(x[len("MODEL_KINEMATICS "):] for x in process.stdout.splitlines()
                             if x.startswith("MODEL_KINEMATICS ")))
    if len(frames) != 92 or any(not re.fullmatch(r"[A-Za-z_][A-Za-z_0-9]*", name) for name in frames):
        raise ValueError("Expected 92 named Strandbeest link frames")

    def pose(name):
        a = frames[name]
        return a[:3], rotation(a[3:]), a[3:]

    def numbers(values):
        return "(" + ", ".join(format(v, ".17g") for v in values) + ")"

    def vector_attr(kind, name, value):
        return f"    {kind} {name} = {numbers(value)}\n"

    text = ['#usda 1.0\n( defaultPrim = "Strandbeest"\n metersPerUnit = 1\n upAxis = "Z"\n)\n',
            'def PhysicsScene "PhysicsScene" {\n vector3f physics:gravityDirection = (0, 0, -1)\n float physics:gravityMagnitude = 9.81\n}\n',
            'def Xform "Strandbeest" ( prepend apiSchemas = ["PhysicsArticulationRootAPI"] ) {\n']
    for link in links:
        name = link.attrib["name"]
        p, _, q = pose(name)
        inertia = link.find("inertial")
        values = inertia.find("inertia").attrib
        diagonal = [float(values[k]) for k in ("ixx", "iyy", "izz")]
        if any(abs(float(values.get(k, 0))) > 1e-12 for k in ("ixy", "ixz", "iyz")) or max(diagonal) != min(diagonal):
            raise ValueError("Expected isotropic Strandbeest inertias")
        origin = inertia.find("origin")
        xyz = [float(x) for x in origin.attrib.get("xyz", "0 0 0").split()] if origin is not None else [0., 0., 0.]
        text.append(f'  def Xform "{name}" ( prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"] ) {{\n')
        text.append(f'    float physics:mass = {float(inertia.find("mass").attrib["value"]):.17g}\n')
        text.extend([vector_attr("point3f", "physics:centerOfMass", xyz),
                     vector_attr("float3", "physics:diagonalInertia", diagonal),
                     vector_attr("quatf", "physics:principalAxes", [1., 0., 0., 0.]),
                     vector_attr("double3", "xformOp:translate", p),
                     vector_attr("quatd", "xformOp:orient", q),
                     '    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n'])
        for index, collision in enumerate(link.findall("collision")):
            sphere = collision.find("geometry/sphere")
            if sphere is None:
                raise ValueError("Expected spherical foot collisions")
            origin = collision.find("origin")
            xyz = [float(x) for x in origin.attrib.get("xyz", "0 0 0").split()] if origin is not None else [0., 0., 0.]
            text.extend([f'    def Sphere "collision_{index}" ( prepend apiSchemas = ["PhysicsCollisionAPI"] ) {{\n',
                         f'      double radius = {float(sphere.attrib["radius"]):.17g}\n',
                         vector_attr("double3", "xformOp:translate", xyz),
                         '      uniform token[] xformOpOrder = ["xformOp:translate"]\n    }\n'])
        text.append('  }\n')
    anchors = []

    def joint(name, parent, child, world_position, world_orientation, fixed=False, loop=False):
        cls = "PhysicsFixedJoint" if fixed else "PhysicsRevoluteJoint"
        text.append(f'  def {cls} "{name}" {{\n')
        for side, body in enumerate((parent, child)):
            bp, br, bq = pose(body)
            local_position = mv(list(zip(*br)), sub(world_position, bp))
            inverse = [bq[0], -bq[1], -bq[2], -bq[3]]
            local_orientation = multiply(inverse, world_orientation)
            text.extend([f'    rel physics:body{side} = </Strandbeest/{body}>\n',
                         vector_attr("point3f", f"physics:localPos{side}", local_position),
                         vector_attr("quatf", f"physics:localRot{side}", local_orientation)])
        if not fixed:
            text.append('    uniform token physics:axis = "Y"\n')
        if loop:
            text.append('    bool physics:excludeFromArticulation = true\n')
            axis = mv(rotation(world_orientation), [0., 1., 0.])
            for sign in (-1, 1):
                anchors.append((parent, child, add(world_position, [sign * .05 * v for v in axis])))
        text.append('  }\n')

    for item in robot.findall("joint"):
        parent, child = item.find("parent").attrib["link"], item.find("child").attrib["link"]
        axis = item.find("axis")
        if axis is not None and item.attrib["type"] != "fixed" and list(map(float, axis.attrib["xyz"].split())) != [0., 1., 0.]:
            raise ValueError("Expected planar Y-axis tree joints")
        p, _, q = pose(child)
        joint(item.attrib["name"], parent, child, p, q, item.attrib["type"] == "fixed")
    equalities = robot.findall("constraints/equality")
    if len(equalities) != 36:
        raise ValueError("Expected 36 active closure hinges")
    for index, item in enumerate(equalities):
        parent, child = item.attrib["body1"], item.attrib["body2"]
        p, r, q = pose(parent)
        joint(f"loop_{index}", parent, child,
              add(p, mv(r, list(map(float, item.attrib["anchor"].split())))), q, loop=True)
    for index in range(1, 7):
        child = f"pair{index:02d}_bar_l"
        p, r, q = pose("crank_axle")
        child_p, _, _ = pose(child)
        y = mv(list(zip(*r)), sub(child_p, p))[1]
        joint(f"loop_bearing_{index}", "crank_axle", child,
              add(p, mv(r, [0., y, 0.])), q, loop=True)
    if len(anchors) != 84:
        raise ValueError("Expected 84 hinge anchor points")
    text.append('}\n')
    (output / "strandbeest.usda").write_text("".join(text))
    (output / "anchors.json").write_text(json.dumps(anchors, indent=2))
    (output / "strandbeest-anchors.txt").write_text("84\n" + "".join(
        f"{p} {c} " + " ".join(format(v, ".17g") for v in xyz) + "\n" for p, c, xyz in anchors))
    licenses = output / "licenses"
    licenses.mkdir(exist_ok=True)
    shutil.copyfile(sources["anymal"] / "LICENSE", licenses / "ANYmal.txt")
    shutil.copyfile(sources["strandbeest"] / "LICENSE.md", licenses / "Strandbeest.md")
    manifest = {"sdk_revision": paths["sdk_revision"], "source_sha256": {
        name: hashlib.sha256(path.read_bytes()).hexdigest()
        for name, path in (("anymal.urdf", anymal), ("strandbeest.urdf", source))},
        "links": len(links), "closure_hinges": 42, "anchor_points": 84}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2))
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--build", type=Path, default=Path("build"))
    parser.add_argument("--output", type=Path)
    parser.add_argument("--config", default="Release")
    args = parser.parse_args()
    print(prepare(json.loads((args.build / f"benchmark_paths_{args.config}.json").read_text()),
                  args.output or args.build / "assets"))
