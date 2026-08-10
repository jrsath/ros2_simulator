#!/usr/bin/env python3

import math
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def origin_matrix(origin):
    # Convert a URDF origin into a transform matrix.
    matrix = np.eye(4)
    if origin is None:
        return matrix

    xyz = np.array([float(value) for value in origin.get("xyz", "0 0 0").split()])
    roll, pitch, yaw = [
        float(value) for value in origin.get("rpy", "0 0 0").split()
    ]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    matrix[:3, :3] = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
    matrix[:3, 3] = xyz
    return matrix


def read_binary_stl(path):
    # Read triangles from a binary STL file.
    data = path.read_bytes()
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    triangles = np.empty((triangle_count, 3, 3), dtype=np.float64)
    for index in range(triangle_count):
        offset = 84 + index * 50 + 12
        for vertex in range(3):
            triangles[index, vertex] = struct.unpack_from(
                "<fff", data, offset + vertex * 12
            )
    return triangles


def write_binary_stl(path, triangles):
    # Write triangles to a binary STL file.
    with path.open("wb") as output:
        output.write(b"auto_cake_feeder cell frame".ljust(80, b"\0"))
        output.write(struct.pack("<I", len(triangles)))
        for triangle in triangles:
            normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
            length = np.linalg.norm(normal)
            if length:
                normal /= length
            output.write(struct.pack("<fff", *normal.astype(np.float32)))
            for vertex in triangle:
                output.write(struct.pack("<fff", *vertex.astype(np.float32)))
            output.write(struct.pack("<H", 0))


def collect_link_poses(robot):
    # Find every fixed link pose from the URDF joint tree.
    links = {link.get("name") for link in robot.findall("link")}
    children = set()
    joints = []
    for joint in robot.findall("joint"):
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        children.add(child)
        joints.append((parent, child, origin_matrix(joint.find("origin"))))

    roots = links - children
    if len(roots) != 1:
        raise RuntimeError(f"Expected one root link, found {sorted(roots)}")

    adjacency = {}
    for parent, child, transform in joints:
        adjacency.setdefault(parent, []).append((child, transform))

    root = next(iter(roots))
    poses = {root: np.eye(4)}
    pending = [root]
    while pending:
        parent = pending.pop()
        for child, transform in adjacency.get(parent, []):
            poses[child] = poses[parent] @ transform
            pending.append(child)
    return poses


def build_mesh(frame_package, output_path):
    # Combine the exported frame links into one static visual mesh.
    urdf_path = frame_package / "urdf" / "frame_assembly.xacro"
    robot = ET.parse(urdf_path).getroot()
    poses = collect_link_poses(robot)
    all_triangles = []

    for link in robot.findall("link"):
        visual = link.find("visual")
        if visual is None:
            continue
        mesh = visual.find("./geometry/mesh")
        if mesh is None:
            continue

        mesh_name = mesh.get("filename").split("/meshes/")[-1]
        scale = np.array([float(value) for value in mesh.get("scale").split()])
        triangles = read_binary_stl(frame_package / "meshes" / mesh_name)
        triangles *= scale

        transform = poses[link.get("name")] @ origin_matrix(visual.find("origin"))
        flat = triangles.reshape(-1, 3)
        flat = (transform[:3, :3] @ flat.T).T + transform[:3, 3]
        all_triangles.append(flat.reshape(-1, 3, 3) * 1000.0)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    write_binary_stl(output_path, np.concatenate(all_triangles))


def main():
    # Build the cell frame mesh from the supplied description package.
    if len(sys.argv) != 3:
        raise SystemExit(
            "Usage: build_cell_frame_mesh.py FRAME_PACKAGE OUTPUT_STL"
        )
    build_mesh(Path(sys.argv[1]), Path(sys.argv[2]))


if __name__ == "__main__":
    main()
