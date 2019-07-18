#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# System libs
import os
import sys
import glob
import json
import argparse
import tempfile
import subprocess

# Third-party libs
import numpy

# Local libs
import paths


ALL_METHODS_RAY = [
    "igl",
    "embree",
    "geogram",
    "ours",
    "morton",
    "hilbert",
    "ours_binary",
    "morton_binary",
    "hilbert_binary",
]

ALL_METHODS_DIST = [
    "igl",
    "geogram",
    "ours",
    "morton",
    "hilbert",
    "ours_binary",
    "morton_binary",
    "hilbert_binary",
]


def ray_tracing(mesh_path, method):
    # Find executable path
    exe_path = paths.get_executable("ray_tracing", paths.PROJECT_DIR)

    args = [exe_path, mesh_path, '-W', '1200', '-H', '800', '-m', method]

    try:
        print(' '.join(args))
        res = subprocess.run(args, stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, check=True)
    except subprocess.CalledProcessError as e:
        print("Return Code: ", e.returncode, file=sys.stderr)
        raise e
    else:
        return res.stdout.decode("utf-8")


def distance_to_mesh(mesh_path, method):
    # Find executable path
    exe_path = paths.get_executable("distance_to_mesh", paths.PROJECT_DIR)

    args = [exe_path, mesh_path, '-s', '50000', '-m', method]

    try:
        print(' '.join(args))
        res = subprocess.run(args, stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, check=True)
    except subprocess.CalledProcessError as e:
        print("Return Code: ", e.returncode, file=sys.stderr)
        raise e
    else:
        return res.stdout.decode("utf-8")


def ray_tracing_all(folder):
    results = []
    for ext in ['off', 'stl', 'obj']:
        for mesh in glob.glob(os.path.join(folder, "*." + ext)):
            for method in ALL_METHODS_RAY:
                results.append(json.loads(ray_tracing(mesh, method)))
    return results


def distance_to_mesh_all(folder):
    results = []
    for ext in ['off', 'stl', 'obj']:
        for mesh in glob.glob(os.path.join(folder, "*." + ext)):
            for method in ALL_METHODS_DIST:
                results.append(json.loads(distance_to_mesh(mesh, method)))
    return results


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", default=None, help="data folder")
    parser.add_argument("-o", "--output", default='results.json', help="output results")
    parser.add_argument("-c", "--case", default='ray_tracing',
                        choices=['ray_tracing', 'distance_to_mesh'], help="output results")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.case == 'ray_tracing':
        res = ray_tracing_all(args.input)
    elif args.case == 'distance_to_mesh':
        res = distance_to_mesh_all(args.input)
    with open(args.output, 'w') as f:
        f.write(json.dumps(res, indent=4))


if __name__ == "__main__":
    main()
