#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# System libs
import json
import argparse

# Third-party libs
import pandas
from pandas.io.json import json_normalize


def show(filename):
    with open(filename, 'r') as f:
        db = json.load(f)
    df = json_normalize(db)
    print(df.groupby('method').describe()['time'].sort_values('mean'))


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", default=None, help="json file")
    return parser.parse_args()


def main():
    args = parse_args()
    show(args.input)


if __name__ == "__main__":
    main()
