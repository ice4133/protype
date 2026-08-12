#!/usr/bin/env python3
"""Trim an HDF5 collection file to a specified frame range [start, end).

Usage:
    python scripts/trim_hdf5.py <input.hdf5> <start> <end> [-o output.hdf5]

Examples:
    # Extract frames 100-500 (saves to input_100_500.hdf5)
    python scripts/trim_hdf5.py data/collection_123.hdf5 100 500

    # Extract frames 0-200, specify output path
    python scripts/trim_hdf5.py data/collection_123.hdf5 0 200 -o trimmed.hdf5
"""

import argparse
import os
import sys

import h5py
import numpy as np


def trim_hdf5(input_path: str, start: int, end: int, output_path=None):
    if not os.path.exists(input_path):
        print(f"Error: {input_path} not found")
        sys.exit(1)

    with h5py.File(input_path, 'r') as hf_in:
        # Find total frames from the first multi-dimensional dataset (recursive)
        total_frames = None
        def find_frames(name, obj):
            nonlocal total_frames
            if total_frames is None and isinstance(obj, h5py.Dataset) and obj.ndim >= 2:
                total_frames = obj.shape[0]
        hf_in.visititems(find_frames)

        if total_frames is None:
            print("Error: no frame-based datasets found")
            sys.exit(1)

        if start < 0 or end > total_frames or start >= end:
            print(f"Error: invalid range [{start}, {end}), total frames: {total_frames}")
            sys.exit(1)

        num_frames = end - start

        if output_path is None:
            base, ext = os.path.splitext(input_path)
            output_path = f"{base}_{start}_{end}{ext}"

        print(f"Input:  {input_path} ({total_frames} frames)")
        print(f"Output: {output_path} (frames [{start}, {end}), {num_frames} frames)")

        with h5py.File(output_path, 'w') as hf_out:
            def copy_item(name, obj):
                if isinstance(obj, h5py.Dataset):
                    if obj.ndim < 2:
                        hf_out.create_dataset(name, data=obj[()], dtype=obj.dtype)
                        print(f"  {name}: scalar")
                    else:
                        sliced = obj[start:end]
                        out_ds = hf_out.create_dataset(
                            name,
                            data=sliced,
                            dtype=obj.dtype,
                            chunks=(1, *obj.shape[1:]),
                        )
                        for attr_name, attr_val in obj.attrs.items():
                            out_ds.attrs[attr_name] = attr_val
                        print(f"  {name}: {obj.shape} -> {out_ds.shape}")
            hf_in.visititems(copy_item)

    print("Done.")


def main():
    parser = argparse.ArgumentParser(description="Trim HDF5 collection to a frame range")
    parser.add_argument("input", help="Input HDF5 file path")
    parser.add_argument("start", type=int, help="Start frame index (inclusive)")
    parser.add_argument("end", type=int, help="End frame index (exclusive)")
    parser.add_argument("-o", "--output", default=None, help="Output file path (default: input_start_end.hdf5)")
    args = parser.parse_args()

    trim_hdf5(args.input, args.start, args.end, args.output)


if __name__ == "__main__":
    main()
