#!/usr/bin/env python3
"""Extract the competition highway lane centerlines from a MORAI MGeo map."""

import argparse
import json
import math
from pathlib import Path


LANE_CHAINS = {
    # Competition-logical lanes. MGeo ego_lane=1 (A2256W000429) is separated
    # from A2256W000418 by the white solid boundary B2256W000059. It belongs
    # to a different straight-through movement, so the usable highway group
    # intentionally maps MGeo lanes 2..5 to logical lanes 1..4.
    "highway_lane1": [
        "A2256W000418", "A2256W000431", "A2256W000435",
        "A2256W000423", "A2256W000432", "A2256W000427",
        "A2256W000444",
    ],
    "highway_lane2": [
        "A2256W000410", "A2256W000430", "A2256W000434",
        "A2256W000422", "A2256W000153", "A2256W000451",
        "A2256W000446",
    ],
    "highway_lane3": [
        "A2256W000409", "A2256W000420", "A2256W000408",
        "A2256W000445",
    ],
    "highway_lane4": ["A2256W000411"],
}


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def load_links(hdmap_dir):
    path = hdmap_dir / "link_set.json"
    with path.open("r", encoding="utf-8-sig") as stream:
        links = json.load(stream)
    return {link["idx"]: link for link in links}


def concatenate_chain(name, chain, links, join_tolerance_m):
    points = []
    for link_id in chain:
        if link_id not in links:
            raise KeyError("{}: missing link {}".format(name, link_id))

        link_points = links[link_id]["points"]
        if len(link_points) < 2:
            raise ValueError("{}: link {} has fewer than two points".format(
                name, link_id))

        if points:
            gap = distance(points[-1], link_points[0])
            if gap > join_tolerance_m:
                raise ValueError(
                    "{}: discontinuity {} -> {} is {:.3f} m".format(
                        name, previous_id, link_id, gap))
            if gap <= 1e-6:
                link_points = link_points[1:]

        points.extend(link_points)
        previous_id = link_id

    return points


def path_length(points):
    return sum(distance(a, b) for a, b in zip(points, points[1:]))


def validate_spacing(name, points, maximum_spacing_m):
    maximum = max(distance(a, b) for a, b in zip(points, points[1:]))
    if maximum > maximum_spacing_m:
        raise ValueError("{}: maximum point spacing is {:.3f} m".format(
            name, maximum))
    return maximum


def write_path(path, points):
    with path.open("w", encoding="ascii", newline="\n") as stream:
        for point in points:
            z = point[2] if len(point) > 2 else 0.0
            stream.write("{:.10f}\t{:.10f}\t{:.10f}\n".format(
                point[0], point[1], z))


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--hdmap-dir", required=True, type=Path,
        help="Directory containing the MORAI MGeo link_set.json")
    parser.add_argument(
        "--output-dir", type=Path,
        default=Path(__file__).resolve().parents[1] / "path_data")
    parser.add_argument("--join-tolerance-m", type=float, default=0.05)
    parser.add_argument("--maximum-spacing-m", type=float, default=0.75)
    return parser.parse_args()


def main():
    args = parse_args()
    links = load_links(args.hdmap_dir)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    metadata = {}
    for name, chain in LANE_CHAINS.items():
        points = concatenate_chain(
            name, chain, links, args.join_tolerance_m)
        maximum_spacing = validate_spacing(
            name, points, args.maximum_spacing_m)
        output_path = args.output_dir / (name + ".txt")
        write_path(output_path, points)
        metadata[name] = {
            "links": chain,
            "point_count": len(points),
            "length_m": round(path_length(points), 3),
            "maximum_spacing_m": round(maximum_spacing, 3),
        }
        print("{}: {} points, {:.1f} m -> {}".format(
            name, len(points), metadata[name]["length_m"], output_path))

    with (args.output_dir / "highway_lane_paths.json").open(
            "w", encoding="utf-8", newline="\n") as stream:
        json.dump(metadata, stream, indent=2)
        stream.write("\n")


if __name__ == "__main__":
    main()
