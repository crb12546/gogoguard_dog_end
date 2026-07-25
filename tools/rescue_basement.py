#!/usr/bin/env python3
from pathlib import Path
import math
import os


ROUTE_IN = Path("/home/unitree/go2_fastlio_ws/src/go2_fastlio_patrol/routes/basement.csv")
ROUTE_OUT = Path("/home/unitree/go2_fastlio_ws/src/go2_fastlio_patrol/routes/basement_rescue.csv")
PCD_IN = Path("/home/unitree/go2_fastlio_ws/maps/console/basement.pcd")
PCD_OUT = Path("/home/unitree/go2_fastlio_ws/maps/console/basement_rescue.pcd")

MAX_ROUTE_STEP_M = 5.0
PCD_MARGIN_M = 25.0
PCD_MIN_Z = -50.0
PCD_MAX_Z = 50.0


def rescue_route():
    lines = ROUTE_IN.read_text().splitlines()
    if not lines:
        raise RuntimeError("empty route csv")

    kept = [lines[0]]
    previous = None
    cut_reason = "no cut"

    for line in lines[1:]:
        fields = line.split(",")
        if len(fields) < 3:
            continue
        x = float(fields[1])
        y = float(fields[2])
        if previous is not None:
            step = math.hypot(x - previous[0], y - previous[1])
            if step > MAX_ROUTE_STEP_M:
                cut_reason = (
                    "cut before id={id} step={step:.3f} "
                    "prev=({px:.3f},{py:.3f}) cur=({x:.3f},{y:.3f})"
                ).format(id=fields[0], step=step, px=previous[0], py=previous[1], x=x, y=y)
                break
        kept.append(line)
        previous = (x, y)

    ROUTE_OUT.write_text("\n".join(kept) + "\n")

    points = []
    for line in kept[1:]:
        fields = line.split(",")
        points.append((float(fields[1]), float(fields[2])))
    if not points:
        raise RuntimeError("no rescued route points")

    min_x = min(point[0] for point in points)
    max_x = max(point[0] for point in points)
    min_y = min(point[1] for point in points)
    max_y = max(point[1] for point in points)

    return {
        "rows": len(points),
        "min_x": min_x,
        "max_x": max_x,
        "min_y": min_y,
        "max_y": max_y,
        "cut_reason": cut_reason,
    }


def point_is_kept(line, bounds):
    fields = line.split()
    if len(fields) < 3:
        return False
    try:
        x = float(fields[0])
        y = float(fields[1])
        z = float(fields[2])
    except ValueError:
        return False
    return (
        bounds["min_x"] <= x <= bounds["max_x"]
        and bounds["min_y"] <= y <= bounds["max_y"]
        and PCD_MIN_Z <= z <= PCD_MAX_Z
    )


def count_pcd_points(bounds):
    count = 0
    in_data = False
    with PCD_IN.open("r") as source:
        for raw_line in source:
            line = raw_line.rstrip("\n")
            if not in_data:
                if line.startswith("DATA"):
                    in_data = True
                continue
            if point_is_kept(line, bounds):
                count += 1
    return count


def rescue_pcd(bounds, count):
    tmp_path = PCD_OUT.with_suffix(".pcd.tmp")
    in_data = False
    with PCD_IN.open("r") as source, tmp_path.open("w") as dest:
        for raw_line in source:
            line = raw_line.rstrip("\n")
            if not in_data:
                if line.startswith("WIDTH "):
                    dest.write("WIDTH {}\n".format(count))
                elif line.startswith("POINTS "):
                    dest.write("POINTS {}\n".format(count))
                else:
                    dest.write(line + "\n")
                if line.startswith("DATA"):
                    in_data = True
                continue
            if point_is_kept(line, bounds):
                dest.write(line + "\n")
    os.replace(str(tmp_path), str(PCD_OUT))


def main():
    route = rescue_route()
    bounds = {
        "min_x": route["min_x"] - PCD_MARGIN_M,
        "max_x": route["max_x"] + PCD_MARGIN_M,
        "min_y": route["min_y"] - PCD_MARGIN_M,
        "max_y": route["max_y"] + PCD_MARGIN_M,
    }
    count = count_pcd_points(bounds)
    if count <= 0:
        raise RuntimeError("pcd rescue filter produced no points")
    rescue_pcd(bounds, count)

    print("route_out={}".format(ROUTE_OUT))
    print("route_rows={}".format(route["rows"]))
    print("route_bbox=x[{:.3f},{:.3f}] y[{:.3f},{:.3f}]".format(route["min_x"], route["max_x"], route["min_y"], route["max_y"]))
    print("cut_reason={}".format(route["cut_reason"]))
    print("pcd_out={}".format(PCD_OUT))
    print("pcd_points={}".format(count))
    print("pcd_filter=x[{:.3f},{:.3f}] y[{:.3f},{:.3f}] z[{:.1f},{:.1f}]".format(bounds["min_x"], bounds["max_x"], bounds["min_y"], bounds["max_y"], PCD_MIN_Z, PCD_MAX_Z))


if __name__ == "__main__":
    main()