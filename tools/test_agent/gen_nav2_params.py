#!/usr/bin/env python3
import argparse
import copy
import pathlib
import sys
from typing import Any, Dict

import yaml


def _deep_get(d: Dict[str, Any], path: str) -> Any:
    cur: Any = d
    for key in path.split("."):
        if not isinstance(cur, dict) or key not in cur:
            return None
        cur = cur[key]
    return cur


def _deep_set(d: Dict[str, Any], path: str, value: Any) -> None:
    cur: Any = d
    parts = path.split(".")
    for key in parts[:-1]:
        if key not in cur or not isinstance(cur[key], dict):
            cur[key] = {}
        cur = cur[key]
    cur[parts[-1]] = value


def generate(base: Dict[str, Any], controller: str) -> Dict[str, Any]:
    out = copy.deepcopy(base)

    cs_params = _deep_get(out, "controller_server.ros__parameters")
    if not isinstance(cs_params, dict):
        raise RuntimeError("base YAML missing controller_server.ros__parameters")

    # Ensure controller plugin name is stable: FollowPath
    cs_params["controller_plugins"] = ["FollowPath"]
    follow: Dict[str, Any]
    if controller == "dwb":
        follow_val = cs_params.get("FollowPath", {})
        follow = follow_val if isinstance(follow_val, dict) else {}
    else:
        # For non-DWB tests, start from a clean controller config to avoid inheriting
        # unrelated parameters (e.g., DWB critics / sampling params).
        follow = {}

    # Default settings used by the agent tools
    cs_params.setdefault("controller_frequency", 20.0)
    cs_params.setdefault("failure_tolerance", 1.0)

    if controller == "pp":
        follow["plugin"] = "pp_controller::Nav2PurePursuitController"

        follow.setdefault("desired_linear_vel", 0.35)
        follow.setdefault("lookahead_dist", 0.6)
        follow.setdefault("min_lookahead_dist", 0.3)
        follow.setdefault("max_lookahead_dist", 1.0)
        follow.setdefault("lookahead_time", 1.5)
        follow.setdefault("use_velocity_scaled_lookahead_dist", False)

        follow.setdefault("min_approach_linear_velocity", 0.05)
        follow.setdefault("approach_velocity_scaling_dist", 0.6)

        follow.setdefault("use_regulated_linear_velocity_scaling", True)
        follow.setdefault("regulated_linear_scaling_min_radius", 0.9)
        follow.setdefault("regulated_linear_scaling_min_speed", 0.25)

        follow.setdefault("use_rotate_to_heading", False)
        follow.setdefault("rotate_to_heading_min_angle", 0.785)
        follow.setdefault("rotate_to_heading_angular_vel", 1.5)

        follow.setdefault("allow_reversing", True)
        follow.setdefault("max_angular_vel", 1.0)
        follow.setdefault("transform_tolerance", 0.2)

    elif controller == "rpp":
        follow["plugin"] = "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"

        follow.setdefault("desired_linear_vel", 0.35)
        follow.setdefault("lookahead_dist", 0.6)
        follow.setdefault("min_lookahead_dist", 0.3)
        follow.setdefault("max_lookahead_dist", 1.0)
        follow.setdefault("lookahead_time", 1.5)
        follow.setdefault("use_velocity_scaled_lookahead_dist", False)
        follow.setdefault("rotate_to_heading_angular_vel", 1.5)
        follow.setdefault("transform_tolerance", 0.2)
        follow.setdefault("min_approach_linear_velocity", 0.05)
        follow.setdefault("approach_velocity_scaling_dist", 0.6)
        follow.setdefault("use_collision_detection", True)
        follow.setdefault("max_allowed_time_to_collision_up_to_carrot", 1.0)
        follow.setdefault("use_regulated_linear_velocity_scaling", True)
        follow.setdefault("use_fixed_curvature_lookahead", False)
        follow.setdefault("allow_reversing", True)

    elif controller == "dwb":
        # Keep whatever the base file had (dwb config)
        pass
    else:
        raise RuntimeError(f"unknown controller: {controller}")

    cs_params["FollowPath"] = follow
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", required=True, help="Base nav2_params.yaml")
    ap.add_argument("--controller", required=True, choices=["pp", "rpp", "dwb"])
    ap.add_argument("--out", required=True, help="Output YAML path")
    args = ap.parse_args()

    base_path = pathlib.Path(args.base)
    out_path = pathlib.Path(args.out)
    base = yaml.safe_load(base_path.read_text())
    if not isinstance(base, dict):
        raise RuntimeError("base YAML must be a mapping")

    generated = generate(base, args.controller)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(yaml.safe_dump(generated, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
