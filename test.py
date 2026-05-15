from __future__ import annotations

import json
from pathlib import Path

import render_test


def main() -> int:
    out_dir = Path("outputs/smoke")
    code = render_test.main(
        [
            "--output-dir",
            str(out_dir),
            "--actual-tag-size-m",
            "0.0465",
            "--calibration-views",
            "10",
        ]
    )
    metrics_path = out_dir / "metrics.json"
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    known = metrics["known_intrinsics"]
    if not known or not all(row["pose"]["success"] for row in known):
        raise AssertionError("known-intrinsics localization failed")
    max_center_error = max(row["camera_center_error_m"] for row in known)
    max_rotation_error = max(row["rotation_error_deg"] for row in known)
    if max_center_error > 0.03:
        raise AssertionError(f"camera-center error too high: {max_center_error:.6f}m")
    if max_rotation_error > 3.0:
        raise AssertionError(f"rotation error too high: {max_rotation_error:.6f}deg")
    print(f"smoke test passed: max center error {max_center_error:.6f}m, max rotation error {max_rotation_error:.6f}deg")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
