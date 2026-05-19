from __future__ import annotations

import argparse
import csv
from pathlib import Path

from src.board import (
    A4_HEIGHT_M,
    A4_WIDTH_M,
    DEFAULT_FAMILY,
    create_irregular_table_layout,
    create_layout_from_placements,
    create_single_tag_layout,
    save_board_layout,
    save_board_pdf_and_preview,
)
from utils.io import ensure_dir, write_json


DEFAULT_SAMPLE_TABLE_WIDTH_M = 2.8
DEFAULT_SAMPLE_TABLE_HEIGHT_M = 2.8


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate one printable AprilTag page per tag.")
    parser.add_argument("--output-dir", default="outputs/multi_tags", help="Directory for generated pages and metadata.")
    parser.add_argument("--count", type=int, default=12, help="Number of tag pages to generate when no CSV is provided.")
    parser.add_argument("--start-id", type=int, default=0, help="First tag id when generating a contiguous id range.")
    parser.add_argument("--tag-size-m", type=float, default=0.045, help="Printed tag edge size in meters.")
    parser.add_argument("--family", default=DEFAULT_FAMILY, help="OpenCV AprilTag dictionary name.")
    parser.add_argument("--paper-width-m", type=float, default=A4_WIDTH_M)
    parser.add_argument("--paper-height-m", type=float, default=A4_HEIGHT_M)
    parser.add_argument("--dpi", type=int, default=300)
    parser.add_argument("--manifest-name", default="tag_pages_manifest.json")
    parser.add_argument("--placements-csv", default=None, help="Optional CSV with columns: id,x_m,y_m,yaw_deg.")
    parser.add_argument(
        "--sample-layout",
        action="store_true",
        help="Also write a deterministic irregular table layout for simulation or dry runs.",
    )
    parser.add_argument("--table-width-m", type=float, default=DEFAULT_SAMPLE_TABLE_WIDTH_M)
    parser.add_argument("--table-height-m", type=float, default=DEFAULT_SAMPLE_TABLE_HEIGHT_M)
    parser.add_argument("--layout-name", default="table_layout.json")
    parser.add_argument("--seed", type=int, default=7)
    return parser.parse_args()


def _read_placements_csv(path: str | Path) -> list[tuple[int, float, float, float]]:
    placements: list[tuple[int, float, float, float]] = []
    with Path(path).open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        required = {"id", "x_m", "y_m"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"{path} is missing CSV column(s): {', '.join(sorted(missing))}")
        for row in reader:
            placements.append(
                (
                    int(row["id"]),
                    float(row["x_m"]),
                    float(row["y_m"]),
                    float(row.get("yaw_deg") or 0.0),
                )
            )
    return placements


def _table_size_from_args(args: argparse.Namespace) -> tuple[float, float] | None:
    if args.table_width_m is None and args.table_height_m is None:
        return None
    if args.table_width_m is None or args.table_height_m is None:
        raise ValueError("--table-width-m and --table-height-m must be provided together")
    return (float(args.table_width_m), float(args.table_height_m))


def main() -> int:
    args = parse_args()
    if args.count <= 0:
        raise ValueError("--count must be positive")
    if args.tag_size_m <= 0:
        raise ValueError("--tag-size-m must be positive")
    if args.placements_csv and args.sample_layout:
        raise ValueError("--placements-csv and --sample-layout are mutually exclusive")

    out_dir = ensure_dir(args.output_dir)
    pages_dir = ensure_dir(out_dir / "pages")
    table_size = _table_size_from_args(args)

    placements = _read_placements_csv(args.placements_csv) if args.placements_csv else None
    tag_ids = [tag_id for tag_id, _, _, _ in placements] if placements else list(range(args.start_id, args.start_id + args.count))

    page_rows = []
    for tag_id in tag_ids:
        page_layout = create_single_tag_layout(
            tag_id,
            tag_size_m=args.tag_size_m,
            family=args.family,
            paper_size_m=(args.paper_width_m, args.paper_height_m),
        )
        pdf_path, png_path = save_board_pdf_and_preview(
            page_layout,
            pages_dir / f"tag_{tag_id:03d}.pdf",
            pages_dir / f"tag_{tag_id:03d}.png",
            dpi=args.dpi,
        )
        page_rows.append({"id": int(tag_id), "pdf": str(pdf_path), "preview": str(png_path)})

    layout_path = None
    if placements is not None:
        layout = create_layout_from_placements(
            placements,
            tag_size_m=args.tag_size_m,
            family=args.family,
            table_size_m=table_size,
        )
        layout_path = save_board_layout(out_dir / args.layout_name, layout)
    elif args.sample_layout:
        layout = create_irregular_table_layout(
            args.count,
            tag_size_m=args.tag_size_m,
            family=args.family,
            table_size_m=table_size,
            start_id=args.start_id,
            seed=args.seed,
        )
        layout_path = save_board_layout(out_dir / args.layout_name, layout)

    manifest = {
        "family": args.family,
        "tag_size_m": float(args.tag_size_m),
        "paper_size_m": [float(args.paper_width_m), float(args.paper_height_m)],
        "pages": page_rows,
        "table_layout": str(layout_path) if layout_path else None,
    }
    manifest_path = write_json(out_dir / args.manifest_name, manifest)

    print(f"manifest: {manifest_path}")
    print(f"pages: {pages_dir}")
    if layout_path:
        print(f"table layout: {layout_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
