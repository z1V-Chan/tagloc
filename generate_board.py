from __future__ import annotations

import argparse
from pathlib import Path

from src.board import create_default_layout, save_board_layout, save_board_pdf_and_preview
from utils.io import ensure_dir


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate an A4 AprilTag board PDF and layout JSON.")
    parser.add_argument("--output-dir", default="outputs/board", help="Directory for generated files.")
    parser.add_argument("--tag-size-m", type=float, default=0.045, help="Nominal tag edge size in meters.")
    parser.add_argument("--family", default="DICT_APRILTAG_36h11", help="OpenCV AprilTag dictionary name.")
    parser.add_argument("--pdf-name", default="a4_apriltag_board.pdf")
    parser.add_argument("--png-name", default="a4_apriltag_board.png")
    parser.add_argument("--layout-name", default="board_layout.json")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    out_dir = ensure_dir(args.output_dir)
    layout = create_default_layout(tag_size_m=args.tag_size_m, family=args.family)
    layout_path = save_board_layout(out_dir / args.layout_name, layout)
    pdf_path, png_path = save_board_pdf_and_preview(
        layout,
        out_dir / args.pdf_name,
        out_dir / args.png_name,
    )
    print(f"layout: {layout_path}")
    print(f"pdf: {Path(pdf_path)}")
    print(f"preview: {Path(png_path)}")


if __name__ == "__main__":
    main()

