from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

from src.board import A4_HEIGHT_M, A4_WIDTH_M, DEFAULT_FAMILY, marker_image
from utils.io import ensure_dir, write_json


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate tiled multi-page AprilTag PDF sheets.")
    parser.add_argument("--output-dir", default="outputs/tag_sheets", help="Directory for generated files.")
    parser.add_argument("--count", type=int, default=48, help="Number of different tags to generate.")
    parser.add_argument("--start-id", type=int, default=0, help="First tag id in the generated range.")
    parser.add_argument("--tag-size-m", type=float, default=0.045, help="Printed tag edge size in meters.")
    parser.add_argument("--family", default=DEFAULT_FAMILY, help="OpenCV AprilTag dictionary name.")
    parser.add_argument("--paper-width-m", type=float, default=A4_WIDTH_M)
    parser.add_argument("--paper-height-m", type=float, default=A4_HEIGHT_M)
    parser.add_argument("--rows", type=int, default=3, help="Tag rows per PDF page.")
    parser.add_argument("--cols", type=int, default=2, help="Tag columns per PDF page.")
    parser.add_argument("--dpi", type=int, default=300)
    parser.add_argument("--pdf-name", default=None, help="Output PDF filename. Defaults to a descriptive name.")
    parser.add_argument("--manifest-name", default="tag_sheet_manifest.json")
    parser.add_argument("--preview-dir", default="previews", help="Directory name for per-page PNG previews.")
    parser.add_argument("--no-labels", action="store_true", help="Do not print tag id labels below tags.")
    parser.add_argument("--no-cut-guides", action="store_true", help="Do not draw faint cell cut guides.")
    return parser.parse_args()


def _validate_args(args: argparse.Namespace) -> None:
    if args.count <= 0:
        raise ValueError("--count must be positive")
    if args.start_id < 0:
        raise ValueError("--start-id must be non-negative")
    if args.tag_size_m <= 0:
        raise ValueError("--tag-size-m must be positive")
    if args.paper_width_m <= 0 or args.paper_height_m <= 0:
        raise ValueError("--paper-width-m and --paper-height-m must be positive")
    if args.rows <= 0 or args.cols <= 0:
        raise ValueError("--rows and --cols must be positive")

    cell_w = args.paper_width_m / args.cols
    cell_h = args.paper_height_m / args.rows
    min_margin = min(cell_w, cell_h) - args.tag_size_m
    if min_margin < 0.020:
        raise ValueError(
            "tag size leaves less than 20mm total whitespace in the smallest cell; "
            "reduce --tag-size-m or use fewer rows/columns"
        )


def _draw_cut_guides(ax: plt.Axes, paper_w: float, paper_h: float, rows: int, cols: int) -> None:
    guide_color = "0.82"
    for col in range(1, cols):
        x = -paper_w / 2.0 + col * paper_w / cols
        ax.plot([x, x], [-paper_h / 2.0, paper_h / 2.0], color=guide_color, linewidth=0.35, linestyle="--")
    for row in range(1, rows):
        y = paper_h / 2.0 - row * paper_h / rows
        ax.plot([-paper_w / 2.0, paper_w / 2.0], [y, y], color=guide_color, linewidth=0.35, linestyle="--")


def _new_page_figure(paper_w: float, paper_h: float, dpi: int) -> tuple[plt.Figure, plt.Axes]:
    width_in = paper_w / 0.0254
    height_in = paper_h / 0.0254
    fig = plt.figure(figsize=(width_in, height_in), dpi=dpi, facecolor="white")
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(-paper_w / 2.0, paper_w / 2.0)
    ax.set_ylim(-paper_h / 2.0, paper_h / 2.0)
    ax.set_aspect("equal")
    ax.axis("off")
    return fig, ax


def _render_page(
    *,
    ax: plt.Axes,
    tag_ids: list[int],
    family: str,
    tag_size_m: float,
    paper_w: float,
    paper_h: float,
    rows: int,
    cols: int,
    labels: bool,
    cut_guides: bool,
) -> list[dict[str, object]]:
    if cut_guides:
        _draw_cut_guides(ax, paper_w, paper_h, rows, cols)

    cell_w = paper_w / cols
    cell_h = paper_h / rows
    half = tag_size_m * 0.5
    marker_px = 1000
    records: list[dict[str, object]] = []

    for slot, tag_id in enumerate(tag_ids):
        row, col = divmod(slot, cols)
        x = -paper_w / 2.0 + (col + 0.5) * cell_w
        y = paper_h / 2.0 - (row + 0.5) * cell_h

        image = marker_image(family, tag_id, marker_px)
        ax.imshow(
            image,
            cmap="gray",
            vmin=0,
            vmax=255,
            extent=(x - half, x + half, y - half, y + half),
            origin="upper",
            interpolation="nearest",
        )
        if labels:
            ax.text(
                x,
                y - half - 0.006,
                f"ID {tag_id}",
                ha="center",
                va="top",
                fontsize=8,
                color="0.25",
            )

        records.append(
            {
                "id": int(tag_id),
                "row": int(row),
                "col": int(col),
                "center_on_page_m": [float(x), float(y)],
            }
        )

    return records


def main() -> int:
    args = parse_args()
    _validate_args(args)

    out_dir = ensure_dir(args.output_dir)
    preview_dir = ensure_dir(out_dir / args.preview_dir)
    pdf_name = args.pdf_name or f"apriltags_{args.count}_{args.rows}x{args.cols}_a4.pdf"
    pdf_path = out_dir / pdf_name

    tag_ids = list(range(args.start_id, args.start_id + args.count))
    tags_per_page = args.rows * args.cols
    page_count = int(math.ceil(args.count / tags_per_page))
    pages: list[dict[str, object]] = []

    with PdfPages(pdf_path) as pdf:
        for page_index in range(page_count):
            page_ids = tag_ids[page_index * tags_per_page : (page_index + 1) * tags_per_page]
            fig, ax = _new_page_figure(args.paper_width_m, args.paper_height_m, args.dpi)
            page_records = _render_page(
                ax=ax,
                tag_ids=page_ids,
                family=args.family,
                tag_size_m=args.tag_size_m,
                paper_w=args.paper_width_m,
                paper_h=args.paper_height_m,
                rows=args.rows,
                cols=args.cols,
                labels=not args.no_labels,
                cut_guides=not args.no_cut_guides,
            )
            pdf.savefig(fig)
            preview_path = preview_dir / f"page_{page_index + 1:02d}.png"
            fig.savefig(preview_path, format="png", dpi=args.dpi)
            plt.close(fig)
            pages.append(
                {
                    "page": int(page_index + 1),
                    "preview": str(preview_path),
                    "tags": page_records,
                }
            )

    manifest = {
        "family": args.family,
        "tag_size_m": float(args.tag_size_m),
        "paper_size_m": [float(args.paper_width_m), float(args.paper_height_m)],
        "rows": int(args.rows),
        "cols": int(args.cols),
        "count": int(args.count),
        "start_id": int(args.start_id),
        "pdf": str(pdf_path),
        "pages": pages,
    }
    manifest_path = write_json(out_dir / args.manifest_name, manifest)

    print(f"pdf: {pdf_path}")
    print(f"manifest: {manifest_path}")
    print(f"previews: {preview_dir}")
    print(f"pages: {page_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
