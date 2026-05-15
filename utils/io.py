from __future__ import annotations

import json
import glob
from pathlib import Path
from typing import Any, Iterable

import yaml


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}


def ensure_dir(path: str | Path) -> Path:
    out = Path(path)
    out.mkdir(parents=True, exist_ok=True)
    return out


def read_data(path: str | Path) -> dict[str, Any]:
    p = Path(path)
    with p.open("r", encoding="utf-8") as f:
        if p.suffix.lower() in {".yaml", ".yml"}:
            data = yaml.safe_load(f)
        else:
            data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"{p} must contain a mapping")
    return data


def write_json(path: str | Path, data: Any) -> Path:
    p = Path(path)
    ensure_dir(p.parent)
    with p.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, sort_keys=True)
        f.write("\n")
    return p


def expand_image_paths(inputs: Iterable[str | Path]) -> list[Path]:
    paths: list[Path] = []
    for item in inputs:
        raw = str(item)
        matches = [Path(p) for p in sorted(glob.glob(raw))] if any(c in raw for c in "*?[]") else [Path(raw)]
        for p in matches:
            if p.is_dir():
                paths.extend(sorted(x for x in p.iterdir() if x.suffix.lower() in IMAGE_SUFFIXES))
            elif p.is_file() and p.suffix.lower() in IMAGE_SUFFIXES:
                paths.append(p)
    deduped: list[Path] = []
    seen: set[Path] = set()
    for p in paths:
        rp = p.resolve()
        if rp not in seen:
            deduped.append(p)
            seen.add(rp)
    return deduped
