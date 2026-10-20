"""Versioned numeric-only trajectory cache; summaries are not sufficient input."""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

import numpy as np

from .postprocess import chain
from .problem import Solution


def json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def baseline_identity(p, branch, source_root):
    """Hash effective physical/numerical settings and the baseline implementation.

    Tracker settings are deliberately excluded: they cannot change this flight path.
    """
    params = asdict(p)
    params.pop("star_tracker", None)
    source_root = Path(source_root)
    files = [path for path in sorted((source_root / "hopper").rglob("*.py"))
             if not path.name.startswith(("star_tracker", "tracker_"))]
    files.append(source_root / "scripts" / "run_roundtrip.py")
    source = hashlib.sha256()
    for path in files:
        source.update(path.relative_to(source_root).as_posix().encode())
        source.update(path.read_bytes().replace(b"\r\n", b"\n"))
    identity = dict(branch=branch, params=params, baseline_source_sha256=source.hexdigest())
    return hashlib.sha256(json.dumps(identity, default=json_value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def save_trajectory(path, p, sol, branch, identity, validation):
    metadata = dict(version=1, branch=branch, baseline_identity=identity, validation=validation,
                    status=sol.status, success=sol.success, m_wet=sol.m_wet, z=sol.z,
                    iterations=sol.iterations, solve_time=sol.solve_time)
    arrays = {"metadata": np.array(json.dumps(metadata, default=json_value))}
    for name in chain(branch):
        ph = sol.phases[name]
        for key in ("X", "U"):
            arrays[f"{name}_{key}"] = np.asarray(ph[key], dtype=float)
        arrays[f"{name}_t"] = np.array(ph["t"], dtype=float)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, **arrays)


def load_trajectory(path, p, branch, identity):
    try:
        with np.load(path, allow_pickle=False) as data:
            metadata = json.loads(str(data["metadata"].item()))
            if (metadata.get("version") != 1 or metadata.get("baseline_identity") != identity or
                    metadata.get("branch") != branch):
                raise ValueError("Stale/incompatible trajectory cache; rerun with --refresh-baseline")
            if not metadata.get("success") or not metadata.get("validation", {}).get("passed"):
                raise ValueError("Trajectory cache is not a validated successful solution")
            phases = {}
            for name in chain(branch):
                kind = name.split("2_")[0] if "2_" in name else name.split("_")[0]
                X, U, duration = data[f"{name}_X"], data[f"{name}_U"], float(data[f"{name}_t"])
                n = p.solver.nodes[kind] + 1
                if (X.shape != (7, n) or U.shape != (4, n) or not np.isfinite(X).all() or
                        not np.isfinite(U).all() or not np.isfinite(duration) or duration <= 0):
                    raise ValueError("Invalid trajectory cache array shapes/values")
                phases[name] = dict(X=X.copy(), U=U.copy(), t=duration, powered=kind != "coast",
                                    kind=kind, branch=branch if kind == "terminal" else None)
    except (KeyError, OSError, TypeError, json.JSONDecodeError) as exc:
        raise ValueError("Full versioned NPZ trajectory required; summary JSON is insufficient") from exc
    return Solution(phases, metadata["z"], metadata["status"], True, metadata["iterations"],
                    metadata["solve_time"], metadata["m_wet"]), metadata["validation"]
