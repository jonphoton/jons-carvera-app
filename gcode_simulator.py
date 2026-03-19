"""
GCode Simulator Engine — parser and vectorized material removal.
Adapted from GCodeSimulator (PyQt6 app) for use as a library module.
No GUI dependencies — uses only numpy and standard library.
"""

import re, math
import numpy as np
from dataclasses import dataclass


# ── Data structures ──────────────────────────────────────────────────────────

@dataclass
class ToolDef:
    number: int
    diameter_mm: float
    color: tuple       # (r, g, b) 0.0-1.0 for matplotlib
    name: str

@dataclass
class Segment:
    x0: float; y0: float; z0: float
    x1: float; y1: float; z1: float
    tool: int
    is_rapid: bool
    line_number: int

DEFAULT_TOOLS = {
    3: ToolDef(3, 25.4 / 8,  (0.20, 0.60, 1.00), 'T3  1/8"'),
    5: ToolDef(5, 25.4 / 32, (1.00, 0.40, 0.10), 'T5  1/32"'),
    6: ToolDef(6, 0.6,       (0.10, 0.90, 0.30), "T6  0.6mm"),
}


# ── G-code parser ────────────────────────────────────────────────────────────

def parse_word(line, letter):
    m = re.search(rf'{letter}([+-]?[\d.]+)', line)
    return float(m.group(1)) if m else None

def arc_to_segments(x0, y0, z0, x1, y1, z1, i, j, clockwise, n_segs=32):
    cx, cy = x0 + i, y0 + j
    r = math.hypot(i, j)
    if r < 1e-9:
        return [(x0, y0, z0, x1, y1, z1)]
    a0 = math.atan2(y0 - cy, x0 - cx)
    a1 = math.atan2(y1 - cy, x1 - cx)
    if clockwise:
        if a1 >= a0: a1 -= 2 * math.pi
    else:
        if a1 <= a0: a1 += 2 * math.pi
    delta = a1 - a0
    if abs(delta) < 1e-9:
        delta = -2 * math.pi if clockwise else 2 * math.pi
    steps = max(4, int(abs(delta) / (2 * math.pi) * n_segs))
    segs = []
    px, py, pz = x0, y0, z0
    for s in range(1, steps + 1):
        t = s / steps
        a = a0 + delta * t
        nx_ = cx + r * math.cos(a)
        ny_ = cy + r * math.sin(a)
        nz_ = z0 + (z1 - z0) * t
        segs.append((px, py, pz, nx_, ny_, nz_))
        px, py, pz = nx_, ny_, nz_
    return segs

def parse_gcode(filepath):
    """Parse a GCode file into raw lines and Segment objects."""
    with open(filepath, 'r') as f:
        raw_lines = f.readlines()
    segments = []
    x = y = z = 0.0
    current_tool = 3
    motion_mode = 0
    for line_idx, raw in enumerate(raw_lines):
        line = raw.split(';')[0].strip().upper()
        if not line: continue
        tc = re.search(r'M6\s*T(\d+)', line)
        if tc: current_tool = int(tc.group(1)); continue
        tc2 = re.search(r'T(\d+)\s*M6', line)
        if tc2: current_tool = int(tc2.group(1)); continue
        gm = re.search(r'G([0123])\b', line)
        if gm: motion_mode = int(gm.group(1))
        nx = parse_word(line, 'X'); ny = parse_word(line, 'Y')
        nz = parse_word(line, 'Z')
        ni = parse_word(line, 'I'); nj = parse_word(line, 'J')
        tx = nx if nx is not None else x
        ty = ny if ny is not None else y
        tz = nz if nz is not None else z
        if nx is None and ny is None and nz is None: continue
        if motion_mode in (0, 1):
            segments.append(Segment(x, y, z, tx, ty, tz, current_tool, motion_mode == 0, line_idx))
            x, y, z = tx, ty, tz
        elif motion_mode in (2, 3):
            ii = ni if ni is not None else 0.0
            jj = nj if nj is not None else 0.0
            for (ax0, ay0, az0, ax1, ay1, az1) in arc_to_segments(
                    x, y, z, tx, ty, tz, ii, jj, motion_mode == 2):
                segments.append(Segment(ax0, ay0, az0, ax1, ay1, az1,
                                        current_tool, False, line_idx))
            x, y, z = tx, ty, tz
    return raw_lines, segments


# ── Header parser ────────────────────────────────────────────────────────────

def parse_gcode_header(raw_lines):
    """Parse metadata from GCode header comments.

    Returns dict with keys:
        'stock_dims': (width, height, thickness) or None
        'plug_dims': (width, height, thickness) or None
        'plug_x_offset': float or None
        'tools': {tool_num: diameter_mm}
    """
    result = {'stock_dims': None, 'plug_dims': None, 'plug_x_offset': None, 'tools': {}}
    for raw in raw_lines[:30]:  # header is in the first ~30 lines
        line = raw.strip()
        if not line.startswith(';'):
            if line and not line.startswith('G') and not line.startswith('M'):
                continue
            if not line.startswith(';'):
                continue
        # Coaster stock: 75.0 x 75.0 x 16.0 mm
        m = re.search(r'Coaster stock:\s*([\d.]+)\s*x\s*([\d.]+)\s*x\s*([\d.]+)', line)
        if m:
            result['stock_dims'] = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
        # Plug stock: 75.0 x 75.0 x 7.0 mm (offset X=110.0)
        m = re.search(r'Plug stock:\s*([\d.]+)\s*x\s*([\d.]+)\s*x\s*([\d.]+).*offset X=([\d.]+)', line)
        if m:
            result['plug_dims'] = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
            result['plug_x_offset'] = float(m.group(4))
        # Tool T3: 3.175mm dia
        m = re.search(r'Tool T(\d+):\s*([\d.]+)\s*mm\s*dia', line)
        if m:
            result['tools'][int(m.group(1))] = float(m.group(2))
    return result


# ── Material removal engine ──────────────────────────────────────────────────

class MaterialRemovalEngine:
    def __init__(self, stock_x, stock_y, stock_z, resolution=0.05, tools=None):
        """
        Args:
            stock_x, stock_y, stock_z: stock dimensions in mm
            resolution: mm per pixel (default 0.05 for embedded use)
            tools: {tool_number: ToolDef} or None for DEFAULT_TOOLS
        """
        self.stock_z = stock_z
        self.res = resolution
        self.nx = int(stock_x / resolution) + 1
        self.ny = int(stock_y / resolution) + 1
        self.hm = np.full((self.ny, self.nx), stock_z, dtype=np.float32)
        self._tools = tools if tools is not None else DEFAULT_TOOLS
        self._masks = {}
        self._step_sizes = {}

    def _mask(self, tool_num):
        if tool_num in self._masks:
            return self._masks[tool_num]
        td = self._tools.get(tool_num)
        radius = (td.diameter_mm / 2.0) if td else 1.0
        r_px = int(math.ceil(radius / self.res))
        ys, xs = np.mgrid[-r_px:r_px+1, -r_px:r_px+1]
        dist = np.sqrt((xs * self.res)**2 + (ys * self.res)**2)
        m = dist <= radius
        self._masks[tool_num] = (xs[m].astype(np.int32), ys[m].astype(np.int32))
        self._step_sizes[tool_num] = max(radius * 0.5, self.res * 2)
        return self._masks[tool_num]

    def apply(self, seg):
        """Apply one segment to the heightmap (vectorized)."""
        if seg.is_rapid:
            return
        dx_m, dy_m = self._mask(seg.tool)
        step = self._step_sizes[seg.tool]
        length = math.sqrt((seg.x1-seg.x0)**2 + (seg.y1-seg.y0)**2 + (seg.z1-seg.z0)**2)
        n_steps = max(1, int(length / step))

        t = np.linspace(0, 1, n_steps + 1, dtype=np.float32)
        cx = seg.x0 + (seg.x1 - seg.x0) * t
        cy = seg.y0 + (seg.y1 - seg.y0) * t
        cz = seg.z0 + (seg.z1 - seg.z0) * t

        below = cz < self.stock_z
        if not np.any(below):
            return
        cx, cy, cz = cx[below], cy[below], cz[below]

        gx = np.round(cx / self.res).astype(np.int32)
        gy = np.round(cy / self.res).astype(np.int32)

        all_px = gx[:, None] + dx_m[None, :]
        all_py = gy[:, None] + dy_m[None, :]
        all_z = np.broadcast_to(cz[:, None], all_px.shape)

        all_px = all_px.ravel()
        all_py = all_py.ravel()
        all_z = all_z.ravel()
        valid = (all_px >= 0) & (all_px < self.nx) & (all_py >= 0) & (all_py < self.ny)
        all_px, all_py, all_z = all_px[valid], all_py[valid], all_z[valid]

        np.minimum.at(self.hm, (all_py, all_px), all_z)


# ── Simulation runner ────────────────────────────────────────────────────────

def run_simulation(segments, stock_x, stock_y, stock_z, tools=None,
                   resolution=0.05, progress_callback=None, cancel_flag=None,
                   num_checkpoints=0):
    """Run full material removal simulation.

    Args:
        segments: list of Segment objects from parse_gcode()
        stock_x, stock_y, stock_z: stock dimensions in mm
        tools: {tool_number: ToolDef} or None for defaults
        resolution: mm per pixel
        progress_callback: called with percentage (0-100) during simulation
        cancel_flag: callable returning True to abort early
        num_checkpoints: if > 0, save heightmap snapshots at evenly spaced
            intervals.  Returns (heightmap, checkpoints) instead of just
            heightmap, where checkpoints is a list of
            (segment_index, heightmap_copy) tuples.

    Returns:
        heightmap as np.float32 array (ny, nx), or None if cancelled.
        If num_checkpoints > 0, returns (heightmap, checkpoints) or
        (None, []) if cancelled.
    """
    engine = MaterialRemovalEngine(stock_x, stock_y, stock_z, resolution, tools)
    total = len(segments)
    report_interval = max(1, total // 100)

    checkpoints = []
    if num_checkpoints > 0 and total > 0:
        # Compute segment indices at which to save checkpoints
        cp_interval = max(1, total // num_checkpoints)
        next_cp = cp_interval
    else:
        cp_interval = 0
        next_cp = total + 1  # never triggers

    for i, seg in enumerate(segments):
        if cancel_flag and cancel_flag():
            if num_checkpoints > 0:
                return None, []
            return None
        engine.apply(seg)
        if cp_interval > 0 and i + 1 >= next_cp:
            checkpoints.append((i, engine.hm.copy()))
            next_cp += cp_interval
        if progress_callback and (i % report_interval == 0):
            progress_callback(int(100 * i / total))
    if progress_callback:
        progress_callback(100)

    # Always include the final state as the last checkpoint
    if num_checkpoints > 0:
        checkpoints.append((total - 1, engine.hm.copy()))
        return engine.hm, checkpoints
    return engine.hm
