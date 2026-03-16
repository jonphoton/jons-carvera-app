# Jon's Carvera App

A desktop application for controlling and generating GCode for the [Makera Carvera](https://www.makera.com/) desktop CNC machine. Built with Python and Tkinter.

## Features

- **SVG Curve Smoothing** — Load SVG files, smooth curves with minimum radius-of-curvature constraints, and export adjusted SVGs
- **Inlay Coaster GCode Generation** — Generate multi-tool pocketing, surfacing, and contour-cut GCode for wood inlay coasters and plugs
  - Multi-tool support with progressive rest machining
  - Contour-parallel pocket clearing
  - Helical ramp entry and arc-linked surfacing passes
  - Separate coaster, plug, and combined joint GCode output
  - Inlay clearance pass for fit adjustment
- **DXF Probe Hole Generation** — Generate GCode for drilling probe/alignment holes from DXF files
- **Serial Connection** — Direct serial communication with the Carvera controller

## Requirements

- Python 3.8+
- Tkinter (included with most Python installations)

### Optional Dependencies

The app uses feature flags and runs with reduced functionality if optional packages are missing:

- `numpy` — Required for most features
- `matplotlib` — SVG visualization and smoothing
- `scipy` — Curve fitting for SVG smoothing
- `svgpathtools` — SVG file parsing
- `shapely` — Polygon operations for GCode toolpath generation
- `numba` — JIT compilation for performance (optional)
- `ezdxf` — DXF file support
- `hidapi` — USB HID communication

## Usage

```bash
# First run creates a venv and installs dependencies automatically
./run.sh
```

Or manually:

```bash
python3 -m venv venv
venv/bin/pip install numpy matplotlib scipy svgpathtools shapely ezdxf hidapi
venv/bin/python UnifiedController_Newest.py
```

## Configuration

Settings are stored in:
- `~/.carvera_unified_config.json` — Global app settings
- `~/.carvera_coaster_settings.json` — Coaster/inlay parameters and tool configurations

## License

Personal project — use at your own risk.
