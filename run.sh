#!/bin/bash
# Run the Carvera Unified Controller app
# Creates a virtual environment on first run and installs dependencies

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="$SCRIPT_DIR/venv"

if [ ! -d "$VENV_DIR" ]; then
    echo "Creating virtual environment..."
    python3 -m venv "$VENV_DIR"
    echo "Installing dependencies..."
    "$VENV_DIR/bin/pip" install --upgrade pip
    "$VENV_DIR/bin/pip" install numpy matplotlib scipy svgpathtools numba shapely ezdxf hidapi
fi

exec "$VENV_DIR/bin/python" "$SCRIPT_DIR/UnifiedController_Newest.py" "$@"
