#!/usr/bin/env python3
"""
Unified Makera Carvera CNC Control Software
Combines: Manual Control, Probing, A-Axis Leveling, and Curve Fitting
All tabs share a single connection with heartbeat management
"""

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

import os
os.environ['MPLCONFIGDIR'] = os.path.join(os.path.expanduser('~'), '.carvera_matplotlib')

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import socket
import threading
import time
import math
import re
import json
import numpy as np
from typing import List, Tuple, Optional, Callable
from dataclasses import dataclass
from enum import Enum

# Optional imports
try:
    import ezdxf
    HAS_EZDXF = True
except ImportError:
    HAS_EZDXF = False
    print("Warning: ezdxf not installed. DXF export will be disabled.")

try:
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
    from matplotlib.figure import Figure
    from matplotlib.widgets import RadioButtons
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("Warning: matplotlib not installed. Curve fitter tab will be disabled.")

try:
    from scipy.optimize import minimize
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False
    print("Warning: scipy not installed. Automatic fitting will be disabled.")

try:
    import hid
    HAS_HID = True
except ImportError:
    HAS_HID = False
    print("Warning: hid not installed. WHB04B-4 pendant support will be disabled.")
    print("  Install with: pip install hid")

# SVG Smoother imports
try:
    from scipy.interpolate import splprep, splev
    from matplotlib.widgets import Slider, Button
    from matplotlib.collections import LineCollection
    from svgpathtools import parse_path, CubicBezier, Line
    from numba import jit
    import xml.etree.ElementTree as ET
    import subprocess
    HAS_SVG_SMOOTHER = True
except ImportError as e:
    HAS_SVG_SMOOTHER = False
    print(f"Warning: SVG Smoother dependencies not fully installed ({e})")
    print("  Install with: pip install svgpathtools numba")

# Shapely for polygon offset operations (coaster GCode generation)
try:
    from shapely.geometry import Polygon as ShapelyPolygon, MultiPolygon, LineString, MultiLineString, Point as ShapelyPoint
    from shapely.ops import unary_union
    from shapely.validation import make_valid
    HAS_SHAPELY = True
except ImportError:
    HAS_SHAPELY = False
    print("Warning: shapely not installed. Coaster GCode generation will be disabled.")
    print("  Install with: pip install shapely")


# =============================================================================
# Data Classes and Enums
# =============================================================================

class Direction(Enum):
    PLUS_X = "+X"
    MINUS_X = "-X"
    PLUS_Y = "+Y"
    MINUS_Y = "-Y"


@dataclass
class Point:
    x: float
    y: float
    z: float = 0.0


@dataclass
class Circle:
    center: Point
    radius: float


@dataclass
class Wall:
    point: Point
    direction: Direction


@dataclass
class InfiniteWall:
    point: Point
    angle: float


@dataclass
class Polygon:
    points: List[Point]


# =============================================================================
# Custom Exceptions for CNC Operations
# =============================================================================

class CNCError(Exception):
    """Base exception for CNC operations."""
    pass

class ProbeNoContact(CNCError):
    """Probe did not make contact within expected distance."""
    pass

class ProbeCrash(CNCError):
    """Probe triggered unexpectedly during a move (crash detected)."""
    pass

class AlarmError(CNCError):
    """Machine entered alarm state."""
    pass


# =============================================================================
# WHB04B-4 Pendant Support
# =============================================================================

class WHB04B4:
    """
    WHB04B-4 wireless CNC pendant controller.
    Supports step-based jogging via MPG wheel.
    """
    
    VENDOR_ID = 0x10CE
    PRODUCT_ID = 0xEB93
    
    # Step sizes in mm
    STEP_SIZES = [0.001, 0.01, 0.1, 1.0]
    
    # Axis codes from pendant (byte index 5)
    AXIS_MAP = {17: "X", 18: "Y", 19: "Z", 20: "A", 21: "B", 22: "C"}
    
    # Step size codes from pendant (byte index 4)
    # Maps rotary switch position to step size index
    STEP_MAP = {
        0x0D: 0,  # x1 -> 0.001mm
        0x0E: 1,  # x10 -> 0.01mm
        0x0F: 2,  # x100 -> 0.1mm
        0x10: 3,  # Lead/x1000 -> 1.0mm
    }
    def __init__(self, jog_callback: Callable[[str, float], None],
                 step_callback: Callable[[float], None] = None,
                 axis_callback: Callable[[str], None] = None):
        """
        Initialize the WHB04B-4 pendant.
        jog_callback: function(axis, distance) called for each jog step
        step_callback: function(step_size) called when step size changes
        axis_callback: function(axis) called when selected axis changes
        """
        self.jog_callback = jog_callback
        self.step_callback = step_callback
        self.axis_callback = axis_callback
        
        self.device = None
        self.running = False
        self.thread = None
        self.enabled = True
        self._jog_busy = False  # Prevent command flooding
        
        # Current state
        self.current_axis = 'X'
        self.step_index = 1  # Default to 0.01mm
        self.step_size = self.STEP_SIZES[self.step_index]
    
    @staticmethod
    def find_device() -> bool:
        """Check if WHB04B-4 device is available."""
        if not HAS_HID:
            return False
        try:
            # Look for device in enumeration
            for d in hid.enumerate():
                if d['vendor_id'] == WHB04B4.VENDOR_ID and d['product_id'] == WHB04B4.PRODUCT_ID:
                    return True
            return False
        except Exception as e:
            print(f"Pendant search error: {e}")
            return False
    
    def connect(self) -> bool:
        """Connect to the WHB04B-4 pendant."""
        if not HAS_HID:
            print("hid library not available")
            return False
        
        try:
            # Find device path first
            device_path = None
            for d in hid.enumerate():
                if d['vendor_id'] == self.VENDOR_ID and d['product_id'] == self.PRODUCT_ID:
                    device_path = d['path']
                    break
            
            if not device_path:
                print("Pendant not found in enumeration")
                return False
            
            # Try different API styles
            opened = False
            
            # Try hidapi style - Device class
            if not opened and hasattr(hid, 'Device'):
                try:
                    self.device = hid.Device(path=device_path)
                    opened = True
                except Exception as e:
                    print(f"hid.Device failed: {e}")
            
            # Try device() then open_path
            if not opened and hasattr(hid, 'device'):
                try:
                    self.device = hid.device()
                    self.device.open_path(device_path)
                    opened = True
                except Exception as e:
                    print(f"open_path failed: {e}")
            
            # Last resort: open by vid/pid
            if not opened and hasattr(hid, 'device'):
                try:
                    self.device = hid.device()
                    self.device.open(self.VENDOR_ID, self.PRODUCT_ID)
                    opened = True
                except Exception as e:
                    print(f"open by vid/pid failed: {e}")
            
            if not opened:
                print("All connection methods failed")
                return False
            
            # Set non-blocking if method exists
            if hasattr(self.device, 'set_nonblocking'):
                self.device.set_nonblocking(True)
            elif hasattr(self.device, 'nonblocking'):
                self.device.nonblocking = True
                
            print("WHB04B-4 connected")
            return True
        except Exception as e:
            print(f"Failed to open pendant: {e}")
            self.device = None
            return False
    
    def disconnect(self):
        """Disconnect from the pendant."""
        self.stop()
        if self.device:
            try:
                self.device.close()
            except:
                pass
            self.device = None
    
    def start(self):
        """Start the pendant polling thread."""
        if not self.device or self.running:
            return
        
        self.running = True
        self.thread = threading.Thread(target=self._poll_loop, daemon=True)
        self.thread.start()
        print("WHB04B-4 polling started")
    
    def stop(self):
        """Stop the pendant polling thread."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
            self.thread = None
    
    def set_enabled(self, enabled: bool):
        """Enable/disable jog output."""
        self.enabled = enabled
    
    def set_step_size(self, size: float):
        """Set step size manually."""
        if size in self.STEP_SIZES:
            self.step_index = self.STEP_SIZES.index(size)
            self.step_size = size
        else:
            # Find closest
            self.step_size = min(self.STEP_SIZES, key=lambda x: abs(x - size))
            self.step_index = self.STEP_SIZES.index(self.step_size)
    
    def _poll_loop(self):
        """Main polling loop for pendant input."""
        while self.running:
            try:
                if self.device:
                    # Try read with short timeout (hidapi Device style)
                    try:
                        data = self.device.read(64, timeout_ms=5)
                    except TypeError:
                        # Fallback: read without timeout (hid.device style)
                        data = self.device.read(64)
                    
                    if data and len(data) >= 7:
                        self._process_packet(data)
                        continue  # Check for more data immediately
                time.sleep(0.005)  # 5ms sleep when no data
            except Exception as e:
                print(f"WHB04B-4 read error: {e}")
                time.sleep(0.5)
    
    def _process_packet(self, data):
        """Process a packet from the pendant."""
        # Step size is at byte index 4
        step_code = data[4]
        if step_code in self.STEP_MAP:
            new_step_index = self.STEP_MAP[step_code]
            if new_step_index != self.step_index:
                self.step_index = new_step_index
                self.step_size = self.STEP_SIZES[new_step_index]
                if self.step_callback:
                    try:
                        self.step_callback(self.step_size)
                    except:
                        pass
        
        # Axis is at byte index 5
        axis_code = data[5]
        axis = self.AXIS_MAP.get(axis_code, None)
        
        # Jog delta is at byte index 6 (signed byte)
        jog_delta = data[6] if data[6] < 128 else data[6] - 256
        
        if axis and axis != self.current_axis:
            self.current_axis = axis
            if self.axis_callback:
                try:
                    self.axis_callback(axis)
                except:
                    pass
        
        # Execute jog if there's movement and we're enabled
        if jog_delta != 0 and self.enabled and axis in ['X', 'Y', 'Z', 'A']:
            distance = jog_delta * self.step_size
            # Run jog in separate thread to avoid blocking HID polling
            if not self._jog_busy:
                threading.Thread(target=self._do_jog, args=(axis, distance), daemon=True).start()
    
    def _do_jog(self, axis: str, distance: float):
        """Execute jog command in separate thread."""
        self._jog_busy = True
        try:
            self.jog_callback(axis, distance)
        except Exception as e:
            print(f"Jog callback error: {e}")
        finally:
            self._jog_busy = False


# =============================================================================
# Shared Carvera Connection Class
# =============================================================================

class SharedCarveraConnection:
    """
    Unified connection handler for Carvera CNC with heartbeat management.
    Shared across all tabs of the application.
    """
    
    PROBE_TOOL_NUMBER = 999990
    
    def __init__(self):
        self.ip = "192.168.5.4"
        self.port = 2222
        self.sock = None
        self.connected = False
        
        # WCS position (work coordinates)
        self.position = Point(0, 0, 0)
        self.a_position = 0.0
        
        # Machine position (absolute machine coordinates)
        self.machine_position = Point(0, 0, 0)
        self.machine_a_position = 0.0
        
        # Tool and WCS info
        self.tool_number = 0
        self.wcs_rotation = 0.0  # G54 rotation in degrees
        self.wcs_offset_x = 0.0  # G54 X offset (machine = WCS + offset)
        self.wcs_offset_y = 0.0  # G54 Y offset
        self.wcs_offset_z = 0.0  # G54 Z offset
        
        # Probing settings (shared across all tabs)
        self.probe_z = 0.0           # Z height for probing operations
        self.probe_z_set = False     # Must be set before probing is allowed
        self.retract_height = 5.0    # Height above probe_z to retract to
        self.slow_probe_speed = 50.0 # Slow probing feedrate (mm/min)
        self.max_probe_distance = 10.0  # Maximum probe distance (mm)
        self.probe_tip_diameter = 3.0   # Probe tip diameter (mm) - loaded from machine
        
        # Heartbeat management
        self.heartbeat_thread = None
        self.heartbeat_running = False
        self.heartbeat_lock = threading.Lock()
        self.operation_in_progress = False
        
        # Callbacks for status updates
        self.status_callbacks: List[Callable] = []
        self.position_callbacks: List[Callable] = []
    
    def add_status_callback(self, callback: Callable):
        """Register a callback for connection status changes."""
        self.status_callbacks.append(callback)
    
    def add_position_callback(self, callback: Callable):
        """Register a callback for position updates."""
        self.position_callbacks.append(callback)
    
    def notify_status(self):
        """Notify all registered callbacks of status change."""
        for cb in self.status_callbacks:
            try:
                cb(self.connected)
            except Exception as e:
                print(f"Status callback error: {e}")
    
    def notify_position(self):
        """Notify all registered callbacks of position update."""
        for cb in self.position_callbacks:
            try:
                cb(self.position, self.a_position)
            except Exception as e:
                print(f"Position callback error: {e}")
    
    def connect(self, ip: str = None, port: int = None) -> bool:
        """Establish connection to Carvera."""
        if ip:
            self.ip = ip
        if port:
            self.port = port
        
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            print(f"Connecting to {self.ip}:{self.port}...")
            self.sock.connect((self.ip, self.port))
            
            # Wait for initial greeting
            time.sleep(0.5)
            try:
                initial = self.sock.recv(4096).decode('utf-8')
                print(f"Initial response: {initial.strip()}")
            except socket.timeout:
                pass
            
            self.connected = True
            print("Connected successfully")
            
            # Initialize coordinate system
            self.send_command("G54")
            self.send_command("G90")
            
            # Query initial WCS rotation
            self.get_wcs_rotation()
            
            # Load probe tip diameter from machine config
            self.load_probe_tip_diameter()
            
            # Start heartbeat
            self.start_heartbeat()
            
            self.notify_status()
            return True
            
        except Exception as e:
            print(f"Connection failed: {e}")
            self.connected = False
            self.notify_status()
            return False
    
    def disconnect(self):
        """Disconnect from Carvera."""
        self.stop_heartbeat()
        
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            self.sock = None
        
        self.connected = False
        self.notify_status()
        print("Disconnected")
    
    def start_heartbeat(self):
        """Start the heartbeat thread."""
        if not self.heartbeat_running:
            self.heartbeat_running = True
            self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
            self.heartbeat_thread.start()
    
    def stop_heartbeat(self):
        """Stop the heartbeat thread."""
        self.heartbeat_running = False
        if self.heartbeat_thread and self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join(timeout=2.0)
            if self.heartbeat_thread.is_alive():
                print("=== WARNING: Heartbeat thread did not stop cleanly")
    
    def _heartbeat_loop(self):
        """Background heartbeat loop."""
        while self.heartbeat_running and self.connected:
            time.sleep(0.5)
            
            # Check again after sleep
            if not self.heartbeat_running or not self.sock:
                break
            
            try:
                # Use timeout on lock acquisition to be interruptible
                lock_acquired = self.heartbeat_lock.acquire(timeout=0.5)
                if not lock_acquired:
                    continue  # Try again next iteration
                
                try:
                    # Check flag again after acquiring lock
                    if not self.heartbeat_running or self.operation_in_progress:
                        continue
                    
                    self.sock.settimeout(0.5)
                    self.sock.sendall("?\n".encode('utf-8'))
                    try:
                        response = self.sock.recv(4096).decode('utf-8')
                        self._parse_status(response)
                    except socket.timeout:
                        pass
                    self.sock.settimeout(5.0)
                finally:
                    self.heartbeat_lock.release()
            except (BrokenPipeError, ConnectionResetError, OSError) as e:
                print(f"Heartbeat connection error: {e}")
                # Connection is dead - mark as disconnected
                self.connected = False
                self.heartbeat_running = False
                try:
                    if self.sock:
                        self.sock.close()
                except:
                    pass
                self.sock = None
                # Notify callbacks that we're disconnected (schedule on main thread if possible)
                self.notify_status()
                break
            except Exception as e:
                print(f"Heartbeat error: {e}")
                break
    
    def _parse_status(self, response: str):
        """Parse status response and update position."""
        try:
            # Parse WPos (work coordinates)
            if 'WPos:' in response:
                pos_str = response.split('WPos:')[1].split('|')[0]
                coords = pos_str.split(',')
                self.position = Point(
                    float(coords[0]),
                    float(coords[1]),
                    float(coords[2])
                )
                if len(coords) > 3:
                    self.a_position = float(coords[3])
            
            # Parse MPos (machine coordinates)
            if 'MPos:' in response:
                mpos_str = response.split('MPos:')[1].split('|')[0]
                mcoords = mpos_str.split(',')
                self.machine_position = Point(
                    float(mcoords[0]),
                    float(mcoords[1]),
                    float(mcoords[2])
                )
                if len(mcoords) > 3:
                    self.machine_a_position = float(mcoords[3])
            
            # Parse tool number (T:999990 format)
            if 'T:' in response:
                try:
                    t_str = response.split('T:')[1].split('|')[0].split(',')[0].split()[0]
                    self.tool_number = int(t_str)
                except:
                    pass
            
            self.notify_position()
        except Exception as e:
            pass  # Silent fail for status parsing
    
    def get_wcs_offset(self) -> tuple:
        """Query current G54 offset and rotation. Returns (x, y, z, rotation)."""
        if not self.connected:
            return (self.wcs_offset_x, self.wcs_offset_y, self.wcs_offset_z, self.wcs_rotation)
        try:
            response = self.send_command("$#")
            
            # Format is [G54:X,Y,Z,A,B,R] where R (rotation) is the 6th value
            g54_match = re.search(r'\[G54:([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+)\]', response)
            if g54_match:
                self.wcs_offset_x = float(g54_match.group(1))
                self.wcs_offset_y = float(g54_match.group(2))
                self.wcs_offset_z = float(g54_match.group(3))
                self.wcs_rotation = float(g54_match.group(6))
            
        except Exception as e:
            print(f"Error parsing WCS offset: {e}")
        return (self.wcs_offset_x, self.wcs_offset_y, self.wcs_offset_z, self.wcs_rotation)
    
    def get_wcs_rotation(self) -> float:
        """Query current G54 rotation angle (legacy method)."""
        self.get_wcs_offset()
        return self.wcs_rotation
    
    def load_probe_tip_diameter(self) -> Optional[float]:
        """Load probe tip diameter from Carvera's configuration."""
        try:
            response = self.send_command("config-get sd zprobe.probe_tip_diameter", timeout=5.0)
            # Response format: "sd: zprobe.probe_tip_diameter is set to 1.646"
            match = re.search(r'is set to\s+([\d.]+)', response)
            if match:
                diameter = float(match.group(1))
                print(f"Loaded probe tip diameter from machine: {diameter}mm")
                self.probe_tip_diameter = diameter
                return diameter
            else:
                print(f"Could not parse probe tip diameter from response: {response.strip()[:100]}")
        except Exception as e:
            print(f"Error reading probe tip diameter: {e}")
        return None
    
    def is_probe_selected(self) -> bool:
        """Check if the 3D probe tool is currently selected."""
        return self.tool_number == self.PROBE_TOOL_NUMBER
    
    def check_probe_ready(self, require_probe_z: bool = True) -> bool:
        """
        Check if probe is ready for probing operations.
        Returns True if ready, False otherwise (and shows error dialog).
        
        Args:
            require_probe_z: If True, also checks that probing height has been set.
        """
        print("=== check_probe_ready called")
        if not self.connected:
            messagebox.showwarning("Not Connected", "Please connect first")
            return False
        
        # Check if probing height has been set
        if require_probe_z and not self.probe_z_set:
            messagebox.showerror(
                "Probing Height Not Set",
                "You must set the probing height before probing.\n\n"
                "Go to the Settings panel and click 'Set' "
                "after positioning the probe at the desired probing height."
            )
            return False
        
        # Refresh status to get current tool
        print("=== Checking current tool...")
        try:
            response = self.send_command("?")
            self._parse_status(response)
            print(f"=== Tool check complete, tool={self.tool_number}")
        except Exception as e:
            print(f"=== Tool check failed: {e}")
        
        if not self.is_probe_selected():
            messagebox.showerror(
                "Wrong Tool Selected",
                f"3D Probe (tool 999990) must be selected for probing operations.\n\n"
                f"Current tool: {self.tool_number}\n\n"
                f"Please select the 3D probe before probing."
            )
            return False
        
        print("=== Probe ready")
        return True
    
    def send_command(self, command: str, timeout: float = 5.0, skip_alarm_check: bool = False) -> str:
        """Send G-code command and receive response with timeout."""
        if not self.connected or not self.sock:
            raise ConnectionError("Not connected to Carvera")
        
        # During operations, heartbeat is stopped so we don't need the lock
        # Outside operations, we need the lock to prevent conflicts with heartbeat
        if self.operation_in_progress:
            return self._send_command_internal(command, timeout, skip_alarm_check)
        else:
            # Try to acquire lock with timeout to avoid deadlock
            lock_acquired = self.heartbeat_lock.acquire(timeout=3.0)
            if not lock_acquired:
                print(f"=== WARNING: Could not acquire lock for command '{command}' after 3s - forcing")
                # Force stop heartbeat and proceed
                self.heartbeat_running = False
                time.sleep(0.5)
                return self._send_command_internal(command, timeout, skip_alarm_check)
            try:
                return self._send_command_internal(command, timeout, skip_alarm_check)
            finally:
                self.heartbeat_lock.release()
    
    def _send_command_internal(self, command: str, timeout: float, skip_alarm_check: bool = False) -> str:
        """Internal send command without lock handling."""
        try:
            self.sock.settimeout(timeout)
            cmd_str = command.strip() + "\n"
            self.sock.sendall(cmd_str.encode('utf-8'))
            time.sleep(0.1)
            response = self.sock.recv(4096).decode('utf-8')
            print(f"Sent: {command} | Response: {response.strip()[:100]}")
            
            # Check for alarm state (skip for unlock commands)
            if not skip_alarm_check and ('ALARM' in response.upper() or '<Alarm' in response):
                raise AlarmError(f"Machine in alarm state: {response.strip()[:200]}")
            
            return response
        except socket.timeout:
            raise TimeoutError(f"Command '{command}' timed out after {timeout}s")
        except AlarmError:
            raise  # Re-raise alarm errors
        except Exception as e:
            raise ConnectionError(f"Command failed: {e}")
    
    def check_status(self) -> str:
        """Check machine status and return state. Raises AlarmError if in alarm."""
        response = self.send_command("?")
        if '<Alarm' in response:
            raise AlarmError(f"Machine in alarm state: {response.strip()[:200]}")
        return response
    
    def wait_for_ok(self, timeout: float = 10.0) -> bool:
        """Wait for 'ok' response (simplified - response already in send_command)."""
        return True
    
    def wait_for_idle(self, timeout: float = 30.0, allow_alarm: bool = False) -> bool:
        """Wait for machine to become idle. Raises AlarmError if alarm detected (unless allow_alarm=True)."""
        start = time.time()
        while time.time() - start < timeout:
            try:
                response = self.send_command("?", skip_alarm_check=True)
                if '<Idle' in response:
                    return True
                if '<Alarm' in response:
                    if allow_alarm:
                        return False  # Return False but don't raise
                    raise AlarmError(f"Machine entered alarm state: {response.strip()[:200]}")
            except AlarmError:
                raise  # Re-raise alarm errors
            except (BrokenPipeError, ConnectionResetError, OSError) as e:
                raise ConnectionError(f"Connection lost: {e}")
            except Exception as e:
                print(f"wait_for_idle error: {e}")
                pass
            time.sleep(0.1)
        raise TimeoutError(f"wait_for_idle timed out after {timeout}s")
    
    def get_position(self) -> Point:
        """Query and return current position."""
        try:
            response = self.send_command("?")
            self._parse_status(response)
        except:
            pass
        return self.position
    
    def begin_operation(self):
        """Mark start of an operation (pauses heartbeat position queries)."""
        print(f"=== BEGIN_OPERATION called, current state: operation_in_progress={self.operation_in_progress}")
        
        # Check if already in an operation
        if self.operation_in_progress:
            print("=== ERROR: Another operation already in progress!")
            raise CNCError("Another operation is already in progress")
        
        # Check if socket is still valid
        if not self.sock or not self.connected:
            raise ConnectionError("Not connected to machine")
        
        self.operation_in_progress = True
        print("=== Stopping heartbeat...")
        self.stop_heartbeat()
        print("=== Heartbeat stopped")
        
        # Small extra delay to ensure thread has fully released resources
        time.sleep(0.1)
        
        # Flush any stale data from socket buffer
        print("=== Flushing socket buffer...")
        try:
            self.sock.setblocking(False)
            while True:
                try:
                    stale = self.sock.recv(4096)
                    if not stale:
                        break
                    print(f"=== Flushed stale data: {stale[:100]}")
                except BlockingIOError:
                    break  # No more data
                except (BrokenPipeError, ConnectionResetError, OSError) as e:
                    print(f"=== Connection error during flush: {e}")
                    self.operation_in_progress = False
                    self.connected = False
                    self.notify_status()
                    raise ConnectionError(f"Connection lost: {e}")
                except:
                    break
        except ConnectionError:
            raise
        except:
            pass
        finally:
            try:
                self.sock.setblocking(True)
                self.sock.settimeout(5.0)
            except:
                pass
        
        # Wait for machine to be idle before starting operation
        print("=== Waiting for machine to be idle...")
        try:
            self.wait_for_idle(timeout=10.0)
            print("=== Machine is idle, proceeding with operation")
        except TimeoutError:
            print("=== ERROR: Machine not idle after 10s timeout")
            self.operation_in_progress = False
            self.start_heartbeat()
            raise CNCError("Machine not idle - cannot start operation")
        except AlarmError:
            print("=== ERROR: Machine in alarm state")
            self.operation_in_progress = False
            self.start_heartbeat()
            raise
        except ConnectionError:
            print("=== ERROR: Connection lost")
            self.operation_in_progress = False
            self.connected = False
            self.notify_status()
            raise
    
    def end_operation(self):
        """Mark end of an operation (resumes heartbeat)."""
        print(f"=== END_OPERATION called")
        self.operation_in_progress = False
        time.sleep(0.1)
        self.start_heartbeat()
    
    def move_to(self, x: float, y: float, z: float, feedrate: int = 1000):
        """Move to specified position. Raises ProbeCrash if probe triggers during move."""
        gcode = f"G0 X{x:.4f} Y{y:.4f} Z{z:.4f} F{feedrate}"
        response = self.send_command(gcode)
        # Check for probe trigger during move (crash)
        if 'PRB:' in response:
            raise ProbeCrash(f"Probe triggered during move to ({x:.3f}, {y:.3f}, {z:.3f}) - possible crash!")
        return response
    
    def move_xy(self, x: float, y: float, feedrate: int = 1000):
        """Move XY only. Raises ProbeCrash if probe triggers during move."""
        gcode = f"G0 X{x:.4f} Y{y:.4f} F{feedrate}"
        response = self.send_command(gcode)
        # Check for probe trigger during move (crash)
        if 'PRB:' in response:
            raise ProbeCrash(f"Probe triggered during XY move to ({x:.3f}, {y:.3f}) - possible crash!")
        return response
    
    def move_z(self, z: float, feedrate: int = 500):
        """Move Z only. Raises ProbeCrash if probe triggers during move."""
        response = self.send_command(f"G0 Z{z:.4f} F{feedrate}")
        # Check for probe trigger during move (crash)
        if 'PRB:' in response:
            raise ProbeCrash(f"Probe triggered during Z move to {z:.3f} - possible crash!")
        return response
    
    def move_a(self, a: float, feedrate: int = 500):
        """Move A axis only."""
        return self.send_command(f"G0 A{a:.4f} F{feedrate}")
    
    def probe_z(self, max_depth: float = -10.0, feedrate: float = 50.0) -> Optional[float]:
        """Probe Z axis downward, return Z at contact."""
        # Calculate timeout based on probe distance and feedrate
        probe_dist = abs(max_depth)
        probe_timeout = max(15.0, (probe_dist / feedrate) * 60.0 + 5.0)
        
        response = self.send_command(f"G38.2 Z{max_depth:.4f} F{feedrate}", timeout=probe_timeout)
        
        try:
            if 'PRB:' in response:
                pos_str = response.split('PRB:')[1].split(':')[0]
                coords = pos_str.split(',')
                return float(coords[2])
        except:
            pass
        return None
    
    def probe_direction(self, direction: Direction, distance: float = None, 
                       probe_offset: float = 0.0) -> Point:
        """Probe in specified direction with double-tap. Raises ProbeNoContact if no contact made.
        
        Args:
            direction: Direction to probe
            distance: Maximum probe distance (uses max_probe_distance if None)
            probe_offset: Offset to apply to probed position (e.g., probe tip radius)
        """
        if distance is None:
            distance = self.max_probe_distance
        
        backoff = 0.5
        slow_speed = self.slow_probe_speed  # Use unified slow probe speed
        
        # Calculate timeouts based on distance and feedrate
        # Fast probe: F200 = 200mm/min, so time = distance/200 * 60 seconds
        # Slow probe uses slow_speed, probing ~0.6mm
        fast_timeout = max(10.0, (distance / 200.0) * 60.0 + 5.0)  # At least 10s, plus 5s buffer
        slow_timeout = max(10.0, (1.0 / slow_speed) * 60.0 + 5.0)  # Time for ~1mm at slow speed
        
        # Determine movement
        if direction == Direction.PLUS_X:
            gcode_fast = f"G38.2 X{distance:.4f} F200"
            gcode_slow = f"G38.2 X{distance:.4f} F{slow_speed:.0f}"
            backoff_gcode = f"G0 X{-backoff:.4f} F500"
            retract_gcode = f"G0 X{-backoff:.4f} F500"
            dir_name = "+X"
        elif direction == Direction.MINUS_X:
            gcode_fast = f"G38.2 X{-distance:.4f} F200"
            gcode_slow = f"G38.2 X{-distance:.4f} F{slow_speed:.0f}"
            backoff_gcode = f"G0 X{backoff:.4f} F500"
            retract_gcode = f"G0 X{backoff:.4f} F500"
            dir_name = "-X"
        elif direction == Direction.PLUS_Y:
            gcode_fast = f"G38.2 Y{distance:.4f} F200"
            gcode_slow = f"G38.2 Y{distance:.4f} F{slow_speed:.0f}"
            backoff_gcode = f"G0 Y{-backoff:.4f} F500"
            retract_gcode = f"G0 Y{-backoff:.4f} F500"
            dir_name = "+Y"
        else:  # MINUS_Y
            gcode_fast = f"G38.2 Y{-distance:.4f} F200"
            gcode_slow = f"G38.2 Y{-distance:.4f} F{slow_speed:.0f}"
            backoff_gcode = f"G0 Y{backoff:.4f} F500"
            retract_gcode = f"G0 Y{backoff:.4f} F500"
            dir_name = "-Y"
        
        # Fast probe (with extended timeout)
        response_fast = self.send_command(gcode_fast, timeout=fast_timeout)
        
        # Wait for probe to physically release from surface before backing off
        time.sleep(0.3)
        
        # Check if fast probe made contact (PRB:x,y,z:1 means contact, :0 means no contact)
        if 'PRB:' in response_fast:
            # Check for :0 at end indicating no contact
            prb_part = response_fast.split('PRB:')[1]
            if ':0' in prb_part or prb_part.strip().endswith(':0'):
                raise ProbeNoContact(f"Fast probe {dir_name} did not make contact within {distance}mm")
        
        # Backoff (check for crash)
        self.send_command("G91")
        backoff_response = self.send_command(backoff_gcode)
        if 'PRB:' in backoff_response:
            raise ProbeCrash(f"Probe triggered during backoff {dir_name}")
        self.send_command("G90")
        self.wait_for_idle()
        
        # Slow probe (with extended timeout)
        response = self.send_command(gcode_slow, timeout=slow_timeout)
        
        # Wait for probe to physically release from surface before retracting
        # This prevents crash detection from triggering during retract
        time.sleep(0.3)
        
        # Check if slow probe made contact
        if 'PRB:' in response:
            prb_part = response.split('PRB:')[1]
            if ':0' in prb_part or prb_part.strip().endswith(':0'):
                raise ProbeNoContact(f"Slow probe {dir_name} did not make contact")
            
            # Get work position after probe
            status = self.send_command("?")
            if 'WPos:' in status:
                pos_str = status.split('WPos:')[1].split('|')[0]
                coords = pos_str.split(',')
                x, y, z = float(coords[0]), float(coords[1]), float(coords[2])
                
                # Apply probe offset
                if direction == Direction.PLUS_X:
                    x += probe_offset
                elif direction == Direction.MINUS_X:
                    x -= probe_offset
                elif direction == Direction.PLUS_Y:
                    y += probe_offset
                else:
                    y -= probe_offset
                
                # Retract (check for crash)
                self.send_command("G91")
                retract_response = self.send_command(retract_gcode)
                if 'PRB:' in retract_response:
                    raise ProbeCrash(f"Probe triggered during retract {dir_name}")
                self.send_command("G90")
                self.wait_for_idle()
                
                return Point(x, y, z)
        
        raise ProbeNoContact(f"Probe {dir_name} failed - no valid response")


# =============================================================================
# Connection Tab
# =============================================================================

class ConnectionTab(ttk.Frame):
    """Tab for managing Carvera connection."""
    
    def __init__(self, parent, connection: SharedCarveraConnection):
        super().__init__(parent)
        self.connection = connection
        self.config_file = os.path.expanduser("~/.carvera_unified_config.json")
        
        self.setup_ui()
        self.load_config()
        
        # Register for button text update
        self.connection.add_status_callback(self.on_status_change)
    
    def setup_ui(self):
        # Connection settings
        conn_frame = ttk.LabelFrame(self, text="Connection Settings", padding=10)
        conn_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(conn_frame, text="IP Address:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.ip_entry = ttk.Entry(conn_frame, width=20)
        self.ip_entry.grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=2, sticky=tk.W, padx=(20, 0), pady=5)
        self.port_entry = ttk.Entry(conn_frame, width=10)
        self.port_entry.grid(row=0, column=3, padx=5, pady=5)
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=4, padx=20, pady=5)
        
        # Machine info
        info_frame = ttk.LabelFrame(self, text="Instructions", padding=10)
        info_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        info_text = """
All functions share this connection. Connect here first, then use any tab.

Ensure that the 3D probe is installed and calibrated.

Before probing:
1. Jog the probe to the desired probing Z height
2. Click "Set" in the Settings panel (above) to set the probing height

Tips:
- Make sure your Carvera is powered on and connected to the network
- The default port is 2222 for most Carvera machines
"""
        info_label = ttk.Label(info_frame, text=info_text, justify=tk.LEFT)
        info_label.pack(anchor=tk.W)
    
    def toggle_connection(self):
        if self.connection.connected:
            self.connection.disconnect()
            self.connect_btn.config(text="Connect")
        else:
            ip = self.ip_entry.get()
            try:
                port = int(self.port_entry.get())
            except ValueError:
                messagebox.showerror("Invalid Port", "Please enter a valid port number")
                return
            
            self.save_config()
            
            if self.connection.connect(ip, port):
                self.connect_btn.config(text="Disconnect")
            else:
                messagebox.showerror("Connection Error", 
                                   f"Failed to connect to {ip}:{port}")
    
    def on_status_change(self, connected: bool):
        if connected:
            self.connect_btn.config(text="Disconnect")
        else:
            self.connect_btn.config(text="Connect")
            # Reset probe_z_set when disconnected (main app will update UI)
            self.connection.probe_z_set = False
    
    def save_config(self):
        config = {}
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    config = json.load(f)
            except:
                pass
        config["ip"] = self.ip_entry.get()
        config["port"] = self.port_entry.get()
        
        try:
            with open(self.config_file, 'w') as f:
                json.dump(config, f)
        except:
            pass
    
    def load_config(self):
        default_ip = "192.168.5.4"
        default_port = "2222"
        
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    config = json.load(f)
                    default_ip = config.get("ip", default_ip)
                    default_port = config.get("port", default_port)
            except:
                pass
        
        self.ip_entry.insert(0, default_ip)
        self.port_entry.insert(0, default_port)

# =============================================================================
# Coordinate Measuring Tab
# =============================================================================

class ControlProbingTab(ttk.Frame):
    """Combined tab for manual control and probing operations."""
    
    def __init__(self, parent, connection: SharedCarveraConnection):
        super().__init__(parent)
        self.connection = connection
        self.config_file = os.path.expanduser("~/.carvera_unified_config.json")
        self.position_marker = None
        
        # View settings for canvas (all in WCS coordinates)
        # Machine envelope is 360x240mm, we'll center on it once connected
        self.zoom_factor = 1.0
        self.center_x = 0.0
        self.center_y = 0.0
        self.work_area_width = 400.0   # Slightly larger than envelope to show it all
        self.work_area_height = 280.0
        
        # Drag state
        self.drag_start_x = None
        self.drag_start_y = None
        self.zoom_box = None
        
        # Features (for probing results and dropped points)
        self.circles: List[Circle] = []
        self.walls: List[Wall] = []
        self.infinite_walls: List[InfiniteWall] = []
        self.polygons: List[Polygon] = []
        self.points: List[Point] = []
        
        self.setup_ui()
        self.load_probe_config()
        
        # Register for position updates
        self.connection.add_position_callback(self.on_position_update)
        
        # Register for status changes to reset view when connected
        self.connection.add_status_callback(self.on_connection_status_change)
    
    def on_connection_status_change(self, connected: bool):
        """Called when connection status changes."""
        if connected:
            # Reset view to show full work envelope now that we have WCS offset
            self.after(500, self.reset_view)  # Slight delay to ensure offset is populated
            # Update probe tip diameter from machine config
            self.after(100, self.update_probe_tip_from_connection)
    
    def update_probe_tip_from_connection(self):
        """Update probe tip entry from connection's loaded value."""
        if self.connection.connected:
            diameter = self.connection.probe_tip_diameter
            # Need to set state to normal to update, then back to readonly
            self.probe_tip_entry.config(state='normal')
            self.probe_tip_entry.delete(0, tk.END)
            self.probe_tip_entry.insert(0, f"{diameter:.2f}")
            self.probe_tip_entry.config(state='readonly')
            print(f"Updated probe tip entry to {diameter}mm from machine config")
    
    def load_probe_config(self):
        """Load probe settings from config (except probe tip diameter which comes from machine)."""
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    config = json.load(f)
                    # Note: probe_tip_diameter is NOT loaded from config - it comes from the machine
                    if "probe_dist" in config:
                        self.probe_dist_entry.delete(0, tk.END)
                        self.probe_dist_entry.insert(0, config["probe_dist"])
                    if "wall_width" in config:
                        self.wall_width_entry.delete(0, tk.END)
                        self.wall_width_entry.insert(0, config["wall_width"])
                    if "bore_radius" in config:
                        self.bore_radius_entry.delete(0, tk.END)
                        self.bore_radius_entry.insert(0, config["bore_radius"])
                    if "radial_angles" in config:
                        self.radial_angles_entry.delete(0, tk.END)
                        self.radial_angles_entry.insert(0, config["radial_angles"])
            except:
                pass
    
    def save_probe_config(self):
        """Save probe settings to config (except probe tip diameter which comes from machine)."""
        try:
            config = {}
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r') as f:
                    config = json.load(f)
            # Note: probe_tip_diameter is NOT saved - it comes from the machine
            config["probe_dist"] = self.probe_dist_entry.get()
            config["wall_width"] = self.wall_width_entry.get()
            config["bore_radius"] = self.bore_radius_entry.get()
            config["radial_angles"] = self.radial_angles_entry.get()
            with open(self.config_file, 'w') as f:
                json.dump(config, f, indent=2)
        except:
            pass
    
    def setup_ui(self):
        # Main container
        main_frame = ttk.Frame(self)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Left: Controls
        left_frame = ttk.Frame(main_frame, width=324)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
        left_frame.pack_propagate(False)
        
        # Drop Point button
        ttk.Button(left_frame, text="Drop Point Here", 
                  command=self.drop_point).pack(fill=tk.X, pady=5)
        
        # Probe parameters
        param_frame = ttk.LabelFrame(left_frame, text="Probe Parameters", padding=5)
        param_frame.pack(fill=tk.X, pady=5)
        
        param_grid = ttk.Frame(param_frame)
        param_grid.pack(fill=tk.X)
        
        ttk.Label(param_grid, text="Tip Ø:").grid(row=0, column=0, sticky=tk.W)
        self.probe_tip_entry = ttk.Entry(param_grid, width=7)
        self.probe_tip_entry.insert(0, "3.0")
        self.probe_tip_entry.config(state='readonly')  # Read-only - comes from machine
        self.probe_tip_entry.grid(row=0, column=1, padx=2)
        
        ttk.Label(param_grid, text="Dist:").grid(row=0, column=2, sticky=tk.W, padx=(8,0))
        self.probe_dist_entry = ttk.Entry(param_grid, width=7)
        self.probe_dist_entry.insert(0, "10.0")
        self.probe_dist_entry.grid(row=0, column=3, padx=2)
        
        ttk.Label(param_grid, text="Bore Ø:").grid(row=1, column=0, sticky=tk.W)
        self.bore_radius_entry = ttk.Entry(param_grid, width=7)
        self.bore_radius_entry.insert(0, "10.0")
        self.bore_radius_entry.grid(row=1, column=1, padx=2)
        
        ttk.Label(param_grid, text="Wall W:").grid(row=1, column=2, sticky=tk.W, padx=(8,0))
        self.wall_width_entry = ttk.Entry(param_grid, width=7)
        self.wall_width_entry.insert(0, "10.0")
        self.wall_width_entry.grid(row=1, column=3, padx=2)
        
        ttk.Label(param_grid, text="Radial N:").grid(row=2, column=0, sticky=tk.W)
        self.radial_angles_entry = ttk.Entry(param_grid, width=7)
        self.radial_angles_entry.insert(0, "8")
        self.radial_angles_entry.grid(row=2, column=1, padx=2)
        
        # Note about probe tip
        ttk.Label(param_frame, text="(Tip Ø loaded from machine)", 
                 font=("TkDefaultFont", 8), foreground="gray").pack(anchor=tk.W)
        
        # Direction
        dir_frame = ttk.LabelFrame(left_frame, text="Probe Direction", padding=5)
        dir_frame.pack(fill=tk.X, pady=5)
        
        self.probe_direction = tk.StringVar(value="+X")
        
        d_frame = ttk.Frame(dir_frame)
        d_frame.pack()
        
        ttk.Radiobutton(d_frame, text="+Y", variable=self.probe_direction,
                       value="+Y").grid(row=0, column=1, pady=1)
        ttk.Radiobutton(d_frame, text="-X", variable=self.probe_direction,
                       value="-X").grid(row=1, column=0, padx=3)
        ttk.Radiobutton(d_frame, text="+X", variable=self.probe_direction,
                       value="+X").grid(row=1, column=2, padx=3)
        ttk.Radiobutton(d_frame, text="-Y", variable=self.probe_direction,
                       value="-Y").grid(row=2, column=1, pady=1)
        
        # Probe actions
        action_frame = ttk.LabelFrame(left_frame, text="Probe Actions", padding=5)
        action_frame.pack(fill=tk.X, pady=5)
        
        btn_grid = ttk.Frame(action_frame)
        btn_grid.pack(fill=tk.X)
        
        ttk.Button(btn_grid, text="Probe Bore",
                  command=self.probe_bore).grid(row=0, column=0, padx=2, pady=2, sticky=tk.EW)
        ttk.Button(btn_grid, text="Dir Probe",
                  command=self.probe_single).grid(row=0, column=1, padx=2, pady=2, sticky=tk.EW)
        ttk.Button(btn_grid, text="Probe Wall",
                  command=self.probe_wall).grid(row=1, column=0, padx=2, pady=2, sticky=tk.EW)
        ttk.Button(btn_grid, text="Radial",
                  command=self.radial_probe).grid(row=1, column=1, padx=2, pady=2, sticky=tk.EW)
        btn_grid.columnconfigure(0, weight=1)
        btn_grid.columnconfigure(1, weight=1)
        
        # Results
        results_frame = ttk.LabelFrame(left_frame, text="Results", padding=5)
        results_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.results_text = tk.Text(results_frame, height=8, width=28)
        self.results_text.pack(fill=tk.BOTH, expand=True)
        
        # Export/Clear
        export_frame = ttk.Frame(left_frame)
        export_frame.pack(fill=tk.X, pady=5)
        
        self.filename_entry = ttk.Entry(export_frame, width=14)
        self.filename_entry.insert(0, "features.dxf")
        self.filename_entry.pack(side=tk.LEFT, padx=2)
        
        ttk.Button(export_frame, text="Export",
                  command=self.export_dxf).pack(side=tk.LEFT, padx=2)
        ttk.Button(export_frame, text="Clear",
                  command=self.clear_features).pack(side=tk.LEFT, padx=2)
        
        # Right: Canvas
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        canvas_frame = ttk.LabelFrame(right_frame, text="Work Area - WCS (Click to Move)", padding=5)
        canvas_frame.pack(fill=tk.BOTH, expand=True)
        
        ttk.Label(canvas_frame, text="Zoom: +/- | Center: C | Reset: R | Drag: Zoom box | Red=Machine limits",
                 foreground="gray").pack(anchor=tk.W)
        
        self.canvas = tk.Canvas(canvas_frame, bg="white", cursor="crosshair")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
        self.canvas.bind("<ButtonPress-1>", self.on_canvas_press)
        self.canvas.bind("<B1-Motion>", self.on_canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_canvas_release)
        self.canvas.bind("<Configure>", self.on_canvas_resize)
        
        # Keyboard bindings
        self.winfo_toplevel().bind("=", lambda e: self.zoom(1.1))
        self.winfo_toplevel().bind("+", lambda e: self.zoom(1.1))
        self.winfo_toplevel().bind("-", lambda e: self.zoom(0.9))
        self.winfo_toplevel().bind("c", lambda e: self.center_on_spindle())
        self.winfo_toplevel().bind("C", lambda e: self.center_on_spindle())
        self.winfo_toplevel().bind("r", lambda e: self.reset_view())
        self.winfo_toplevel().bind("R", lambda e: self.reset_view())
    
    def on_position_update(self, pos: Point, a: float):
        self.draw_position_marker(pos.x, pos.y)
    
    # Canvas methods
    def get_scale(self) -> float:
        """Get uniform scale (pixels per mm) to maintain square aspect ratio."""
        canvas_w = self.canvas.winfo_width() - 20
        canvas_h = self.canvas.winfo_height() - 20
        if canvas_w <= 0 or canvas_h <= 0:
            return 1.0
        # Use the smaller dimension to determine scale, so content fits in both directions
        scale_x = canvas_w / self.work_area_width
        scale_y = canvas_h / self.work_area_height
        return min(scale_x, scale_y)
    
    def world_to_canvas_x(self, world_x: float) -> float:
        canvas_w = self.canvas.winfo_width() - 20
        scale = self.get_scale()
        relative_x = world_x - self.center_x
        return 10 + canvas_w/2 + relative_x * scale
    
    def world_to_canvas_y(self, world_y: float) -> float:
        canvas_h = self.canvas.winfo_height() - 20
        scale = self.get_scale()
        relative_y = world_y - self.center_y
        return 10 + canvas_h/2 - relative_y * scale
    
    def canvas_to_world_x(self, canvas_x: float) -> float:
        canvas_w = self.canvas.winfo_width() - 20
        scale = self.get_scale()
        if scale <= 0:
            return self.center_x
        relative_x = (canvas_x - 10 - canvas_w/2) / scale
        return relative_x + self.center_x
    
    def canvas_to_world_y(self, canvas_y: float) -> float:
        canvas_h = self.canvas.winfo_height() - 20
        scale = self.get_scale()
        if scale <= 0:
            return self.center_y
        relative_y = (canvas_h/2 - (canvas_y - 10)) / scale
        return relative_y + self.center_y
    
    def redraw_canvas(self):
        self.canvas.delete("all")
        
        canvas_w = self.canvas.winfo_width() - 20
        canvas_h = self.canvas.winfo_height() - 20
        
        if canvas_w <= 0 or canvas_h <= 0:
            return
        
        # Border
        self.canvas.create_rectangle(10, 10, canvas_w + 10, canvas_h + 10, 
                                    outline="black", width=2)
        
        # Calculate visible world range based on uniform scale
        scale = self.get_scale()
        if scale <= 0:
            return
        visible_half_w = canvas_w / (2 * scale)
        visible_half_h = canvas_h / (2 * scale)
        
        # Draw Carvera work envelope boundary in WCS coordinates
        # Machine envelope is X: -360 to 0, Y: -240 to 0 in machine coords
        # WCS = Machine - G54_offset, so envelope in WCS:
        # X: -360 - offset_x to 0 - offset_x
        # Y: -240 - offset_y to 0 - offset_y
        env_mach_x1, env_mach_x2 = -360, 0
        env_mach_y1, env_mach_y2 = -240, 0
        env_wcs_x1 = env_mach_x1 - self.connection.wcs_offset_x
        env_wcs_x2 = env_mach_x2 - self.connection.wcs_offset_x
        env_wcs_y1 = env_mach_y1 - self.connection.wcs_offset_y
        env_wcs_y2 = env_mach_y2 - self.connection.wcs_offset_y
        
        env_x1 = self.world_to_canvas_x(env_wcs_x1)
        env_y1 = self.world_to_canvas_y(env_wcs_y2)  # Note: y2 is top in WCS
        env_x2 = self.world_to_canvas_x(env_wcs_x2)
        env_y2 = self.world_to_canvas_y(env_wcs_y1)  # y1 is bottom in WCS
        self.canvas.create_rectangle(env_x1, env_y1, env_x2, env_y2, 
                                    outline="red", width=2, dash=(5, 3))
        
        # Grid - draw lines at 10mm intervals within visible area
        grid_min_x = int((self.center_x - visible_half_w) / 10) * 10
        grid_max_x = int((self.center_x + visible_half_w) / 10 + 1) * 10
        grid_min_y = int((self.center_y - visible_half_h) / 10) * 10
        grid_max_y = int((self.center_y + visible_half_h) / 10 + 1) * 10
        
        for i in range(grid_min_x, grid_max_x + 1, 10):
            x = self.world_to_canvas_x(i)
            if 10 <= x <= canvas_w + 10:
                self.canvas.create_line(x, 10, x, canvas_h + 10, fill="lightgray", dash=(2, 2))
        
        for i in range(grid_min_y, grid_max_y + 1, 10):
            y = self.world_to_canvas_y(i)
            if 10 <= y <= canvas_h + 10:
                self.canvas.create_line(10, y, canvas_w + 10, y, fill="lightgray", dash=(2, 2))
        
        # Axes
        x_axis = self.world_to_canvas_x(0)
        y_axis = self.world_to_canvas_y(0)
        if 10 <= x_axis <= canvas_w + 10:
            self.canvas.create_line(x_axis, 10, x_axis, canvas_h + 10, fill="gray", width=2)
        if 10 <= y_axis <= canvas_h + 10:
            self.canvas.create_line(10, y_axis, canvas_w + 10, y_axis, fill="gray", width=2)
        
        # Label the origin if visible
        if 10 <= x_axis <= canvas_w + 10 and 10 <= y_axis <= canvas_h + 10:
            self.canvas.create_text(x_axis + 5, y_axis - 5, text="(0,0)", 
                                   anchor="sw", fill="gray", font=("TkDefaultFont", 8))
        
        # Draw features
        scale = self.get_scale()
        
        for circle in self.circles:
            cx = self.world_to_canvas_x(circle.center.x)
            cy = self.world_to_canvas_y(circle.center.y)
            # Scale radius to canvas pixels using uniform scale
            r_canvas = circle.radius * scale
            self.canvas.create_oval(cx-r_canvas, cy-r_canvas, cx+r_canvas, cy+r_canvas,
                                   outline="blue", width=2)
            self.canvas.create_oval(cx-3, cy-3, cx+3, cy+3, fill="blue")
        
        for wall in self.infinite_walls:
            angle_rad = math.radians(wall.angle)
            max_dist = math.sqrt(self.work_area_width**2 + self.work_area_height**2)
            dx = math.cos(angle_rad)
            dy = math.sin(angle_rad)
            x1 = wall.point.x - dx * max_dist
            y1 = wall.point.y - dy * max_dist
            x2 = wall.point.x + dx * max_dist
            y2 = wall.point.y + dy * max_dist
            cx1, cy1 = self.world_to_canvas_x(x1), self.world_to_canvas_y(y1)
            cx2, cy2 = self.world_to_canvas_x(x2), self.world_to_canvas_y(y2)
            self.canvas.create_line(cx1, cy1, cx2, cy2, fill="purple", width=2)
        
        for poly in self.polygons:
            if len(poly.points) > 1:
                coords = []
                for pt in poly.points:
                    coords.extend([self.world_to_canvas_x(pt.x), self.world_to_canvas_y(pt.y)])
                coords.extend([coords[0], coords[1]])  # Close polygon
                self.canvas.create_line(coords, fill="orange", width=2)
        
        for pt in self.points:
            cx = self.world_to_canvas_x(pt.x)
            cy = self.world_to_canvas_y(pt.y)
            self.canvas.create_oval(cx-3, cy-3, cx+3, cy+3, fill="green", outline="darkgreen")
        
        # Position marker
        if self.connection.connected:
            pos = self.connection.position
            self.draw_position_marker(pos.x, pos.y)
    
    def draw_position_marker(self, x: float, y: float):
        if self.position_marker:
            self.canvas.delete(self.position_marker)
        
        cx = self.world_to_canvas_x(x)
        cy = self.world_to_canvas_y(y)
        
        self.position_marker = self.canvas.create_oval(
            cx-5, cy-5, cx+5, cy+5,
            fill="red", outline="darkred", width=2, tags="position"
        )
    
    def on_canvas_resize(self, event):
        self.redraw_canvas()
    
    def on_canvas_press(self, event):
        self.drag_start_x = event.x
        self.drag_start_y = event.y
    
    def on_canvas_drag(self, event):
        if self.drag_start_x and self.drag_start_y:
            if self.zoom_box:
                self.canvas.delete(self.zoom_box)
            self.zoom_box = self.canvas.create_rectangle(
                self.drag_start_x, self.drag_start_y, event.x, event.y,
                outline="blue", width=2, dash=(5, 5)
            )
    
    def on_canvas_release(self, event):
        if not self.drag_start_x or not self.drag_start_y:
            return
        
        if self.zoom_box:
            self.canvas.delete(self.zoom_box)
            self.zoom_box = None
        
        drag_dist = math.sqrt((event.x - self.drag_start_x)**2 + 
                              (event.y - self.drag_start_y)**2)
        
        if drag_dist < 5:
            self.move_to_click(event.x, event.y)
        else:
            self.zoom_to_box(self.drag_start_x, self.drag_start_y, event.x, event.y)
        
        self.drag_start_x = None
        self.drag_start_y = None
    
    def move_to_click(self, canvas_x: float, canvas_y: float):
        if not self.connection.connected:
            messagebox.showwarning("Not Connected", "Please connect first")
            return
        
        # Check if probing height is set
        if not self.connection.probe_z_set:
            messagebox.showerror(
                "Probing Height Not Set",
                "You must set the probing height before moving.\n\n"
                "Go to the Settings panel and click 'Set' "
                "after positioning the probe at the desired probing height."
            )
            return
        
        world_x = self.canvas_to_world_x(canvas_x)
        world_y = self.canvas_to_world_y(canvas_y)
        
        # Calculate retract Z (probe_z + retract_height)
        retract_z = self.connection.probe_z + self.connection.retract_height
        
        try:
            self.connection.begin_operation()
            # Move to retract height first
            self.connection.move_z(retract_z)
            self.connection.wait_for_idle()
            # Move XY
            self.connection.move_xy(world_x, world_y)
            self.connection.wait_for_idle()
            # Move down to probe height
            self.connection.move_z(self.connection.probe_z)
            self.connection.wait_for_idle()
            self.connection.end_operation()
        except ProbeCrash as e:
            self.connection.end_operation()
            messagebox.showerror("Probe Crash", f"Probe triggered during move:\n{e}")
        except AlarmError as e:
            self.connection.end_operation()
            messagebox.showerror("Machine Alarm", f"Machine entered alarm state:\n{e}")
        except TimeoutError as e:
            self.connection.end_operation()
            messagebox.showerror("Operation Timed Out", f"Move timed out:\n{e}")
        except Exception as e:
            self.connection.end_operation()
            messagebox.showerror("Move Error", str(e))
    
    def zoom_to_box(self, x1, y1, x2, y2):
        wx1 = self.canvas_to_world_x(x1)
        wy1 = self.canvas_to_world_y(y1)
        wx2 = self.canvas_to_world_x(x2)
        wy2 = self.canvas_to_world_y(y2)
        
        self.center_x = (wx1 + wx2) / 2.0
        self.center_y = (wy1 + wy2) / 2.0
        
        box_w = abs(wx2 - wx1)
        box_h = abs(wy2 - wy1)
        
        if box_w < 0.1 or box_h < 0.1:
            return
        
        # Base dimensions for zoom calculation (full envelope + margin)
        base_width = 400.0
        base_height = 280.0
        
        self.zoom_factor = min(base_width / (box_w * 1.1), base_height / (box_h * 1.1))
        self.zoom_factor = max(0.1, min(20.0, self.zoom_factor))
        
        self.work_area_width = base_width / self.zoom_factor
        self.work_area_height = base_height / self.zoom_factor
        
        self.redraw_canvas()
    
    def zoom(self, factor: float):
        # Base dimensions for zoom calculation (full envelope + margin)
        base_width = 400.0
        base_height = 280.0
        
        new_zoom = self.zoom_factor * factor
        if 0.1 <= new_zoom <= 20.0:
            self.zoom_factor = new_zoom
            self.work_area_width = base_width / self.zoom_factor
            self.work_area_height = base_height / self.zoom_factor
            self.redraw_canvas()
    
    def reset_view(self):
        """Reset view to show full Carvera work envelope in WCS coordinates."""
        # Machine envelope: X -360 to 0, Y -240 to 0
        # In WCS: subtract the G54 offset
        # Center of envelope in WCS:
        env_center_x = -180.0 - self.connection.wcs_offset_x
        env_center_y = -120.0 - self.connection.wcs_offset_y
        
        self.zoom_factor = 1.0
        self.center_x = env_center_x
        self.center_y = env_center_y
        self.work_area_width = 400.0   # Slightly larger than 360 to show full envelope
        self.work_area_height = 280.0  # Slightly larger than 240
        self.redraw_canvas()
    
    def center_on_spindle(self):
        if self.connection.connected:
            pos = self.connection.get_position()
            self.center_x = pos.x
            self.center_y = pos.y
            self.redraw_canvas()
    
    def drop_point(self):
        if not self.connection.connected:
            messagebox.showwarning("Not Connected", "Please connect first")
            return
        
        pos = self.connection.get_position()
        self.points.append(Point(pos.x, pos.y, pos.z))
        self.log_result(f"Point: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
        self.redraw_canvas()
    
    def log_result(self, message: str):
        self.results_text.insert(tk.END, message + "\n")
        self.results_text.see(tk.END)
    
    def get_probe_offset(self) -> float:
        try:
            return float(self.probe_tip_entry.get()) / 2.0
        except:
            return 0.0
    
    # Probing methods
    def probe_bore(self):
        print("=== PROBE_BORE button clicked")
        try:
            if not self.connection.check_probe_ready():
                return
        except Exception as e:
            print(f"=== check_probe_ready failed: {e}")
            messagebox.showerror("Connection Error", f"Failed to check probe status:\n{e}")
            return
        
        try:
            bore_diameter = float(self.bore_radius_entry.get())
            rbore = bore_diameter / 2.0  # Convert diameter to radius for probing distance
            offset = self.get_probe_offset()
            
            print(f"=== Starting bore probe, diameter={bore_diameter}, offset={offset}")
            self.connection.begin_operation()
            current = self.connection.get_position()
            print(f"=== Starting position: ({current.x:.3f}, {current.y:.3f}, {current.z:.3f})")
            
            # === FIRST PASS: Find approximate center ===
            self.log_result("Pass 1: Finding center...")
            
            px1 = self.connection.probe_direction(Direction.PLUS_X, rbore, offset)
            self.connection.move_xy(current.x, current.y)
            self.connection.wait_for_idle()
            time.sleep(0.1)
            
            mx1 = self.connection.probe_direction(Direction.MINUS_X, rbore, offset)
            self.connection.move_xy(current.x, current.y)
            self.connection.wait_for_idle()
            time.sleep(0.1)
            
            py1 = self.connection.probe_direction(Direction.PLUS_Y, rbore, offset)
            self.connection.move_xy(current.x, current.y)
            self.connection.wait_for_idle()
            time.sleep(0.1)
            
            my1 = self.connection.probe_direction(Direction.MINUS_Y, rbore, offset)
            
            # Calculate approximate center
            cx1 = (px1.x + mx1.x) / 2.0
            cy1 = (py1.y + my1.y) / 2.0
            
            self.log_result(f"  Approx center: ({cx1:.3f}, {cy1:.3f})")
            
            # === MOVE TO CENTER ===
            self.connection.move_xy(cx1, cy1)
            self.connection.wait_for_idle()
            time.sleep(0.2)
            
            # === SECOND PASS: Measure from center for accurate diameter ===
            self.log_result("Pass 2: Measuring diameter...")
            
            px2 = self.connection.probe_direction(Direction.PLUS_X, rbore, offset)
            self.connection.move_xy(cx1, cy1)
            self.connection.wait_for_idle()
            time.sleep(0.2)
            
            mx2 = self.connection.probe_direction(Direction.MINUS_X, rbore, offset)
            self.connection.move_xy(cx1, cy1)
            self.connection.wait_for_idle()
            time.sleep(0.2)
            
            py2 = self.connection.probe_direction(Direction.PLUS_Y, rbore, offset)
            self.connection.move_xy(cx1, cy1)
            self.connection.wait_for_idle()
            time.sleep(0.2)
            
            my2 = self.connection.probe_direction(Direction.MINUS_Y, rbore, offset)
            
            # Calculate refined center and true diameter from second pass
            cx = (px2.x + mx2.x) / 2.0
            cy = (py2.y + my2.y) / 2.0
            
            # Now these are true diameters (probed from center)
            dx = px2.x - mx2.x
            dy = py2.y - my2.y
            d = (dx + dy) / 2.0  # Average diameter
            r = d / 2.0
            
            # Move to the refined center
            self.connection.move_xy(cx, cy)
            self.connection.wait_for_idle()
            
            self.connection.end_operation()
            
            self.circles.append(Circle(Point(cx, cy), r))
            self.log_result(f"Bore: ({cx:.3f}, {cy:.3f}) Ø={d:.3f}")
            self.save_probe_config()
            self.redraw_canvas()
                
        except ProbeNoContact as e:
            self.connection.end_operation()
            self.log_result(f"Probe No Contact: {e}")
            messagebox.showerror("Probe No Contact", f"Bore probe failed - no contact:\n{e}")
        except ProbeCrash as e:
            self.connection.end_operation()
            self.log_result(f"Probe Crash: {e}")
            messagebox.showerror("Probe Crash", f"Probe triggered during move:\n{e}")
        except AlarmError as e:
            self.connection.end_operation()
            self.log_result(f"Alarm: {e}")
            messagebox.showerror("Machine Alarm", f"Machine entered alarm state:\n{e}")
        except TimeoutError as e:
            self.connection.end_operation()
            self.log_result(f"Timeout: {e}")
            messagebox.showerror("Operation Timed Out", f"Bore probe timed out:\n{e}")
        except Exception as e:
            self.connection.end_operation()
            self.log_result(f"Probe Error: {e}")
            messagebox.showerror("Probe Error", f"Bore probe failed:\n{e}")
    
    def probe_single(self):
        print("=== PROBE_SINGLE button clicked")
        try:
            if not self.connection.check_probe_ready():
                return
        except Exception as e:
            print(f"=== check_probe_ready failed: {e}")
            messagebox.showerror("Connection Error", f"Failed to check probe status:\n{e}")
            return
        
        try:
            dist = float(self.probe_dist_entry.get())
            offset = self.get_probe_offset()
            direction = Direction(self.probe_direction.get())
            
            self.connection.begin_operation()
            result = self.connection.probe_direction(direction, dist, offset)
            self.connection.end_operation()
            
            self.points.append(result)
            self.log_result(f"Point: ({result.x:.3f}, {result.y:.3f}, {result.z:.3f})")
            self.save_probe_config()
            self.redraw_canvas()
                
        except ProbeNoContact as e:
            self.connection.end_operation()
            self.log_result(f"Probe No Contact: {e}")
            messagebox.showerror("Probe No Contact", f"Single probe failed - no contact:\n{e}")
        except ProbeCrash as e:
            self.connection.end_operation()
            self.log_result(f"Probe Crash: {e}")
            messagebox.showerror("Probe Crash", f"Probe triggered during move:\n{e}")
        except AlarmError as e:
            self.connection.end_operation()
            self.log_result(f"Alarm: {e}")
            messagebox.showerror("Machine Alarm", f"Machine entered alarm state:\n{e}")
        except TimeoutError as e:
            self.connection.end_operation()
            self.log_result(f"Timeout: {e}")
            messagebox.showerror("Operation Timed Out", f"Single probe timed out:\n{e}")
        except Exception as e:
            self.connection.end_operation()
            self.log_result(f"Probe Error: {e}")
            messagebox.showerror("Probe Error", f"Single probe failed:\n{e}")
    
    def probe_wall(self):
        print("=== PROBE_WALL button clicked")
        try:
            if not self.connection.check_probe_ready():
                return
        except Exception as e:
            print(f"=== check_probe_ready failed: {e}")
            messagebox.showerror("Connection Error", f"Failed to check probe status:\n{e}")
            return
        
        try:
            dist = float(self.probe_dist_entry.get())
            width = float(self.wall_width_entry.get())
            offset = self.get_probe_offset()
            direction = Direction(self.probe_direction.get())
            
            print(f"=== Starting wall probe, dist={dist}, width={width}, direction={direction}")
            self.connection.begin_operation()
            current = self.connection.get_position()
            print(f"=== Starting position: ({current.x:.3f}, {current.y:.3f}, {current.z:.3f})")
            
            if direction in [Direction.PLUS_X, Direction.MINUS_X]:
                print(f"=== Moving to first point: ({current.x:.3f}, {current.y - width/2:.3f})")
                self.connection.move_xy(current.x, current.y - width/2)
                self.connection.wait_for_idle()
                time.sleep(0.1)
                p1 = self.connection.probe_direction(direction, dist, offset)
                
                print(f"=== Moving to second point: ({current.x:.3f}, {current.y + width/2:.3f})")
                self.connection.move_xy(current.x, current.y + width/2)
                self.connection.wait_for_idle()
                time.sleep(0.1)
                p2 = self.connection.probe_direction(direction, dist, offset)
            else:
                print(f"=== Moving to first point: ({current.x - width/2:.3f}, {current.y:.3f})")
                self.connection.move_xy(current.x - width/2, current.y)
                self.connection.wait_for_idle()
                time.sleep(0.1)
                p1 = self.connection.probe_direction(direction, dist, offset)
                
                print(f"=== Moving to second point: ({current.x + width/2:.3f}, {current.y:.3f})")
                self.connection.move_xy(current.x + width/2, current.y)
                self.connection.wait_for_idle()
                time.sleep(0.1)
                p2 = self.connection.probe_direction(direction, dist, offset)
            
            self.connection.end_operation()
            
            # Calculate wall angle from the two points (this is correct even with offset errors,
            # because both points have the same error which cancels in the difference)
            dy = p2.y - p1.y
            dx = p2.x - p1.x
            angle = math.degrees(math.atan2(dy, dx))
            angle_rad = math.radians(angle)
            
            # Correct positions for tilted wall contact
            # The probe_direction assumed perpendicular contact, but for a tilted wall
            # the contact point on the probe ball is offset from the pole.
            # 
            # For wall at angle θ:
            # - PLUS_X:  correction = offset * (sin(θ) - 1, -cos(θ))
            # - MINUS_X: correction = offset * (1 - sin(θ), cos(θ))
            # - PLUS_Y:  correction = offset * (-sin(θ), cos(θ) - 1)
            # - MINUS_Y: correction = offset * (sin(θ), 1 - cos(θ))
            
            sin_a = math.sin(angle_rad)
            cos_a = math.cos(angle_rad)
            
            if direction == Direction.PLUS_X:
                corr_x = offset * (sin_a - 1)
                corr_y = offset * (-cos_a)
            elif direction == Direction.MINUS_X:
                corr_x = offset * (1 - sin_a)
                corr_y = offset * cos_a
            elif direction == Direction.PLUS_Y:
                corr_x = offset * (-sin_a)
                corr_y = offset * (cos_a - 1)
            else:  # MINUS_Y
                corr_x = offset * sin_a
                corr_y = offset * (1 - cos_a)
            
            p1_corrected = Point(p1.x + corr_x, p1.y + corr_y, p1.z)
            p2_corrected = Point(p2.x + corr_x, p2.y + corr_y, p2.z)
            
            midpoint = Point((p1_corrected.x + p2_corrected.x)/2, 
                           (p1_corrected.y + p2_corrected.y)/2, p1.z)
            self.infinite_walls.append(InfiniteWall(midpoint, angle))
            
            self.log_result(f"Wall: ({midpoint.x:.3f}, {midpoint.y:.3f}) {angle:.1f}°")
            self.save_probe_config()
            self.redraw_canvas()
                
        except ProbeNoContact as e:
            self.connection.end_operation()
            self.log_result(f"Probe No Contact: {e}")
            messagebox.showerror("Probe No Contact", f"Wall probe failed - no contact:\n{e}")
        except ProbeCrash as e:
            self.connection.end_operation()
            self.log_result(f"Probe Crash: {e}")
            messagebox.showerror("Probe Crash", f"Probe triggered during move:\n{e}")
        except AlarmError as e:
            self.connection.end_operation()
            self.log_result(f"Alarm: {e}")
            messagebox.showerror("Machine Alarm", f"Machine entered alarm state:\n{e}")
        except TimeoutError as e:
            self.connection.end_operation()
            self.log_result(f"Timeout: {e}")
            messagebox.showerror("Operation Timed Out", f"Wall probe timed out:\n{e}")
        except Exception as e:
            self.connection.end_operation()
            self.log_result(f"Probe Error: {e}")
            messagebox.showerror("Probe Error", f"Wall probe failed:\n{e}")
    
    def radial_probe(self):
        print("=== RADIAL_PROBE button clicked")
        try:
            if not self.connection.check_probe_ready():
                return
        except Exception as e:
            print(f"=== check_probe_ready failed: {e}")
            messagebox.showerror("Connection Error", f"Failed to check probe status:\n{e}")
            return
        
        try:
            dist = float(self.probe_dist_entry.get())
            nangles = int(self.radial_angles_entry.get())
            offset = self.get_probe_offset()
            
            self.connection.begin_operation()
            current = self.connection.get_position()
            
            probe_points = []
            
            for i in range(nangles):
                angle = (2 * math.pi * i) / nangles
                dx = dist * math.cos(angle)
                dy = dist * math.sin(angle)
                
                self.connection.move_to(current.x, current.y, current.z)
                time.sleep(0.2)
                
                if abs(dx) > abs(dy):
                    direction = Direction.PLUS_X if dx > 0 else Direction.MINUS_X
                    d = abs(dx)
                else:
                    direction = Direction.PLUS_Y if dy > 0 else Direction.MINUS_Y
                    d = abs(dy)
                
                result = self.connection.probe_direction(direction, d, offset)
                probe_points.append(result)
            
            self.connection.end_operation()
            
            self.polygons.append(Polygon(probe_points))
            self.log_result(f"Radial: {len(probe_points)} points")
            self.save_probe_config()
            self.redraw_canvas()
                
        except ProbeNoContact as e:
            self.connection.end_operation()
            self.log_result(f"Probe No Contact: {e}")
            messagebox.showerror("Probe No Contact", f"Radial probe failed - no contact:\n{e}")
        except ProbeCrash as e:
            self.connection.end_operation()
            self.log_result(f"Probe Crash: {e}")
            messagebox.showerror("Probe Crash", f"Probe triggered during move:\n{e}")
        except AlarmError as e:
            self.connection.end_operation()
            self.log_result(f"Alarm: {e}")
            messagebox.showerror("Machine Alarm", f"Machine entered alarm state:\n{e}")
        except TimeoutError as e:
            self.connection.end_operation()
            self.log_result(f"Timeout: {e}")
            messagebox.showerror("Operation Timed Out", f"Radial probe timed out:\n{e}")
        except Exception as e:
            self.connection.end_operation()
            self.log_result(f"Probe Error: {e}")
            messagebox.showerror("Probe Error", f"Radial probe failed:\n{e}")
    
    def export_dxf(self):
        if not HAS_EZDXF:
            messagebox.showerror("Not Available", "ezdxf not installed")
            return
        
        filename = self.filename_entry.get()
        if not filename.endswith('.dxf'):
            filename += '.dxf'
        
        try:
            doc = ezdxf.new('R2010')
            msp = doc.modelspace()
            
            for c in self.circles:
                msp.add_circle((c.center.x, c.center.y), c.radius)
            
            for w in self.infinite_walls:
                angle_rad = math.radians(w.angle)
                length = 100
                x1 = w.point.x - length * math.cos(angle_rad)
                y1 = w.point.y - length * math.sin(angle_rad)
                x2 = w.point.x + length * math.cos(angle_rad)
                y2 = w.point.y + length * math.sin(angle_rad)
                msp.add_line((x1, y1), (x2, y2))
            
            for poly in self.polygons:
                if len(poly.points) > 1:
                    pts = [(p.x, p.y) for p in poly.points]
                    pts.append(pts[0])
                    msp.add_lwpolyline(pts)
            
            for p in self.points:
                msp.add_point((p.x, p.y))
            
            doc.saveas(filename)
            self.log_result(f"Exported: {filename}")
            messagebox.showinfo("Export", f"Saved to {filename}")
            
        except Exception as e:
            messagebox.showerror("Export Error", str(e))
    
    def clear_features(self):
        self.circles.clear()
        self.walls.clear()
        self.infinite_walls.clear()
        self.polygons.clear()
        self.points.clear()
        self.results_text.delete(1.0, tk.END)
        self.redraw_canvas()


# =============================================================================
# Level A-Axis Tab
# =============================================================================

class LevelAAxisTab(ttk.Frame):
    """Tab for leveling the A-axis (4th axis)."""
    
    def __init__(self, parent, connection: SharedCarveraConnection):
        super().__init__(parent)
        self.connection = connection
        self.setup_ui()
    
    def setup_ui(self):
        # Main container with two columns
        main_frame = ttk.Frame(self)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Left column: Instructions and parameters
        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        # Instructions
        instr_frame = ttk.LabelFrame(left_frame, text="Instructions", padding=10)
        instr_frame.pack(fill=tk.X, pady=5)
        
        # Title (bold)
        title_label = ttk.Label(instr_frame, text="A-Axis Leveling Procedure:", 
                                font=("TkDefaultFont", 9, "bold"))
        title_label.pack(anchor=tk.W)
        
        instructions = """1. Rotate A-axis to make stock as level as possible by eye.
2. Jog probe to above the stock, centered on A-Axis.
3. Click "Level A-Axis" - this will probe at 3 Y locations, remove A-axis tilt, and reset A-axis WCS zero."""
        ttk.Label(instr_frame, text=instructions, justify=tk.LEFT).pack(anchor=tk.W)
        
        # Parameters
        param_frame = ttk.LabelFrame(left_frame, text="Leveling Parameters", padding=10)
        param_frame.pack(fill=tk.X, pady=5)
        
        param_grid = ttk.Frame(param_frame)
        param_grid.pack(fill=tk.X)
        
        ttk.Label(param_grid, text="Y Distance (mm):").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.y_dist_entry = ttk.Entry(param_grid, width=10)
        self.y_dist_entry.insert(0, "15.0")
        self.y_dist_entry.grid(row=0, column=1, padx=5, pady=2)
        
        ttk.Label(param_grid, text="(Uses probe speed from Settings panel)", 
                 foreground="gray").grid(row=1, column=0, columnspan=2, sticky=tk.W, pady=2)
        
        # Action button
        self.level_btn = ttk.Button(left_frame, text="Level A-Axis Starting at Current Location", 
                                   command=self.level_a_axis)
        self.level_btn.pack(fill=tk.X, pady=10)
        
        # Right column: Results
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)
        
        result_frame = ttk.LabelFrame(right_frame, text="Results", padding=10)
        result_frame.pack(fill=tk.BOTH, expand=True)
        
        self.result_text = tk.Text(result_frame, height=20, width=50)
        self.result_text.pack(fill=tk.BOTH, expand=True)
    
    def log(self, message: str):
        self.result_text.insert(tk.END, message + "\n")
        self.result_text.see(tk.END)
        self.update()
    
    def probe_z_down(self, retract_z: float) -> Optional[float]:
        """Probe downward and return Z position. Raises ProbeNoContact if no contact."""
        feedrate = self.connection.slow_probe_speed
        max_depth = self.connection.max_probe_distance
        # Calculate timeout based on max depth at feedrate + buffer
        probe_timeout = max(15.0, (max_depth / feedrate) * 60.0 + 5.0)
        
        response = self.connection.send_command(f"G38.2 Z-{max_depth:.4f} F{feedrate:.0f}", timeout=probe_timeout)
        
        # Wait for probe to release from surface before retracting
        time.sleep(0.3)
        
        # Check for no contact
        if 'PRB:' in response:
            prb_part = response.split('PRB:')[1]
            if ':0' in prb_part or prb_part.strip().endswith(':0'):
                raise ProbeNoContact("Z probe did not make contact")
        
        # Get position
        status = self.connection.send_command("?")
        
        try:
            if 'WPos:' in status:
                pos_str = status.split('WPos:')[1].split('|')[0]
                coords = pos_str.split(',')
                z = float(coords[2])
                
                # Retract
                self.connection.move_z(retract_z)
                self.connection.wait_for_idle()
                
                return z
        except Exception as e:
            self.log(f"Probe parse error: {e}")
        
        return None
    
    def level_a_axis(self):
        print("=== LEVEL_A_AXIS button clicked")
        try:
            # Level A-Axis doesn't require probe_z to be set - uses current position
            if not self.connection.check_probe_ready(require_probe_z=False):
                return
        except Exception as e:
            print(f"=== check_probe_ready failed: {e}")
            messagebox.showerror("Connection Error", f"Failed to check probe status:\n{e}")
            return
        
        try:
            dy = float(self.y_dist_entry.get())
            
            self.result_text.delete(1.0, tk.END)
            self.log("Starting A-axis leveling...")
            self.log(f"Using probe speed: {self.connection.slow_probe_speed:.0f} mm/min")
            
            self.connection.begin_operation()
            
            # Ensure correct mode
            self.connection.send_command("G54")
            self.connection.send_command("G90")
            
            # Get current position - use current Z as the safe/retract height
            pos = self.connection.get_position()
            x0 = pos.x
            y0 = pos.y
            z0 = pos.z  # Use current Z as safe height
            a0 = self.connection.a_position
            
            self.log(f"Starting position: X={x0:.3f}, Y={y0:.3f}, Z={z0:.3f}, A={a0:.3f}")
            self.log(f"Using current Z={z0:.3f} as retract height")
            
            # Probe at center (already at position)
            self.log("\nProbing at center...")
            z_center = self.probe_z_down(z0)
            if z_center is None:
                raise Exception("Center probe failed")
            self.log(f"  Z center = {z_center:.3f}")
            
            # Probe at +Y
            self.log(f"\nProbing at Y+{dy}...")
            self.connection.move_to(x0, y0 + dy, z0)
            self.connection.wait_for_idle()
            z_plus = self.probe_z_down(z0)
            if z_plus is None:
                raise Exception("+Y probe failed")
            self.log(f"  Z plus = {z_plus:.3f}")
            
            # Probe at -Y
            self.log(f"\nProbing at Y-{dy}...")
            self.connection.move_to(x0, y0 - dy, z0)
            self.connection.wait_for_idle()
            z_minus = self.probe_z_down(z0)
            if z_minus is None:
                raise Exception("-Y probe failed")
            self.log(f"  Z minus = {z_minus:.3f}")
            
            # Return to center
            self.connection.move_to(x0, y0, z0)
            self.connection.wait_for_idle()
            
            # Calculate correction angle
            # Positive dz means -Y side is higher, need positive A rotation to level
            dz = z_minus - z_plus
            dtheta = math.degrees(math.atan2(dz, 2.0 * dy))
            
            self.log(f"\nHeight difference (Z_minus - Z_plus): {dz:.4f} mm")
            self.log(f"Correction angle: {dtheta:.4f}°")
            
            # Apply correction
            new_a = a0 + dtheta
            self.log(f"\nApplying A-axis correction: {a0:.3f}° → {new_a:.3f}°")
            
            self.connection.move_a(new_a)
            self.connection.wait_for_idle()
            
            # Set current A position as WCS zero (only A-axis, not X/Y/Z)
            self.log("\nSetting current A position as WCS A=0...")
            self.connection.send_command("G10 L20 P1 A0")
            self.connection.wait_for_idle()
            
            self.log(f"\n✓ A-axis leveling complete!")
            self.log(f"  A-axis corrected by {dtheta:.4f}°")
            self.log(f"  A-axis WCS origin set to current position (A=0)")
            
            self.connection.end_operation()
            
        except ProbeNoContact as e:
            self.connection.end_operation()
            self.log(f"\n✗ Probe No Contact: {e}")
            messagebox.showerror("Probe No Contact", f"Probe failed - no contact:\n{e}")
        except ProbeCrash as e:
            self.connection.end_operation()
            self.log(f"\n✗ Probe Crash: {e}")
            messagebox.showerror("Probe Crash", f"Probe triggered during move:\n{e}")
        except AlarmError as e:
            self.connection.end_operation()
            self.log(f"\n✗ Alarm: {e}")
            messagebox.showerror("Machine Alarm", f"Machine entered alarm state:\n{e}")
        except TimeoutError as e:
            self.connection.end_operation()
            self.log(f"\n✗ Timeout: {e}")
            messagebox.showerror("Operation Timed Out", f"A-axis leveling timed out:\n{e}")
        except Exception as e:
            self.connection.end_operation()
            self.log(f"\n✗ Error: {e}")
            messagebox.showerror("Error", f"A-axis leveling failed:\n{e}")


# =============================================================================
# Curve Fitter Tab (requires matplotlib)
# =============================================================================

if HAS_MATPLOTLIB:
    class CurveFitterTab(ttk.Frame):
        """Tab for G-code based part finding, coordinate refinement and curve fitting."""
        
        # Config file for persistent settings
        CONFIG_FILE = os.path.join(os.path.expanduser("~"), ".carvera_partfinder.json")
        
        def __init__(self, parent, connection: SharedCarveraConnection):
            super().__init__(parent)
            self.connection = connection
            
            # State
            self.outer_curve_points = None
            self.original_outer_curve = None
            self.rotation_matrix = np.eye(2)
            self.translation = np.array([0.0, 0.0])
            
            self.mode = 'refine'  # 'refine', 'move', 'probe', 'fit'
            
            # Refine state
            self.clicked_points = []
            self.refine_point1 = None  # (x, y) - saved point 1 coordinates
            self.refine_point2 = None  # (x, y) - saved point 2 coordinates
            
            # Note: Probe Z and retract height are now stored in connection object
            # Access via self.connection.probe_z and self.connection.retract_height
            
            # Probe state
            self.n_probe_points = 20
            self.probe_distance = 5.0
            self.PsSamp = None
            self.NsSamp = None
            
            # Drag state for probe points
            self.dragging_point_idx = None
            self.probe_scatter = None  # Reference to scatter plot for picking
            
            # Fit state
            self.probed_points_work = []
            self.fit_rotation_angle = 0.0
            self.fit_translation = np.array([0.0, 0.0])
            
            # Load saved settings
            self.load_config()
            
            self.setup_ui()
        
        def load_config(self):
            """Load saved settings from config file."""
            try:
                if os.path.exists(self.CONFIG_FILE):
                    with open(self.CONFIG_FILE, 'r') as f:
                        config = json.load(f)
                    if 'refine_point1' in config and config['refine_point1']:
                        self.refine_point1 = tuple(config['refine_point1'])
                    if 'refine_point2' in config and config['refine_point2']:
                        self.refine_point2 = tuple(config['refine_point2'])
                    if 'probe_distance' in config:
                        self.probe_distance = config['probe_distance']
                    if 'n_probe_points' in config:
                        self.n_probe_points = config['n_probe_points']
            except Exception as e:
                print(f"Could not load config: {e}")
        
        def save_config(self):
            """Save settings to config file."""
            try:
                config = {
                    'refine_point1': list(self.refine_point1) if self.refine_point1 else None,
                    'refine_point2': list(self.refine_point2) if self.refine_point2 else None,
                    'probe_distance': self.probe_distance,
                    'n_probe_points': self.n_probe_points,
                }
                with open(self.CONFIG_FILE, 'w') as f:
                    json.dump(config, f, indent=2)
            except Exception as e:
                print(f"Could not save config: {e}")
        
        def setup_ui(self):
            # Main container
            main_frame = ttk.Frame(self)
            main_frame.pack(fill=tk.BOTH, expand=True)
            
            # Left: Controls (wider for controls to fit)
            left_frame = ttk.Frame(main_frame, width=336)
            left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
            left_frame.pack_propagate(False)
            
            # G-code loading
            load_frame = ttk.LabelFrame(left_frame, text="G-code File", padding=10)
            load_frame.pack(fill=tk.X, pady=5)
            
            ttk.Button(load_frame, text="Load G-code...",
                      command=self.load_gcode).pack(fill=tk.X)
            
            self.gcode_label = ttk.Label(load_frame, text="No file loaded", wraplength=310)
            self.gcode_label.pack(pady=5)
            
            # Mode selection
            mode_frame = ttk.LabelFrame(left_frame, text="Mode", padding=10)
            mode_frame.pack(fill=tk.X, pady=5)
            
            self.mode_var = tk.StringVar(value="refine")
            
            ttk.Radiobutton(mode_frame, text="1. Refine Coordinates", 
                           variable=self.mode_var, value="refine",
                           command=self.on_mode_change).pack(anchor=tk.W)
            ttk.Radiobutton(mode_frame, text="2. Test Move",
                           variable=self.mode_var, value="move",
                           command=self.on_mode_change).pack(anchor=tk.W)
            ttk.Radiobutton(mode_frame, text="3. Probe Perimeter",
                           variable=self.mode_var, value="probe",
                           command=self.on_mode_change).pack(anchor=tk.W)
            ttk.Radiobutton(mode_frame, text="4. Fit to Points",
                           variable=self.mode_var, value="fit",
                           command=self.on_mode_change).pack(anchor=tk.W)
            
            # Mode-specific controls container
            self.controls_frame = ttk.Frame(left_frame)
            self.controls_frame.pack(fill=tk.BOTH, expand=True, pady=5)
            
            # Status
            self.status_label = ttk.Label(left_frame, text="Load G-code to begin",
                                         foreground="blue", wraplength=310)
            self.status_label.pack(fill=tk.X, pady=5)
            
            # Right: Matplotlib canvas
            right_frame = ttk.Frame(main_frame)
            right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)
            
            self.fig = Figure(figsize=(8, 6), dpi=100)
            self.ax = self.fig.add_subplot(111)
            
            self.canvas = FigureCanvasTkAgg(self.fig, master=right_frame)
            self.canvas.draw()
            self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
            
            # Toolbar
            toolbar = NavigationToolbar2Tk(self.canvas, right_frame)
            toolbar.update()
            
            # Connect mouse events for clicking and dragging probe points
            self.canvas.mpl_connect('button_press_event', self.on_plot_click)
            self.canvas.mpl_connect('motion_notify_event', self.on_plot_motion)
            self.canvas.mpl_connect('button_release_event', self.on_plot_release)
            
            # Initial mode setup
            self.setup_refine_controls()
        
        def load_gcode(self):
            filename = filedialog.askopenfilename(
                filetypes=[("G-code files", "*.cnc *.nc *.gcode"), ("All files", "*.*")]
            )
            
            if not filename:
                return
            
            try:
                with open(filename, 'r') as f:
                    gcode = f.read()
                
                points = self.extract_outer_curve(gcode)
                
                if len(points) < 3:
                    messagebox.showerror("Error", "Not enough points found in G-code")
                    return
                
                self.outer_curve_points = points.copy()
                self.original_outer_curve = points.copy()
                self.rotation_matrix = np.eye(2)
                self.translation = np.array([0.0, 0.0])
                
                self.gcode_label.config(text=f"Loaded: {os.path.basename(filename)}\n({len(points)} points)")
                self.status_label.config(text="G-code loaded. Select mode to begin.")
                
                self.plot()
                
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load G-code: {e}")
        
        def extract_outer_curve(self, gcode_text: str) -> np.ndarray:
            """Extract points from G-code where cutting moves occur at Z=0.
            Handles G0, G1, G2, G3 moves including arc interpolation.
            Only processes XY plane (G17) arcs, ignores XZ (G18) and YZ (G19) arcs."""
            current_z = 0.0
            current_x = 0.0
            current_y = 0.0
            points = []
            current_mode = None  # 'G0', 'G1', 'G2', 'G3'
            current_plane = 'G17'  # Default XY plane (only affects arcs)
            
            for line in gcode_text.split('\n'):
                line = line.strip()
                if not line or line.startswith(';') or line.startswith('('):
                    continue
                
                # Remove comments
                if ';' in line:
                    line = line.split(';')[0].strip()
                if '(' in line:
                    line = re.sub(r'\([^)]*\)', '', line).strip()
                
                # Check for plane selection (G17=XY, G18=XZ, G19=YZ) - only affects arcs
                if re.search(r'\bG17\b', line, re.IGNORECASE):
                    current_plane = 'G17'
                if re.search(r'\bG18\b', line, re.IGNORECASE):
                    current_plane = 'G18'
                if re.search(r'\bG19\b', line, re.IGNORECASE):
                    current_plane = 'G19'
                
                # More robust mode detection using regex word boundaries
                # Match G0, G00, G1, G01, G2, G02, G3, G03 (not G20, G21, etc.)
                g0_match = re.search(r'\bG0*0(?!\d)', line, re.IGNORECASE)
                g1_match = re.search(r'\bG0*1(?!\d)', line, re.IGNORECASE)
                g2_match = re.search(r'\bG0*2(?!\d)', line, re.IGNORECASE)
                g3_match = re.search(r'\bG0*3(?!\d)', line, re.IGNORECASE)
                
                if g0_match:
                    current_mode = 'G0'
                elif g1_match:
                    current_mode = 'G1'
                elif g2_match:
                    current_mode = 'G2'
                elif g3_match:
                    current_mode = 'G3'
                
                # Parse coordinates
                x_match = re.search(r'X([-+]?\d*\.?\d+)', line, re.IGNORECASE)
                y_match = re.search(r'Y([-+]?\d*\.?\d+)', line, re.IGNORECASE)
                z_match = re.search(r'Z([-+]?\d*\.?\d+)', line, re.IGNORECASE)
                i_match = re.search(r'I([-+]?\d*\.?\d+)', line, re.IGNORECASE)
                j_match = re.search(r'J([-+]?\d*\.?\d+)', line, re.IGNORECASE)
                
                new_x = float(x_match.group(1)) if x_match else current_x
                new_y = float(y_match.group(1)) if y_match else current_y
                new_z = float(z_match.group(1)) if z_match else current_z
                
                # Process moves at cutting depth (Z=0)
                if current_mode == 'G1' and abs(new_z) < 0.01:
                    # Linear moves - always process (plane doesn't affect linear moves)
                    if x_match or y_match:
                        points.append([new_x, new_y])
                elif current_mode in ('G2', 'G3') and current_plane == 'G17' and abs(new_z) < 0.01:
                    # Arc moves - only process XY plane (G17) arcs
                    i_val = float(i_match.group(1)) if i_match else 0.0
                    j_val = float(j_match.group(1)) if j_match else 0.0
                    
                    # Only process if we have I or J (center offset) or movement
                    if i_match or j_match or (x_match or y_match):
                        # Center of arc (relative to start point)
                        cx = current_x + i_val
                        cy = current_y + j_val
                        
                        # Calculate start and end angles
                        start_angle = math.atan2(current_y - cy, current_x - cx)
                        end_angle = math.atan2(new_y - cy, new_x - cx)
                        radius = math.sqrt((current_x - cx)**2 + (current_y - cy)**2)
                        
                        if radius > 0.001:  # Only process if radius is meaningful
                            # Determine arc direction and normalize angles
                            if current_mode == 'G2':  # Clockwise
                                if end_angle >= start_angle:
                                    end_angle -= 2 * math.pi
                            else:  # G3 - Counter-clockwise
                                if end_angle <= start_angle:
                                    end_angle += 2 * math.pi
                            
                            # Interpolate arc with ~1mm spacing or at least 8 points
                            arc_length = abs(end_angle - start_angle) * radius
                            n_points = max(8, int(arc_length / 1.0))
                            
                            for k in range(1, n_points + 1):
                                t = k / n_points
                                angle = start_angle + t * (end_angle - start_angle)
                                ax = cx + radius * math.cos(angle)
                                ay = cy + radius * math.sin(angle)
                                points.append([ax, ay])
                
                # Update current position
                current_x = new_x
                current_y = new_y
                current_z = new_z
            
            return np.array(points) if points else np.zeros((0, 2))
        
        def on_mode_change(self):
            self.mode = self.mode_var.get()
            
            # Clear controls
            for widget in self.controls_frame.winfo_children():
                widget.destroy()
            
            if self.mode == 'refine':
                self.setup_refine_controls()
            elif self.mode == 'move':
                self.setup_move_controls()
            elif self.mode == 'probe':
                self.setup_probe_controls()
            elif self.mode == 'fit':
                self.setup_fit_controls()
            
            self.plot()
        
        def setup_refine_controls(self):
            frame = ttk.LabelFrame(self.controls_frame, text="Refine Controls", padding=10)
            frame.pack(fill=tk.X)
            
            ttk.Label(frame, text="Click 2 points on plot, then enter\ntheir actual WCS coordinates:").pack(anchor=tk.W)
            
            # Point 1: X, Y entries side by side with current position button
            ttk.Label(frame, text="Point 1:").pack(anchor=tk.W, pady=(10,0))
            p1_frame = ttk.Frame(frame)
            p1_frame.pack(anchor=tk.W)
            self.x1_entry = ttk.Entry(p1_frame, width=7)
            self.x1_entry.pack(side=tk.LEFT)
            ttk.Label(p1_frame, text=",").pack(side=tk.LEFT, padx=2)
            self.y1_entry = ttk.Entry(p1_frame, width=7)
            self.y1_entry.pack(side=tk.LEFT)
            tk.Button(p1_frame, text="Probe Pos", font=("TkDefaultFont", 8),
                      command=lambda: self.fill_current_pos(self.x1_entry, self.y1_entry)).pack(side=tk.LEFT, padx=(5,0))
            
            # Point 2: X, Y entries side by side with current position button
            ttk.Label(frame, text="Point 2:").pack(anchor=tk.W, pady=(10,0))
            p2_frame = ttk.Frame(frame)
            p2_frame.pack(anchor=tk.W)
            self.x2_entry = ttk.Entry(p2_frame, width=7)
            self.x2_entry.pack(side=tk.LEFT)
            ttk.Label(p2_frame, text=",").pack(side=tk.LEFT, padx=2)
            self.y2_entry = ttk.Entry(p2_frame, width=7)
            self.y2_entry.pack(side=tk.LEFT)
            tk.Button(p2_frame, text="Probe Pos", font=("TkDefaultFont", 8),
                      command=lambda: self.fill_current_pos(self.x2_entry, self.y2_entry)).pack(side=tk.LEFT, padx=(5,0))
            
            # Restore saved point values
            if self.refine_point1:
                self.x1_entry.insert(0, str(self.refine_point1[0]))
                self.y1_entry.insert(0, str(self.refine_point1[1]))
            if self.refine_point2:
                self.x2_entry.insert(0, str(self.refine_point2[0]))
                self.y2_entry.insert(0, str(self.refine_point2[1]))
            
            # Bind comma to move to next field (like tab)
            def on_comma(event):
                event.widget.tk_focusNext().focus()
                return "break"  # Prevent comma from being inserted
            
            self.x1_entry.bind(',', on_comma)
            self.y1_entry.bind(',', on_comma)
            self.x2_entry.bind(',', on_comma)
            self.y2_entry.bind(',', on_comma)
            
            ttk.Button(frame, text="Clear Selected",
                      command=self.clear_selected_points).pack(fill=tk.X, pady=(10,2))
            ttk.Button(frame, text="Apply Transformation",
                      command=self.apply_refine).pack(fill=tk.X, pady=2)
            
            self.clicked_points = []
            self.status_label.config(text="Click 2 points on the plot")
        
        def clear_selected_points(self):
            """Clear the selected points on the plot."""
            self.clicked_points = []
            self.plot()
            self.status_label.config(text="Selection cleared. Click 2 points on the plot")
        
        def fill_current_pos(self, x_entry, y_entry):
            """Fill entry fields with current WCS X and Y position."""
            if not self.connection.connected:
                messagebox.showwarning("Not Connected", "Please connect first")
                return
            
            try:
                status = self.connection.send_command("?")
                if 'WPos:' in status:
                    pos_str = status.split('WPos:')[1].split('|')[0]
                    coords = pos_str.split(',')
                    x = float(coords[0])
                    y = float(coords[1])
                    
                    x_entry.delete(0, tk.END)
                    x_entry.insert(0, f"{x:.3f}")
                    y_entry.delete(0, tk.END)
                    y_entry.insert(0, f"{y:.3f}")
            except TimeoutError as e:
                messagebox.showerror("Operation Timed Out", f"Get position timed out:\n{e}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to get position: {e}")
        
        def setup_move_controls(self):
            # Test move section
            move_frame = ttk.LabelFrame(self.controls_frame, text="Test Move", padding=10)
            move_frame.pack(fill=tk.X, pady=5)
            
            ttk.Label(move_frame, text="Click on the plot to move\nthe spindle & verify approximate\npart orientation.\n\nMoves at retract height.", 
                     wraplength=310).pack()
            
            self.status_label.config(text="Click on plot to test move")
        
        def setup_probe_controls(self):
            frame = ttk.LabelFrame(self.controls_frame, text="Probe Controls", padding=5)
            frame.pack(fill=tk.X)
            
            # Instructions
            ttk.Label(frame, text="Drag points on plot to adjust.\nClick +/- to change count.", 
                     font=("TkDefaultFont", 8), foreground="gray").pack(anchor=tk.W)
            
            ttk.Label(frame, text="Number of points:").pack(anchor=tk.W, pady=(5,0))
            
            n_frame = ttk.Frame(frame)
            n_frame.pack(fill=tk.X)
            
            self.n_label = ttk.Label(n_frame, text=str(self.n_probe_points), width=4)
            self.n_label.pack(side=tk.LEFT, padx=2)
            
            ttk.Button(n_frame, text="-", width=2,
                      command=lambda: self.adjust_n(-1)).pack(side=tk.LEFT, padx=1)
            ttk.Button(n_frame, text="+", width=2,
                      command=lambda: self.adjust_n(1)).pack(side=tk.LEFT, padx=1)
            
            ttk.Label(frame, text="Probe distance (mm):").pack(anchor=tk.W, pady=(5,0))
            
            d_frame = ttk.Frame(frame)
            d_frame.pack(fill=tk.X)
            
            self.d_label = ttk.Label(d_frame, text=f"{self.probe_distance:.1f}", width=2)
            self.d_label.pack(side=tk.LEFT)
            
            for txt, val in [("-1", -1.0), ("-.1", -0.1), ("+.1", 0.1), ("+1", 1.0)]:
                ttk.Button(d_frame, text=txt,
                          command=lambda v=val: self.adjust_d(v)).pack(side=tk.LEFT)
            
            ttk.Separator(frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=5)
            
            # Make probe button more prominent
            self.probe_btn = ttk.Button(frame, text="▶ Probe Now",
                      command=self.execute_probing)
            self.probe_btn.pack(fill=tk.X, pady=5)
            
            self.update_probe_visualization()
        
        def setup_fit_controls(self):
            frame = ttk.LabelFrame(self.controls_frame, text="Fit Controls", padding=5)
            frame.pack(fill=tk.X)
            
            # Rotation
            ttk.Label(frame, text="Rotation (°):").pack(anchor=tk.W)
            
            r_frame = ttk.Frame(frame)
            r_frame.pack(fill=tk.X)
            
            self.rot_label = ttk.Label(r_frame, text=f"{self.fit_rotation_angle:.2f}", width=4)
            self.rot_label.pack(side=tk.LEFT)
            
            for txt, val in [("-1", -1.0), ("-.1", -0.1), ("+.1", 0.1), ("+1", 1.0)]:
                ttk.Button(r_frame, text=txt,
                          command=lambda v=val: self.adjust_rot(v)).pack(side=tk.LEFT)
            
            # X translation
            ttk.Label(frame, text="X offset (mm):").pack(anchor=tk.W, pady=(5,0))
            
            tx_frame = ttk.Frame(frame)
            tx_frame.pack(fill=tk.X)
            
            self.tx_label = ttk.Label(tx_frame, text=f"{self.fit_translation[0]:.2f}", width=4)
            self.tx_label.pack(side=tk.LEFT)
            
            for txt, val in [("-1", -1.0), ("-.1", -0.1), ("+.1", 0.1), ("+1", 1.0)]:
                ttk.Button(tx_frame, text=txt,
                          command=lambda v=val: self.adjust_tx(v)).pack(side=tk.LEFT)
            
            # Y translation
            ttk.Label(frame, text="Y offset (mm):").pack(anchor=tk.W, pady=(5,0))
            
            ty_frame = ttk.Frame(frame)
            ty_frame.pack(fill=tk.X)
            
            self.ty_label = ttk.Label(ty_frame, text=f"{self.fit_translation[1]:.2f}", width=4)
            self.ty_label.pack(side=tk.LEFT)
            
            for txt, val in [("-1", -1.0), ("-.1", -0.1), ("+.1", 0.1), ("+1", 1.0)]:
                ttk.Button(ty_frame, text=txt,
                          command=lambda v=val: self.adjust_ty(v)).pack(side=tk.LEFT)
            
            if HAS_SCIPY:
                ttk.Button(frame, text="Auto Fit",
                          command=self.auto_fit).pack(fill=tk.X, pady=5)
            
            ttk.Button(frame, text="Update Machine WCS",
                      command=self.update_wcs).pack(fill=tk.X, pady=2)
            
            if len(self.probed_points_work) == 0:
                self.status_label.config(text="No probed points. Run probing first.")
            else:
                self.status_label.config(text=f"{len(self.probed_points_work)} probed points")
        
        def adjust_n(self, delta: int):
            self.n_probe_points = max(3, self.n_probe_points + delta)
            self.n_label.config(text=str(self.n_probe_points))
            self.update_probe_visualization()
            self.save_config()
        
        def adjust_d(self, delta: float):
            self.probe_distance = max(0.1, self.probe_distance + delta)
            self.d_label.config(text=f"{self.probe_distance:.1f}")
            self.update_probe_visualization()
            self.save_config()
        
        def adjust_rot(self, delta: float):
            self.fit_rotation_angle += delta
            self.rot_label.config(text=f"{self.fit_rotation_angle:.2f}")
            self.plot()
        
        def adjust_tx(self, delta: float):
            self.fit_translation[0] += delta
            self.tx_label.config(text=f"{self.fit_translation[0]:.2f}")
            self.plot()
        
        def adjust_ty(self, delta: float):
            self.fit_translation[1] += delta
            self.ty_label.config(text=f"{self.fit_translation[1]:.2f}")
            self.plot()
        
        def update_probe_visualization(self):
            if self.outer_curve_points is not None:
                self.PsSamp, self.NsSamp = self.sample_curve_uniform(
                    self.outer_curve_points, self.n_probe_points
                )
            self.plot()
        
        def sample_curve_uniform(self, points: np.ndarray, n_samples: int):
            """Sample n points uniformly along curve with inward normals."""
            segments = np.diff(points, axis=0)
            segment_lengths = np.linalg.norm(segments, axis=1)
            cumulative = np.concatenate([[0], np.cumsum(segment_lengths)])
            total_length = cumulative[-1]
            
            distances = np.linspace(0, total_length, n_samples, endpoint=False)
            
            PsSamp = []
            NsSamp = []
            
            for dist in distances:
                idx = np.searchsorted(cumulative, dist, side='right') - 1
                idx = min(idx, len(segments) - 1)
                
                dist_in_seg = dist - cumulative[idx]
                seg_len = segment_lengths[idx]
                
                t = dist_in_seg / seg_len if seg_len > 0 else 0
                
                p1 = points[idx]
                p2 = points[idx + 1]
                sampled = p1 + t * (p2 - p1)
                
                tangent = p2 - p1
                norm = np.linalg.norm(tangent)
                if norm > 0:
                    tangent = tangent / norm
                
                normal = np.array([-tangent[1], tangent[0]])
                
                PsSamp.append(sampled)
                NsSamp.append(normal)
            
            return np.array(PsSamp), np.array(NsSamp)
        
        def plot(self):
            self.ax.clear()
            
            if self.outer_curve_points is None:
                self.canvas.draw()
                return
            
            # Plot curve
            self.ax.plot(self.outer_curve_points[:, 0], self.outer_curve_points[:, 1],
                        'b-', linewidth=2, label='Curve')
            
            if self.mode == 'refine':
                # Plot clicked points
                if len(self.clicked_points) > 0:
                    clicked = np.array(self.clicked_points)
                    self.ax.scatter(clicked[:, 0], clicked[:, 1],
                                   c='cyan', s=200, marker='x', linewidths=3,
                                   zorder=10, label='Clicked')
                    for i, pt in enumerate(clicked):
                        self.ax.annotate(f'P{i+1}', xy=(pt[0], pt[1]),
                                        xytext=(5, 5), textcoords='offset points',
                                        fontsize=12, color='cyan', weight='bold')
            
            elif self.mode == 'probe':
                if self.PsSamp is not None and self.NsSamp is not None:
                    # Larger scatter points for easier dragging (s=150)
                    self.ax.scatter(self.PsSamp[:, 0], self.PsSamp[:, 1],
                                   c='red', s=150, zorder=5, label='Sample points (drag to move)',
                                   edgecolors='darkred', linewidths=2, picker=True)
                    
                    for i in range(len(self.PsSamp)):
                        start = self.PsSamp[i] + self.probe_distance * self.NsSamp[i]
                        
                        self.ax.arrow(self.PsSamp[i, 0], self.PsSamp[i, 1],
                                     self.NsSamp[i, 0] * self.probe_distance,
                                     self.NsSamp[i, 1] * self.probe_distance,
                                     head_width=0.3, head_length=0.2,
                                     fc='orange', ec='orange', alpha=0.5)
            
            elif self.mode == 'fit':
                if len(self.probed_points_work) > 0:
                    probed = np.array(self.probed_points_work)
                    self.ax.scatter(probed[:, 0], probed[:, 1],
                                   c='red', s=100, marker='o',
                                   zorder=10, label='Probed', alpha=0.7)
                
                # Plot fitted curve
                angle_rad = np.deg2rad(self.fit_rotation_angle)
                rot = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
                               [np.sin(angle_rad), np.cos(angle_rad)]])
                rot_inv = rot.T
                
                translated = self.outer_curve_points + self.fit_translation
                fitted = (rot_inv @ translated.T).T
                
                self.ax.plot(fitted[:, 0], fitted[:, 1],
                            'g--', linewidth=2, label='Fitted', alpha=0.8)
            
            self.ax.axis('equal')
            self.ax.grid(True, alpha=0.3)
            if self.mode != 'refine':
                self.ax.legend()
            self.ax.set_xlabel('X')
            self.ax.set_ylabel('Y')
            
            self.canvas.draw()
        
        def on_plot_click(self, event):
            if event.inaxes != self.ax:
                return
            
            if self.outer_curve_points is None:
                return
            
            if self.mode == 'refine':
                if len(self.clicked_points) < 2:
                    self.clicked_points.append([event.xdata, event.ydata])
                    self.status_label.config(
                        text=f"Point {len(self.clicked_points)} marked at ({event.xdata:.3f}, {event.ydata:.3f})"
                    )
                    self.plot()
                    
                    if len(self.clicked_points) == 2:
                        self.status_label.config(text="Enter coordinates and click Apply")
            
            elif self.mode == 'move':
                if not self.connection.connected:
                    messagebox.showwarning("Not Connected", "Please connect first")
                    return
                if not self.connection.probe_z_set:
                    messagebox.showerror(
                        "Probing Height Not Set",
                        "You must set the probing height before moving.\n\n"
                        "Go to the Settings panel and click 'Set'."
                    )
                    return
                try:
                    # Use unified settings from connection - stay at retract height
                    retract_z = self.connection.probe_z + self.connection.retract_height
                    
                    self.connection.begin_operation()
                    self.connection.move_z(retract_z)
                    self.connection.wait_for_idle()
                    self.connection.move_xy(event.xdata, event.ydata)
                    self.connection.wait_for_idle()
                    # Stay at retract height (don't descend to probe_z)
                    self.connection.end_operation()
                    self.status_label.config(text=f"Moved to ({event.xdata:.3f}, {event.ydata:.3f}) at Z={retract_z:.1f}")
                except ProbeCrash as e:
                    self.connection.end_operation()
                    messagebox.showerror("Probe Crash", f"Probe triggered during move:\n{e}")
                except AlarmError as e:
                    self.connection.end_operation()
                    messagebox.showerror("Machine Alarm", f"Machine entered alarm state:\n{e}")
                except TimeoutError as e:
                    self.connection.end_operation()
                    messagebox.showerror("Operation Timed Out", f"Move timed out:\n{e}")
                except Exception as e:
                    self.connection.end_operation()
                    messagebox.showerror("Move Error", f"Move failed:\n{e}")
            
            elif self.mode == 'probe':
                # Check if we clicked near a probe point to start dragging
                if self.PsSamp is not None and len(self.PsSamp) > 0:
                    click_pt = np.array([event.xdata, event.ydata])
                    distances = np.linalg.norm(self.PsSamp - click_pt, axis=1)
                    min_idx = np.argmin(distances)
                    
                    # Calculate threshold based on plot scale
                    xlim = self.ax.get_xlim()
                    ylim = self.ax.get_ylim()
                    plot_scale = max(xlim[1] - xlim[0], ylim[1] - ylim[0])
                    threshold = plot_scale * 0.02  # 2% of plot size
                    
                    if distances[min_idx] < threshold:
                        self.dragging_point_idx = min_idx
                        self.status_label.config(text=f"Dragging point {min_idx + 1}")
        
        def on_plot_motion(self, event):
            """Handle mouse motion for dragging probe points along the curve."""
            if self.mode != 'probe' or self.dragging_point_idx is None:
                return
            if event.inaxes != self.ax:
                return
            if self.PsSamp is None or self.outer_curve_points is None:
                return
            if len(self.outer_curve_points) < 2:
                return
            
            mouse_pt = np.array([event.xdata, event.ydata])
            
            # Find closest point along any segment of the curve
            best_dist = float('inf')
            best_point = None
            best_seg_idx = 0
            
            n_curve = len(self.outer_curve_points)
            for i in range(n_curve):
                # Segment from point i to point (i+1) % n_curve
                p1 = self.outer_curve_points[i]
                p2 = self.outer_curve_points[(i + 1) % n_curve]
                
                # Vector from p1 to p2
                seg = p2 - p1
                seg_len_sq = np.dot(seg, seg)
                
                if seg_len_sq < 1e-12:
                    # Degenerate segment, just use p1
                    closest = p1
                    t = 0.0
                else:
                    # Project mouse point onto the line, clamp to segment
                    t = np.dot(mouse_pt - p1, seg) / seg_len_sq
                    t = max(0.0, min(1.0, t))
                    closest = p1 + t * seg
                
                dist = np.linalg.norm(mouse_pt - closest)
                if dist < best_dist:
                    best_dist = dist
                    best_point = closest
                    best_seg_idx = i
                    best_t = t
            
            # Snap to the closest point on the curve
            self.PsSamp[self.dragging_point_idx] = best_point.copy()
            
            # Compute normal from the segment direction
            p1 = self.outer_curve_points[best_seg_idx]
            p2 = self.outer_curve_points[(best_seg_idx + 1) % n_curve]
            tangent = p2 - p1
            tangent_len = np.linalg.norm(tangent)
            if tangent_len > 1e-9:
                tangent = tangent / tangent_len
                # Normal is perpendicular, pointing inward (assuming CCW curve)
                self.NsSamp[self.dragging_point_idx] = np.array([-tangent[1], tangent[0]])
            
            self.plot()
        
        def on_plot_release(self, event):
            """Handle mouse release to end dragging."""
            if self.dragging_point_idx is not None:
                self.status_label.config(text=f"Point {self.dragging_point_idx + 1} moved along curve")
                self.dragging_point_idx = None
        
        def apply_refine(self):
            if len(self.clicked_points) < 2:
                messagebox.showwarning("Error", "Need 2 clicked points")
                return
            
            try:
                x1 = float(self.x1_entry.get())
                y1 = float(self.y1_entry.get())
                x2 = float(self.x2_entry.get())
                y2 = float(self.y2_entry.get())
            except ValueError:
                messagebox.showerror("Error", "Invalid coordinates")
                return
            
            # Save point values for next time (in memory and to file)
            self.refine_point1 = (x1, y1)
            self.refine_point2 = (x2, y2)
            self.save_config()
            
            clicked = np.array(self.clicked_points)
            actual = np.array([[x1, y1], [x2, y2]])
            
            # Calculate transformation
            clicked_center = np.mean(clicked, axis=0)
            actual_center = np.mean(actual, axis=0)
            
            clicked_centered = clicked - clicked_center
            actual_centered = actual - actual_center
            
            H = clicked_centered.T @ actual_centered
            U, S, Vt = np.linalg.svd(H)
            rot = Vt.T @ U.T
            
            if np.linalg.det(rot) < 0:
                Vt[-1, :] *= -1
                rot = Vt.T @ U.T
            
            trans = actual_center - (rot @ clicked_center)
            
            # Apply
            self.rotation_matrix = rot @ self.rotation_matrix
            self.translation = rot @ self.translation + trans
            
            self.outer_curve_points = (self.rotation_matrix @ self.original_outer_curve.T).T + self.translation
            
            self.clicked_points = []
            
            angle = np.rad2deg(np.arctan2(self.rotation_matrix[1, 0], self.rotation_matrix[0, 0]))
            self.status_label.config(
                text=f"Applied: rot={angle:.2f}°, trans=({self.translation[0]:.3f}, {self.translation[1]:.3f})"
            )
            
            self.plot()
        
        def execute_probing(self):
            print("=== EXECUTE_PROBING button clicked")
            try:
                if not self.connection.check_probe_ready():
                    return
            except Exception as e:
                print(f"=== check_probe_ready failed: {e}")
                messagebox.showerror("Connection Error", f"Failed to check probe status:\n{e}")
                return
            
            if self.PsSamp is None or self.NsSamp is None:
                return
            
            # Use unified settings from connection
            probe_z = self.connection.probe_z
            retract_z = probe_z + self.connection.retract_height
            slow_speed = self.connection.slow_probe_speed
            
            # Calculate probe positions
            probe_positions = self.PsSamp + self.probe_distance * self.NsSamp
            probe_normals = -self.NsSamp
            
            self.status_label.config(text="Probing...")
            self.update()
            
            self.connection.begin_operation()
            
            probed = []
            
            try:
                self.connection.send_command("G90")
                
                for i in range(len(probe_positions)):
                    self.status_label.config(text=f"Probing {i+1}/{len(probe_positions)}...")
                    self.update()
                    
                    start = probe_positions[i]
                    direction = probe_normals[i]
                    
                    # Move to retract Z (probe_z + retract_height)
                    self.connection.move_z(retract_z)
                    self.connection.wait_for_idle()
                    
                    # Move to start XY
                    self.connection.move_xy(start[0], start[1])
                    self.connection.wait_for_idle()
                    
                    # Move to probe Z height
                    self.connection.move_z(probe_z)
                    self.connection.wait_for_idle()
                    
                    # Probe using unified max_probe_distance
                    max_probe_dist = self.connection.max_probe_distance
                    probe_x = direction[0] * max_probe_dist
                    probe_y = direction[1] * max_probe_dist
                    
                    # Calculate timeout: probe distance at F300 + buffer
                    fast_timeout = max(10.0, (max_probe_dist / 300.0) * 60.0 + 5.0)
                    
                    # Fast probe
                    fast_response = self.connection.send_command(
                        f"G38.2 X{probe_x:.4f} Y{probe_y:.4f} F300",
                        timeout=fast_timeout
                    )
                    
                    # Wait for probe to release from surface
                    time.sleep(0.3)
                    
                    # Check for no contact during fast probe
                    if 'PRB:' in fast_response:
                        prb_part = fast_response.split('PRB:')[1]
                        if ':0' in prb_part or prb_part.strip().endswith(':0'):
                            raise ProbeNoContact(f"Fast probe at point {i+1} did not make contact")
                    
                    self.connection.wait_for_idle()
                    
                    # Retract (check for crash on retract)
                    self.connection.send_command("G91")
                    retract_response = self.connection.send_command(f"G0 X{-direction[0]*0.5:.4f} Y{-direction[1]*0.5:.4f} F500")
                    if 'PRB:' in retract_response:
                        raise ProbeCrash(f"Probe triggered during retract after point {i+1}")
                    self.connection.send_command("G90")
                    self.connection.wait_for_idle()
                    
                    # Slow probe using unified slow_probe_speed
                    slow_timeout = max(10.0, (1.0 / slow_speed) * 60.0 + 5.0)
                    probe_x_slow = direction[0] * 0.6
                    probe_y_slow = direction[1] * 0.6
                    slow_response = self.connection.send_command(
                        f"G38.2 X{probe_x_slow:.4f} Y{probe_y_slow:.4f} F{slow_speed:.0f}",
                        timeout=slow_timeout
                    )
                    
                    # Wait for probe to release from surface
                    time.sleep(0.3)
                    
                    # Check for no contact during slow probe
                    if 'PRB:' in slow_response:
                        prb_part = slow_response.split('PRB:')[1]
                        if ':0' in prb_part or prb_part.strip().endswith(':0'):
                            raise ProbeNoContact(f"Slow probe at point {i+1} did not make contact")
                    
                    # Get position
                    status = self.connection.send_command("?")
                    if 'WPos:' in status:
                        pos_str = status.split('WPos:')[1].split('|')[0]
                        coords = pos_str.split(',')
                        x, y = float(coords[0]), float(coords[1])
                        probed.append([x, y])
                    else:
                        raise CNCError(f"Could not get position after probe at point {i+1}")
                    
                    # Final retract (check for crash)
                    self.connection.send_command("G91")
                    final_retract = self.connection.send_command(f"G0 X{-direction[0]*0.3:.4f} Y{-direction[1]*0.3:.4f} F500")
                    if 'PRB:' in final_retract:
                        raise ProbeCrash(f"Probe triggered during final retract after point {i+1}")
                    self.connection.send_command("G90")
                    
                    time.sleep(0.1)
                
                # Return to retract Z
                self.connection.move_z(retract_z)
                self.connection.wait_for_idle()
                
                self.probed_points_work = probed
                self.status_label.config(text=f"Probed {len(probed)} points. Switch to Fit mode.")
                
            except ProbeNoContact as e:
                self.status_label.config(text=f"No Contact: {e}")
                messagebox.showerror("Probe No Contact", f"Probe failed - no contact:\n{e}")
            except ProbeCrash as e:
                self.status_label.config(text=f"Crash: {e}")
                messagebox.showerror("Probe Crash", f"Probe triggered during move:\n{e}")
            except AlarmError as e:
                self.status_label.config(text=f"Alarm: {e}")
                messagebox.showerror("Machine Alarm", f"Machine entered alarm state:\n{e}")
            except TimeoutError as e:
                self.status_label.config(text=f"Timeout: {e}")
                messagebox.showerror("Operation Timed Out", f"Probing timed out:\n{e}")
            except Exception as e:
                self.status_label.config(text=f"Error: {e}")
                messagebox.showerror("Probe Error", f"Probing failed:\n{e}")
            finally:
                self.connection.end_operation()
        
        def auto_fit(self):
            if not HAS_SCIPY:
                messagebox.showerror("Not Available", "scipy not installed")
                return
            
            if len(self.probed_points_work) == 0:
                messagebox.showwarning("No Data", "No probed points available")
                return
            
            self.status_label.config(text="Optimizing...")
            self.update()
            
            outer_curve = self.outer_curve_points.copy()
            probe_points = np.array(self.probed_points_work)
            
            def compute_error(params):
                theta, x0, y0, s = params
                cos_t = np.cos(theta)
                sin_t = np.sin(theta)
                rot = np.array([[cos_t, -sin_t], [sin_t, cos_t]])
                
                p0 = np.array([x0, y0])
                transformed = -p0 + (rot @ (s * probe_points).T).T
                
                seg_p1 = outer_curve[:-1]
                seg_p2 = outer_curve[1:]
                v = seg_p2 - seg_p1
                vv = np.sum(v * v, axis=1)
                
                pts_exp = transformed[:, np.newaxis, :]
                p1_exp = seg_p1[np.newaxis, :, :]
                
                w = pts_exp - p1_exp
                wv = np.sum(w * v[np.newaxis, :, :], axis=2)
                t = np.clip(wv / (vv[np.newaxis, :] + 1e-20), 0.0, 1.0)
                
                nearest = p1_exp + t[:, :, np.newaxis] * v[np.newaxis, :, :]
                dist_sq = np.sum((pts_exp - nearest)**2, axis=2)
                dist_sq = np.maximum(dist_sq, 1e-20)
                
                recip_sq = 1.0 / dist_sq
                sum_recip = np.sum(recip_sq, axis=1)
                point_errors = 1.0 / (sum_recip + 1e-20)
                
                return np.sum(point_errors)
            
            # Initial guess
            theta0 = np.deg2rad(self.fit_rotation_angle)
            x0_init = self.fit_translation[0]
            y0_init = self.fit_translation[1]
            
            bounds = [(-10, 10), (-10, 10), (-10, 10), (0.9, 1.1)]
            
            result = minimize(
                compute_error,
                [theta0, x0_init, y0_init, 1.0],
                method='L-BFGS-B',
                bounds=bounds,
                options={'maxiter': 15000}
            )
            
            if result.success:
                theta, x0, y0, s = result.x
                self.fit_rotation_angle = np.rad2deg(theta)
                self.fit_translation[0] = x0
                self.fit_translation[1] = y0
                
                self.rot_label.config(text=f"{self.fit_rotation_angle:.2f}")
                self.tx_label.config(text=f"{self.fit_translation[0]:.2f}")
                self.ty_label.config(text=f"{self.fit_translation[1]:.2f}")
                
                self.status_label.config(text=f"Fit complete! Scale={s:.4f}")
                self.plot()
            else:
                self.status_label.config(text="Optimization failed")
        
        def update_wcs(self):
            if not self.connection.connected:
                messagebox.showwarning("Not Connected", "Please connect first")
                return
            
            try:
                # Calculate combined transformation
                refine_angle = np.arctan2(self.rotation_matrix[1, 0], self.rotation_matrix[0, 0])
                fit_angle_rad = np.deg2rad(self.fit_rotation_angle)
                
                fit_rot_inv = np.array([[np.cos(-fit_angle_rad), -np.sin(-fit_angle_rad)],
                                        [np.sin(-fit_angle_rad), np.cos(-fit_angle_rad)]])
                
                combined_rot = fit_rot_inv @ self.rotation_matrix
                combined_angle = np.rad2deg(np.arctan2(combined_rot[1, 0], combined_rot[0, 0]))
                combined_trans = fit_rot_inv @ self.translation + self.fit_translation
                
                # Get current G54
                self.connection.begin_operation()
                response = self.connection.send_command("$#")
                
                g54_match = re.search(
                    r'\[G54:([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+)',
                    response
                )
                
                if g54_match:
                    cur_x = float(g54_match.group(1))
                    cur_y = float(g54_match.group(2))
                    cur_z = float(g54_match.group(3))
                    
                    new_x = cur_x + combined_trans[0]
                    new_y = cur_y + combined_trans[1]
                    
                    self.connection.send_command(
                        f"G10 L2 P1 X{new_x:.4f} Y{new_y:.4f} Z{cur_z:.4f} R{combined_angle:.4f}"
                    )
                    
                    # Update stored rotation value
                    self.connection.wcs_rotation = combined_angle
                    
                    self.connection.end_operation()
                    
                    self.status_label.config(
                        text=f"WCS updated: R={combined_angle:.2f}°, X+={combined_trans[0]:.3f}, Y+={combined_trans[1]:.3f}"
                    )
                else:
                    self.connection.end_operation()
                    messagebox.showerror("Error", "Could not read current G54 offsets")
                    
            except AlarmError as e:
                self.connection.end_operation()
                messagebox.showerror("Machine Alarm", f"Machine entered alarm state:\n{e}")
            except TimeoutError as e:
                self.connection.end_operation()
                messagebox.showerror("Operation Timed Out", f"Update WCS timed out:\n{e}")
            except Exception as e:
                self.connection.end_operation()
                messagebox.showerror("Error", f"Update WCS failed:\n{e}")


# =============================================================================
# SVG Smoother Tab
# =============================================================================

if HAS_SVG_SMOOTHER and HAS_MATPLOTLIB:
    
    # SVG Smoother Settings
    SVG_SIMPLIFY_TOL = 0.01
    SVG_SPLINE_STEP = 0.005
    SVG_SPLINE_MIN_SAMPLES = 120
    SVG_SPLINE_MAX_SAMPLES = 2500
    SVG_MEASURED_ROC_SAMPLE_STEP = 0.5
    SVG_MAX_RELAX_ITERS = 800

    # Coaster GCode defaults
    COASTER_DEFAULT_DIAMETER = 95.0
    COASTER_DEFAULT_STOCK_W = 110.0
    COASTER_DEFAULT_STOCK_H = 110.0
    COASTER_DEFAULT_STOCK_T = 12.0
    COASTER_DEFAULT_PLUG_W = 110.0
    COASTER_DEFAULT_PLUG_H = 110.0
    COASTER_DEFAULT_PLUG_T = 6.0
    COASTER_DEFAULT_TARGET_T = 6.0
    COASTER_DEFAULT_INLAY_DEPTH = 1.5
    COASTER_DEFAULT_GLUE_GAP = 0.1
    COASTER_DEFAULT_RPM = 10000
    COASTER_DEFAULT_RETRACT = 5.0
    COASTER_DEFAULT_TOOLS = [
        {"diameter": "6.35", "tool_num": "1", "feedrate": "1500", "ramp_feed": "500", "contour_feed": "800", "stepdown": "1.0", "rpm": "10000", "stepover": "50"},
        {"diameter": "3.175", "tool_num": "2", "feedrate": "1000", "ramp_feed": "333", "stepdown": "0.5", "rpm": "10000", "stepover": "50"},
        {"diameter": "1.0", "tool_num": "3", "feedrate": "600", "ramp_feed": "200", "stepdown": "0.3", "rpm": "10000", "stepover": "50"},
    ]
    COASTER_SETTINGS_FILE = os.path.expanduser("~/.carvera_coaster_settings.json")

    # Matrix helper functions
    def _svg_mat_identity():
        return np.eye(3, dtype=float)
    
    def _svg_mat_mul(A, B):
        return A @ B
    
    def _svg_mat_translate(tx, ty=0.0):
        M = _svg_mat_identity()
        M[0, 2] = float(tx)
        M[1, 2] = float(ty)
        return M
    
    def _svg_mat_scale(sx, sy=None):
        if sy is None:
            sy = sx
        M = _svg_mat_identity()
        M[0, 0] = float(sx)
        M[1, 1] = float(sy)
        return M
    
    def _svg_mat_rotate(angle_deg, cx=0.0, cy=0.0):
        a = np.deg2rad(float(angle_deg))
        c, s = np.cos(a), np.sin(a)
        R = _svg_mat_identity()
        R[0, 0], R[0, 1], R[1, 0], R[1, 1] = c, -s, s, c
        if cx != 0.0 or cy != 0.0:
            return _svg_mat_mul(_svg_mat_translate(cx, cy), _svg_mat_mul(R, _svg_mat_translate(-cx, -cy)))
        return R
    
    def _svg_mat_matrix(a, b, c, d, e, f):
        M = _svg_mat_identity()
        M[0, 0], M[0, 1], M[0, 2] = float(a), float(c), float(e)
        M[1, 0], M[1, 1], M[1, 2] = float(b), float(d), float(f)
        return M
    
    _svg_num_re = re.compile(r"[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?")
    
    def _svg_parse_floats(s):
        return [float(x) for x in _svg_num_re.findall(s)]
    
    def svg_parse_transform(transform_str):
        if not transform_str or not transform_str.strip():
            return _svg_mat_identity()
        func_re = re.compile(r"([a-zA-Z]+)\s*\(([^)]*)\)")
        funcs = func_re.findall(transform_str.strip())
        if not funcs:
            return _svg_mat_identity()
        M = _svg_mat_identity()
        for name, args in funcs:
            vals = _svg_parse_floats(args)
            if name == "matrix" and len(vals) >= 6:
                T = _svg_mat_matrix(*vals[:6])
            elif name == "translate":
                T = _svg_mat_translate(vals[0] if vals else 0, vals[1] if len(vals) > 1 else 0)
            elif name == "scale":
                T = _svg_mat_scale(vals[0] if vals else 1, vals[1] if len(vals) > 1 else None)
            elif name == "rotate" and vals:
                T = _svg_mat_rotate(vals[0], vals[1] if len(vals) > 2 else 0, vals[2] if len(vals) > 2 else 0)
            else:
                T = _svg_mat_identity()
            M = _svg_mat_mul(T, M)
        return M
    
    def svg_apply_affine(points, M):
        pts = np.asarray(points, dtype=float)
        if len(pts) == 0:
            return pts
        ones = np.ones((pts.shape[0], 1), dtype=float)
        h = np.hstack([pts, ones])
        return (M @ h.T).T[:, :2]
    
    # Bezier and curve functions
    @jit(nopython=True, fastmath=True, cache=True)
    def _svg_bernstein(n, i, t):
        binom = 1.0
        for k in range(1, i + 1):
            binom = binom * (n - i + k) / k
        return binom * (t ** i) * ((1.0 - t) ** (n - i))
    
    @jit(nopython=True, fastmath=True, cache=True)
    def _svg_eval_bezier(control_points, t_values):
        n = len(control_points) - 1
        m = len(t_values)
        out = np.zeros((m, 2))
        for idx in range(m):
            t = t_values[idx]
            for j in range(n + 1):
                b = _svg_bernstein(n, j, t)
                out[idx, 0] += b * control_points[j, 0]
                out[idx, 1] += b * control_points[j, 1]
        return out
    
    class SVGBezierCurve:
        def __init__(self, control_points):
            self.control_points = np.array(control_points, dtype=np.float64)
        
        def evaluate(self, t):
            t = np.atleast_1d(t).astype(np.float64)
            pts = _svg_eval_bezier(self.control_points, t)
            return pts[0] if len(t) == 1 else pts
        
        def arc_length(self, num_samples=60):
            t = np.linspace(0, 1, num_samples)
            pts = self.evaluate(t)
            return float(np.sum(np.sqrt(np.sum(np.diff(pts, axis=0) ** 2, axis=1))))
    
    @jit(nopython=True, fastmath=True, cache=True)
    def _svg_compute_roc(p1, p2, p3):
        ax, ay = p2[0] - p1[0], p2[1] - p1[1]
        bx, by = p3[0] - p2[0], p3[1] - p2[1]
        cx, cy = p3[0] - p1[0], p3[1] - p1[1]
        a = (ax*ax + ay*ay) ** 0.5
        b = (bx*bx + by*by) ** 0.5
        c = (cx*cx + cy*cy) ** 0.5
        s = 0.5 * (a + b + c)
        area_sq = s * (s - a) * (s - b) * (s - c)
        if area_sq < 1e-12:
            return 1e30
        return (a * b * c) / (4.0 * (area_sq ** 0.5))
    
    @jit(nopython=True, fastmath=True, cache=True)
    def _svg_relax_curve(pts, min_roc, is_closed, max_iters):
        n = pts.shape[0]
        if n < 3:
            return pts
        for _ in range(max_iters):
            modified = False
            if is_closed:
                for i in range(n):
                    im1 = i - 1 if i > 0 else n - 1
                    ip1 = i + 1 if i < n - 1 else 0
                    if _svg_compute_roc(pts[im1], pts[i], pts[ip1]) < min_roc:
                        pts[i, 0] = 0.5 * (pts[im1, 0] + pts[ip1, 0])
                        pts[i, 1] = 0.5 * (pts[im1, 1] + pts[ip1, 1])
                        modified = True
            else:
                for i in range(1, n - 1):
                    if _svg_compute_roc(pts[i-1], pts[i], pts[i+1]) < min_roc:
                        pts[i, 0] = 0.5 * (pts[i-1, 0] + pts[i+1, 0])
                        pts[i, 1] = 0.5 * (pts[i-1, 1] + pts[i+1, 1])
                        modified = True
            if not modified:
                break
        return pts
    
    @jit(nopython=True, fastmath=True, cache=True)
    def _svg_roc_along_polyline(points, is_closed):
        n = points.shape[0]
        rocs = np.empty(n, dtype=np.float64)
        if n < 3:
            for i in range(n):
                rocs[i] = 1e30
            return rocs
        if is_closed:
            for i in range(n):
                im1 = i - 1 if i > 0 else n - 1
                ip1 = i + 1 if i < n - 1 else 0
                rocs[i] = _svg_compute_roc(points[im1], points[i], points[ip1])
        else:
            rocs[0] = rocs[n-1] = 1e30
            for i in range(1, n - 1):
                rocs[i] = _svg_compute_roc(points[i-1], points[i], points[i+1])
        return rocs
    
    def _svg_is_closed(points, tol=None):
        pts = np.asarray(points, dtype=float)
        if len(pts) < 2:
            return False
        if tol is None:
            d = np.sqrt(np.sum(np.diff(pts, axis=0) ** 2, axis=1))
            tol = 0.01 * (np.mean(d) if len(d) else 1.0)
        return np.linalg.norm(pts[0] - pts[-1]) < tol
    
    def _svg_resample_uniform(points, spacing):
        pts = np.asarray(points, dtype=float)
        if len(pts) < 2:
            return pts
        diffs = np.diff(pts, axis=0)
        seg = np.sqrt(np.sum(diffs**2, axis=1))
        cum = np.concatenate([[0.0], np.cumsum(seg)])
        total = cum[-1]
        if total < spacing * 0.1:
            return pts[:2] if len(pts) >= 2 else pts
        m = max(3, int(np.floor(total / spacing)) + 1)
        target = np.linspace(0.0, total, m)
        idx = np.clip(np.searchsorted(cum, target, side="right") - 1, 0, len(cum) - 2)
        denom = (cum[idx + 1] - cum[idx]) + 1e-12
        lt = np.clip((target - cum[idx]) / denom, 0.0, 1.0)
        out = (1.0 - lt)[:, None] * pts[idx] + lt[:, None] * pts[idx + 1]
        out[0], out[-1] = pts[0], pts[-1]
        return out
    
    def _svg_simplify_rdp(points, tol):
        pts = np.asarray(points, dtype=float)
        n = len(pts)
        if n <= 2:
            return pts
        tol2 = tol * tol
        keep = np.zeros(n, dtype=bool)
        keep[0] = keep[-1] = True
        stack = [(0, n - 1)]
        while stack:
            i0, i1 = stack.pop()
            A, B = pts[i0], pts[i1]
            max_d2, idx = -1.0, -1
            for i in range(i0 + 1, i1):
                AB = B - A
                denom = AB[0]*AB[0] + AB[1]*AB[1]
                if denom <= 1e-24:
                    d = pts[i] - A
                    d2 = d[0]*d[0] + d[1]*d[1]
                else:
                    t = np.clip(((pts[i][0]-A[0])*AB[0] + (pts[i][1]-A[1])*AB[1]) / denom, 0, 1)
                    proj = A + t * AB
                    d = pts[i] - proj
                    d2 = d[0]*d[0] + d[1]*d[1]
                if d2 > max_d2:
                    max_d2, idx = d2, i
            if max_d2 > tol2 and idx != -1:
                keep[idx] = True
                stack.append((i0, idx))
                stack.append((idx, i1))
        return pts[keep] if np.sum(keep) >= 2 else pts[[0, -1]]
    
    def _svg_simplify_polyline(points, tol, closed=False):
        pts = np.asarray(points, dtype=float)
        if len(pts) < 3:
            return pts
        if closed:
            if np.linalg.norm(pts[0] - pts[-1]) < 1e-9:
                pts0 = pts[:-1]
            else:
                pts0 = pts
            k = int(np.argmin(pts0[:, 0] + pts0[:, 1]))
            ptsr = np.vstack([pts0[k:], pts0[:k], pts0[k:k+1]])
            simp = _svg_simplify_rdp(ptsr, tol)
            if np.linalg.norm(simp[0] - simp[-1]) > 1e-9:
                simp = np.vstack([simp, simp[0]])
            return simp
        return _svg_simplify_rdp(pts, tol)
    
    def _svg_polyline_length(points):
        pts = np.asarray(points, dtype=float)
        if len(pts) < 2:
            return 0.0
        return float(np.sum(np.sqrt(np.sum(np.diff(pts, axis=0)**2, axis=1))))
    
    def _svg_fit_spline(points, smoothing=0, num_samples=None, closed=False):
        pts = np.asarray(points, dtype=float)
        # Remove near-duplicate points
        if len(pts) >= 2:
            step = np.linalg.norm(np.diff(pts, axis=0), axis=1)
            keep = np.hstack([[True], step > 1e-9])
            pts = pts[keep]
        if closed and len(pts) >= 2 and np.linalg.norm(pts[0] - pts[-1]) < 1e-6:
            pts = pts[:-1]
        n = len(pts)
        if n < 3:
            return pts
        k = min(3, n - 1)
        tck, _u = splprep([pts[:, 0], pts[:, 1]], s=smoothing, k=k, per=bool(closed))
        if num_samples is None:
            L = _svg_polyline_length(pts)
            num_samples = int(np.clip(L / max(SVG_SPLINE_STEP, 1e-9), SVG_SPLINE_MIN_SAMPLES, SVG_SPLINE_MAX_SAMPLES))
            num_samples = max(num_samples, 3)
        u_new = np.linspace(0, 1, num_samples)
        x_new, y_new = splev(u_new, tck)
        return np.column_stack([x_new, y_new])
    
    def _svg_densify(points, spacing):
        pts = np.asarray(points, dtype=float)
        if len(pts) < 2:
            return pts
        dense = [pts[0]]
        for i in range(len(pts) - 1):
            p1, p2 = pts[i], pts[i + 1]
            dist = np.linalg.norm(p2 - p1)
            num = max(1, int(np.ceil(dist / spacing)))
            for j in range(1, num + 1):
                t = j / num
                dense.append((1 - t) * p1 + t * p2)
        return np.asarray(dense, dtype=float)
    
    def _svg_sample_bezier_uniform(bezier, spacing):
        arc = bezier.arc_length()
        if arc < spacing * 0.1:
            return np.array([bezier.control_points[0], bezier.control_points[-1]], dtype=float)
        num = max(3, int(np.ceil(arc / spacing)) + 1)
        return _svg_resample_uniform(bezier.evaluate(np.linspace(0, 1, num)), spacing)
    
    def _svg_sample_path_uniform(path_segments, spacing, join_tol=1e-9):
        pieces = []
        last = None
        for bez in path_segments:
            seg_pts = _svg_sample_bezier_uniform(bez, spacing)
            if last is not None and len(seg_pts) and np.linalg.norm(seg_pts[0] - last) < join_tol:
                seg_pts = seg_pts[1:]
            if len(seg_pts):
                pieces.append(seg_pts)
                last = seg_pts[-1]
        return np.vstack(pieces) if pieces else np.zeros((0, 2), dtype=float)
    
    def _svg_circle_to_cubics(cx, cy, r):
        k = 0.5522847498307936
        cx, cy, r = float(cx), float(cy), float(r)
        p = lambda x, y: np.array([x, y], dtype=float)
        return [
            np.vstack([p(cx+r, cy), p(cx+r, cy+k*r), p(cx+k*r, cy+r), p(cx, cy+r)]),
            np.vstack([p(cx, cy+r), p(cx-k*r, cy+r), p(cx-r, cy+k*r), p(cx-r, cy)]),
            np.vstack([p(cx-r, cy), p(cx-r, cy-k*r), p(cx-k*r, cy-r), p(cx, cy-r)]),
            np.vstack([p(cx, cy-r), p(cx+k*r, cy-r), p(cx+r, cy-k*r), p(cx+r, cy)]),
        ]
    
    def _svg_ellipse_to_cubics(cx, cy, rx, ry):
        k = 0.5522847498307936
        cx, cy, rx, ry = float(cx), float(cy), float(rx), float(ry)
        p = lambda x, y: np.array([x, y], dtype=float)
        return [
            np.vstack([p(cx+rx, cy), p(cx+rx, cy+k*ry), p(cx+k*rx, cy+ry), p(cx, cy+ry)]),
            np.vstack([p(cx, cy+ry), p(cx-k*rx, cy+ry), p(cx-rx, cy+k*ry), p(cx-rx, cy)]),
            np.vstack([p(cx-rx, cy), p(cx-rx, cy-k*ry), p(cx-k*rx, cy-ry), p(cx, cy-ry)]),
            np.vstack([p(cx, cy-ry), p(cx+k*rx, cy-ry), p(cx+rx, cy-k*ry), p(cx+rx, cy)]),
        ]
    
    def _svg_as_float(x, default=0.0):
        try:
            return float(x)
        except:
            return float(default)
    
    def _svg_strip_ns(tag):
        return tag.split("}", 1)[-1] if "}" in tag else tag
    
    def _svg_load_paths(filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        svg_root_attribs = {k: root.attrib[k] for k in ("width", "height", "viewBox", "preserveAspectRatio") if k in root.attrib}
        all_paths = []
        
        def visit(node, M_parent):
            tag = _svg_strip_ns(node.tag)
            tstr = node.attrib.get("transform", "")
            M_here = _svg_mat_mul(svg_parse_transform(tstr), M_parent) if tstr else M_parent
            
            if tag == "path":
                d = node.attrib.get("d", "").strip()
                if d:
                    path_obj = parse_path(d)
                    segs = []
                    for seg in path_obj:
                        if isinstance(seg, CubicBezier):
                            cps = np.array([[seg.start.real, seg.start.imag], [seg.control1.real, seg.control1.imag],
                                           [seg.control2.real, seg.control2.imag], [seg.end.real, seg.end.imag]], dtype=float)
                            segs.append(SVGBezierCurve(svg_apply_affine(cps, M_here)))
                        elif isinstance(seg, Line):
                            cps = np.array([[seg.start.real, seg.start.imag], [seg.end.real, seg.end.imag]], dtype=float)
                            segs.append(SVGBezierCurve(svg_apply_affine(cps, M_here)))
                    if segs:
                        all_paths.append(segs)
            elif tag == "circle":
                r = _svg_as_float(node.attrib.get("r", 0))
                if r > 0:
                    cx, cy = _svg_as_float(node.attrib.get("cx", 0)), _svg_as_float(node.attrib.get("cy", 0))
                    all_paths.append([SVGBezierCurve(svg_apply_affine(cps, M_here)) for cps in _svg_circle_to_cubics(cx, cy, r)])
            elif tag == "ellipse":
                rx, ry = _svg_as_float(node.attrib.get("rx", 0)), _svg_as_float(node.attrib.get("ry", 0))
                if rx > 0 and ry > 0:
                    cx, cy = _svg_as_float(node.attrib.get("cx", 0)), _svg_as_float(node.attrib.get("cy", 0))
                    all_paths.append([SVGBezierCurve(svg_apply_affine(cps, M_here)) for cps in _svg_ellipse_to_cubics(cx, cy, rx, ry)])
            elif tag == "line":
                cps = np.array([[_svg_as_float(node.attrib.get("x1", 0)), _svg_as_float(node.attrib.get("y1", 0))],
                               [_svg_as_float(node.attrib.get("x2", 0)), _svg_as_float(node.attrib.get("y2", 0))]], dtype=float)
                all_paths.append([SVGBezierCurve(svg_apply_affine(cps, M_here))])
            elif tag in ("polyline", "polygon"):
                vals = _svg_parse_floats(node.attrib.get("points", ""))
                if len(vals) >= 4:
                    pts = np.array(vals[:len(vals)//2*2], dtype=float).reshape(-1, 2)
                    if tag == "polygon" and np.linalg.norm(pts[0] - pts[-1]) > 1e-9:
                        pts = np.vstack([pts, pts[0]])
                    pts = svg_apply_affine(pts, M_here)
                    segs = [SVGBezierCurve(np.array([pts[i], pts[i+1]], dtype=float)) for i in range(len(pts)-1)]
                    if segs:
                        all_paths.append(segs)
            elif tag == "rect":
                w, h = _svg_as_float(node.attrib.get("width", 0)), _svg_as_float(node.attrib.get("height", 0))
                if w > 0 and h > 0:
                    x, y = _svg_as_float(node.attrib.get("x", 0)), _svg_as_float(node.attrib.get("y", 0))
                    pts = np.array([[x, y], [x+w, y], [x+w, y+h], [x, y+h], [x, y]], dtype=float)
                    pts = svg_apply_affine(pts, M_here)
                    segs = [SVGBezierCurve(np.array([pts[i], pts[i+1]], dtype=float)) for i in range(4)]
                    all_paths.append(segs)
            for child in list(node):
                visit(child, M_here)
        
        visit(root, _svg_mat_identity())
        return all_paths, svg_root_attribs
    
    def _svg_finite_points(points):
        pts = np.asarray(points, dtype=float)
        if pts.ndim != 2 or pts.shape[1] != 2:
            return np.zeros((0, 2), dtype=float)
        mask = np.isfinite(pts[:, 0]) & np.isfinite(pts[:, 1])
        return pts[mask]
    
    def _svg_relax_and_fit(path_segs, min_roc):
        spacing = float(min_roc) / 10.0
        sampled = _svg_sample_path_uniform(path_segs, spacing)
        closed = _svg_is_closed(sampled)
        relaxed = _svg_relax_curve(sampled.astype(np.float64).copy(), float(min_roc), bool(closed), SVG_MAX_RELAX_ITERS)
        dense = _svg_densify(relaxed, 2.0)
        curve = _svg_fit_spline(dense, smoothing=0, closed=closed)
        return [_svg_simplify_polyline(curve, SVG_SIMPLIFY_TOL, closed=closed)]
    
    def _svg_closed_all_below_min(path_curves, min_roc):
        closed_any = False
        rocs_all = []
        for curve_pts in path_curves:
            pts = _svg_finite_points(curve_pts)
            if len(pts) < 3:
                continue
            closed = _svg_is_closed(pts)
            closed_any = closed_any or closed
            if SVG_MEASURED_ROC_SAMPLE_STEP > 0:
                pts_u = _svg_resample_uniform(pts, SVG_MEASURED_ROC_SAMPLE_STEP)
                if closed and np.linalg.norm(pts_u[0] - pts_u[-1]) > 1e-6:
                    pts_u = np.vstack([pts_u, pts_u[0]])
                pts = pts_u
            if len(pts) < 3:
                continue
            rocs = _svg_roc_along_polyline(pts.astype(np.float64), bool(closed))
            finite = rocs[np.isfinite(rocs)]
            if len(finite):
                rocs_all.append(finite)
        if not closed_any:
            return False
        if not rocs_all:
            return True
        return bool(np.all(np.concatenate(rocs_all) < float(min_roc)))
    
    def _svg_process(filename, min_roc):
        svg_paths, svg_root_attribs = _svg_load_paths(filename)
        if not svg_paths:
            return None
        fitted_list = []
        for path_segs in svg_paths:
            fitted = _svg_relax_and_fit(path_segs, min_roc)
            if not _svg_closed_all_below_min(fitted, min_roc):
                fitted_list.append(fitted)
        return {"original_paths": svg_paths, "fitted": fitted_list, "min_roc": float(min_roc), "svg_root_attribs": svg_root_attribs}
    
    def _svg_measured_min_roc(fitted_paths):
        min_roc = np.inf
        for path_curves in fitted_paths:
            for curve_pts in path_curves:
                pts = np.asarray(curve_pts, dtype=float)
                if len(pts) < 3:
                    continue
                closed = _svg_is_closed(pts)
                if SVG_MEASURED_ROC_SAMPLE_STEP > 0:
                    pts_u = _svg_resample_uniform(pts, SVG_MEASURED_ROC_SAMPLE_STEP)
                    if closed and np.linalg.norm(pts_u[0] - pts_u[-1]) > 1e-6:
                        pts_u = np.vstack([pts_u, pts_u[0]])
                    pts = pts_u
                if len(pts) < 3:
                    continue
                rocs = _svg_roc_along_polyline(pts.astype(np.float64), bool(closed))
                finite = rocs[np.isfinite(rocs)]
                if len(finite):
                    min_roc = min(min_roc, float(np.min(finite)))
        return min_roc if np.isfinite(min_roc) else np.inf
    
    @jit(nopython=True, fastmath=True, cache=True)
    def _svg_hsv_to_rgb(h, s, v):
        h = h - np.floor(h)
        i = int(h * 6.0)
        f = h * 6.0 - i
        p, q, t = v * (1.0 - s), v * (1.0 - f * s), v * (1.0 - (1.0 - f) * s)
        i = i % 6
        if i == 0: return v, t, p
        if i == 1: return q, v, p
        if i == 2: return p, v, t
        if i == 3: return p, q, v
        if i == 4: return t, p, v
        return v, p, q
    
    def _svg_linecollection_roc(points, min_roc_scale, ax=None):
        pts = np.asarray(points, dtype=np.float64)
        if len(pts) < 2:
            return None
        closed = _svg_is_closed(pts)
        rocs = _svg_roc_along_polyline(pts.astype(np.float64), bool(closed))
        seg_roc = 0.5 * (rocs[:-1] + rocs[1:])
        eps = 1e-12
        lo, hi = np.log10(min_roc_scale + eps), np.log10(10.0 * min_roc_scale + eps)
        t = np.clip((np.log10(seg_roc + eps) - lo) / (hi - lo + 1e-12), 0.0, 1.0)
        hues = 0.0 + 0.6 * t
        colors = np.empty((len(hues), 4), dtype=float)
        for i, h in enumerate(hues):
            r, g, b = _svg_hsv_to_rgb(float(h), 0.85, 0.95)
            colors[i] = [r, g, b, 1.0]
        segments = np.stack([pts[:-1], pts[1:]], axis=1)
        xmin, xmax = ax.get_xlim() if ax else (0, 1)
        ymin, ymax = ax.get_ylim() if ax else (0, 1)
        span = max(abs(xmax - xmin), abs(ymax - ymin), 1e-9)
        lw = float(np.clip(0.0018 * span, 0.6, 2.0))
        return LineCollection(segments, colors=colors, linewidths=lw)
    
    def _svg_catmull_rom_to_bezier(points):
        pts = np.asarray(points, dtype=float)
        n = len(pts)
        if n < 2:
            return []
        segs = []
        for i in range(n - 1):
            p0 = pts[i - 1] if i > 0 else pts[i]
            p1, p2 = pts[i], pts[i + 1]
            p3 = pts[i + 2] if (i + 2) < n else pts[i + 1]
            segs.append((p1, p1 + (p2 - p0) / 6.0, p2 - (p3 - p1) / 6.0, p2))
        return segs
    
    def _svg_compute_bbox(fitted_paths):
        all_pts = []
        for path_curves in fitted_paths:
            for curve_pts in path_curves:
                pts = _svg_finite_points(curve_pts)
                if len(pts):
                    all_pts.append(pts)
        if not all_pts:
            return None
        pts = np.vstack(all_pts)
        return float(np.min(pts[:, 0])), float(np.min(pts[:, 1])), float(np.max(pts[:, 0])), float(np.max(pts[:, 1]))
    
    def _svg_export_fitted(input_filename, fitted_paths, svg_root_attribs=None):
        base, ext = os.path.splitext(input_filename)
        if ext.lower() != ".svg":
            return None
        output_filename = f"{base}_adjusted.svg"
        
        viewbox = svg_root_attribs.get("viewBox") if svg_root_attribs else None
        if viewbox is None:
            bbox = _svg_compute_bbox(fitted_paths)
            if bbox is None:
                with open(output_filename, "w", encoding="utf-8") as f:
                    f.write('<?xml version="1.0" encoding="UTF-8"?>\n<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 10 10">\n</svg>\n')
                return output_filename
            xmin, ymin, xmax, ymax = bbox
            w, h = max(1e-6, xmax - xmin), max(1e-6, ymax - ymin)
            viewbox = f"{xmin:.6f} {ymin:.6f} {w:.6f} {h:.6f}"
        
        bbox = _svg_compute_bbox(fitted_paths)
        stroke_width = 0.0012 * max(1e-6, bbox[2] - bbox[0], bbox[3] - bbox[1]) if bbox else 1.0
        
        width = svg_root_attribs.get("width", "") if svg_root_attribs else ""
        height = svg_root_attribs.get("height", "") if svg_root_attribs else ""
        preserve = svg_root_attribs.get("preserveAspectRatio", "") if svg_root_attribs else ""
        
        attribs = [f'viewBox="{viewbox}"']
        if width: attribs.append(f'width="{width}"')
        if height: attribs.append(f'height="{height}"')
        if preserve: attribs.append(f'preserveAspectRatio="{preserve}"')
        
        lines = ['<?xml version="1.0" encoding="UTF-8"?>', f'<svg xmlns="http://www.w3.org/2000/svg" {" ".join(attribs)}>']
        
        for path_curves in fitted_paths:
                for curve_pts in path_curves:
                    curve_pts = _svg_finite_points(curve_pts)
                    if len(curve_pts) < 2:
                        continue
                    closed = _svg_is_closed(curve_pts)
                    # Export as polyline path — the fitted curves already have many dense
                    # smooth points from spline fitting, so this faithfully reproduces
                    # what matplotlib shows.  The previous Catmull-Rom conversion created
                    # cusps because at i=0 it set p0=p1 (zero tangent) and at i=n-2 it
                    # set p3=p2, collapsing the control handles at every endpoint.
                    d_parts = [f"M {curve_pts[0][0]:.6f},{curve_pts[0][1]:.6f}"]
                    for pt in curve_pts[1:]:
                        d_parts.append(f"L {pt[0]:.6f},{pt[1]:.6f}")
                    if closed:
                        d_parts.append("Z")
                    lines.append(f'<path d="{" ".join(d_parts)}" fill="none" stroke="black" stroke-width="{stroke_width:.6f}"/>')
        
        lines.append("</svg>")
        with open(output_filename, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))
        return output_filename

    # =========================================================================
    # Coaster GCode Generation Functions
    # =========================================================================

    def _coaster_svg_to_machine(fitted_paths, coaster_diameter, stock_w, stock_h):
        """Transform SVG fitted paths to machine coordinates.

        Machine origin: lower-left corner of stock, Z=0 at stock bottom.
        SVG Y-axis points down; machine Y-axis points up.
        Design is centered within the stock.

        Returns (machine_paths, circle_center) where machine_paths is a list
        of numpy arrays (N,2) in machine coords and circle_center is (cx, cy).
        """
        bbox = _svg_compute_bbox(fitted_paths)
        if bbox is None:
            return [], (stock_w / 2.0, stock_h / 2.0)
        svg_xmin, svg_ymin, svg_xmax, svg_ymax = bbox
        svg_cx = (svg_xmin + svg_xmax) / 2.0
        svg_cy = (svg_ymin + svg_ymax) / 2.0
        mc_cx = stock_w / 2.0
        mc_cy = stock_h / 2.0

        machine_paths = []
        for path_curves in fitted_paths:
            for curve_pts in path_curves:
                pts = np.asarray(curve_pts, dtype=float)
                if len(pts) < 2:
                    continue
                new_pts = np.empty_like(pts)
                new_pts[:, 0] = (pts[:, 0] - svg_cx) + mc_cx
                new_pts[:, 1] = -(pts[:, 1] - svg_cy) + mc_cy
                machine_paths.append(new_pts)
        return machine_paths, (mc_cx, mc_cy)

    def _coaster_overlay_coords(fitted_paths, coaster_diameter, stock_w, stock_h):
        """Compute overlay shapes (stock rect & circle) in SVG coordinate space
        for visualization on the existing matplotlib plot."""
        bbox = _svg_compute_bbox(fitted_paths)
        if bbox is None:
            return None, None
        svg_xmin, svg_ymin, svg_xmax, svg_ymax = bbox
        svg_cx = (svg_xmin + svg_xmax) / 2.0
        svg_cy = (svg_ymin + svg_ymax) / 2.0

        # Stock rectangle corners in SVG space (centered on design)
        half_w = stock_w / 2.0
        half_h = stock_h / 2.0
        # Machine center maps to SVG center, but Y is flipped
        # Machine (0,0) in SVG space: x = 0 - mc_cx + svg_cx, y = -(0 - mc_cy) + svg_cy
        # Machine (stock_w, stock_h) in SVG: x = stock_w - mc_cx + svg_cx, y = -(stock_h - mc_cy) + svg_cy
        stock_rect_svg = np.array([
            [svg_cx - half_w, svg_cy - half_h],
            [svg_cx + half_w, svg_cy - half_h],
            [svg_cx + half_w, svg_cy + half_h],
            [svg_cx - half_w, svg_cy + half_h],
            [svg_cx - half_w, svg_cy - half_h],
        ])

        # Circle in SVG space (centered on design center)
        r = coaster_diameter / 2.0
        theta = np.linspace(0, 2 * np.pi, 361)
        circle_svg = np.column_stack([svg_cx + r * np.cos(theta),
                                      svg_cy + r * np.sin(theta)])
        return stock_rect_svg, circle_svg

    if HAS_SHAPELY:

        def _coaster_safe_polygon(coords):
            """Create a valid Shapely Polygon, handling degenerate cases."""
            try:
                poly = ShapelyPolygon(coords)
                if not poly.is_valid:
                    poly = make_valid(poly)
                if poly.is_empty or poly.area < 1e-6:
                    return None
                if poly.geom_type == 'MultiPolygon':
                    return max(poly.geoms, key=lambda p: p.area)
                elif poly.geom_type == 'Polygon':
                    return poly
                return None
            except Exception:
                return None

        def _coaster_paths_to_shapely(machine_paths):
            """Convert machine-coord polylines to Shapely Polygons with hole detection."""
            polygons_raw = []
            for path in machine_paths:
                if _svg_is_closed(path) and len(path) >= 4:
                    poly = _coaster_safe_polygon(path)
                    if poly is not None:
                        polygons_raw.append(poly)
            if not polygons_raw:
                return []
            # Sort by area descending for hole detection
            polygons_raw.sort(key=lambda p: p.area, reverse=True)
            result = []
            used = set()
            for i, outer in enumerate(polygons_raw):
                if i in used:
                    continue
                holes = []
                for j in range(i + 1, len(polygons_raw)):
                    if j in used:
                        continue
                    if outer.contains(polygons_raw[j]):
                        holes.append(list(polygons_raw[j].exterior.coords))
                        used.add(j)
                if holes:
                    result.append(ShapelyPolygon(list(outer.exterior.coords), holes))
                else:
                    result.append(outer)
            return result

        def _coaster_circle_polygon(cx, cy, radius, n=360):
            """Create a Shapely circular polygon."""
            theta = np.linspace(0, 2 * np.pi, n + 1)[:-1]
            coords = [(cx + radius * math.cos(t), cy + radius * math.sin(t)) for t in theta]
            return ShapelyPolygon(coords)

        def _coaster_raster_fill(polygon, stepover):
            """Generate zigzag raster fill lines within a Shapely polygon.

            Returns a list of numpy arrays, each (N,2), representing
            continuous cutting passes.
            """
            if polygon.is_empty or polygon.area < 1e-6:
                return []
            bounds = polygon.bounds  # (minx, miny, maxx, maxy)
            passes = []
            y = bounds[1] + stepover * 0.5
            direction = 1  # 1=left-to-right, -1=right-to-left
            x_margin = 0.5
            while y < bounds[3]:
                scan = LineString([(bounds[0] - x_margin, y), (bounds[2] + x_margin, y)])
                clipped = polygon.intersection(scan)
                if not clipped.is_empty:
                    segs = []
                    if clipped.geom_type == 'LineString':
                        segs = [clipped]
                    elif clipped.geom_type == 'MultiLineString':
                        segs = list(clipped.geoms)
                    # Sort segments left-to-right or right-to-left
                    segs = [s for s in segs if s.length > 0.01]
                    segs.sort(key=lambda s: s.bounds[0], reverse=(direction < 0))
                    for seg in segs:
                        coords = list(seg.coords)
                        if direction < 0:
                            coords.reverse()
                        passes.append(np.array(coords))
                y += stepover
                direction *= -1
            return passes

        def _coaster_contour_parallel_fill(polygon, stepover):
            """Generate contour-parallel (offset spiral) toolpath within a polygon.

            Repeatedly offsets the polygon inward by stepover, extracting
            contour rings from outside to inside.  Adjacent rings are
            connected by rotating each ring to start at the point nearest
            the previous ring's endpoint, minimising rapid traverses.

            Returns a list of numpy arrays, each a closed contour ring.
            """
            if polygon.is_empty or polygon.area < 1e-6:
                return []

            rings = []
            offset = stepover * 0.5  # first ring at half-stepover from boundary
            while True:
                shrunk = polygon.buffer(-offset, join_style=2, mitre_limit=2.0)
                if shrunk.is_empty:
                    break
                geoms = [shrunk] if shrunk.geom_type == 'Polygon' else list(shrunk.geoms)
                for geom in geoms:
                    if geom.is_empty or geom.area < 1e-6:
                        continue
                    coords = np.array(geom.exterior.coords)
                    if len(coords) >= 3:
                        rings.append(coords)
                    for interior in geom.interiors:
                        coords = np.array(interior.coords)
                        if len(coords) >= 3:
                            rings.append(coords)
                offset += stepover

            # Connect rings: rotate each ring to start at the point
            # nearest the previous ring's endpoint
            for i in range(1, len(rings)):
                prev_end = rings[i - 1][-1]
                ring = rings[i]
                dists = np.sum((ring - prev_end) ** 2, axis=1)
                nearest = int(np.argmin(dists))
                if nearest != 0:
                    # Rotate ring and re-close
                    # Ring is closed (first==last), so drop the closing point,
                    # roll, then re-append the closing point.
                    open_ring = ring[:-1] if np.linalg.norm(ring[0] - ring[-1]) < 1e-6 else ring
                    rotated = np.roll(open_ring, -nearest, axis=0)
                    rings[i] = np.vstack([rotated, rotated[:1]])

            return rings

        def _coaster_surface_raster(cx, cy, radius, tool_diameter, stepover_frac=0.5):
            """Generate surfacing raster passes over the coaster circle area.

            Only surfaces within the circle (plus a margin for the contour cut
            tool path) rather than the entire stock.
            """
            # Surface slightly beyond the coaster radius to leave clean
            # material for the contour cut
            surf_radius = radius + tool_diameter
            circle = _coaster_circle_polygon(cx, cy, surf_radius)
            stepover = tool_diameter * stepover_frac
            return _coaster_raster_fill(circle, stepover)

        def _coaster_pocket_toolpath(polygon, tool_radius, stepover_frac=0.5):
            """Generate pocket clearing toolpath: offset inward by tool_radius,
            then raster fill.  Raster fill gives complete coverage for complex
            concave shapes (contour-parallel collapses too quickly on narrow
            features like dragon legs/horns)."""
            offset_poly = polygon.buffer(-tool_radius, join_style=2, mitre_limit=2.0)
            if offset_poly.is_empty:
                return []
            stepover = tool_radius * 2.0 * stepover_frac
            if offset_poly.geom_type == 'MultiPolygon':
                passes = []
                for p in offset_poly.geoms:
                    passes.extend(_coaster_raster_fill(p, stepover))
                return passes
            return _coaster_raster_fill(offset_poly, stepover)

        def _coaster_rest_machining_toolpath(polygon, prev_tool_radius, curr_tool_radius, stepover_frac=0.5):
            """Generate rest machining passes: contour-parallel rings where
            previous tool couldn't reach, with raster fallback for remainder.

            Generates rings at offsets from curr_r to prev_r along the original
            polygon boundary (spaced by stepover to limit engagement).  Areas
            where rings collapse (narrow concave features) are raster-filled.
            """
            curr_r = curr_tool_radius
            prev_r = prev_tool_radius
            stepover = curr_r * 2.0 * stepover_frac

            # Check current tool can reach inside polygon
            reachable_curr = polygon.buffer(-curr_r, join_style=2, mitre_limit=2.0)
            if reachable_curr.is_empty:
                return []

            # Compute rest area (for remainder detection later)
            reachable_prev = polygon.buffer(-prev_r, join_style=2, mitre_limit=2.0)
            rest_area = reachable_curr.difference(reachable_prev) if not reachable_prev.is_empty else reachable_curr
            if rest_area.is_empty:
                return []

            # --- Ring offsets from curr_r to prev_r ---
            ring_offsets = []
            d = curr_r
            while d <= prev_r + 1e-9:
                ring_offsets.append(d)
                d += stepover
            # Ensure final offset reaches prev_r
            if ring_offsets and ring_offsets[-1] < prev_r - 1e-6:
                ring_offsets.append(prev_r)
            if not ring_offsets:
                ring_offsets = [curr_r]

            # --- Extract contour rings at each offset ---
            rings = []
            for d in ring_offsets:
                shrunk = polygon.buffer(-d, join_style=2, mitre_limit=2.0)
                if shrunk.is_empty:
                    continue
                geoms = [shrunk] if shrunk.geom_type == 'Polygon' else list(shrunk.geoms)
                for geom in geoms:
                    if geom.is_empty or geom.area < 1e-6:
                        continue
                    coords = np.array(geom.exterior.coords)
                    if len(coords) >= 3:
                        rings.append(coords)
                    for interior in geom.interiors:
                        hcoords = np.array(interior.coords)
                        if len(hcoords) >= 3:
                            rings.append(hcoords)

            # --- Filter tiny ring fragments ---
            min_perimeter = stepover * 4.0
            filtered = []
            for r in rings:
                perimeter = np.sum(np.sqrt(np.sum(np.diff(r, axis=0)**2, axis=1)))
                if perimeter >= min_perimeter:
                    filtered.append(r)
            rings = filtered

            # --- Connect rings: rotate each to start near previous endpoint ---
            for i in range(1, len(rings)):
                prev_end = rings[i - 1][-1]
                ring = rings[i]
                dists = np.sum((ring - prev_end) ** 2, axis=1)
                nearest = int(np.argmin(dists))
                if nearest != 0:
                    open_ring = ring[:-1] if np.linalg.norm(ring[0] - ring[-1]) < 1e-6 else ring
                    rotated = np.roll(open_ring, -nearest, axis=0)
                    rings[i] = np.vstack([rotated, rotated[:1]])

            # --- Raster fill any remainder not covered by contour rings ---
            raster_passes = []
            min_area = stepover * stepover
            if rings:
                ring_lines = [LineString(r) for r in rings if len(r) >= 2]
                if ring_lines:
                    ring_coverage = unary_union(ring_lines).buffer(curr_r, join_style=2, mitre_limit=2.0)
                    remainder = rest_area.difference(ring_coverage)
                else:
                    remainder = rest_area
            else:
                remainder = rest_area

            # Handle GeometryCollection from difference()
            if remainder is not None and not remainder.is_empty:
                if remainder.geom_type == 'GeometryCollection':
                    poly_geoms = [g for g in remainder.geoms
                                  if g.geom_type in ('Polygon', 'MultiPolygon')]
                    remainder = unary_union(poly_geoms) if poly_geoms else None

            if remainder is not None and not remainder.is_empty and remainder.area > min_area:
                if remainder.geom_type == 'MultiPolygon':
                    for p in remainder.geoms:
                        if p.area > min_area:
                            raster_passes.extend(_coaster_raster_fill(p, stepover))
                elif remainder.geom_type == 'Polygon':
                    raster_passes.extend(_coaster_raster_fill(remainder, stepover))
                # Filter degenerate micro-segments
                raster_passes = [p for p in raster_passes
                                 if np.linalg.norm(p[-1] - p[0]) >= stepover]

            return rings + raster_passes

        def _coaster_contour_pass(polygon, tool_radius):
            """Generate a contour pass along the pocket boundary at tool_radius offset.

            Returns a list of numpy arrays representing the contour path(s).
            """
            offset_poly = polygon.buffer(-tool_radius, join_style=2, mitre_limit=2.0)
            if offset_poly.is_empty:
                return []
            passes = []
            if offset_poly.geom_type == 'MultiPolygon':
                polys = list(offset_poly.geoms)
            else:
                polys = [offset_poly]
            for p in polys:
                coords = np.array(p.exterior.coords)
                if len(coords) > 1:
                    passes.append(coords)
                for hole in p.interiors:
                    hcoords = np.array(hole.coords)
                    if len(hcoords) > 1:
                        passes.append(hcoords)
            return passes

        # ----- GCode emission helpers -----

        def _coaster_gcode_header(rpm, tool_number):
            """GCode program header."""
            return [
                "; Coaster Inlay GCode — generated by Carvera JonApp",
                "G21 ; mm mode",
                "G90 ; absolute positioning",
                f"M6 T{tool_number}",
                f"M3 S{rpm}",
                "G4 P3 ; spindle spinup dwell",
            ]

        def _coaster_gcode_footer(retract_z):
            """GCode program footer."""
            return [
                "M5 ; spindle off",
                f"G0 Z{retract_z:.3f}",
                "G0 X0 Y0",
                "M2 ; program end",
            ]

        def _coaster_gcode_tool_change(tool_number, rpm, retract_z):
            """GCode tool change sequence."""
            return [
                f"",
                f"M5",
                f"G0 Z{retract_z:.3f}",
                f"M6 T{tool_number}",
                f"M3 S{rpm}",
                "G4 P3",
            ]

        def _coaster_gcode_helical_ramp(cx, cy, ramp_radius, z_from, z_to,
                                         stepdown, ramp_feed):
            """Generate a helical ramp at (cx, cy) to descend from z_from to z_to.

            The tool traces small circles of ramp_radius, descending by stepdown
            per revolution.  If ramp_radius is very small, falls back to a
            gentle linear ramp along X.
            """
            lines = []
            if ramp_radius < 0.3:
                # Too small for a helix — position first, then linear ramp
                ramp_len = max(2.0, abs(z_from - z_to) * 3.0)
                lines.append(f"G0 X{cx:.4f} Y{cy:.4f}")
                lines.append(f"G0 Z{z_from + 0.3:.3f}")
                lines.append(f"G1 X{cx + ramp_len:.4f} Y{cy:.4f} Z{z_to:.3f} F{ramp_feed:.0f}")
                lines.append(f"G1 X{cx:.4f} Y{cy:.4f} F{ramp_feed:.0f}")
                return lines
            # Start at 3-o'clock on the ramp circle
            sx = cx + ramp_radius
            sy = cy
            lines.append(f"G0 X{sx:.4f} Y{sy:.4f}")
            lines.append(f"G0 Z{z_from + 0.3:.3f}")
            z = z_from
            half = stepdown / 2.0
            while z > z_to + 1e-6:
                z1 = max(z - half, z_to)
                lines.append(f"G2 X{cx - ramp_radius:.4f} Y{cy:.4f} Z{z1:.3f} I{-ramp_radius:.4f} J0 F{ramp_feed:.0f}")
                z2 = max(z1 - half, z_to)
                lines.append(f"G2 X{cx + ramp_radius:.4f} Y{cy:.4f} Z{z2:.3f} I{ramp_radius:.4f} J0 F{ramp_feed:.0f}")
                z = z2
            # Final full circle at target depth to clean the ramp hole
            lines.append(f"G2 X{cx - ramp_radius:.4f} Y{cy:.4f} Z{z_to:.3f} I{-ramp_radius:.4f} J0 F{ramp_feed:.0f}")
            lines.append(f"G2 X{cx + ramp_radius:.4f} Y{cy:.4f} Z{z_to:.3f} I{ramp_radius:.4f} J0 F{ramp_feed:.0f}")
            return lines

        def _coaster_gcode_raster_at_depth(passes_2d, z_depth, feedrate, retract_z,
                                               stepdown, tool_diameter, prev_z=None,
                                               ramp_feed=None, z_top=None):
            """Convert 2D passes to GCode at a fixed Z depth.

            Uses a helical ramp entry for the first pass, then traverses
            between subsequent passes at a safe height above the stock
            surface (z_top) to avoid dragging through uncut material.
            """
            if ramp_feed is None:
                ramp_feed = feedrate / 3.0
            lines = []
            ramp_from = prev_z if prev_z is not None else retract_z
            ramp_done = False
            ramp_radius = tool_diameter * 0.4  # slightly less than tool radius
            # Traverse above the stock surface to avoid gouging uncut material
            traverse_z = (z_top + 1.0) if z_top is not None else retract_z
            for pss in passes_2d:
                pts = np.asarray(pss)
                if len(pts) < 2:
                    continue
                if not ramp_done:
                    # Helical ramp entry at the midpoint of the first pass
                    lines.append(f"G0 Z{retract_z:.3f}")
                    mid_idx = len(pts) // 2
                    ramp_cx, ramp_cy = pts[mid_idx][0], pts[mid_idx][1]
                    lines.extend(_coaster_gcode_helical_ramp(
                        ramp_cx, ramp_cy, ramp_radius,
                        ramp_from, z_depth, stepdown, ramp_feed))
                    ramp_done = True
                # Lift to shallow clearance, rapid to pass start, plunge
                lines.append(f"G0 Z{traverse_z:.3f}")
                lines.append(f"G0 X{pts[0][0]:.4f} Y{pts[0][1]:.4f}")
                lines.append(f"G1 Z{z_depth:.3f} F{ramp_feed:.0f}")
                for pt in pts[1:]:
                    lines.append(f"G1 X{pt[0]:.4f} Y{pt[1]:.4f} F{feedrate:.0f}")
            return lines

        def _coaster_gcode_surface_at_depth(passes_2d, z_depth, feedrate, retract_z,
                                               stepdown, tool_diameter, prev_z=None,
                                               ramp_feed=None):
            """Convert 2D surface raster passes to GCode at a fixed Z depth.

            Instead of lifting between rows, extends 1 tool diameter past
            the end of each row and makes a semicircular arc into the next
            row for continuous cutting.
            """
            if ramp_feed is None:
                ramp_feed = feedrate / 3.0
            lines = []
            valid = []
            for pss in passes_2d:
                pts = np.asarray(pss)
                if len(pts) >= 2:
                    valid.append(pts)
            if not valid:
                return lines

            ramp_from = prev_z if prev_z is not None else retract_z
            ramp_radius = tool_diameter * 0.4
            overrun = tool_diameter

            # Helical ramp entry at midpoint of first pass
            pts = valid[0]
            lines.append(f"G0 Z{retract_z:.3f}")
            mid_idx = len(pts) // 2
            lines.extend(_coaster_gcode_helical_ramp(
                pts[mid_idx][0], pts[mid_idx][1], ramp_radius,
                ramp_from, z_depth, stepdown, ramp_feed))

            # Move to start of first pass and cut it
            lines.append(f"G1 X{pts[0][0]:.4f} Y{pts[0][1]:.4f} F{feedrate:.0f}")
            for pt in pts[1:]:
                lines.append(f"G1 X{pt[0]:.4f} Y{pt[1]:.4f} F{feedrate:.0f}")

            # Link subsequent passes with overrun + semicircular arc
            for idx in range(1, len(valid)):
                prev_pts = valid[idx - 1]
                curr_pts = valid[idx]
                prev_end = prev_pts[-1]
                going_right = prev_end[0] > prev_pts[0][0]

                # Extend 1 tool diameter past end of row
                ext_x = prev_end[0] + (overrun if going_right else -overrun)
                lines.append(f"G1 X{ext_x:.4f} Y{prev_end[1]:.4f} F{feedrate:.0f}")

                # Semicircular arc to next row's Y level
                next_start = curr_pts[0]
                dy = next_start[1] - prev_end[1]
                if abs(dy) > 0.001:
                    # Right side: G3 (CCW), Left side: G2 (CW)
                    arc_cmd = "G3" if going_right else "G2"
                    lines.append(f"{arc_cmd} X{ext_x:.4f} Y{next_start[1]:.4f} I0 J{dy / 2:.4f} F{feedrate:.0f}")
                else:
                    lines.append(f"G1 X{ext_x:.4f} Y{next_start[1]:.4f} F{feedrate:.0f}")

                # Move to start of next pass and cut it
                lines.append(f"G1 X{next_start[0]:.4f} Y{next_start[1]:.4f} F{feedrate:.0f}")
                for pt in curr_pts[1:]:
                    lines.append(f"G1 X{pt[0]:.4f} Y{pt[1]:.4f} F{feedrate:.0f}")

            return lines

        def _coaster_gcode_surface_with_stepdown(passes_2d, total_depth, stepdown,
                                                  feedrate, retract_z, z_top,
                                                  tool_diameter=6.35, ramp_feed=None):
            """Emit surface raster passes at successive depth layers.

            Like pocket_with_stepdown but uses arc-linked rows instead of
            lifting between passes.
            """
            lines = []
            if not passes_2d or total_depth <= 0:
                return lines
            z = z_top
            prev_z = z_top
            z_bottom = z_top - total_depth
            while z > z_bottom + 1e-6:
                z -= stepdown
                if z < z_bottom:
                    z = z_bottom
                lines.append(f"; surface pass Z={z:.3f}")
                lines.extend(_coaster_gcode_surface_at_depth(
                    passes_2d, z, feedrate, retract_z, stepdown,
                    tool_diameter, prev_z, ramp_feed))
                prev_z = z
            return lines

        def _coaster_gcode_pocket_with_stepdown(passes_2d, total_depth, stepdown,
                                                 feedrate, retract_z, z_top,
                                                 tool_diameter=6.35, ramp_feed=None):
            """Emit raster passes at successive depth layers from z_top downward.

            Each depth layer begins with a helical ramp entry before rastering.
            """
            lines = []
            if not passes_2d or total_depth <= 0:
                return lines
            z = z_top
            prev_z = z_top
            z_bottom = z_top - total_depth
            while z > z_bottom + 1e-6:
                z -= stepdown
                if z < z_bottom:
                    z = z_bottom
                lines.append(f"; depth pass Z={z:.3f}")
                lines.extend(_coaster_gcode_raster_at_depth(
                    passes_2d, z, feedrate, retract_z, stepdown,
                    tool_diameter, prev_z, ramp_feed, z_top))
                prev_z = z
            return lines

        def _coaster_gcode_spiral_contour(cx, cy, cut_radius, stepdown, total_depth,
                                           feedrate, retract_z, z_top, ramp_feed=None):
            """Emit GCode for spiral ramp contour using semicircular G2 arcs.

            Entry is via the first spiral arc itself — no straight plunge.
            """
            if ramp_feed is None:
                ramp_feed = feedrate / 2.0
            lines = []
            # Start at 3-o'clock position, rapid to just above surface
            start_x = cx + cut_radius
            start_y = cy
            lines.append(f"G0 Z{retract_z:.3f}")
            lines.append(f"G0 X{start_x:.4f} Y{start_y:.4f}")
            lines.append(f"G0 Z{z_top + 0.5:.3f}")

            z = z_top
            z_bottom = z_top - total_depth
            half_step = stepdown / 2.0

            # First arc ramps from above the surface into the cut
            z1 = max(z - half_step, z_bottom)
            lines.append(f"G2 X{cx - cut_radius:.4f} Y{cy:.4f} Z{z1:.3f} I{-cut_radius:.4f} J0 F{ramp_feed:.0f}")
            z2 = max(z1 - half_step, z_bottom)
            lines.append(f"G2 X{cx + cut_radius:.4f} Y{cy:.4f} Z{z2:.3f} I{cut_radius:.4f} J0 F{feedrate:.0f}")
            z = z2

            while z > z_bottom + 1e-6:
                # Semicircle 1: 3 o'clock -> 9 o'clock (clockwise)
                z1 = max(z - half_step, z_bottom)
                lines.append(f"G2 X{cx - cut_radius:.4f} Y{cy:.4f} Z{z1:.3f} I{-cut_radius:.4f} J0 F{feedrate:.0f}")
                # Semicircle 2: 9 o'clock -> 3 o'clock (clockwise)
                z2 = max(z1 - half_step, z_bottom)
                lines.append(f"G2 X{cx + cut_radius:.4f} Y{cy:.4f} Z{z2:.3f} I{cut_radius:.4f} J0 F{feedrate:.0f}")
                z = z2

            # Spring pass at full depth
            lines.append(f"; spring pass")
            lines.append(f"G2 X{cx - cut_radius:.4f} Y{cy:.4f} Z{z_bottom:.3f} I{-cut_radius:.4f} J0 F{feedrate:.0f}")
            lines.append(f"G2 X{cx + cut_radius:.4f} Y{cy:.4f} Z{z_bottom:.3f} I{cut_radius:.4f} J0 F{feedrate:.0f}")

            # Retract
            lines.append(f"G0 Z{retract_z:.3f}")
            return lines

        # ----- Top-level GCode assembly -----

        def _coaster_generate_coaster_gcode(params):
            """Generate complete coaster GCode.

            params keys: machine_paths, circle_center, coaster_diameter,
                stock_w, stock_h, stock_thickness, target_thickness,
                inlay_depth, retract_z, tools (list sorted largest first,
                each with diameter, feedrate, stepdown, rpm, stepover_frac,
                tool_number)
            """
            tools = sorted(params["tools"], key=lambda t: -t["diameter"])
            largest = tools[0]
            cx, cy = params["circle_center"]
            coaster_r = params["coaster_diameter"] / 2.0
            stock_t = params["stock_thickness"]
            target_t = params["target_thickness"]
            inlay_depth = params["inlay_depth"]
            retract_z = stock_t + params["retract_z"]

            lines = [
                f"; Coaster Inlay GCode",
                f"; Stock: {params['stock_w']:.1f} x {params['stock_h']:.1f} x {stock_t:.1f} mm",
                f"; Coaster diameter: {params['coaster_diameter']:.1f} mm",
                f"; Target thickness: {target_t:.1f} mm, Inlay depth: {inlay_depth:.1f} mm",
                f"; Origin: lower-left corner of stock, Z=0 at stock bottom",
                f"; Stock top: Z={stock_t:.1f}, Surfaced top: Z={target_t:.1f}",
                f"; Inlay bottom: Z={target_t - inlay_depth:.1f}",
                "",
            ]
            lines.extend(_coaster_gcode_header(largest["rpm"], largest["tool_number"]))

            # Phase 1: Surface coaster area to target thickness
            surface_depth = stock_t - target_t
            if surface_depth > 0.01:
                lines.append("")
                lines.append("; === PHASE 1: Surface to coaster thickness ===")
                raster = _coaster_surface_raster(cx, cy, coaster_r,
                                                  largest["diameter"])
                lines.extend(_coaster_gcode_surface_with_stepdown(
                    raster, surface_depth, largest["stepdown"],
                    largest["feedrate"], retract_z, stock_t,
                    largest["diameter"], largest["ramp_feed"]))

            # Phases 2+: Inlay pockets with progressive tools
            # (cut pockets BEFORE contour so the coaster is still held in stock)
            polys = _coaster_paths_to_shapely(params["machine_paths"])
            if polys:
                for i, tool in enumerate(tools):
                    tool_r = tool["diameter"] / 2.0
                    so_frac = tool.get("stepover_frac", 0.5)
                    phase_num = 2 + i
                    lines.append("")
                    if i == 0:
                        lines.append(f"; === PHASE {phase_num}: Pocket with {tool['diameter']:.2f}mm tool ===")
                    else:
                        lines.append(f"; === PHASE {phase_num}: Rest machining with {tool['diameter']:.2f}mm tool ===")
                        lines.extend(_coaster_gcode_tool_change(tool["tool_number"], tool["rpm"], retract_z))
                    is_last_tool = (i == len(tools) - 1)
                    for poly in polys:
                        if i == 0:
                            passes = _coaster_pocket_toolpath(poly, tool_r, so_frac)
                        else:
                            prev_r = tools[i - 1]["diameter"] / 2.0
                            passes = _coaster_rest_machining_toolpath(poly, prev_r, tool_r, so_frac)
                        if passes:
                            lines.extend(_coaster_gcode_pocket_with_stepdown(
                                passes, inlay_depth, tool["stepdown"],
                                tool["feedrate"], retract_z, target_t,
                                tool["diameter"], tool["ramp_feed"]))
                        # Single-tool: pocket raster needs a contour finish.
                        # Multi-tool: rest machining already includes the
                        # innermost contour ring, so no separate pass needed.
                        if is_last_tool and i == 0:
                            contour = _coaster_contour_pass(poly, tool_r)
                            if contour:
                                lines.extend(_coaster_gcode_pocket_with_stepdown(
                                    contour, inlay_depth, tool["stepdown"],
                                    tool["feedrate"], retract_z, target_t,
                                    tool["diameter"], tool["ramp_feed"]))

            # Final phase: Contour cut circular coaster outline (last, so
            # the coaster stays attached to the stock during pocketing)
            last_phase = 2 + len(tools) if polys else 2
            lines.append("")
            lines.append(f"; === PHASE {last_phase}: Contour cut coaster circle ===")
            # Switch back to largest tool for contour if we changed tools
            if len(tools) > 1 and polys:
                lines.extend(_coaster_gcode_tool_change(largest["tool_number"], largest["rpm"], retract_z))
            tool_r = largest["diameter"] / 2.0
            cut_radius = coaster_r + tool_r  # outside contour
            contour_feed = largest.get("contour_feed", largest["feedrate"])
            lines.extend(_coaster_gcode_spiral_contour(
                cx, cy, cut_radius, largest["stepdown"], target_t,
                contour_feed, retract_z, target_t, largest["ramp_feed"]))

            lines.append("")
            lines.extend(_coaster_gcode_footer(retract_z))
            return "\n".join(lines)

        def _coaster_generate_plug_gcode(params):
            """Generate complete plug GCode.

            Same params as coaster plus: plug_stock_w, plug_stock_h,
                plug_stock_thickness, glue_gap
            """
            tools = sorted(params["tools"], key=lambda t: -t["diameter"])
            largest = tools[0]
            # Plug paths are transformed to plug stock coordinates
            plug_w = params["plug_stock_w"]
            plug_h = params["plug_stock_h"]
            plug_t = params["plug_stock_thickness"]
            target_t = params["plug_target_thickness"]
            inlay_depth = params["inlay_depth"]
            glue_gap = params["glue_gap"]
            retract_z = plug_t + params["retract_z"]
            coaster_r = params["coaster_diameter"] / 2.0

            # Re-transform paths to plug stock coordinates
            plug_paths, plug_center = _coaster_svg_to_machine(
                params["fitted_paths"], params["coaster_diameter"], plug_w, plug_h)

            cx, cy = plug_center

            plug_depth = inlay_depth + glue_gap  # plug features taller than coaster pockets for glue space

            lines = [
                f"; Plug Inlay GCode",
                f"; Stock: {plug_w:.1f} x {plug_h:.1f} x {plug_t:.1f} mm",
                f"; Coaster diameter: {params['coaster_diameter']:.1f} mm",
                f"; Plug target thickness: {target_t:.1f} mm",
                f"; Plug clearing depth: {plug_depth:.1f} mm (inlay {inlay_depth:.1f} + glue gap {glue_gap:.2f})",
                f"; Origin: lower-left corner of stock, Z=0 at stock bottom",
                "",
            ]
            lines.extend(_coaster_gcode_header(largest["rpm"], largest["tool_number"]))

            # Phase 1: Surface plug stock to target thickness
            surface_depth = plug_t - target_t
            if surface_depth > 0.01:
                lines.append("")
                lines.append("; === PHASE 1: Surface plug stock to thickness ===")
                raster = _coaster_surface_raster(cx, cy, coaster_r,
                                                  largest["diameter"])
                lines.extend(_coaster_gcode_surface_with_stepdown(
                    raster, surface_depth, largest["stepdown"],
                    largest["feedrate"], retract_z, plug_t,
                    largest["diameter"], largest["ramp_feed"]))

            # Phases 2+: Clear around islands with progressive tools
            # (cut BEFORE contour so the plug is still held in stock)
            circle_poly = _coaster_circle_polygon(cx, cy, coaster_r)
            svg_polys = _coaster_paths_to_shapely(plug_paths)
            if svg_polys:
                # Islands are the SVG shapes (no XY inset — glue gap is in Z only)
                islands = unary_union(svg_polys)
                clearing_area = circle_poly.difference(islands)

                for i, tool in enumerate(tools):
                    tool_r = tool["diameter"] / 2.0
                    so_frac = tool.get("stepover_frac", 0.5)
                    phase_num = 2 + i
                    lines.append("")
                    if i == 0:
                        lines.append(f"; === PHASE {phase_num}: Clear around islands with {tool['diameter']:.2f}mm tool ===")
                    else:
                        lines.append(f"; === PHASE {phase_num}: Rest machining with {tool['diameter']:.2f}mm tool ===")
                        lines.extend(_coaster_gcode_tool_change(tool["tool_number"], tool["rpm"], retract_z))

                    is_last_tool = (i == len(tools) - 1)
                    if i == 0:
                        passes = _coaster_pocket_toolpath(clearing_area, tool_r, so_frac)
                    else:
                        prev_r = tools[i - 1]["diameter"] / 2.0
                        passes = _coaster_rest_machining_toolpath(clearing_area, prev_r, tool_r, so_frac)
                    if passes:
                        lines.extend(_coaster_gcode_pocket_with_stepdown(
                            passes, plug_depth, tool["stepdown"],
                            tool["feedrate"], retract_z, target_t,
                            tool["diameter"], tool["ramp_feed"]))
                    if is_last_tool and i == 0:
                        contour_passes = _coaster_contour_pass(clearing_area, tool_r)
                        if contour_passes:
                            lines.extend(_coaster_gcode_pocket_with_stepdown(
                                contour_passes, plug_depth, tool["stepdown"],
                                tool["feedrate"], retract_z, target_t,
                                tool["diameter"], tool["ramp_feed"]))

            # Final phase: Contour cut circular plug (last)
            last_phase = 2 + len(tools) if svg_polys else 2
            lines.append("")
            lines.append(f"; === PHASE {last_phase}: Contour cut plug circle ===")
            if len(tools) > 1 and svg_polys:
                lines.extend(_coaster_gcode_tool_change(largest["tool_number"], largest["rpm"], retract_z))
            tool_r = largest["diameter"] / 2.0
            cut_radius = coaster_r + tool_r
            contour_feed = largest.get("contour_feed", largest["feedrate"])
            lines.extend(_coaster_gcode_spiral_contour(
                cx, cy, cut_radius, largest["stepdown"], target_t,
                contour_feed, retract_z, target_t, largest["ramp_feed"]))

            lines.append("")
            lines.extend(_coaster_gcode_footer(retract_z))
            return "\n".join(lines)

        def _coaster_generate_joint_gcode(params):
            """Generate joint GCode for both coaster and plug in one file.

            Operations are grouped by tool: each tool machines both parts
            before switching to the next tool.

            params keys: machine_paths, circle_center, coaster_diameter,
                stock_w, stock_h, stock_thickness, target_thickness,
                inlay_depth, retract_z, tools (each with rpm, stepover_frac),
                fitted_paths, plug_stock_w, plug_stock_h,
                plug_stock_thickness, glue_gap, joint_horiz_offset
            """
            tools = sorted(params["tools"], key=lambda t: -t["diameter"])
            largest = tools[0]

            # Coaster coordinates (as-is, origin at coaster stock lower-left)
            coaster_cx, coaster_cy = params["circle_center"]
            coaster_r = params["coaster_diameter"] / 2.0
            coaster_stock_t = params["stock_thickness"]
            coaster_target_t = params["target_thickness"]
            plug_target_t = params["plug_target_thickness"]
            inlay_depth = params["inlay_depth"]
            glue_gap = params["glue_gap"]
            plug_depth = inlay_depth + glue_gap  # plug features taller for glue space

            # Plug coordinates: offset in X from coaster origin
            x_offset = params["joint_horiz_offset"]
            plug_w = params["plug_stock_w"]
            plug_h = params["plug_stock_h"]
            plug_t = params["plug_stock_thickness"]

            # Transform plug paths to plug stock coords, then apply X offset
            plug_paths_raw, plug_center_raw = _coaster_svg_to_machine(
                params["fitted_paths"], params["coaster_diameter"], plug_w, plug_h)
            plug_paths = []
            for path in plug_paths_raw:
                offset_path = path.copy()
                offset_path[:, 0] += x_offset
                plug_paths.append(offset_path)
            plug_cx = plug_center_raw[0] + x_offset
            plug_cy = plug_center_raw[1]

            # Shared retract Z: must clear the taller stock
            retract_z = max(coaster_stock_t, plug_t) + params["retract_z"]

            lines = [
                f"; Joint Coaster + Plug Inlay GCode",
                f"; Coaster stock: {params['stock_w']:.1f} x {params['stock_h']:.1f} x {coaster_stock_t:.1f} mm",
                f"; Plug stock: {plug_w:.1f} x {plug_h:.1f} x {plug_t:.1f} mm (offset X={x_offset:.1f})",
                f"; Coaster diameter: {params['coaster_diameter']:.1f} mm",
                f"; Coaster target thickness: {coaster_target_t:.1f} mm, Plug target thickness: {plug_target_t:.1f} mm",
                f"; Inlay depth: {inlay_depth:.1f} mm, Plug clearing depth: {plug_depth:.1f} mm (glue gap {glue_gap:.2f})",
                f"; Origin: lower-left corner of coaster stock, Z=0 at stock bottom",
                f"; Plug lower-left at X={x_offset:.1f}",
                "",
            ]
            lines.extend(_coaster_gcode_header(largest["rpm"], largest["tool_number"]))

            # === Phase 1: Surface both parts with largest tool ===
            lines.append("")
            lines.append("; === PHASE 1: Surface both parts to target thickness ===")

            # Surface coaster
            surface_depth_coaster = coaster_stock_t - coaster_target_t
            if surface_depth_coaster > 0.01:
                lines.append("; --- Coaster surfacing ---")
                raster = _coaster_surface_raster(coaster_cx, coaster_cy, coaster_r,
                                                  largest["diameter"])
                lines.extend(_coaster_gcode_surface_with_stepdown(
                    raster, surface_depth_coaster, largest["stepdown"],
                    largest["feedrate"], retract_z, coaster_stock_t,
                    largest["diameter"], largest["ramp_feed"]))

            # Surface plug
            surface_depth_plug = plug_t - plug_target_t
            if surface_depth_plug > 0.01:
                lines.append("; --- Plug surfacing ---")
                raster = _coaster_surface_raster(plug_cx, plug_cy, coaster_r,
                                                  largest["diameter"])
                lines.extend(_coaster_gcode_surface_with_stepdown(
                    raster, surface_depth_plug, largest["stepdown"],
                    largest["feedrate"], retract_z, plug_t,
                    largest["diameter"], largest["ramp_feed"]))

            # === Phases 2+: Inlay/clearing operations grouped by tool ===
            coaster_polys = _coaster_paths_to_shapely(params["machine_paths"])

            # Prepare plug clearing area (no XY inset — glue gap is in Z only)
            circle_poly_plug = _coaster_circle_polygon(plug_cx, plug_cy, coaster_r)
            plug_svg_polys = _coaster_paths_to_shapely(plug_paths)
            plug_clearing_area = None
            if plug_svg_polys:
                islands = unary_union(plug_svg_polys)
                plug_clearing_area = circle_poly_plug.difference(islands)

            has_detail_ops = bool(coaster_polys) or plug_clearing_area is not None

            for i, tool in enumerate(tools):
                tool_r = tool["diameter"] / 2.0
                so_frac = tool.get("stepover_frac", 0.5)
                phase_num = 2 + i
                lines.append("")

                if i == 0:
                    lines.append(f"; === PHASE {phase_num}: Pocket/clear with {tool['diameter']:.2f}mm tool ===")
                else:
                    lines.append(f"; === PHASE {phase_num}: Rest machining with {tool['diameter']:.2f}mm tool ===")
                    lines.extend(_coaster_gcode_tool_change(tool["tool_number"], tool["rpm"], retract_z))

                is_last_tool = (i == len(tools) - 1)

                # Coaster inlay pockets
                if coaster_polys:
                    lines.append(f"; --- Coaster inlay pockets ---")
                    for poly in coaster_polys:
                        if i == 0:
                            passes = _coaster_pocket_toolpath(poly, tool_r, so_frac)
                        else:
                            prev_r = tools[i - 1]["diameter"] / 2.0
                            passes = _coaster_rest_machining_toolpath(poly, prev_r, tool_r, so_frac)
                        if passes:
                            lines.extend(_coaster_gcode_pocket_with_stepdown(
                                passes, inlay_depth, tool["stepdown"],
                                tool["feedrate"], retract_z, coaster_target_t,
                                tool["diameter"], tool["ramp_feed"]))
                        if is_last_tool and i == 0:
                            contour = _coaster_contour_pass(poly, tool_r)
                            if contour:
                                lines.extend(_coaster_gcode_pocket_with_stepdown(
                                    contour, inlay_depth, tool["stepdown"],
                                    tool["feedrate"], retract_z, coaster_target_t,
                                    tool["diameter"], tool["ramp_feed"]))

                # Plug clearing around islands
                if plug_clearing_area is not None:
                    lines.append(f"; --- Plug clearing around islands ---")
                    if i == 0:
                        passes = _coaster_pocket_toolpath(plug_clearing_area, tool_r, so_frac)
                    else:
                        prev_r = tools[i - 1]["diameter"] / 2.0
                        passes = _coaster_rest_machining_toolpath(plug_clearing_area, prev_r, tool_r, so_frac)
                    if passes:
                        lines.extend(_coaster_gcode_pocket_with_stepdown(
                            passes, plug_depth, tool["stepdown"],
                            tool["feedrate"], retract_z, plug_target_t,
                            tool["diameter"], tool["ramp_feed"]))
                    if is_last_tool and i == 0:
                        contour_passes = _coaster_contour_pass(plug_clearing_area, tool_r)
                        if contour_passes:
                            lines.extend(_coaster_gcode_pocket_with_stepdown(
                                contour_passes, plug_depth, tool["stepdown"],
                                tool["feedrate"], retract_z, plug_target_t,
                                tool["diameter"], tool["ramp_feed"]))

            # === Final phase: Contour cuts for both parts ===
            last_phase = 2 + len(tools) if has_detail_ops else 2
            lines.append("")
            lines.append(f"; === PHASE {last_phase}: Contour cut both circles ===")
            if len(tools) > 1 and has_detail_ops:
                lines.extend(_coaster_gcode_tool_change(largest["tool_number"], largest["rpm"], retract_z))

            tool_r = largest["diameter"] / 2.0
            cut_radius = coaster_r + tool_r
            contour_feed = largest.get("contour_feed", largest["feedrate"])

            lines.append("; --- Coaster contour ---")
            lines.extend(_coaster_gcode_spiral_contour(
                coaster_cx, coaster_cy, cut_radius, largest["stepdown"], coaster_target_t,
                contour_feed, retract_z, coaster_target_t, largest["ramp_feed"]))

            lines.append("; --- Plug contour ---")
            lines.extend(_coaster_gcode_spiral_contour(
                plug_cx, plug_cy, cut_radius, largest["stepdown"], plug_target_t,
                contour_feed, retract_z, plug_target_t, largest["ramp_feed"]))

            lines.append("")
            lines.extend(_coaster_gcode_footer(retract_z))
            return "\n".join(lines)

        def _coaster_generate_clearance_gcode(params):
            """Generate inlay clearance pass GCode.

            Runs a contour pass around each inlay feature, offset outward
            by clearance_offset, using the smallest enabled tool.
            Single full-depth cut, going around each feature twice to
            handle tool deflection.
            """
            tools = sorted(params["tools"], key=lambda t: -t["diameter"])
            smallest = tools[-1]
            tool_r = smallest["diameter"] / 2.0

            stock_t = params["stock_thickness"]
            target_t = params["target_thickness"]
            inlay_depth = params["inlay_depth"]
            clearance_offset = params["clearance_offset"]
            retract_z = stock_t + params["retract_z"]
            z_bottom = target_t - inlay_depth

            lines = [
                f"; Inlay Clearance Pass GCode",
                f"; Clearance offset: {clearance_offset:.3f} mm",
                f"; Tool: {smallest['diameter']:.2f} mm (tool #{smallest['tool_number']})",
                f"; Cut depth: Z={z_bottom:.3f} (single full-depth pass)",
                f"; Two laps per feature for tool deflection",
                f"; Origin: lower-left corner of stock, Z=0 at stock bottom",
                "",
            ]
            lines.extend(_coaster_gcode_header(smallest["rpm"], smallest["tool_number"]))

            polys = _coaster_paths_to_shapely(params["machine_paths"])
            if polys:
                for pi, poly in enumerate(polys):
                    # Offset polygon outward by clearance amount
                    enlarged = poly.buffer(clearance_offset, join_style=2, mitre_limit=2.0)
                    if enlarged.is_empty:
                        continue
                    contour = _coaster_contour_pass(enlarged, tool_r)
                    if not contour:
                        continue
                    lines.append(f"")
                    lines.append(f"; --- Feature {pi + 1}: clearance contour (2 laps) ---")
                    # Two laps at full depth for tool deflection
                    double_contour = contour + contour
                    lines.extend(_coaster_gcode_raster_at_depth(
                        double_contour, z_bottom, smallest["feedrate"],
                        retract_z, inlay_depth, smallest["diameter"],
                        target_t, smallest["ramp_feed"], target_t))

            lines.append("")
            lines.extend(_coaster_gcode_footer(retract_z))
            return "\n".join(lines)

    class SVGSmootherTab(ttk.Frame):
        """Tab for SVG curve smoothing with ROC constraints and coaster GCode generation."""

        def __init__(self, parent):
            super().__init__(parent)

            self.filename = None
            self.results = None
            self.min_roc = 1.0
            self.roc_min = 0.1
            self.roc_max = 30.0

            self._debounce_ms = 200
            self._timer_id = None
            self._last_roc = None
            self._overlay_timer = None

            # Inlay region selection
            self._selectable_regions = []  # list of dicts: {polygon, members}
            self._selected_indices = set()  # indices into _selectable_regions

            # SVG transform state (scale + position)
            self._raw_svg_paths = None   # raw loaded SVGBezierCurve paths
            self._raw_svg_attribs = None
            self._svg_orig_width = None  # original SVG bbox width
            self._svg_orig_center = None # original SVG bbox center (cx, cy)
            self._svg_width_var = tk.StringVar(value="")
            self._svg_pos_x_var = tk.StringVar(value="")
            self._svg_pos_y_var = tk.StringVar(value="")
            self._svg_transform_timer = None

            # Drag state
            self._drag_start_pixel = None
            self._drag_start_pos = None
            self._dragged = False
            self._drag_dpx = 1.0  # data units per pixel (frozen at drag start)
            self._drag_dpy = 1.0

            # Coaster parameter variables (set defaults, then override from saved)
            self.coaster_diameter_var = tk.StringVar(value=str(COASTER_DEFAULT_DIAMETER))
            self.coaster_stock_w_var = tk.StringVar(value=str(COASTER_DEFAULT_STOCK_W))
            self.coaster_stock_h_var = tk.StringVar(value=str(COASTER_DEFAULT_STOCK_H))
            self.coaster_stock_t_var = tk.StringVar(value=str(COASTER_DEFAULT_STOCK_T))
            self.plug_stock_w_var = tk.StringVar(value=str(COASTER_DEFAULT_PLUG_W))
            self.plug_stock_h_var = tk.StringVar(value=str(COASTER_DEFAULT_PLUG_H))
            self.plug_stock_t_var = tk.StringVar(value=str(COASTER_DEFAULT_PLUG_T))
            self.target_thickness_var = tk.StringVar(value=str(COASTER_DEFAULT_TARGET_T))
            self.plug_target_thickness_var = tk.StringVar(value=str(COASTER_DEFAULT_TARGET_T))
            self.inlay_depth_var = tk.StringVar(value=str(COASTER_DEFAULT_INLAY_DEPTH))
            self.glue_gap_var = tk.StringVar(value=str(COASTER_DEFAULT_GLUE_GAP))
            self.spindle_rpm_var = tk.StringVar(value=str(COASTER_DEFAULT_RPM))
            self.retract_height_var = tk.StringVar(value=str(COASTER_DEFAULT_RETRACT))
            self.joint_horiz_offset_var = tk.StringVar(value="120.0")
            self.clearance_offset_var = tk.StringVar(value="0.1")

            self.tool_vars = []
            for i, defaults in enumerate(COASTER_DEFAULT_TOOLS):
                tv = {
                    "enabled": tk.BooleanVar(value=True),
                    "diameter": tk.StringVar(value=defaults["diameter"]),
                    "tool_num": tk.StringVar(value=defaults["tool_num"]),
                    "feedrate": tk.StringVar(value=defaults["feedrate"]),
                    "ramp_feed": tk.StringVar(value=defaults["ramp_feed"]),
                    "stepdown": tk.StringVar(value=defaults["stepdown"]),
                    "rpm": tk.StringVar(value=defaults["rpm"]),
                    "stepover": tk.StringVar(value=defaults["stepover"]),
                }
                if "contour_feed" in defaults:
                    tv["contour_feed"] = tk.StringVar(value=defaults["contour_feed"])
                self.tool_vars.append(tv)

            self._load_coaster_settings()
            self.setup_ui()

        def _load_coaster_settings(self):
            """Load coaster parameters from saved config file."""
            if not os.path.exists(COASTER_SETTINGS_FILE):
                return
            try:
                with open(COASTER_SETTINGS_FILE, 'r') as f:
                    cfg = json.load(f)
            except Exception:
                return
            for key, var in [
                ("coaster_diameter", self.coaster_diameter_var),
                ("coaster_stock_w", self.coaster_stock_w_var),
                ("coaster_stock_h", self.coaster_stock_h_var),
                ("coaster_stock_t", self.coaster_stock_t_var),
                ("plug_stock_w", self.plug_stock_w_var),
                ("plug_stock_h", self.plug_stock_h_var),
                ("plug_stock_t", self.plug_stock_t_var),
                ("target_thickness", self.target_thickness_var),
                ("plug_target_thickness", self.plug_target_thickness_var),
                ("inlay_depth", self.inlay_depth_var),
                ("glue_gap", self.glue_gap_var),
                ("spindle_rpm", self.spindle_rpm_var),
                ("retract_height", self.retract_height_var),
                ("joint_horiz_offset", self.joint_horiz_offset_var),
                ("clearance_offset", self.clearance_offset_var),
            ]:
                if key in cfg:
                    var.set(str(cfg[key]))
            # Backward compat: if no separate plug target thickness, use coaster's
            if "plug_target_thickness" not in cfg and "target_thickness" in cfg:
                self.plug_target_thickness_var.set(str(cfg["target_thickness"]))
            # Backward compat: use global spindle_rpm as default for per-tool rpm
            global_rpm = cfg.get("spindle_rpm", str(COASTER_DEFAULT_RPM))
            for i in range(3):
                tkey = f"tool_{i}"
                if tkey in cfg:
                    tcfg = cfg[tkey]
                    for field in ("enabled", "diameter", "tool_num", "feedrate", "ramp_feed",
                                  "stepdown", "rpm", "stepover"):
                        if field in tcfg:
                            if field == "enabled":
                                self.tool_vars[i][field].set(bool(tcfg[field]))
                            else:
                                self.tool_vars[i][field].set(str(tcfg[field]))
                    # Tool 0 has contour_feed
                    if i == 0 and "contour_feed" in tcfg:
                        self.tool_vars[0]["contour_feed"].set(str(tcfg["contour_feed"]))
                    # If no per-tool rpm saved, use the global spindle_rpm
                    if "rpm" not in tcfg:
                        self.tool_vars[i]["rpm"].set(str(global_rpm))
                    # If no ramp_feed saved, default to feedrate / 3
                    if "ramp_feed" not in tcfg:
                        try:
                            ff = float(self.tool_vars[i]["feedrate"].get())
                            self.tool_vars[i]["ramp_feed"].set(str(int(ff / 3)))
                        except (ValueError, tk.TclError):
                            pass
            # Restore min ROC slider (UI may not exist yet at init time)
            if "min_roc" in cfg:
                try:
                    roc = float(cfg["min_roc"])
                    roc = max(self.roc_min, min(self.roc_max, roc))
                    self.min_roc = roc
                    if hasattr(self, 'roc_var'):
                        self.roc_var.set(np.log10(roc))
                        self.roc_label.config(text=f"{roc:.3f}")
                except (ValueError, TypeError):
                    pass

        def _save_coaster_settings(self, *_args):
            """Save coaster parameters to config file."""
            cfg = {}
            for key, var in [
                ("coaster_diameter", self.coaster_diameter_var),
                ("coaster_stock_w", self.coaster_stock_w_var),
                ("coaster_stock_h", self.coaster_stock_h_var),
                ("coaster_stock_t", self.coaster_stock_t_var),
                ("plug_stock_w", self.plug_stock_w_var),
                ("plug_stock_h", self.plug_stock_h_var),
                ("plug_stock_t", self.plug_stock_t_var),
                ("target_thickness", self.target_thickness_var),
                ("plug_target_thickness", self.plug_target_thickness_var),
                ("inlay_depth", self.inlay_depth_var),
                ("glue_gap", self.glue_gap_var),
                ("spindle_rpm", self.spindle_rpm_var),
                ("retract_height", self.retract_height_var),
                ("joint_horiz_offset", self.joint_horiz_offset_var),
                ("clearance_offset", self.clearance_offset_var),
            ]:
                cfg[key] = var.get()
            for i in range(3):
                tcfg = {
                    "enabled": self.tool_vars[i]["enabled"].get(),
                    "diameter": self.tool_vars[i]["diameter"].get(),
                    "tool_num": self.tool_vars[i]["tool_num"].get(),
                    "feedrate": self.tool_vars[i]["feedrate"].get(),
                    "ramp_feed": self.tool_vars[i]["ramp_feed"].get(),
                    "stepdown": self.tool_vars[i]["stepdown"].get(),
                    "rpm": self.tool_vars[i]["rpm"].get(),
                    "stepover": self.tool_vars[i]["stepover"].get(),
                }
                if "contour_feed" in self.tool_vars[i]:
                    tcfg["contour_feed"] = self.tool_vars[i]["contour_feed"].get()
                cfg[f"tool_{i}"] = tcfg
            cfg["min_roc"] = self.min_roc
            try:
                with open(COASTER_SETTINGS_FILE, 'w') as f:
                    json.dump(cfg, f, indent=2)
            except Exception:
                pass

        def _on_param_change(self, *_args):
            """Called when any coaster parameter changes — save and update overlay."""
            self._save_coaster_settings()
            # Debounced overlay update
            if self._overlay_timer is not None:
                self.after_cancel(self._overlay_timer)
            self._overlay_timer = self.after(300, self._refresh_overlay)

        def _refresh_overlay(self):
            """Refresh the plot overlay after parameter changes."""
            self._overlay_timer = None
            if self.results is not None:
                self.plot_results()

        def _make_entry_row(self, parent, label_text, var, width=8):
            """Create a label + entry row, returning the entry widget."""
            row = ttk.Frame(parent)
            row.pack(fill=tk.X, pady=1)
            ttk.Label(row, text=label_text, width=14, anchor=tk.W).pack(side=tk.LEFT)
            entry = ttk.Entry(row, textvariable=var, width=width)
            entry.pack(side=tk.LEFT, padx=2)
            entry.bind("<FocusOut>", self._on_param_change)
            entry.bind("<Return>", self._on_param_change)
            return entry

        def _make_stock_row(self, parent, label_text, w_var, h_var, t_var):
            """Create a stock dimension section: label on top, W/H/T fields below."""
            ttk.Label(parent, text=label_text, anchor=tk.W).pack(fill=tk.X, pady=(4, 0))
            row = ttk.Frame(parent)
            row.pack(fill=tk.X, pady=1)
            for lbl, var in [("W:", w_var), ("H:", h_var), ("T:", t_var)]:
                ttk.Label(row, text=lbl).pack(side=tk.LEFT, padx=(6, 0))
                e = ttk.Entry(row, textvariable=var, width=6)
                e.pack(side=tk.LEFT, padx=(0, 2))
                e.bind("<FocusOut>", self._on_param_change)
                e.bind("<Return>", self._on_param_change)
            ttk.Label(row, text="mm").pack(side=tk.LEFT, padx=(2, 0))

        def setup_ui(self):
            # Main layout: controls on left, plot on right
            main_frame = ttk.Frame(self)
            main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

            # Left side: scrollable controls panel
            controls_outer = ttk.Frame(main_frame, width=360)
            controls_outer.pack(side=tk.LEFT, fill=tk.Y, padx=5)
            controls_outer.pack_propagate(False)

            self._ctrl_canvas = tk.Canvas(controls_outer, highlightthickness=0)
            ctrl_scrollbar = ttk.Scrollbar(controls_outer, orient=tk.VERTICAL,
                                            command=self._ctrl_canvas.yview)
            ctrl_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
            self._ctrl_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
            self._ctrl_canvas.configure(yscrollcommand=ctrl_scrollbar.set)

            controls_frame = ttk.Frame(self._ctrl_canvas)
            self._ctrl_window = self._ctrl_canvas.create_window((0, 0), window=controls_frame,
                                                                  anchor=tk.NW)

            def _on_frame_cfg(event):
                self._ctrl_canvas.configure(scrollregion=self._ctrl_canvas.bbox("all"))
            controls_frame.bind("<Configure>", _on_frame_cfg)

            def _on_canvas_cfg(event):
                self._ctrl_canvas.itemconfig(self._ctrl_window, width=event.width)
            self._ctrl_canvas.bind("<Configure>", _on_canvas_cfg)

            # Mousewheel scrolling only when cursor is over controls
            def _bind_mw(event):
                self._ctrl_canvas.bind_all("<MouseWheel>",
                    lambda e: self._ctrl_canvas.yview_scroll(-1 * (e.delta // 120 or (-1 if e.delta < 0 else 1)), "units"))
            def _unbind_mw(event):
                self._ctrl_canvas.unbind_all("<MouseWheel>")
            self._ctrl_canvas.bind("<Enter>", _bind_mw)
            self._ctrl_canvas.bind("<Leave>", _unbind_mw)

            # --- SVG Smoother controls (original) ---
            ttk.Button(controls_frame, text="Open SVG...", command=self.open_file).pack(fill=tk.X, pady=5)

            # SVG size and position controls
            svg_xform_frame = ttk.Frame(controls_frame)
            svg_xform_frame.pack(fill=tk.X, pady=2)
            ttk.Label(svg_xform_frame, text="Width (mm):").pack(side=tk.LEFT)
            self._svg_width_entry = ttk.Entry(svg_xform_frame,
                                               textvariable=self._svg_width_var, width=8)
            self._svg_width_entry.pack(side=tk.LEFT, padx=2)
            self._svg_width_entry.config(state=tk.DISABLED)
            self._svg_width_entry.bind("<FocusOut>", self._on_svg_transform_entry)
            self._svg_width_entry.bind("<Return>", self._on_svg_transform_entry)

            svg_pos_frame = ttk.Frame(controls_frame)
            svg_pos_frame.pack(fill=tk.X, pady=1)
            ttk.Label(svg_pos_frame, text="X:").pack(side=tk.LEFT, padx=(0, 0))
            self._svg_pos_x_entry = ttk.Entry(svg_pos_frame,
                                               textvariable=self._svg_pos_x_var, width=8)
            self._svg_pos_x_entry.pack(side=tk.LEFT, padx=2)
            self._svg_pos_x_entry.config(state=tk.DISABLED)
            self._svg_pos_x_entry.bind("<FocusOut>", self._on_svg_transform_entry)
            self._svg_pos_x_entry.bind("<Return>", self._on_svg_transform_entry)
            ttk.Label(svg_pos_frame, text="Y:").pack(side=tk.LEFT, padx=(6, 0))
            self._svg_pos_y_entry = ttk.Entry(svg_pos_frame,
                                               textvariable=self._svg_pos_y_var, width=8)
            self._svg_pos_y_entry.pack(side=tk.LEFT, padx=2)
            self._svg_pos_y_entry.config(state=tk.DISABLED)
            self._svg_pos_y_entry.bind("<FocusOut>", self._on_svg_transform_entry)
            self._svg_pos_y_entry.bind("<Return>", self._on_svg_transform_entry)

            slider_frame = ttk.LabelFrame(controls_frame, text="Min ROC", padding=5)
            slider_frame.pack(fill=tk.X, pady=5)
            self.roc_var = tk.DoubleVar(value=np.log10(self.min_roc))
            self.roc_scale = ttk.Scale(slider_frame, from_=np.log10(self.roc_min),
                                       to=np.log10(self.roc_max), variable=self.roc_var,
                                       orient=tk.HORIZONTAL, command=self.on_slider_change)
            self.roc_scale.pack(fill=tk.X)
            self.roc_scale.state(['disabled'])
            self.roc_label = ttk.Label(slider_frame, text=f"{self.min_roc:.3f}")
            self.roc_label.pack()

            self.export_btn = ttk.Button(controls_frame, text="Export adjusted",
                                          command=self.export_svg, state=tk.DISABLED)
            self.export_btn.pack(fill=tk.X, pady=5)

            self.roc_info = ttk.Label(controls_frame, text="minimum ROC @ output: —",
                                       font=("Courier", 9))
            self.roc_info.pack(fill=tk.X, pady=5)

            # --- Coaster Geometry ---
            geo_frame = ttk.LabelFrame(controls_frame, text="Coaster Geometry", padding=5)
            geo_frame.pack(fill=tk.X, pady=5)

            self._make_entry_row(geo_frame, "Diameter (mm):", self.coaster_diameter_var)
            self._make_stock_row(geo_frame, "Coaster stock:", self.coaster_stock_w_var,
                                 self.coaster_stock_h_var, self.coaster_stock_t_var)
            self._make_stock_row(geo_frame, "Plug stock:", self.plug_stock_w_var,
                                 self.plug_stock_h_var, self.plug_stock_t_var)
            self._make_entry_row(geo_frame, "Coaster target thick (mm):", self.target_thickness_var)
            self._make_entry_row(geo_frame, "Plug target thick (mm):", self.plug_target_thickness_var)
            self._make_entry_row(geo_frame, "Inlay depth (mm):", self.inlay_depth_var)
            self._make_entry_row(geo_frame, "Glue gap Z (mm):", self.glue_gap_var)

            # --- Machine Settings ---
            mach_frame = ttk.LabelFrame(controls_frame, text="Machine Settings", padding=5)
            mach_frame.pack(fill=tk.X, pady=5)
            self._make_entry_row(mach_frame, "Retract Z (mm):", self.retract_height_var)

            # --- Tool entries ---
            tool_labels = ["Tool 1 (Surfacing, Roughing, and Perimeter Cutout)", "Tool 2 (Semi-finish)", "Tool 3 (Finish)"]
            for i, tl in enumerate(tool_labels):
                tframe = ttk.LabelFrame(controls_frame, text=tl, padding=5)
                tframe.pack(fill=tk.X, pady=3)
                if i > 0:
                    cb = ttk.Checkbutton(tframe, text="Enable", variable=self.tool_vars[i]["enabled"],
                                          command=self._on_param_change)
                    cb.pack(anchor=tk.W)
                self._make_entry_row(tframe, "Diameter (mm):", self.tool_vars[i]["diameter"])
                self._make_entry_row(tframe, "Tool # (1-6):", self.tool_vars[i]["tool_num"])
                self._make_entry_row(tframe, "RPM:", self.tool_vars[i]["rpm"])
                self._make_entry_row(tframe, "Feed (mm/min):", self.tool_vars[i]["feedrate"])
                self._make_entry_row(tframe, "Ramp feed (mm/min):", self.tool_vars[i]["ramp_feed"])
                if "contour_feed" in self.tool_vars[i]:
                    self._make_entry_row(tframe, "Contour feed (mm/min):", self.tool_vars[i]["contour_feed"])
                self._make_entry_row(tframe, "Stepdown (mm):", self.tool_vars[i]["stepdown"])
                self._make_entry_row(tframe, "Stepover (%):", self.tool_vars[i]["stepover"])

            # --- Generate buttons ---
            ttk.Separator(controls_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=8)

            gcode_enabled = HAS_SHAPELY
            self.gen_coaster_btn = ttk.Button(controls_frame, text="Generate Coaster GCode",
                                               command=self.generate_coaster_gcode)
            self.gen_coaster_btn.pack(fill=tk.X, pady=3)
            self.gen_plug_btn = ttk.Button(controls_frame, text="Generate Plug GCode",
                                            command=self.generate_plug_gcode)
            self.gen_plug_btn.pack(fill=tk.X, pady=3)

            # Joint GCode section
            ttk.Separator(controls_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=4)
            self.gen_joint_btn = ttk.Button(controls_frame, text="Generate Joint GCode",
                                             command=self.generate_joint_gcode)
            self.gen_joint_btn.pack(fill=tk.X, pady=3)
            joint_offset_row = ttk.Frame(controls_frame)
            joint_offset_row.pack(fill=tk.X, pady=1)
            ttk.Label(joint_offset_row, text="LL Corner Stock Horiz. Offset:",
                      anchor=tk.W).pack(side=tk.LEFT)
            joint_offset_entry = ttk.Entry(joint_offset_row,
                                           textvariable=self.joint_horiz_offset_var, width=6)
            joint_offset_entry.pack(side=tk.LEFT, padx=2)
            joint_offset_entry.bind("<FocusOut>", self._on_param_change)
            joint_offset_entry.bind("<Return>", self._on_param_change)
            ttk.Label(joint_offset_row, text="mm").pack(side=tk.LEFT)

            # Inlay clearance pass section
            ttk.Separator(controls_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=4)
            self.gen_clearance_btn = ttk.Button(controls_frame,
                text="Generate Inlay Clearance Pass",
                command=self.generate_clearance_gcode)
            self.gen_clearance_btn.pack(fill=tk.X, pady=3)
            clearance_row = ttk.Frame(controls_frame)
            clearance_row.pack(fill=tk.X, pady=1)
            ttk.Label(clearance_row, text="Offset:",
                      anchor=tk.W).pack(side=tk.LEFT)
            clearance_entry = ttk.Entry(clearance_row,
                                        textvariable=self.clearance_offset_var, width=6)
            clearance_entry.pack(side=tk.LEFT, padx=2)
            clearance_entry.bind("<FocusOut>", self._on_param_change)
            clearance_entry.bind("<Return>", self._on_param_change)
            ttk.Label(clearance_row, text="mm").pack(side=tk.LEFT)

            if not gcode_enabled:
                self.gen_coaster_btn.config(state=tk.DISABLED)
                self.gen_plug_btn.config(state=tk.DISABLED)
                self.gen_joint_btn.config(state=tk.DISABLED)
                self.gen_clearance_btn.config(state=tk.DISABLED)
                ttk.Label(controls_frame, text="Install shapely for GCode",
                          foreground="red").pack(fill=tk.X)

            self.gcode_status = ttk.Label(controls_frame, text="", font=("Courier", 8),
                                           wraplength=300)
            self.gcode_status.pack(fill=tk.X, pady=5)

            # Right side: matplotlib plot
            plot_frame = ttk.Frame(main_frame)
            plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

            self.fig = Figure(figsize=(8, 6))
            self.ax = self.fig.add_subplot(111)
            self.ax.set_aspect("equal")
            self.ax.grid(True, alpha=0.3)
            self.ax.set_title("No file loaded")

            self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
            self.canvas.draw()
            self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

            self._toolbar = NavigationToolbar2Tk(self.canvas, plot_frame)
            self._toolbar.update()

            # Connect click handler for inlay region selection
            self.canvas.mpl_connect('button_press_event', self._on_mouse_press)
            self.canvas.mpl_connect('motion_notify_event', self._on_mouse_motion)
            self.canvas.mpl_connect('button_release_event', self._on_mouse_release)

        # --- Original SVG smoother methods ---

        def open_file(self):
            fname = filedialog.askopenfilename(
                title="Open SVG",
                filetypes=[("SVG files", "*.svg"), ("All files", "*.*")]
            )
            if not fname:
                return
            self.filename = fname

            # Load raw SVG paths and compute original bbox
            raw_paths, raw_attribs = _svg_load_paths(fname)
            if not raw_paths:
                messagebox.showwarning("Empty SVG", "No paths found in SVG file.")
                return
            self._raw_svg_paths = raw_paths
            self._raw_svg_attribs = raw_attribs
            bbox = self._compute_raw_bbox()
            if bbox:
                xmin, ymin, xmax, ymax = bbox
                self._svg_orig_width = xmax - xmin
                self._svg_orig_center = ((xmin + xmax) / 2.0, (ymin + ymax) / 2.0)
                self._svg_width_var.set(f"{self._svg_orig_width:.2f}")
                self._svg_pos_x_var.set(f"{self._svg_orig_center[0]:.2f}")
                self._svg_pos_y_var.set(f"{self._svg_orig_center[1]:.2f}")
            # Enable transform entries
            self._svg_width_entry.config(state=tk.NORMAL)
            self._svg_pos_x_entry.config(state=tk.NORMAL)
            self._svg_pos_y_entry.config(state=tk.NORMAL)

            self.roc_scale.state(['!disabled'])
            self.export_btn.config(state=tk.NORMAL)
            self.run_processing()

        def on_slider_change(self, val):
            roc = 10.0 ** float(val)
            self._last_roc = roc
            self.roc_label.config(text=f"{roc:.3f}")
            if self._timer_id:
                self.after_cancel(self._timer_id)
            self._timer_id = self.after(self._debounce_ms, self.debounced_update)

        def debounced_update(self):
            self._timer_id = None
            if self._last_roc is not None and self.filename:
                self.min_roc = self._last_roc
                self._save_coaster_settings()
                self.run_processing()

        def run_processing(self):
            if not self.filename:
                return
            if self._raw_svg_paths is not None:
                scale, dx, dy = self._get_svg_transform()
                transformed_paths = self._transform_raw_paths(scale, dx, dy)
                fitted_list = []
                for path_segs in transformed_paths:
                    fitted = _svg_relax_and_fit(path_segs, self.min_roc)
                    if not _svg_closed_all_below_min(fitted, self.min_roc):
                        fitted_list.append(fitted)
                self.results = {
                    "original_paths": transformed_paths,
                    "fitted": fitted_list,
                    "min_roc": float(self.min_roc),
                    "svg_root_attribs": self._raw_svg_attribs,
                }
            else:
                self.results = _svg_process(self.filename, self.min_roc)
            self._build_selectable_regions()
            self.plot_results()
            self.update_roc_info()

        def update_roc_info(self):
            if self.results is None or not self.results.get("fitted"):
                self.roc_info.config(text="minimum ROC @ output: —")
                return
            rmin = _svg_measured_min_roc(self.results["fitted"])
            if np.isinf(rmin) or rmin > 1e29:
                self.roc_info.config(text="minimum ROC @ output: ∞")
            else:
                self.roc_info.config(text=f"minimum ROC @ output: {rmin:.4f}")

        def export_svg(self):
            if not self.filename or not self.results:
                return
            out = _svg_export_fitted(self.filename, self.results["fitted"],
                                      self.results.get("svg_root_attribs"))
            if out:
                messagebox.showinfo("Export Complete", f"Saved to:\n{out}")

        # --- SVG transform helpers ---

        def _compute_raw_bbox(self):
            """Compute bounding box from raw SVG bezier paths by sampling."""
            if not self._raw_svg_paths:
                return None
            all_pts = []
            t = np.linspace(0, 1, 20)
            for path_segs in self._raw_svg_paths:
                for bez in path_segs:
                    pts = bez.evaluate(t)
                    if isinstance(pts, np.ndarray) and pts.ndim == 2:
                        all_pts.append(pts)
            if not all_pts:
                return None
            pts = np.vstack(all_pts)
            return (float(np.min(pts[:, 0])), float(np.min(pts[:, 1])),
                    float(np.max(pts[:, 0])), float(np.max(pts[:, 1])))

        def _get_svg_transform(self):
            """Compute (scale, dx, dy) from width/position entries.

            Transform: new_pt = orig_pt * scale + (dx, dy)
            where scale and offset are chosen so that the design is scaled
            uniformly around its original center and then translated to
            the position given by the pos entries.
            """
            if self._svg_orig_width is None or self._svg_orig_center is None:
                return 1.0, 0.0, 0.0
            try:
                new_width = float(self._svg_width_var.get())
                if new_width <= 0:
                    new_width = self._svg_orig_width
                scale = new_width / self._svg_orig_width
            except (ValueError, tk.TclError):
                scale = 1.0
            try:
                pos_x = float(self._svg_pos_x_var.get())
                pos_y = float(self._svg_pos_y_var.get())
            except (ValueError, tk.TclError):
                pos_x, pos_y = self._svg_orig_center
            # new_pt = (orig_pt - orig_center) * scale + (pos_x, pos_y)
            #        = orig_pt * scale + (pos_x - orig_center_x * scale,
            #                             pos_y - orig_center_y * scale)
            ocx, ocy = self._svg_orig_center
            dx = pos_x - ocx * scale
            dy = pos_y - ocy * scale
            return scale, dx, dy

        def _transform_raw_paths(self, scale, dx, dy):
            """Return raw SVG paths with scale and translation applied."""
            transformed = []
            for path_segs in self._raw_svg_paths:
                new_segs = []
                for bez in path_segs:
                    new_cps = bez.control_points * scale
                    new_cps[:, 0] += dx
                    new_cps[:, 1] += dy
                    new_segs.append(SVGBezierCurve(new_cps))
                transformed.append(new_segs)
            return transformed

        def _on_svg_transform_entry(self, *_args):
            """Called when SVG width or position entry changes."""
            if self._svg_transform_timer is not None:
                self.after_cancel(self._svg_transform_timer)
            self._svg_transform_timer = self.after(300, self._do_svg_transform_update)

        def _do_svg_transform_update(self):
            self._svg_transform_timer = None
            if self._raw_svg_paths is not None:
                self.run_processing()

        # --- Drag and click handlers ---

        def _on_mouse_press(self, event):
            if event.inaxes != self.ax or event.button != 1:
                return
            if self._toolbar.mode:
                return
            self._drag_start_pixel = (event.x, event.y)
            try:
                self._drag_start_pos = (float(self._svg_pos_x_var.get()),
                                         float(self._svg_pos_y_var.get()))
            except (ValueError, tk.TclError):
                self._drag_start_pixel = None
                return
            self._dragged = False
            # Freeze pixel-to-data conversion at drag start
            inv = self.ax.transData.inverted()
            p0 = inv.transform((0, 0))
            p1 = inv.transform((100, 0))
            p2 = inv.transform((0, 100))
            self._drag_dpx = (p1[0] - p0[0]) / 100.0
            self._drag_dpy = (p2[1] - p0[1]) / 100.0

        def _on_mouse_motion(self, event):
            if self._drag_start_pixel is None:
                return
            if event.x is None or event.y is None:
                return
            px_dx = event.x - self._drag_start_pixel[0]
            px_dy = event.y - self._drag_start_pixel[1]
            if not self._dragged:
                if abs(px_dx) < 4 and abs(px_dy) < 4:
                    return
                self._dragged = True
            data_dx = px_dx * self._drag_dpx
            data_dy = px_dy * self._drag_dpy
            new_x = self._drag_start_pos[0] + data_dx
            new_y = self._drag_start_pos[1] + data_dy
            self._svg_pos_x_var.set(f"{new_x:.2f}")
            self._svg_pos_y_var.set(f"{new_y:.2f}")
            # Debounced reprocess during drag
            if self._svg_transform_timer is not None:
                self.after_cancel(self._svg_transform_timer)
            self._svg_transform_timer = self.after(50, self._do_svg_transform_update)

        def _on_mouse_release(self, event):
            if self._drag_start_pixel is None:
                return
            was_dragged = self._dragged
            self._drag_start_pixel = None
            self._dragged = False
            if was_dragged:
                # Cancel pending debounced update and do immediate reprocess
                if self._svg_transform_timer is not None:
                    self.after_cancel(self._svg_transform_timer)
                    self._svg_transform_timer = None
                self.run_processing()
            else:
                # Click — dispatch to selection handler
                self._handle_plot_click(event)

        def plot_results(self):
            self.ax.clear()

            if self.results is None:
                self.ax.set_title("No file loaded")
                self.ax.grid(True, alpha=0.3)
                self.canvas.draw_idle()
                return

            self.ax.set_title(os.path.basename(self.filename) if self.filename else "")

            # Original in gray
            for path in self.results["original_paths"]:
                for bez in path:
                    t = np.linspace(0, 1, 80)
                    pts = bez.evaluate(t)
                    self.ax.plot(pts[:, 0], pts[:, 1], color="0.6", linewidth=1.2)

            self.ax.set_aspect("equal", adjustable="datalim")
            self.ax.autoscale_view()

            # Fitted paths with ROC coloring
            for path_curves in self.results["fitted"]:
                for curve_pts in path_curves:
                    lc = _svg_linecollection_roc(curve_pts, self.min_roc, ax=self.ax)
                    if lc is not None:
                        self.ax.add_collection(lc)

            # Draw selected inlay regions (filled highlight)
            self._draw_selected_regions()

            # Overlay: stock rectangle and coaster circle
            self._draw_overlay()

            # Selection count label
            n_sel = len(self._selected_indices)
            n_total = len(self._selectable_regions)
            if n_total > 0:
                self.ax.set_title(
                    f"{os.path.basename(self.filename)}  —  "
                    f"{n_sel}/{n_total} inlay regions selected (click to toggle)")

            self.ax.set_aspect("equal")
            self.ax.grid(True, alpha=0.3)
            self.canvas.draw_idle()

        def _draw_overlay(self):
            """Draw coaster stock rectangle and circle overlay on the plot.

            The overlay is anchored to the original SVG center so that it
            stays fixed when the user drags the design around.
            """
            if self.results is None or not self.results.get("fitted"):
                return
            try:
                coaster_d = float(self.coaster_diameter_var.get())
                stock_w = float(self.coaster_stock_w_var.get())
                stock_h = float(self.coaster_stock_h_var.get())
                plug_w = float(self.plug_stock_w_var.get())
                plug_h = float(self.plug_stock_h_var.get())
            except (ValueError, tk.TclError):
                return

            # Use the original (un-dragged) center so overlays stay fixed
            if self._svg_orig_center is not None:
                svg_cx, svg_cy = self._svg_orig_center
            else:
                bbox = _svg_compute_bbox(self.results["fitted"])
                if bbox is None:
                    return
                svg_cx = (bbox[0] + bbox[2]) / 2.0
                svg_cy = (bbox[1] + bbox[3]) / 2.0

            # Coaster stock rectangle
            half_w, half_h = stock_w / 2.0, stock_h / 2.0
            stock_rect = np.array([
                [svg_cx - half_w, svg_cy - half_h],
                [svg_cx + half_w, svg_cy - half_h],
                [svg_cx + half_w, svg_cy + half_h],
                [svg_cx - half_w, svg_cy + half_h],
                [svg_cx - half_w, svg_cy - half_h],
            ])
            self.ax.plot(stock_rect[:, 0], stock_rect[:, 1],
                         color="orange", linestyle="--", linewidth=1.5,
                         label="Coaster stock")

            # Plug stock overlay (only if different from coaster stock)
            if abs(plug_w - stock_w) > 0.01 or abs(plug_h - stock_h) > 0.01:
                half_pw, half_ph = plug_w / 2.0, plug_h / 2.0
                plug_rect = np.array([
                    [svg_cx - half_pw, svg_cy - half_ph],
                    [svg_cx + half_pw, svg_cy - half_ph],
                    [svg_cx + half_pw, svg_cy + half_ph],
                    [svg_cx - half_pw, svg_cy + half_ph],
                    [svg_cx - half_pw, svg_cy - half_ph],
                ])
                self.ax.plot(plug_rect[:, 0], plug_rect[:, 1],
                             color="dodgerblue", linestyle="--", linewidth=1.0,
                             label="Plug stock")

            # Coaster circle
            r = coaster_d / 2.0
            theta = np.linspace(0, 2 * np.pi, 361)
            circle = np.column_stack([svg_cx + r * np.cos(theta),
                                      svg_cy + r * np.sin(theta)])
            self.ax.plot(circle[:, 0], circle[:, 1],
                         color="limegreen", linestyle="-", linewidth=2.0,
                         label="Coaster circle")

        # --- Inlay region selection ---

        def _build_selectable_regions(self):
            """Build clickable inlay regions from fitted paths.

            Each closed contour becomes its own independently selectable
            region. Polygon hierarchy (holes) is handled later at GCode
            generation time, not at the selection stage.
            """
            self._selectable_regions = []
            self._selected_indices = set()
            if not self.results or not self.results.get("fitted"):
                return
            if not HAS_SHAPELY:
                return
            for pg_idx, path_curves in enumerate(self.results["fitted"]):
                for c_idx, curve_pts in enumerate(path_curves):
                    pts = np.asarray(curve_pts, dtype=float)
                    if len(pts) >= 4 and _svg_is_closed(pts):
                        poly = _coaster_safe_polygon(pts)
                        if poly is not None:
                            self._selectable_regions.append({
                                "polygon": poly,
                                "members": [(pg_idx, c_idx)],
                            })

        def _handle_plot_click(self, event):
            """Handle click on the plot to select/deselect inlay regions."""
            if event.inaxes != self.ax:
                return
            if not self._selectable_regions:
                return

            click = ShapelyPoint(event.xdata, event.ydata)

            # Find the smallest region containing the click point
            best = None
            best_area = float('inf')
            for i, region in enumerate(self._selectable_regions):
                if region["polygon"].contains(click):
                    if region["polygon"].area < best_area:
                        best = i
                        best_area = region["polygon"].area

            if best is not None:
                if best in self._selected_indices:
                    self._selected_indices.discard(best)
                else:
                    self._selected_indices.add(best)
                self.plot_results()

        def _draw_selected_regions(self):
            """Draw filled highlight for selected inlay regions.

            For each selected polygon, any other closed paths contained
            within it are subtracted as holes so letters like 'O' display
            correctly as a ring rather than a solid disk.
            """
            if not HAS_SHAPELY or not self._selectable_regions:
                return
            from matplotlib.patches import PathPatch
            from matplotlib.path import Path as MplPath

            for i in self._selected_indices:
                poly = self._selectable_regions[i]["polygon"]
                # Find contained inner paths to treat as holes
                holes = []
                for j, other in enumerate(self._selectable_regions):
                    if j == i:
                        continue
                    if poly.contains(other["polygon"]):
                        holes.append(other["polygon"])
                # Build a matplotlib Path with holes
                verts = list(poly.exterior.coords)
                codes = ([MplPath.MOVETO] +
                         [MplPath.LINETO] * (len(verts) - 2) +
                         [MplPath.CLOSEPOLY])
                for hole_poly in holes:
                    hcoords = list(hole_poly.exterior.coords)
                    verts.extend(hcoords)
                    codes.extend([MplPath.MOVETO] +
                                 [MplPath.LINETO] * (len(hcoords) - 2) +
                                 [MplPath.CLOSEPOLY])
                path = MplPath(verts, codes)
                patch = PathPatch(path, facecolor='cyan', alpha=0.3,
                                  edgecolor='cyan', linewidth=2)
                self.ax.add_patch(patch)

        def _get_selected_fitted_paths(self):
            """Return fitted paths filtered to selected inlay regions.

            Also includes inner closed paths contained within selected
            regions so that _coaster_paths_to_shapely can build correct
            polygon hierarchy (holes for letters like 'O').
            """
            if not self._selected_indices:
                return []
            selected_members = set()
            for i in self._selected_indices:
                for pg_idx, c_idx in self._selectable_regions[i]["members"]:
                    selected_members.add((pg_idx, c_idx))
                # Include contained inner paths as hole boundaries
                poly = self._selectable_regions[i]["polygon"]
                for j, other in enumerate(self._selectable_regions):
                    if j == i or j in self._selected_indices:
                        continue
                    if poly.contains(other["polygon"]):
                        for pg_idx, c_idx in other["members"]:
                            selected_members.add((pg_idx, c_idx))
            filtered = []
            for pg_idx, path_curves in enumerate(self.results["fitted"]):
                filtered_curves = []
                for c_idx, curve_pts in enumerate(path_curves):
                    if (pg_idx, c_idx) in selected_members:
                        filtered_curves.append(curve_pts)
                if filtered_curves:
                    filtered.append(filtered_curves)
            return filtered

        # --- GCode generation methods ---

        def _collect_params(self):
            """Read all UI values into a params dict. Raises ValueError on invalid input."""
            def _f(var, name):
                try:
                    v = float(var.get())
                    if v <= 0:
                        raise ValueError(f"{name} must be positive")
                    return v
                except (ValueError, tk.TclError):
                    raise ValueError(f"Invalid value for {name}: '{var.get()}'")

            params = {
                "coaster_diameter": _f(self.coaster_diameter_var, "Coaster diameter"),
                "stock_w": _f(self.coaster_stock_w_var, "Coaster stock width"),
                "stock_h": _f(self.coaster_stock_h_var, "Coaster stock height"),
                "stock_thickness": _f(self.coaster_stock_t_var, "Coaster stock thickness"),
                "plug_stock_w": _f(self.plug_stock_w_var, "Plug stock width"),
                "plug_stock_h": _f(self.plug_stock_h_var, "Plug stock height"),
                "plug_stock_thickness": _f(self.plug_stock_t_var, "Plug stock thickness"),
                "target_thickness": _f(self.target_thickness_var, "Coaster target thickness"),
                "plug_target_thickness": _f(self.plug_target_thickness_var, "Plug target thickness"),
                "inlay_depth": _f(self.inlay_depth_var, "Inlay depth"),
                "retract_z": _f(self.retract_height_var, "Retract height"),
            }
            try:
                params["glue_gap"] = float(self.glue_gap_var.get())
                if params["glue_gap"] < 0:
                    raise ValueError("Glue gap cannot be negative")
            except (ValueError, tk.TclError):
                raise ValueError(f"Invalid glue gap: '{self.glue_gap_var.get()}'")

            # Collect enabled tools
            tools = []
            for i, tv in enumerate(self.tool_vars):
                if not tv["enabled"].get():
                    continue
                t = {
                    "diameter": _f(tv["diameter"], f"Tool {i+1} diameter"),
                    "feedrate": _f(tv["feedrate"], f"Tool {i+1} feedrate"),
                    "ramp_feed": _f(tv["ramp_feed"], f"Tool {i+1} ramp feed"),
                    "stepdown": _f(tv["stepdown"], f"Tool {i+1} stepdown"),
                    "rpm": int(_f(tv["rpm"], f"Tool {i+1} RPM")),
                }
                if "contour_feed" in tv:
                    t["contour_feed"] = _f(tv["contour_feed"], f"Tool {i+1} contour feed")
                try:
                    tn = int(tv["tool_num"].get())
                    if tn < 1 or tn > 6:
                        raise ValueError
                    t["tool_number"] = tn
                except (ValueError, tk.TclError):
                    raise ValueError(f"Tool {i+1} number must be 1-6")
                try:
                    so = float(tv["stepover"].get())
                    if so <= 0 or so > 100:
                        raise ValueError
                    t["stepover_frac"] = so / 100.0
                except (ValueError, tk.TclError):
                    raise ValueError(f"Tool {i+1} stepover must be 1-100%")
                tools.append(t)

            if not tools:
                raise ValueError("At least one tool must be enabled")
            params["tools"] = tools
            return params

        def _validate_params(self, params):
            """Validate parameter relationships. Returns (ok, error_msg)."""
            if params["target_thickness"] >= params["stock_thickness"]:
                return False, "Coaster target thickness must be less than coaster stock thickness"
            if params["plug_target_thickness"] >= params["plug_stock_thickness"]:
                return False, "Plug target thickness must be less than plug stock thickness"
            if params["inlay_depth"] >= params["target_thickness"]:
                return False, "Inlay depth must be less than coaster target thickness"
            if params["inlay_depth"] >= params["plug_target_thickness"]:
                return False, "Inlay depth must be less than plug target thickness"
            if params["coaster_diameter"] >= min(params["stock_w"], params["stock_h"]):
                return False, "Coaster diameter must be smaller than stock dimensions"
            if params["coaster_diameter"] >= min(params["plug_stock_w"], params["plug_stock_h"]):
                return False, "Coaster diameter must be smaller than plug stock dimensions"
            # Check tool diameters are reasonable
            for t in params["tools"]:
                if t["diameter"] > params["coaster_diameter"]:
                    return False, f"Tool diameter {t['diameter']}mm exceeds coaster diameter"
            return True, ""

        def generate_coaster_gcode(self):
            """Button handler: generate and save coaster GCode."""
            if not self.results or not self.results.get("fitted"):
                messagebox.showwarning("No SVG", "Please load and process an SVG first")
                return
            if not self._selected_indices:
                messagebox.showwarning("No Inlays Selected",
                    "Click on closed regions in the plot to select inlay areas first.")
                return
            try:
                params = self._collect_params()
            except ValueError as e:
                messagebox.showerror("Invalid Parameters", str(e))
                return
            ok, err = self._validate_params(params)
            if not ok:
                messagebox.showerror("Validation Error", err)
                return

            # Transform only selected paths to machine coordinates
            selected_fitted = self._get_selected_fitted_paths()
            machine_paths, circle_center = _coaster_svg_to_machine(
                selected_fitted, params["coaster_diameter"],
                params["stock_w"], params["stock_h"])
            params["machine_paths"] = machine_paths
            params["circle_center"] = circle_center

            try:
                gcode = _coaster_generate_coaster_gcode(params)
            except Exception as e:
                messagebox.showerror("GCode Error", f"Failed to generate GCode:\n{e}")
                return

            # Save dialog
            default_name = ""
            if self.filename:
                base = os.path.splitext(os.path.basename(self.filename))[0]
                default_name = f"{base}_coaster.nc"
            outfile = filedialog.asksaveasfilename(
                title="Save Coaster GCode",
                defaultextension=".nc",
                initialfile=default_name,
                filetypes=[("GCode files", "*.nc *.gcode"), ("All files", "*.*")]
            )
            if not outfile:
                return
            with open(outfile, 'w') as f:
                f.write(gcode)
            n_lines = gcode.count('\n') + 1
            self.gcode_status.config(text=f"Coaster GCode saved: {n_lines} lines")
            messagebox.showinfo("GCode Saved", f"Coaster GCode saved to:\n{outfile}")

        def generate_plug_gcode(self):
            """Button handler: generate and save plug GCode."""
            if not self.results or not self.results.get("fitted"):
                messagebox.showwarning("No SVG", "Please load and process an SVG first")
                return
            if not self._selected_indices:
                messagebox.showwarning("No Inlays Selected",
                    "Click on closed regions in the plot to select inlay areas first.")
                return
            try:
                params = self._collect_params()
            except ValueError as e:
                messagebox.showerror("Invalid Parameters", str(e))
                return
            ok, err = self._validate_params(params)
            if not ok:
                messagebox.showerror("Validation Error", err)
                return

            # Pass only selected paths for plug generation
            params["fitted_paths"] = self._get_selected_fitted_paths()

            try:
                gcode = _coaster_generate_plug_gcode(params)
            except Exception as e:
                messagebox.showerror("GCode Error", f"Failed to generate GCode:\n{e}")
                return

            default_name = ""
            if self.filename:
                base = os.path.splitext(os.path.basename(self.filename))[0]
                default_name = f"{base}_plug.nc"
            outfile = filedialog.asksaveasfilename(
                title="Save Plug GCode",
                defaultextension=".nc",
                initialfile=default_name,
                filetypes=[("GCode files", "*.nc *.gcode"), ("All files", "*.*")]
            )
            if not outfile:
                return
            with open(outfile, 'w') as f:
                f.write(gcode)
            n_lines = gcode.count('\n') + 1
            self.gcode_status.config(text=f"Plug GCode saved: {n_lines} lines")
            messagebox.showinfo("GCode Saved", f"Plug GCode saved to:\n{outfile}")

        def generate_joint_gcode(self):
            """Button handler: generate and save joint coaster+plug GCode."""
            if not self.results or not self.results.get("fitted"):
                messagebox.showwarning("No SVG", "Please load and process an SVG first")
                return
            if not self._selected_indices:
                messagebox.showwarning("No Inlays Selected",
                    "Click on closed regions in the plot to select inlay areas first.")
                return
            try:
                params = self._collect_params()
            except ValueError as e:
                messagebox.showerror("Invalid Parameters", str(e))
                return
            ok, err = self._validate_params(params)
            if not ok:
                messagebox.showerror("Validation Error", err)
                return

            # Parse horizontal offset
            try:
                horiz_offset = float(self.joint_horiz_offset_var.get())
                if horiz_offset <= 0:
                    raise ValueError("must be positive")
            except (ValueError, tk.TclError):
                messagebox.showerror("Invalid Parameters",
                    f"Invalid horizontal offset: '{self.joint_horiz_offset_var.get()}'")
                return
            params["joint_horiz_offset"] = horiz_offset

            # Transform selected paths to coaster machine coordinates
            selected_fitted = self._get_selected_fitted_paths()
            machine_paths, circle_center = _coaster_svg_to_machine(
                selected_fitted, params["coaster_diameter"],
                params["stock_w"], params["stock_h"])
            params["machine_paths"] = machine_paths
            params["circle_center"] = circle_center

            # Pass fitted paths for plug (will be re-transformed internally)
            params["fitted_paths"] = selected_fitted

            try:
                gcode = _coaster_generate_joint_gcode(params)
            except Exception as e:
                messagebox.showerror("GCode Error", f"Failed to generate GCode:\n{e}")
                return

            default_name = ""
            if self.filename:
                base = os.path.splitext(os.path.basename(self.filename))[0]
                default_name = f"{base}_joint.nc"
            outfile = filedialog.asksaveasfilename(
                title="Save Joint GCode",
                defaultextension=".nc",
                initialfile=default_name,
                filetypes=[("GCode files", "*.nc *.gcode"), ("All files", "*.*")]
            )
            if not outfile:
                return
            with open(outfile, 'w') as f:
                f.write(gcode)
            n_lines = gcode.count('\n') + 1
            self.gcode_status.config(text=f"Joint GCode saved: {n_lines} lines")
            messagebox.showinfo("GCode Saved", f"Joint GCode saved to:\n{outfile}")

        def generate_clearance_gcode(self):
            """Button handler: generate and save inlay clearance pass GCode."""
            if not self.results or not self.results.get("fitted"):
                messagebox.showwarning("No SVG", "Please load and process an SVG first")
                return
            if not self._selected_indices:
                messagebox.showwarning("No Inlays Selected",
                    "Click on closed regions in the plot to select inlay areas first.")
                return
            try:
                params = self._collect_params()
            except ValueError as e:
                messagebox.showerror("Invalid Parameters", str(e))
                return
            ok, err = self._validate_params(params)
            if not ok:
                messagebox.showerror("Validation Error", err)
                return

            # Parse clearance offset
            try:
                clearance_offset = float(self.clearance_offset_var.get())
                if clearance_offset <= 0:
                    raise ValueError("must be positive")
            except (ValueError, tk.TclError):
                messagebox.showerror("Invalid Parameters",
                    f"Invalid clearance offset: '{self.clearance_offset_var.get()}'")
                return
            params["clearance_offset"] = clearance_offset

            # Transform selected paths to coaster machine coordinates
            selected_fitted = self._get_selected_fitted_paths()
            machine_paths, circle_center = _coaster_svg_to_machine(
                selected_fitted, params["coaster_diameter"],
                params["stock_w"], params["stock_h"])
            params["machine_paths"] = machine_paths
            params["circle_center"] = circle_center

            try:
                gcode = _coaster_generate_clearance_gcode(params)
            except Exception as e:
                messagebox.showerror("GCode Error", f"Failed to generate GCode:\n{e}")
                return

            default_name = ""
            if self.filename:
                base = os.path.splitext(os.path.basename(self.filename))[0]
                default_name = f"{base}_clearance.nc"
            outfile = filedialog.asksaveasfilename(
                title="Save Clearance Pass GCode",
                defaultextension=".nc",
                initialfile=default_name,
                filetypes=[("GCode files", "*.nc *.gcode"), ("All files", "*.*")]
            )
            if not outfile:
                return
            with open(outfile, 'w') as f:
                f.write(gcode)
            n_lines = gcode.count('\n') + 1
            self.gcode_status.config(text=f"Clearance GCode saved: {n_lines} lines")
            messagebox.showinfo("GCode Saved", f"Clearance pass GCode saved to:\n{outfile}")


# =============================================================================
# Main Application
# =============================================================================

class CarveraUnifiedApp:
    """Main application window."""
    
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Makera Carvera Unified Control")
        self.root.geometry("1600x900")
        
        # Config file for settings
        self.config_file = os.path.expanduser("~/.carvera_unified_config.json")
        
        # Shared connection
        self.connection = SharedCarveraConnection()
        
        # Load settings from config file
        self.load_settings()
        
        # Global header frame
        header_frame = ttk.Frame(self.root)
        header_frame.pack(fill=tk.X, padx=5, pady=2)
        
        # Left side: Status box with rounded border
        status_box = tk.LabelFrame(header_frame, text=" Status ", 
                                   font=("TkDefaultFont", 10, "bold"),
                                   padx=8, pady=5, relief=tk.GROOVE, bd=2)
        status_box.pack(side=tk.LEFT, fill=tk.Y, padx=5)
        
        # Connection status indicator
        self.conn_indicator = tk.Label(status_box, text="● Carvera Disconnected",
                                        font=("TkDefaultFont", 10, "bold"),
                                        fg="red")
        self.conn_indicator.pack(anchor=tk.W)
        
        # WCS coordinates
        wcs_frame = tk.Frame(status_box)
        wcs_frame.pack(fill=tk.X)
        tk.Label(wcs_frame, text="WCS:", width=6, anchor=tk.W).pack(side=tk.LEFT)
        self.wcs_label = tk.Label(wcs_frame, 
                                   text="X: 0.000  Y: 0.000  Z: 0.000  A: 0.000",
                                   font=("Courier", 10, "bold"), fg="blue")
        self.wcs_label.pack(side=tk.LEFT)
        
        # Machine coordinates
        mach_frame = tk.Frame(status_box)
        mach_frame.pack(fill=tk.X)
        tk.Label(mach_frame, text="Mach:", width=6, anchor=tk.W).pack(side=tk.LEFT)
        self.mach_label = tk.Label(mach_frame,
                                    text="X: 0.000  Y: 0.000  Z: 0.000  A: 0.000",
                                    font=("Courier", 10), fg="gray")
        self.mach_label.pack(side=tk.LEFT)
        
        # WCS Rotation and Tool (underneath coordinates)
        info_frame = tk.Frame(status_box)
        info_frame.pack(fill=tk.X)
        tk.Label(info_frame, text="WCS Rot:", width=8, anchor=tk.W).pack(side=tk.LEFT)
        self.rotation_label = tk.Label(info_frame, text="0.000°",
                                        font=("Courier", 10, "bold"), width=8)
        self.rotation_label.pack(side=tk.LEFT)
        tk.Label(info_frame, text="Tool:").pack(side=tk.LEFT, padx=(10, 0))
        self.tool_label = tk.Label(info_frame, text="None",
                                    font=("Courier", 10, "bold"))
        self.tool_label.pack(side=tk.LEFT, padx=5)
        
        # Pendant status indicator
        self.pendant_label = tk.Label(info_frame, text="No Pendant",
                                       font=("TkDefaultFont", 9),
                                       fg="red")
        self.pendant_label.pack(side=tk.LEFT, padx=(20, 0))
        
        # Pendant step size display (only shown if pendant available)
        self.pendant_step_frame = tk.Frame(info_frame)
        self.pendant_step_frame.pack(side=tk.LEFT, padx=(10, 0))
        tk.Label(self.pendant_step_frame, text="Step:").pack(side=tk.LEFT)
        self.pendant_step_label = tk.Label(self.pendant_step_frame, text="0.01",
                                            font=("Courier", 10, "bold"), width=5)
        self.pendant_step_label.pack(side=tk.LEFT, padx=2)
        tk.Label(self.pendant_step_frame, text="mm").pack(side=tk.LEFT)
        
        # Middle: Settings box
        settings_box = tk.LabelFrame(header_frame, text=" Settings ", 
                                     font=("TkDefaultFont", 10, "bold"),
                                     padx=5, pady=3, relief=tk.GROOVE, bd=2)
        settings_box.pack(side=tk.LEFT, padx=5, fill=tk.Y)
        
        # Probing height row
        probe_z_frame = tk.Frame(settings_box)
        probe_z_frame.pack(fill=tk.X, pady=1)
        tk.Label(probe_z_frame, text="Probe Z:", width=10, anchor=tk.W).pack(side=tk.LEFT)
        self.probe_z_label = tk.Label(probe_z_frame, text="NOT SET", 
                                       font=("Courier", 10, "bold"), fg="red", width=10)
        self.probe_z_label.pack(side=tk.LEFT)
        ttk.Button(probe_z_frame, text="Set", width=4,
                  command=self.set_probe_z).pack(side=tk.LEFT, padx=2)
        
        # Retract height row
        retract_frame = tk.Frame(settings_box)
        retract_frame.pack(fill=tk.X, pady=1)
        tk.Label(retract_frame, text="Retract:", width=10, anchor=tk.W).pack(side=tk.LEFT)
        self.retract_height_entry = ttk.Entry(retract_frame, width=6)
        self.retract_height_entry.pack(side=tk.LEFT)
        tk.Label(retract_frame, text="mm above").pack(side=tk.LEFT, padx=2)
        
        # Slow probe speed row
        speed_frame = tk.Frame(settings_box)
        speed_frame.pack(fill=tk.X, pady=1)
        tk.Label(speed_frame, text="Slow speed:", width=10, anchor=tk.W).pack(side=tk.LEFT)
        self.slow_speed_entry = ttk.Entry(speed_frame, width=6)
        self.slow_speed_entry.pack(side=tk.LEFT)
        tk.Label(speed_frame, text="mm/min").pack(side=tk.LEFT, padx=2)
        
        # Max probe distance row
        max_dist_frame = tk.Frame(settings_box)
        max_dist_frame.pack(fill=tk.X, pady=1)
        tk.Label(max_dist_frame, text="Max probe:", width=10, anchor=tk.W).pack(side=tk.LEFT)
        self.max_probe_entry = ttk.Entry(max_dist_frame, width=6)
        self.max_probe_entry.pack(side=tk.LEFT)
        tk.Label(max_dist_frame, text="mm").pack(side=tk.LEFT, padx=2)
        
        # Right side: Jog box with rounded border
        jog_box = tk.LabelFrame(header_frame, text=" Jog ", 
                                font=("TkDefaultFont", 10, "bold"),
                                padx=5, pady=3, relief=tk.GROOVE, bd=2)
        jog_box.pack(side=tk.RIGHT, padx=5)
        
        # Unlock button (inside jog box area, to the right)
        ttk.Button(jog_box, text="Unlock", width=7,
                  command=self.unlock_machine).pack(side=tk.RIGHT, padx=(10, 0))
        
        # Z jog (vertical column) - LEFT
        z_frame = tk.Frame(jog_box)
        z_frame.pack(side=tk.LEFT, padx=5)
        
        tk.Label(z_frame, text="Z", font=("TkDefaultFont", 9)).pack()
        ttk.Button(z_frame, text="+10", width=4,
                  command=lambda: self.jog_axis('Z', 10)).pack(pady=1)
        ttk.Button(z_frame, text="+1", width=4,
                  command=lambda: self.jog_axis('Z', 1)).pack(pady=1)
        ttk.Button(z_frame, text="-1", width=4,
                  command=lambda: self.jog_axis('Z', -1)).pack(pady=1)
        ttk.Button(z_frame, text="-10", width=4,
                  command=lambda: self.jog_axis('Z', -10)).pack(pady=1)
        
        # Separator
        ttk.Separator(jog_box, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=5)
        
        # X/Y in + shape using grid - MIDDLE
        xy_frame = tk.Frame(jog_box)
        xy_frame.pack(side=tk.LEFT, padx=5)
        
        # Row 0: +Y buttons (top)
        ttk.Button(xy_frame, text="+10", width=3,
                  command=lambda: self.jog_axis('Y', 10)).grid(row=0, column=2, pady=1)
        ttk.Button(xy_frame, text="+1", width=3,
                  command=lambda: self.jog_axis('Y', 1)).grid(row=1, column=2, pady=1)
        
        # Row 2: -X buttons, label, +X buttons (middle)
        ttk.Button(xy_frame, text="-10", width=3,
                  command=lambda: self.jog_axis('X', -10)).grid(row=2, column=0, padx=1)
        ttk.Button(xy_frame, text="-1", width=3,
                  command=lambda: self.jog_axis('X', -1)).grid(row=2, column=1, padx=1)
        tk.Label(xy_frame, text="X/Y", font=("TkDefaultFont", 9)).grid(row=2, column=2)
        ttk.Button(xy_frame, text="+1", width=3,
                  command=lambda: self.jog_axis('X', 1)).grid(row=2, column=3, padx=1)
        ttk.Button(xy_frame, text="+10", width=3,
                  command=lambda: self.jog_axis('X', 10)).grid(row=2, column=4, padx=1)
        
        # Row 3-4: -Y buttons (bottom)
        ttk.Button(xy_frame, text="-1", width=3,
                  command=lambda: self.jog_axis('Y', -1)).grid(row=3, column=2, pady=1)
        ttk.Button(xy_frame, text="-10", width=3,
                  command=lambda: self.jog_axis('Y', -10)).grid(row=4, column=2, pady=1)
        
        # Separator
        ttk.Separator(jog_box, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=5)
        
        # A jog (vertical column) - RIGHT
        a_frame = tk.Frame(jog_box)
        a_frame.pack(side=tk.LEFT, padx=5)
        
        tk.Label(a_frame, text="A", font=("TkDefaultFont", 9)).pack()
        ttk.Button(a_frame, text="+10°", width=4,
                  command=lambda: self.jog_axis('A', 10)).pack(pady=1)
        ttk.Button(a_frame, text="+1°", width=4,
                  command=lambda: self.jog_axis('A', 1)).pack(pady=1)
        ttk.Button(a_frame, text="-1°", width=4,
                  command=lambda: self.jog_axis('A', -1)).pack(pady=1)
        ttk.Button(a_frame, text="-10°", width=4,
                  command=lambda: self.jog_axis('A', -10)).pack(pady=1)
        
        # Register for position updates
        self.connection.add_position_callback(self.on_global_position_update)
        self.connection.add_status_callback(self.on_global_status_change)
        self.connection.add_status_callback(self.on_connection_change)
        
        # Initialize settings UI with saved values
        self.init_settings_ui()
        
        # Configure ttk style for colored tab text
        style = ttk.Style()
        
        # Simple fixed indicator frame
        indicator_frame = ttk.Frame(self.root)
        indicator_frame.pack(fill=tk.X, padx=5, pady=(5, 0))
        
        ttk.Label(indicator_frame, text="Current tab:").pack(side=tk.LEFT, padx=(0, 5))
        self._tab_type_label = tk.Label(indicator_frame, text=" Hardware Control ", 
                 bg="#a0c8f0", fg="#000080", font=("TkDefaultFont", 9, "bold"),
                 relief="ridge", padx=5, pady=2)
        self._tab_type_label.pack(side=tk.LEFT)
        
        # Create notebook for tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Bind tab change event
        self.notebook.bind("<<NotebookTabChanged>>", self._on_tab_changed)
        
        # Add hardware tabs (blue text)
        self.connection_tab = ConnectionTab(self.notebook, self.connection)
        self.notebook.add(self.connection_tab, text="Connection")
        
        self.control_tab = ControlProbingTab(self.notebook, self.connection)
        self.notebook.add(self.control_tab, text="Coordinate Measuring")
        
        if HAS_MATPLOTLIB:
            self.fitter_tab = CurveFitterTab(self.notebook, self.connection)
            self.notebook.add(self.fitter_tab, text="Part Finder")
        
        self.level_tab = LevelAAxisTab(self.notebook, self.connection)
        self.notebook.add(self.level_tab, text="Level A-Axis")
        
        # SVG Smoother tab (utility - no hardware, green text)
        if HAS_SVG_SMOOTHER and HAS_MATPLOTLIB:
            self.svg_smoother_tab = SVGSmootherTab(self.notebook)
            self.notebook.add(self.svg_smoother_tab, text="SVG Smoother")
            self._has_utility_tab = True
        else:
            self._has_utility_tab = False
        
        # Initialize pendant
        self.pendant = None
        self.init_pendant()
        
        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
    
    def _on_tab_changed(self, event):
        """Update the tab type indicator when tab changes."""
        try:
            current_tab = self.notebook.index(self.notebook.select())
            tab_count = self.notebook.index("end")
            
            # Last tab (SVG Smoother) is software-only if it exists
            if self._has_utility_tab and current_tab == tab_count - 1:
                self._tab_type_label.config(text=" Software Only ", 
                    bg="#a0e0a0", fg="#006000")
            else:
                self._tab_type_label.config(text=" Hardware Control ", 
                    bg="#a0c8f0", fg="#000080")
        except:
            pass
    
    def load_settings(self):
        """Load settings from config file."""
        default_retract = "5.0"
        default_slow_speed = "50.0"
        default_max_probe = "10.0"
        
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    config = json.load(f)
                    default_retract = str(config.get("retract_height", default_retract))
                    default_slow_speed = str(config.get("slow_probe_speed", default_slow_speed))
                    default_max_probe = str(config.get("max_probe_distance", default_max_probe))
            except:
                pass
        
        # Initialize connection object with loaded values
        try:
            self.connection.retract_height = float(default_retract)
        except:
            pass
        try:
            self.connection.slow_probe_speed = float(default_slow_speed)
        except:
            pass
        try:
            self.connection.max_probe_distance = float(default_max_probe)
        except:
            pass
        
        # Store defaults for UI initialization later
        self._default_retract = default_retract
        self._default_slow_speed = default_slow_speed
        self._default_max_probe = default_max_probe
    
    def init_settings_ui(self):
        """Initialize settings UI with loaded values (called after UI creation)."""
        self.retract_height_entry.insert(0, self._default_retract)
        self.slow_speed_entry.insert(0, self._default_slow_speed)
        self.max_probe_entry.insert(0, self._default_max_probe)
    
    def save_settings(self):
        """Save settings to config file."""
        config = {}
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    config = json.load(f)
            except:
                pass
        
        config["retract_height"] = self.retract_height_entry.get()
        config["slow_probe_speed"] = self.slow_speed_entry.get()
        config["max_probe_distance"] = self.max_probe_entry.get()
        
        # Update connection object
        try:
            self.connection.retract_height = float(self.retract_height_entry.get())
        except:
            pass
        try:
            self.connection.slow_probe_speed = float(self.slow_speed_entry.get())
        except:
            pass
        try:
            self.connection.max_probe_distance = float(self.max_probe_entry.get())
        except:
            pass
        
        try:
            with open(self.config_file, 'w') as f:
                json.dump(config, f)
        except:
            pass
    
    def set_probe_z(self):
        """Set the probing height to current Z position."""
        if not self.connection.connected:
            messagebox.showwarning("Not Connected", "Please connect first")
            return
        
        pos = self.connection.get_position()
        self.connection.probe_z = pos.z
        self.connection.probe_z_set = True
        self.probe_z_label.config(text=f"{pos.z:.3f}", fg="green")
        self.save_settings()
    
    def on_connection_change(self, connected: bool):
        """Handle connection status changes for settings UI."""
        if not connected:
            # Reset probe_z_set when disconnected
            self.connection.probe_z_set = False
            self.probe_z_label.config(text="NOT SET", fg="red")
    
    def jog_axis(self, axis: str, distance: float):
        """Jog an axis by the specified distance (for button presses)."""
        if not self.connection.connected:
            messagebox.showwarning("Not Connected", "Please connect first")
            return
        
        try:
            self.connection.begin_operation()
            self.connection.send_command("G91")  # Relative mode
            response = self.connection.send_command(f"G0 {axis}{distance:.3f} F1000")
            # Check for probe crash during jog
            if 'PRB:' in response:
                raise ProbeCrash(f"Probe triggered during {axis} jog")
            self.connection.send_command("G90")  # Back to absolute
            self.connection.wait_for_idle()
            self.connection.end_operation()
        except ProbeCrash as e:
            self.connection.end_operation()
            messagebox.showerror("Probe Crash", f"Probe triggered during jog:\n{e}")
        except AlarmError as e:
            self.connection.end_operation()
            messagebox.showerror("Machine Alarm", f"Machine entered alarm state:\n{e}")
        except TimeoutError as e:
            self.connection.end_operation()
            messagebox.showerror("Operation Timed Out", f"Jog timed out:\n{e}")
        except Exception as e:
            self.connection.end_operation()
            messagebox.showerror("Jog Error", str(e))
    
    def unlock_machine(self):
        """Send unlock command to clear alarm state."""
        if not self.connection.connected:
            messagebox.showwarning("Not Connected", "Please connect first")
            return
        
        try:
            # Don't use begin_operation() here - it waits for idle which won't happen in alarm state
            # Instead, manually stop heartbeat and send command directly
            self.connection.stop_heartbeat()
            time.sleep(0.2)
            
            # Check if socket is still valid
            if not self.connection.sock:
                messagebox.showerror("Connection Lost", 
                    "Connection to machine was lost.\n"
                    "Please disconnect and reconnect.")
                self.connection.connected = False
                self.connection.notify_status()
                return
            
            # Flush any stale data
            try:
                self.connection.sock.setblocking(False)
                while True:
                    try:
                        self.connection.sock.recv(4096)
                    except BlockingIOError:
                        break
                    except:
                        break
            except:
                pass
            finally:
                try:
                    self.connection.sock.setblocking(True)
                    self.connection.sock.settimeout(5.0)
                except:
                    pass
            
            # Send unlock command directly (skip alarm check)
            try:
                self.connection.sock.sendall("$X\n".encode('utf-8'))
            except (BrokenPipeError, ConnectionResetError, OSError) as e:
                messagebox.showerror("Connection Lost", 
                    f"Connection to machine was lost: {e}\n"
                    "Please disconnect and reconnect.")
                self.connection.connected = False
                self.connection.notify_status()
                return
            
            time.sleep(0.3)
            
            # Try to read response
            try:
                response = self.connection.sock.recv(4096).decode('utf-8')
                print(f"Unlock response: {response.strip()[:100]}")
            except:
                pass
            
            # Wait a moment for machine to process
            time.sleep(0.5)
            
            # Check status
            try:
                self.connection.sock.sendall("?\n".encode('utf-8'))
                time.sleep(0.2)
                status = self.connection.sock.recv(4096).decode('utf-8')
                print(f"Status after unlock: {status.strip()[:100]}")
                
                # Restart heartbeat
                self.connection.start_heartbeat()
                
                if '<Alarm' in status:
                    messagebox.showwarning("Unlock", 
                        "Unlock command sent, but machine still reports alarm.\n"
                        "You may need to press the physical reset button or power cycle.")
                else:
                    messagebox.showinfo("Unlock", "Machine unlocked successfully.")
            except (BrokenPipeError, ConnectionResetError, OSError) as e:
                messagebox.showerror("Connection Lost", 
                    f"Connection to machine was lost: {e}\n"
                    "Please disconnect and reconnect.")
                self.connection.connected = False
                self.connection.notify_status()
                return
            except Exception as e:
                self.connection.start_heartbeat()
                messagebox.showwarning("Unlock", f"Unlock command sent. Could not verify status: {e}")
                
        except Exception as e:
            # Make sure heartbeat restarts if possible
            try:
                self.connection.start_heartbeat()
            except:
                pass
            messagebox.showerror("Unlock Error", str(e))
    
    def init_pendant(self):
        """Initialize pendant if available."""
        if not HAS_HID:
            self.pendant_step_frame.pack_forget()  # Hide step selector
            return
        
        if WHB04B4.find_device():
            self.pendant = WHB04B4(
                self.pendant_jog,
                self.on_pendant_step_change,
                self.on_pendant_axis_change
            )
            if self.pendant.connect():
                self.pendant.start()
                self.pendant_label.config(text="Pendant connected", foreground="green")
                # Update step display with initial value
                self.pendant_step_label.config(text=f"{self.pendant.step_size}")
            else:
                self.pendant_label.config(text="Pendant error", foreground="red")
                self.pendant = None
        else:
            self.pendant_step_frame.pack_forget()  # Hide step display if no pendant
    
    def pendant_jog(self, axis: str, distance: float):
        """Jog callback for pendant. Distance is already delta * step_size."""
        if not self.connection.connected:
            return
        
        try:
            self.connection.begin_operation()
            # Combined command for lower latency
            response = self.connection.send_command(f"G91 G0 {axis}{distance:.4f} F1000")
            # Check for probe crash during jog
            if 'PRB:' in response:
                raise ProbeCrash(f"Probe triggered during {axis} jog")
            self.connection.send_command("G90")
            self.connection.end_operation()
        except ProbeCrash as e:
            self.connection.end_operation()
            print(f"Pendant jog probe crash: {e}")
            messagebox.showerror("Probe Crash", f"Probe triggered during jog:\n{e}")
        except AlarmError as e:
            self.connection.end_operation()
            print(f"Pendant jog alarm: {e}")
            messagebox.showerror("Machine Alarm", f"Machine entered alarm state:\n{e}")
        except Exception as e:
            self.connection.end_operation()
            print(f"Pendant jog error: {e}")
    
    def on_pendant_axis_change(self, axis: str):
        """Called when pendant axis selection changes."""
        # Could update UI to show current axis if desired
        pass
    
    def on_pendant_step_change(self, step_size: float):
        """Called when pendant step size knob changes."""
        self.pendant_step_label.config(text=f"{step_size}")
    
    def on_global_position_update(self, pos: Point, a: float):
        """Update global position displays."""
        # WCS position
        self.wcs_label.config(
            text=f"X: {pos.x:8.3f}  Y: {pos.y:8.3f}  Z: {pos.z:8.3f}  A: {a:8.3f}"
        )
        # Machine position
        mpos = self.connection.machine_position
        ma = self.connection.machine_a_position
        self.mach_label.config(
            text=f"X: {mpos.x:8.3f}  Y: {mpos.y:8.3f}  Z: {mpos.z:8.3f}  A: {ma:8.3f}"
        )
        
        # Tool display
        tool_num = self.connection.tool_number
        if tool_num == SharedCarveraConnection.PROBE_TOOL_NUMBER:
            self.tool_label.config(text="3D Probe", foreground="green")
        elif tool_num == 0:
            self.tool_label.config(text="None", foreground="gray")
        else:
            self.tool_label.config(text=str(tool_num), foreground="black")
        
        # WCS Rotation display
        rot = self.connection.wcs_rotation
        self.rotation_label.config(text=f"{rot:.3f}°")
    
    def on_global_status_change(self, connected: bool):
        """Update global connection indicator."""
        if connected:
            self.conn_indicator.config(text="● Carvera Connected", foreground="green")
        else:
            self.conn_indicator.config(text="● Carvera Disconnected", foreground="red")
    
    def on_close(self):
        # Disconnect pendant if connected
        if self.pendant:
            self.pendant.disconnect()
        
        if self.connection.connected:
            self.connection.disconnect()
        self.root.destroy()
    
    def run(self):
        self.root.mainloop()


def main():
    app = CarveraUnifiedApp()
    app.run()


if __name__ == "__main__":
    main()