#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# ROS2 + Pygame pose / marker GUI
#
# Requirements:
#   - ROS 2 (rclpy, geometry_msgs)
#   - pygame
#   - pyyaml
#   - ament_index_python
#
# Run (inside your ROS2 workspace):
#   ros2 run <your_package> pose_marker_gui.py
#
# NOTE: Set PACKAGE_NAME below to your actual package name.

import os
import math
import sys
from typing import List, Optional, Tuple

import pygame
import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory


# ===== USER CONFIG =====
PACKAGE_NAME = 'virtual_safety_net'    # <-- change to your package name
POSE_TOPIC   = 'pose'              # <-- topic name for PoseStamped


# ===== ROS2 NODE =====

class PoseSubscriberNode(Node):
    def __init__(self):
        super().__init__('pose_marker_gui')
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.subscription = self.create_subscription(
            PoseStamped, POSE_TOPIC, self.pose_callback, qos_profile=pose_qos
        )
        self.latest_pose = None  # (x, y, yaw)

    def pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        # yaw from quaternion (2D)
        # ref: standard yaw formula
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.latest_pose = (x, y, yaw)


# ===== PYGAME GUI =====

class Marker:
    def __init__(self, x: float, y: float, marked: bool = False):
        self.x = x
        self.y = y
        self.marked = marked  # True = “marked location”, False = just POI


class Canvas:
    def __init__(self, rect: pygame.Rect):
        self.rect = rect
        self.zoom = 50.0         # pixels per world unit
        self.pan_x = 0.0         # additional pixel offset
        self.pan_y = 0.0
        self.is_panning = False
        self.last_mouse_pos = None

        # World “axes limits” (metadata, user-editable via View > Canvas)
        self.x_min = -10.0
        self.x_max = 10.0
        self.y_min = -10.0
        self.y_max = 10.0

    # --- world <-> screen transforms ---

    def world_to_screen(self, wx: float, wy: float) -> Tuple[int, int]:
        cx = self.rect.centerx
        cy = self.rect.centery
        sx = cx + wx * self.zoom + self.pan_x
        sy = cy - wy * self.zoom + self.pan_y
        return int(sx), int(sy)

    def screen_to_world(self, sx: int, sy: int) -> Tuple[float, float]:
        cx = self.rect.centerx
        cy = self.rect.centery
        wx = (sx - cx - self.pan_x) / self.zoom
        wy = -(sy - cy - self.pan_y) / self.zoom
        return wx, wy

    # --- panning & zooming ---

    def start_pan(self, mouse_pos: Tuple[int, int]):
        if self.rect.collidepoint(mouse_pos):
            self.is_panning = True
            self.last_mouse_pos = mouse_pos

    def update_pan(self, mouse_pos: Tuple[int, int]):
        if self.is_panning and self.last_mouse_pos is not None:
            dx = mouse_pos[0] - self.last_mouse_pos[0]
            dy = mouse_pos[1] - self.last_mouse_pos[1]
            self.pan_x += dx
            self.pan_y += dy
            self.last_mouse_pos = mouse_pos

    def end_pan(self):
        self.is_panning = False
        self.last_mouse_pos = None

    def apply_zoom(self, mouse_pos: Tuple[int, int], wheel_y: int):
        if not self.rect.collidepoint(mouse_pos):
            return

        # Zoom factor
        zoom_factor = 1.1 ** wheel_y
        if (self.zoom * zoom_factor) < 5.0:
            return  # avoid too small

        # Keep world position under cursor fixed
        mx, my = mouse_pos
        wx_before, wy_before = self.screen_to_world(mx, my)
        self.zoom *= zoom_factor
        sx_after, sy_after = self.world_to_screen(wx_before, wy_before)
        dx = mx - sx_after
        dy = my - sy_after
        self.pan_x += dx
        self.pan_y += dy

    def update_rect_on_resize(self, new_rect: pygame.Rect, old_rect: pygame.Rect):
        # Keep zoom & pan, but adjust for changed center so visual center stays
        old_cx, old_cy = old_rect.centerx, old_rect.centery
        new_cx, new_cy = new_rect.centerx, new_rect.centery
        dcx = new_cx - old_cx
        dcy = new_cy - old_cy
        self.pan_x += dcx
        self.pan_y += dcy
        self.rect = new_rect


class CanvasSettingsOverlay:
    """Simple overlay to edit x_min/x_max/y_min/y_max with text input."""
    def __init__(self, canvas: Canvas, font: pygame.font.Font):
        self.canvas = canvas
        self.font = font
        self.active = False
        self.fields = ['x_min', 'x_max', 'y_min', 'y_max']
        self.values = {
            'x_min': str(canvas.x_min),
            'x_max': str(canvas.x_max),
            'y_min': str(canvas.y_min),
            'y_max': str(canvas.y_max),
        }
        self.current_field_idx = 0

    def open(self):
        self.active = True
        # sync current values
        self.values['x_min'] = str(self.canvas.x_min)
        self.values['x_max'] = str(self.canvas.x_max)
        self.values['y_min'] = str(self.canvas.y_min)
        self.values['y_max'] = str(self.canvas.y_max)

    def close_and_apply(self):
        # parse floats and apply
        try:
            self.canvas.x_min = float(self.values['x_min'])
            self.canvas.x_max = float(self.values['x_max'])
            self.canvas.y_min = float(self.values['y_min'])
            self.canvas.y_max = float(self.values['y_max'])
        except ValueError:
            # ignore invalid input, keep old values
            pass
        self.active = False

    def handle_event(self, event):
        if not self.active:
            return
        if event.type == pygame.KEYDOWN:
            field = self.fields[self.current_field_idx]
            if event.key == pygame.K_RETURN:
                self.close_and_apply()
            elif event.key == pygame.K_TAB:
                self.current_field_idx = (self.current_field_idx + 1) % len(self.fields)
            elif event.key == pygame.K_BACKSPACE:
                self.values[field] = self.values[field][:-1]
            elif event.key == pygame.K_ESCAPE:
                self.active = False
            else:
                # accept text (including minus, dot, digits)
                if event.unicode:
                    self.values[field] += event.unicode

    def draw(self, screen: pygame.Surface):
        if not self.active:
            return
        w, h = screen.get_size()
        overlay_rect = pygame.Rect(w//2 - 200, h//2 - 120, 400, 240)
        pygame.draw.rect(screen, (30, 30, 30), overlay_rect)
        pygame.draw.rect(screen, (200, 200, 200), overlay_rect, 2)

        title_surf = self.font.render("Canvas Limits (Enter = apply, Esc = cancel, Tab = next)", True, (255, 255, 255))
        screen.blit(title_surf, (overlay_rect.x + 10, overlay_rect.y + 10))

        y_start = overlay_rect.y + 50
        for i, key in enumerate(self.fields):
            label = f"{key}: "
            label_surf = self.font.render(label, True, (220, 220, 220))
            screen.blit(label_surf, (overlay_rect.x + 20, y_start + i*40))

            val = self.values[key]
            is_active = (i == self.current_field_idx)
            color = (255, 255, 0) if is_active else (200, 200, 200)
            val_surf = self.font.render(val, True, color)
            screen.blit(val_surf, (overlay_rect.x + 120, y_start + i*40))


class PoseMarkerGUI:
    def __init__(self, node: PoseSubscriberNode, executor: SingleThreadedExecutor):
        pygame.init()
        pygame.display.set_caption("Pose & Marker GUI")
        self.screen = pygame.display.set_mode((1000, 700), pygame.RESIZABLE)
        self.clock = pygame.time.Clock()

        self.node = node
        self.executor = executor

        self.running = True

        # Layout
        self.toolbar_height = 30
        self.toolbar_color = (50, 50, 50)
        self.bg_color = (20, 20, 20)

        width, height = self.screen.get_size()
        canvas_rect = pygame.Rect(
            0, self.toolbar_height, width, height - self.toolbar_height - 40
        )
        self.canvas = Canvas(canvas_rect)

        self.font = pygame.font.SysFont(None, 20)
        self.large_font = pygame.font.SysFont(None, 28)

        # Menus
        self.show_file_menu = False
        self.show_view_menu = False

        self.file_menu_items = ["Save", "Open", "Exit"]
        self.view_menu_items = ["Canvas"]

        # Markers & selection
        self.markers: List[Marker] = []
        self.selected_marker: Optional[int] = None

        # Buttons
        self.mark_button_visible = False
        self.remove_button_visible = False
        self.mark_button_rect = pygame.Rect(10, height - 30, 80, 20)
        self.remove_button_rect = pygame.Rect(100, height - 30, 80, 20)

        # Canvas settings overlay
        self.canvas_settings = CanvasSettingsOverlay(self.canvas, self.font)

    # ===== MAIN LOOP =====

    def run(self):
        while self.running:
            # Allow ROS callbacks
            self.executor.spin_once(timeout_sec=0.0)

            for event in pygame.event.get():
                self.handle_event(event)

            self.draw()
            pygame.display.flip()
            self.clock.tick(60)

        pygame.quit()

    # ===== EVENT HANDLING =====

    def handle_event(self, event):
        # First, give overlay a chance
        if self.canvas_settings.active:
            self.canvas_settings.handle_event(event)
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.VIDEORESIZE:
                self.handle_resize(event)
            return

        if event.type == pygame.QUIT:
            self.running = False

        elif event.type == pygame.VIDEORESIZE:
            self.handle_resize(event)

        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # left click
                self.handle_left_click(event.pos)
                # start panning if clicked on canvas (not on toolbar or menu)
                if self.canvas.rect.collidepoint(event.pos) and not self.click_on_toolbar(event.pos):
                    self.canvas.start_pan(event.pos)
            elif event.button == 4:  # wheel up
                self.canvas.apply_zoom(event.pos, +1)
            elif event.button == 5:  # wheel down
                self.canvas.apply_zoom(event.pos, -1)

        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                self.canvas.end_pan()

        elif event.type == pygame.MOUSEMOTION:
            if self.canvas.is_panning:
                self.canvas.update_pan(event.pos)

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                self.running = False

    def handle_resize(self, event: pygame.event.Event):
        new_w, new_h = event.w, event.h
        old_rect = self.canvas.rect.copy()
        self.screen = pygame.display.set_mode((new_w, new_h), pygame.RESIZABLE)
        new_canvas_rect = pygame.Rect(
            0,
            self.toolbar_height,
            new_w,
            new_h - self.toolbar_height - 40,
        )
        self.canvas.update_rect_on_resize(new_canvas_rect, old_rect)
        # Update button positions
        self.mark_button_rect = pygame.Rect(10, new_h - 30, 80, 20)
        self.remove_button_rect = pygame.Rect(100, new_h - 30, 80, 20)
    def has_valid_marked_hull(self) -> bool:
        """Return True iff there are at least 3 non-colinear marked points."""
        pts = [(m.x, m.y) for m in self.markers if m.marked]

        # Need at least 3 points
        if len(pts) < 3:
            print("[WARN] Need at least 3 marked points to form a convex hull.")
            return False

        # Check if there exists a triple of points that is non-colinear
        eps = 1e-9
        n = len(pts)
        for i in range(n):
            x1, y1 = pts[i]
            for j in range(i + 1, n):
                x2, y2 = pts[j]
                for k in range(j + 1, n):
                    x3, y3 = pts[k]
                    # Twice signed area of triangle (p1, p2, p3)
                    area2 = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)
                    if abs(area2) > eps:
                        return True  # found a non-colinear triple

        print("[WARN] All marked points are colinear; convex hull is degenerate.")
        return False

    def click_on_toolbar(self, pos: Tuple[int, int]) -> bool:
        x, y = pos
        return y < self.toolbar_height

    def handle_left_click(self, pos: Tuple[int, int]):
        x, y = pos
        # Click in toolbar: open/toggle menus
        if y < self.toolbar_height:
            self.handle_toolbar_click(pos)
            return

        # Click in dropdown menus
        if self.show_file_menu or self.show_view_menu:
            if self.handle_menu_click(pos):
                return

        # Click on buttons
        if self.mark_button_visible and self.mark_button_rect.collidepoint(pos):
            self.handle_mark_button()
            return
        if self.remove_button_visible and self.remove_button_rect.collidepoint(pos):
            self.handle_remove_button()
            return

        # Click on canvas
        if self.canvas.rect.collidepoint(pos):
            self.handle_canvas_click(pos)

    # ===== TOOLBAR & MENUS =====

    def handle_toolbar_click(self, pos: Tuple[int, int]):
        x, y = pos
        # File tab region
        file_rect = pygame.Rect(10, 0, 60, self.toolbar_height)
        view_rect = pygame.Rect(80, 0, 60, self.toolbar_height)

        if file_rect.collidepoint(pos):
            self.show_file_menu = not self.show_file_menu
            self.show_view_menu = False
        elif view_rect.collidepoint(pos):
            self.show_view_menu = not self.show_view_menu
            self.show_file_menu = False
        else:
            self.show_file_menu = False
            self.show_view_menu = False

    def handle_menu_click(self, pos: Tuple[int, int]) -> bool:
        x, y = pos
        handled = False

        if self.show_file_menu:
            base_rect = pygame.Rect(10, self.toolbar_height, 80, len(self.file_menu_items) * 22)
            if base_rect.collidepoint(pos):
                index = (y - base_rect.y) // 22
                if 0 <= index < len(self.file_menu_items):
                    item = self.file_menu_items[index]
                    if item == "Save":
                        self.save_markers_to_yaml()
                    elif item == "Open":
                        self.load_markers_from_yaml()
                    elif item == "Exit":
                        self.running = False
                    handled = True
            self.show_file_menu = False

        if self.show_view_menu:
            base_rect = pygame.Rect(80, self.toolbar_height, 100, len(self.view_menu_items) * 22)
            if base_rect.collidepoint(pos):
                index = (y - base_rect.y) // 22
                if 0 <= index < len(self.view_menu_items):
                    item = self.view_menu_items[index]
                    if item == "Canvas":
                        self.canvas_settings.open()
                    handled = True
            self.show_view_menu = False

        return handled

    # ===== CANVAS CLICK & MARKERS =====

    def handle_canvas_click(self, pos: Tuple[int, int]):
        """Handle clicks inside the canvas.

        Rules:
        - Click on existing marker:
            * Select that marker.
            * Remove all other green (unmarked) markers.
        - Click on empty space:
            * Remove all green markers.
            * Create exactly one new green marker at the click.
        """
        clicked_marker_idx = self.find_marker_at_screen_pos(pos)

        if clicked_marker_idx is not None:
            # Clicked on an existing marker.
            # We want to:
            #   - keep the clicked marker
            #   - keep all red (marked) markers
            #   - remove all other green markers

            kept_markers = []
            new_selected = None

            for idx, m in enumerate(self.markers):
                if idx == clicked_marker_idx:
                    # always keep the clicked one (green or red)
                    new_selected = len(kept_markers)
                    kept_markers.append(m)
                else:
                    # keep only marked (red) markers
                    if m.marked:
                        kept_markers.append(m)
                    # any other green marker is dropped

            self.markers = kept_markers
            self.selected_marker = new_selected

        else:
            # Clicked empty space:
            #   - remove ALL green markers
            #   - add exactly one new green point at click
            self.markers = [m for m in self.markers if m.marked]

            wx, wy = self.canvas.screen_to_world(*pos)
            self.markers.append(Marker(wx, wy, marked=False))
            self.selected_marker = len(self.markers) - 1

        # Update Mark / Remove buttons based on the current selection
        self.update_button_visibility()

    def find_marker_at_screen_pos(self, pos: Tuple[int, int]) -> Optional[int]:
        mx, my = pos
        base_radius_world = 0.15  # logical radius in world units

        for i, m in enumerate(self.markers):
            sx, sy = self.canvas.world_to_screen(m.x, m.y)
            radius_px = max(5, int(base_radius_world * self.canvas.zoom))
            dist = math.hypot(mx - sx, my - sy)

            if dist <= radius_px:
                return i

        return None


    def update_button_visibility(self):
        if self.selected_marker is None:
            self.mark_button_visible = False
            self.remove_button_visible = False
            return

        marker = self.markers[self.selected_marker]
        if marker.marked:
            self.mark_button_visible = False
            self.remove_button_visible = True
        else:
            self.mark_button_visible = True
            self.remove_button_visible = False

    def handle_mark_button(self):
        if self.selected_marker is None:
            return
        self.markers[self.selected_marker].marked = True
        self.update_button_visibility()

    def handle_remove_button(self):
        if self.selected_marker is None:
            return
        self.markers[self.selected_marker].marked = False
        self.update_button_visibility()

    # ===== SAVE / LOAD YAML =====

    def get_config_dir(self) -> str:
        """Return the config directory for this package (create if needed)."""
        try:
            pkg_share = get_package_share_directory(PACKAGE_NAME)
        except Exception:
            pkg_share = os.getcwd()

        config_dir = os.path.join(pkg_share, 'config')
        os.makedirs(config_dir, exist_ok=True)
        return config_dir

    def save_markers_to_yaml(self):
        """Save ONLY marked points to a user-selected YAML file.

        YAML format:
            points:
            - {x: ..., y: ...}
        """

        # Collect marked points
        marked_pts = [(m.x, m.y) for m in self.markers if m.marked]

        # Check convex hull condition: ≥3 non-colinear points
        if not self.has_valid_marked_hull():
            # has_valid_marked_hull already prints a message
            return
        self.release_pygame_focus()
        # Use a file dialog
        try:
            import tkinter as tk
            from tkinter import filedialog
        except ImportError:
            print("[ERROR] tkinter not available; cannot open save dialog.")
            return

        root = tk.Tk()
        root.withdraw()  # hide main Tk window

        initialdir = self.get_config_dir()
        os.makedirs(initialdir, exist_ok=True)

        file_path = filedialog.asksaveasfilename(
            initialdir=initialdir,
            title="Save marked points",
            defaultextension=".yaml",
            filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")]
        )
        root.destroy()
        self.restore_pygame_focus()
        if not file_path:
            # User cancelled
            return

        data = {
            'points': [
                {'x': float(x), 'y': float(y)}
                for x, y in marked_pts
            ]
        }

        try:
            with open(file_path, 'w') as f:
                yaml.safe_dump(data, f)
            print(f"[INFO] Saved {len(marked_pts)} marked points to {file_path}")
        except Exception as e:
            print(f"[ERROR] Failed to save markers: {e}")


    def load_markers_from_yaml(self):
        """Load points from a YAML file and treat them as marked points."""
        self.release_pygame_focus()
        try:
            import tkinter as tk
            from tkinter import filedialog
        except ImportError:
            print("[ERROR] tkinter not available; cannot open load dialog.")
            return

        root = tk.Tk()
        root.withdraw()

        initialdir = self.get_config_dir()
        os.makedirs(initialdir, exist_ok=True)

        file_path = filedialog.askopenfilename(
            initialdir=initialdir,
            title="Open marked points",
            filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")]
        )
        root.destroy()
        self.restore_pygame_focus()
        if not file_path:
            # User cancelled
            return

        if not os.path.exists(file_path):
            print(f"[WARN] YAML file not found: {file_path}")
            return

        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f) or {}
            pts = data.get('points', [])

            self.markers = []
            for p in pts:
                x = float(p.get('x', 0.0))
                y = float(p.get('y', 0.0))
                # Loaded points are considered "marked" (red)
                self.markers.append(Marker(x, y, marked=True))

            self.selected_marker = None
            self.update_button_visibility()
            print(f"[INFO] Loaded {len(self.markers)} marked points from {file_path}")
        except Exception as e:
            print(f"[ERROR] Failed to load markers: {e}")
    def release_pygame_focus(self):
        """Allow Tk dialogs to receive mouse input."""
        try:
            import pygame
            pygame.event.set_grab(False)
        except Exception:
            pass

    def restore_pygame_focus(self):
        """Re-enable normal Pygame window focus behavior."""
        try:
            import pygame
            pygame.event.set_grab(True)
        except Exception:
            pass


    # ===== DRAWING =====

    def draw_toolbar(self):
        pygame.draw.rect(self.screen, self.toolbar_color, (0, 0, self.screen.get_width(), self.toolbar_height))
        # File tab
        file_rect = pygame.Rect(10, 0, 60, self.toolbar_height)
        view_rect = pygame.Rect(80, 0, 60, self.toolbar_height)
        pygame.draw.rect(self.screen, (80, 80, 80), file_rect)
        pygame.draw.rect(self.screen, (80, 80, 80), view_rect)

        file_text = self.font.render("File", True, (255, 255, 255))
        view_text = self.font.render("View", True, (255, 255, 255))
        self.screen.blit(file_text, (file_rect.x + 15, 7))
        self.screen.blit(view_text, (view_rect.x + 12, 7))

        # Dropdown menus
        if self.show_file_menu:
            base_rect = pygame.Rect(10, self.toolbar_height, 80, len(self.file_menu_items)*22)
            pygame.draw.rect(self.screen, (60, 60, 60), base_rect)
            pygame.draw.rect(self.screen, (200, 200, 200), base_rect, 1)
            for i, item in enumerate(self.file_menu_items):
                text = self.font.render(item, True, (255, 255, 255))
                self.screen.blit(text, (base_rect.x + 5, base_rect.y + 3 + i*22))

        if self.show_view_menu:
            base_rect = pygame.Rect(80, self.toolbar_height, 100, len(self.view_menu_items)*22)
            pygame.draw.rect(self.screen, (60, 60, 60), base_rect)
            pygame.draw.rect(self.screen, (200, 200, 200), base_rect, 1)
            for i, item in enumerate(self.view_menu_items):
                text = self.font.render(item, True, (255, 255, 255))
                self.screen.blit(text, (base_rect.x + 5, base_rect.y + 3 + i*22))

    def draw_buttons(self):
        # Draw Mark / Remove buttons at bottom
        if self.mark_button_visible:
            pygame.draw.rect(self.screen, (60, 120, 60), self.mark_button_rect)
            pygame.draw.rect(self.screen, (255, 255, 255), self.mark_button_rect, 1)
            txt = self.font.render("Mark", True, (255, 255, 255))
            self.screen.blit(txt, (self.mark_button_rect.x + 15, self.mark_button_rect.y + 2))

        if self.remove_button_visible:
            pygame.draw.rect(self.screen, (120, 60, 60), self.remove_button_rect)
            pygame.draw.rect(self.screen, (255, 255, 255), self.remove_button_rect, 1)
            txt = self.font.render("Remove", True, (255, 255, 255))
            self.screen.blit(txt, (self.remove_button_rect.x + 5, self.remove_button_rect.y + 2))

    def draw_canvas_background(self):
        pygame.draw.rect(self.screen, (30, 30, 30), self.canvas.rect)

        # optional: draw axes cross at origin
        origin_sx, origin_sy = self.canvas.world_to_screen(0.0, 0.0)
        if self.canvas.rect.collidepoint(origin_sx, origin_sy):
            pygame.draw.line(self.screen, (80, 80, 200),
                             (self.canvas.rect.left, origin_sy),
                             (self.canvas.rect.right, origin_sy), 1)
            pygame.draw.line(self.screen, (80, 200, 80),
                             (origin_sx, self.canvas.rect.top),
                             (origin_sx, self.canvas.rect.bottom), 1)

        # show canvas limits metadata
        limits_text = f"x:[{self.canvas.x_min:.1f}, {self.canvas.x_max:.1f}], y:[{self.canvas.y_min:.1f}, {self.canvas.y_max:.1f}], zoom:{self.canvas.zoom:.1f}"
        txt_surf = self.font.render(limits_text, True, (200, 200, 200))
        self.screen.blit(txt_surf, (self.canvas.rect.x + 5, self.canvas.rect.y + 5))

    def draw_pose(self):
        if self.node.latest_pose is None:
            return
        x, y, yaw = self.node.latest_pose
        sx, sy = self.canvas.world_to_screen(x, y)
        base_radius_world = 0.2
        radius_px = max(6, int(base_radius_world * self.canvas.zoom))

        # circle
        pygame.draw.circle(self.screen, (0, 180, 255), (sx, sy), radius_px)

        # heading arrow
        arrow_len = radius_px * 1.8
        ex = sx + arrow_len * math.cos(yaw)
        ey = sy - arrow_len * math.sin(yaw)
        pygame.draw.line(self.screen, (255, 255, 255), (sx, sy), (int(ex), int(ey)), 2)

    def draw_markers(self):
        base_radius_world = 0.15
        for idx, m in enumerate(self.markers):
            sx, sy = self.canvas.world_to_screen(m.x, m.y)
            radius_px = max(5, int(base_radius_world * self.canvas.zoom))
            if m.marked:
                color = (255, 80, 80)   # marked locations = red-ish
            else:
                color = (80, 255, 80)   # unmarked POI = green-ish

            pygame.draw.circle(self.screen, color, (sx, sy), radius_px)

            # highlight selected
            if self.selected_marker == idx:
                pygame.draw.circle(self.screen, (255, 255, 255), (sx, sy), radius_px+2, 2)

    def draw(self):
        self.screen.fill(self.bg_color)

        # 1) Draw canvas and everything inside it
        self.draw_canvas_background()
        self.draw_pose()
        self.draw_markers()
        self.draw_buttons()

        # 2) Draw toolbar + its dropdown menus on top of the canvas
        self.draw_toolbar()

        # 3) Draw the canvas settings overlay on top of everything
        self.canvas_settings.draw(self.screen)



# ===== MAIN ENTRYPOINT =====

def main(argv=None):
    rclpy.init(args=argv)
    node = PoseSubscriberNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    gui = PoseMarkerGUI(node, executor)
    try:
        gui.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
