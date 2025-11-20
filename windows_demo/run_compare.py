import os, tempfile, math
os.environ.setdefault("MPLCONFIGDIR", os.path.join(tempfile.gettempdir(), "mpl"))

import tkinter as tk
from tkinter import ttk, simpledialog

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import matplotlib.image as mpimg
from matplotlib.collections import LineCollection

import math

try:
    from PIL import Image, ImageTk
except Exception:
    Image = None
    ImageTk = None

from utils.gridmap import GridMap
from utils.costmap import CostMap
from algorithms.dfs import dfs_steps
from algorithms.bfs import bfs_steps
from algorithms.dijkstra import dijkstra_steps
from algorithms.astar import astar_steps, HEURISTICS

HEX = {
    "free":       "#f5f5f5",
    "obstacle":   "#1e1e1e",
    "start":      "#32cd32",
    "goal":       "#dc143c",
    "grid_line":  "#c8c8c8",
}
HEX_VIS = {
    "visited": "#b3d9ff",
    "frontier":"#ffcc80",
    "path":    "#ffff00"
}

LOGO_PATH   = "utils/RCI_LAB_LOGO.png" #"C:/Users/home/Desktop/Grid_Based_Planning/code/utils/RCI_LAB_LOGO.png"
LOGO_MAX_W  = 100
LOGO_MARGIN = 10

def hex_to_rgb255(h: str):
    h = h.strip()
    if h.startswith('#'): h = h[1:]
    if len(h) == 3: h = ''.join([c*2 for c in h])
    return tuple(int(h[i:i+2], 16) for i in (0,2,4))

ALGO_REGISTRY = {
    "dijkstra": dict(label="Dijkstra", title="Dijkstra", factory=lambda gm: dijkstra_steps(gm)),
    "astar":    dict(label="A*",       title=lambda h: f"A* (heuristic = {h})", factory=lambda h: (lambda gm: astar_steps(gm, heuristic=h))),
    "bfs":      dict(label="BFS",      title="BFS", factory=lambda gm: bfs_steps(gm)),
    "dfs":      dict(label="DFS",      title="DFS", factory=lambda gm: dfs_steps(gm)),
}

class CompareApp(tk.Tk):
    def __init__(self, H=5, W=5):
        super().__init__()
        self.title("RCI Lab | Grid Based Planning Algorithm Comparing - DFS / BFS / Dijkstra / A*")
        self.geometry("1420x860")

        self._icon_img_ref = None
        if os.path.exists(LOGO_PATH) and Image is not None and ImageTk is not None:
            try:
                ic = Image.open(LOGO_PATH).convert("RGBA").resize((32, 32), Image.LANCZOS)
                self._icon_img_ref = ImageTk.PhotoImage(ic)
                self.iconphoto(True, self._icon_img_ref)
            except Exception:
                pass

        self.gm = GridMap(H, W, diag=False, diag_safety=True)
        self.heuristic = "manhattan"
        self.mode = tk.StringVar(value='obstacle')
        self.conn = tk.StringVar(value='4')
        self.interval_ms = 30

        self.axes = []
        self.states = []
        self.gens = []
        self.step_counts = []
        self.base_meshes = []
        self.cm_meshes   = []
        self.vis_meshes  = []
        self.info_texts  = [] 

        self._ani = None
        self.anim_running = False
        self._logo_arr = None

        self._costmap = CostMap.zeros(H, W)
        self.gm.set_costmap(self._costmap)

        # top bar
        bar_top = ttk.Frame(self); bar_top.pack(side=tk.TOP, fill=tk.X)
        self.lbl_size = ttk.Label(bar_top, text=f"Size: {W}×{H} cells")
        self.lbl_size.pack(side=tk.LEFT, padx=6)
        ttk.Button(bar_top, text="Heuristic (H)",       command=self._cycle_heur).pack(side=tk.LEFT, padx=6)
        ttk.Button(bar_top, text="Play/Pause (Enter)",  command=self._play_pause).pack(side=tk.LEFT, padx=6)
        ttk.Button(bar_top, text="Step (N)",            command=self._step_once).pack(side=tk.LEFT, padx=4)
        ttk.Button(bar_top, text="Reset (R)",           command=lambda:self.reset_search(True)).pack(side=tk.LEFT, padx=4)
        ttk.Button(bar_top, text="Clear (C)",           command=lambda:self.reset_search(False)).pack(side=tk.LEFT, padx=4)
        ttk.Button(bar_top, text="Resize Map",          command=self._resize_popup).pack(side=tk.RIGHT, padx=6)
        ttk.Button(bar_top, text="Speed (ms)",          command=self._speed_popup).pack(side=tk.RIGHT, padx=6)
        ttk.Button(bar_top, text="Custom Costmap",      command=self._open_costmap_editor).pack(side=tk.RIGHT, padx=6)

        # middle bar
        bar_mid = ttk.Frame(self); bar_mid.pack(side=tk.TOP, fill=tk.X, pady=(2,0))
        ttk.Label(bar_mid, text="Mode:").pack(side=tk.LEFT, padx=(6,2))
        for key, label in (('obstacle','Obstacle (O)'),
                           ('start','Start (S)'),
                           ('goal','Goal (G)'),
                           ('erase','Eraser (E)')):
            ttk.Radiobutton(bar_mid, text=label, value=key, variable=self.mode).pack(side=tk.LEFT, padx=6)

        ttk.Label(bar_mid, text="Connectivity:").pack(side=tk.LEFT, padx=(16,4))
        ttk.Radiobutton(bar_mid, text="4-neigh", value='4', variable=self.conn, command=self._on_conn).pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(bar_mid, text="8-neigh", value='8', variable=self.conn, command=self._on_conn).pack(side=tk.LEFT, padx=2)

        ttk.Label(bar_mid, text="Algorithms:").pack(side=tk.LEFT, padx=(16,4))
        algo_keys_order = ("dfs", "bfs", "dijkstra", "astar")
        self.var_chk = {k: tk.BooleanVar(value=(k=="dfs")) for k in algo_keys_order}
        for key in algo_keys_order:
            ttk.Checkbutton(bar_mid, text=ALGO_REGISTRY[key]["label"], variable=self.var_chk[key],
                            command=self._on_algos_changed).pack(side=tk.LEFT, padx=4)

        self.diag_safety = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            bar_mid, text="Anti corner-cut", variable=self.diag_safety,
            command=lambda: (self.gm.set_diag_safety(self.diag_safety.get()),
                             self.label_status.config(text=f"Diagonal safety = {'ON' if self.diag_safety.get() else 'OFF'}"))
        ).pack(side=tk.LEFT, padx=(10,2))

        # statics toggle bar
        bar_stats = ttk.Frame(self); bar_stats.pack(side=tk.TOP, fill=tk.X, pady=(2,4))
        ttk.Label(bar_stats, text="Show:").pack(side=tk.LEFT, padx=(6,4))
        self.show_steps  = tk.BooleanVar(value=True)
        self.show_path   = tk.BooleanVar(value=True)
        self.show_cost   = tk.BooleanVar(value=True)
        self.show_detail = tk.BooleanVar(value=False)
        for (txt, var) in (("steps", self.show_steps),
                           ("path(hop)",  self.show_path),
                           ("cost",  self.show_cost),
                           ("detail",self.show_detail)):
            ttk.Checkbutton(bar_stats, text=txt, variable=var, command=self._update_info_labels).pack(side=tk.LEFT, padx=4)

        # canvas setting
        self.fig = plt.Figure(figsize=(12.5, 7.0))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(fill=tk.BOTH, expand=True)

        self._grid_cols = []
        self.canvas.mpl_connect("resize_event",
            lambda e: (self._place_logo(), self._update_grid_linewidths(), self._place_info_labels()))
        self.bind("<Configure>", lambda e: (self._place_logo(), self._update_grid_linewidths(), self._place_info_labels()))

        # bottom frame
        bottom = ttk.Frame(self); bottom.pack(side=tk.BOTTOM, fill=tk.X)
        self.label_hints  = ttk.Label(bottom, anchor='w',
            text="Keys: O/S/G/E mode · D 4/8 · H heuristic · Enter Play/Pause · N step · R reset · C clear")
        self.label_status = ttk.Label(bottom, anchor='e', text="Ready")
        self.label_hints.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(6,4))
        self.label_status.pack(side=tk.RIGHT, padx=(4,6))

        # event
        self.canvas.mpl_connect("button_press_event",   self._on_press)
        self.canvas.mpl_connect("motion_notify_event",  self._on_move)
        self.canvas.mpl_connect("button_release_event", self._on_release)
        self.canvas.mpl_connect("key_press_event",      self._on_key)

        self._dragging = False; self._last_rc = None

        self._rebuild_layout()
        self._update_info_labels()

    # title setting
    def _title_for(self, key: str) -> str:
        if key == "dfs":       return "DFS"
        if key == "bfs":       return "BFS"
        if key == "dijkstra":  return "Dijkstra"
        if key == "astar":     return f"A* (heuristic = {self.heuristic})"
        return key.upper()

    def _refresh_titles(self):
        for i, ax in enumerate(self.axes):
            key = self.selected_algos[i]
            ax.set_title(self._title_for(key), pad=8, fontsize=11)  # 볼드 제거
        self.canvas.draw_idle()

    # layout func.
    def _rebuild_layout(self):
        self.selected_algos = [k for k in ("dfs", "bfs", "dijkstra", "astar") if self.var_chk[k].get()]
        if not self.selected_algos:
            self.selected_algos = ["dfs"]
            self.var_chk["dfs"].set(True)

        show_n = len(self.selected_algos)
        if show_n <= 1:   rows, cols = 1, 1
        elif show_n == 2: rows, cols = 1, 2
        elif show_n == 3: rows, cols = 1, 3
        else:             rows, cols = 2, 2
        self._layout_rows, self._layout_cols = rows, cols

        self.fig.clf()
        gs = self.fig.add_gridspec(rows, cols, wspace=0.08, hspace=0.25)

        self.axes = []
        self.base_meshes = []
        self.cm_meshes   = []
        self.vis_meshes  = []
        self._grid_cols  = []
        self.info_texts  = []

        self.states = [None] * show_n
        self.gens   = [None] * show_n
        self.step_counts = [0] * show_n

        H, W = self.gm.height, self.gm.width
        X = np.arange(-0.5, W + 0.5, 1.0)
        Y = np.arange(-0.5, H + 0.5, 1.0)

        base_rgba = self._base_rgba()
        cm_rgba = self._cost_rgba_from_raw()
        face_base = self._rgba255_to01(base_rgba)
        face_cm   = self._rgba255_to01(cm_rgba)
        face_vis0 = self._rgba255_to01(np.zeros((H, W, 4), np.uint8))

        for i in range(show_n):
            r = i // cols
            c = i % cols
            ax = self.fig.add_subplot(gs[r, c])

            ax.set_xlim(-0.5, W - 0.5)
            ax.set_ylim(H - 0.5, -0.5)
            ax.set_aspect('equal', adjustable='box')
            ax.set_xticks([]); ax.set_yticks([])
            ax.set_xmargin(0); ax.set_ymargin(0)
            for sp in ax.spines.values():
                sp.set_visible(False)

            ax.set_title(self._title_for(self.selected_algos[i]), pad=8, fontsize=11)

            pc_base = ax.pcolormesh(
                X, Y, np.zeros((H, W)),
                shading='flat', edgecolors='none', antialiased=False, zorder=1
            )
            pc_base.set_array(None); pc_base.set_cmap(None)
            pc_base.set_facecolors(face_base)
            self.base_meshes.append(pc_base)

            pc_cost = ax.pcolormesh(
                X, Y, np.zeros((H, W)),
                shading='flat', edgecolors='none', antialiased=False, zorder=2
            )
            pc_cost.set_array(None); pc_cost.set_cmap(None)
            pc_cost.set_facecolors(face_cm)
            self.cm_meshes.append(pc_cost)

            pc_vis = ax.pcolormesh(
                X, Y, np.zeros((H, W)),
                shading='flat', edgecolors='none', antialiased=False, zorder=3
            )
            pc_vis.set_array(None); pc_vis.set_cmap(None)
            pc_vis.set_facecolors(face_vis0)
            self.vis_meshes.append(pc_vis)

            self._add_vector_grid(ax)
            self.axes.append(ax)

        # rebuild all
        self._build_info_labels()
        self._redraw_all_images()
        self.canvas.draw_idle()
        self._place_logo()
        self._update_grid_linewidths()
        self._refresh_titles()
        self._place_info_labels()

    # grid line -> grid is made by vector for alignment.
    # ordinary line X
    def _add_vector_grid(self, ax):
        ax.set_xmargin(0); ax.set_ymargin(0)
        for sp in ax.spines.values():
            sp.set_visible(False)
        ax.set_frame_on(False)

        W, H = self.gm.width, self.gm.height
        xs = np.arange(-0.5, W + 0.5, 1.0)
        ys = np.arange(-0.5, H + 0.5, 1.0)

        seg_v = [[(x, -0.5), (x, H - 0.5)] for x in xs]
        seg_h = [[(-0.5, y), (W - 0.5, y)] for y in ys]

        lw = self._calc_grid_lw_pts(ax, target_px=1.0)
        color = HEX["grid_line"]

        v = LineCollection(seg_v, colors=color, linewidths=lw,
                           antialiased=False, capstyle='butt', joinstyle='miter',
                           zorder=10, clip_on=False)
        h = LineCollection(seg_h, colors=color, linewidths=lw,
                           antialiased=False, capstyle='butt', joinstyle='miter',
                           zorder=10, clip_on=False)
        ax.add_collection(v); ax.add_collection(h)
        self._grid_cols.append((ax, v, h))

    def _calc_grid_lw_pts(self, ax, target_px=1.0):
        dpi = ax.figure.dpi
        return target_px * 72.0 / dpi

    def _update_grid_linewidths(self):
        if not getattr(self, "_grid_cols", None):
            return
        for ax, v, h in self._grid_cols:
            lw = self._calc_grid_lw_pts(ax, target_px=1.0)
            v.set_linewidths(lw)
            h.set_linewidths(lw)
        self.canvas.draw_idle()

    # color utility
    def _rgba255_to01(self, arr):
        return (arr.astype(np.float32).reshape(-1, 4) / 255.0)

    def _base_rgba(self):
        H, W = self.gm.height, self.gm.width
        rgba = np.empty((H, W, 4), dtype=np.uint8)
        r_free, g_free, b_free = hex_to_rgb255(HEX["free"])
        rgba[:, :, 0] = r_free; rgba[:, :, 1] = g_free; rgba[:, :, 2] = b_free; rgba[:, :, 3] = 255
        ys, xs = np.where(self.gm.grid == 1)
        r_obs, g_obs, b_obs = hex_to_rgb255(HEX["obstacle"])
        rgba[ys, xs, 0] = r_obs; rgba[ys, xs, 1] = g_obs; rgba[ys, xs, 2] = b_obs
        if self.gm.start is not None:
            rs, cs = self.gm.start
            rgba[rs, cs, :3] = hex_to_rgb255(HEX["start"])
        if self.gm.goal is not None:
            rg, cg = self.gm.goal
            rgba[rg, cg, :3] = hex_to_rgb255(HEX["goal"])
        return rgba

    def _vis_cells_rgba(self, state):
        H, W = self.gm.height, self.gm.width
        rgba = np.zeros((H, W, 4), dtype=np.uint8)
        if not state: return rgba
        rv = (*hex_to_rgb255(HEX_VIS["visited"]),  140)
        rf = (*hex_to_rgb255(HEX_VIS["frontier"]), 180)
        rp = (*hex_to_rgb255(HEX_VIS["path"]),     220)
        def paint(cells, col):
            for r, c in (cells or []):
                if self.gm.start and (r, c) == self.gm.start: continue
                if self.gm.goal  and (r, c) == self.gm.goal:  continue
                rgba[r, c] = col
        paint(state.get("visited"),  rv)
        paint(state.get("frontier"), rf)
        paint(state.get("path"),     rp)
        return rgba

    def _cost_rgba_from_raw(self):
        H, W = self.gm.height, self.gm.width
        if self._costmap is not None and hasattr(self._costmap, "cost") and \
           getattr(self._costmap.cost, "shape", None) == (H, W):
            C = np.asarray(self._costmap.cost, dtype=float)
        else:
            C = np.zeros((H, W), dtype=float)

        vmax = 5.0
        t = np.clip(C / max(1e-9, vmax), 0.0, 1.0)
        cm_rgba = np.zeros((H, W, 4), dtype=np.uint8)
        c0 = np.array([0xEE, 0xE6, 0xF7], np.float32)
        c1 = np.array([0x6A, 0x1B, 0x9A], np.float32)
        tt = t**0.6
        rgb = (c0 + (c1 - c0) * tt[..., None]).astype(np.uint8)
        cm_rgba[..., :3] = rgb

        active = (t > 0.0) & (self.gm.grid != 1)
        if self.gm.start is not None: active[self.gm.start] = False
        if self.gm.goal  is not None: active[self.gm.goal]  = False

        alpha = np.zeros_like(t, dtype=np.uint8)
        alpha[active] = (50 + 205 * t[active]).astype(np.uint8)
        cm_rgba[..., 3] = alpha
        return cm_rgba

    # rendering and animation settning
    def _redraw_all_images(self):
        for pc in (self.base_meshes + self.cm_meshes + self.vis_meshes):
            if pc.get_array() is not None:
                pc.set_array(None)
                pc.set_cmap(None)

        base_fc = self._rgba255_to01(self._base_rgba())
        for pc in self.base_meshes:
            pc.set_facecolors(base_fc)

        cm_fc = self._rgba255_to01(self._cost_rgba_from_raw())
        for pc in self.cm_meshes:
            pc.set_facecolors(cm_fc)

        for i, pc in enumerate(self.vis_meshes):
            state = self.states[i] if i < len(self.states) else None
            vis_fc = self._rgba255_to01(self._vis_cells_rgba(state))
            pc.set_facecolors(vis_fc)

        self.canvas.draw_idle()
        self._update_info_labels()

    def _start_generators(self):
        self.gens = []
        for key in self.selected_algos[:len(self.axes)]:
            if key == "astar":
                self.gens.append(astar_steps(self.gm, heuristic=self.heuristic))
            elif key == "dijkstra":
                self.gens.append(dijkstra_steps(self.gm))
            elif key == "bfs":
                self.gens.append(bfs_steps(self.gm))
            elif key == "dfs":
                self.gens.append(dfs_steps(self.gm))
        self.states = [None]*len(self.gens)
        self.step_counts = [0]*len(self.gens)
        self._update_info_labels()

    def _tick(self, _):
        advanced = False
        for i, gen in enumerate(self.gens):
            if gen is None: continue
            try:
                self.states[i] = next(gen)
                self.step_counts[i] += 1
                advanced = True
            except StopIteration:
                self.gens[i] = None
        if advanced:
            self._redraw_all_images()

        if self.gens and all(g is None for g in self.gens):
            self.anim_running = False
            if self._ani is not None:
                self._ani.event_source.stop(); self._ani = None
            self.label_status.config(text="Done")
            self._update_info_labels()

    # input and editting
    def _event_to_rc(self, ev):
        if ev.inaxes not in self.axes: return None
        if ev.xdata is None or ev.ydata is None: return None
        c = int(np.clip(np.round(ev.xdata), 0, self.gm.width  - 1))
        r = int(np.clip(np.round(ev.ydata), 0, self.gm.height - 1))
        return (r, c)

    def _apply_mode_to_cell(self, r, c):
        m = self.mode.get()
        if m == 'obstacle':
            self.gm.grid[r, c] = 1
        elif m == 'erase':
            self.gm.grid[r, c] = 0
            if self.gm.start is not None and (r, c) == self.gm.start:
                self.gm.start = None
            if self.gm.goal  is not None and (r, c) == self.gm.goal:
                self.gm.goal  = None
            if self._costmap is not None and getattr(self._costmap, "cost", None) is not None:
                if self._costmap.cost.shape == (self.gm.height, self.gm.width):
                    self._costmap.set_value(r, c, 0.0)
        elif m == 'start':
            if self.gm.is_free((r,c)): self.gm.set_start((r,c))
        elif m == 'goal':
            if self.gm.is_free((r,c)): self.gm.set_goal((r,c))

    def _paint_segment(self, rc0, rc1):
        (r0,c0), (r1,c1) = rc0, rc1
        dr, dc = abs(r1-r0), abs(c1-c0)
        sr, sc = (1 if r0<r1 else -1), (1 if c0<c1 else -1)
        err = dr - dc; r, c = r0, c0
        while True:
            self._apply_mode_to_cell(r,c)
            if r==r1 and c==c1: break
            e2 = 2*err
            if e2 > -dc: err -= dc; r += sr
            if e2 <  dr: err += dr; c += sc
        self._redraw_all_images()

    def _on_press(self, ev):
        if ev.button != 1: return
        rc = self._event_to_rc(ev)
        if rc is None: return
        self._dragging = True; self._last_rc = rc
        self._paint_segment(rc, rc)

    def _on_move(self, ev):
        if not self._dragging: return
        rc = self._event_to_rc(ev)
        if rc is None or rc == self._last_rc: return
        self._paint_segment(self._last_rc, rc)
        self._last_rc = rc

    def _on_release(self, ev):
        if ev.button != 1: return
        self._dragging = False; self._last_rc = None

    def _on_key(self, ev):
        k = (ev.key or "").lower()
        if   k == 'o': self.mode.set('obstacle')
        elif k == 's': self.mode.set('start')
        elif k == 'g': self.mode.set('goal')
        elif k == 'e': self.mode.set('erase')
        elif k == 'd': self.conn.set('8' if self.conn.get()=='4' else '4'); self._on_conn()
        elif k == 'h': self._cycle_heur()
        elif k in ('enter',' '): self._play_pause()
        elif k == 'n': self._step_once()
        elif k == 'r': self.reset_search(True)
        elif k == 'c': self.reset_search(False)

    # button
    def _on_algos_changed(self):
        self._rebuild_layout()
        self._place_logo()
        self._refresh_titles()
        self._update_info_labels()

    def _on_conn(self):
        self.gm.set_connectivity(self.conn.get() == '8')
        self.heuristic = 'octile' if self.gm.diag else 'manhattan'
        self.canvas.draw_idle()
        self._refresh_titles()
        self._update_info_labels()

    def _cycle_heur(self):
        hs = list(HEURISTICS.keys()); i = hs.index(self.heuristic)
        self.heuristic = hs[(i+1) % len(hs)]
        self.canvas.draw_idle()
        self.label_status.config(text=f"Heuristic = {self.heuristic}")
        self._refresh_titles()
        self._update_info_labels()

    def _resize_popup(self):
        W = simpledialog.askinteger("Resize Map", "Width (cells):",  minvalue=5, maxvalue=200, parent=self)
        H = simpledialog.askinteger("Resize Map", "Height (cells):", minvalue=5, maxvalue=200, parent=self)
        if not W or not H: return
        self.gm = GridMap(H, W, diag=self.gm.diag, diag_safety=self.gm.diag_safety)
        self.lbl_size.config(text=f"Size: {W}×{H} cells")
        if self._costmap is None or getattr(self._costmap, "cost", None) is None or self._costmap.cost.shape != (H, W):
            self._costmap = CostMap.zeros(H, W)
        self.gm.set_costmap(self._costmap)
        self.reset_search(keep_map=True)
        self._rebuild_layout()
        self._redraw_all_images()
        self._place_logo()
        self._update_info_labels()

    def _speed_popup(self):
        cur = self.interval_ms
        val = simpledialog.askinteger(
            "Playback Speed",
            f"재생 간격(ms)을 입력하세요 (기본 30ms). 현재: {cur} ms",
            minvalue=1, maxvalue=2000, parent=self
        )
        if val:
            self.interval_ms = int(val)
            self.label_status.config(text=f"Interval = {self.interval_ms} ms")

    def _open_costmap_editor(self):
        if self._costmap is None or self._costmap.cost.shape != (self.gm.height, self.gm.width):
            self._costmap = CostMap.zeros(self.gm.height, self.gm.width)
            self.gm.set_costmap(self._costmap)
        self._CostmapEditor(self, self.gm, self._costmap, on_apply=self._apply_costmap)

    def _apply_costmap(self, cm: CostMap):
        self._costmap = cm.copy()
        self.gm.set_costmap(self._costmap)
        self.reset_search(keep_map=True)
        self._redraw_all_images()
        self.label_status.config(text="Custom costmap applied")
        self._update_info_labels()

    def _play_pause(self):
        if not self.anim_running:
            if self.gm.start is None or self.gm.goal is None:
                self.label_status.config(text="Set Start & Goal first"); return
            self._start_generators()
            self.anim_running = True
            self._ani = FuncAnimation(self.fig, self._tick, interval=self.interval_ms,
                                      cache_frame_data=False, save_count=10000)
            self._tick(None)
            self._ani.event_source.start()
            self.canvas.draw_idle()
            self.label_status.config(text=f"Playing ({self.interval_ms} ms)")
        else:
            self.anim_running = False
            if self._ani is not None:
                self._ani.event_source.stop(); self._ani = None
            self.label_status.config(text="Paused")

    def _step_once(self):
        if self._ani is not None: return
        if not self.gens or all(g is None for g in self.gens):
            if self.gm.start is None or self.gm.goal is None:
                self.label_status.config(text="Set Start & Goal first"); return
            self._start_generators()
        self._tick(None)

    def reset_search(self, keep_map=True):
        if not keep_map:
            self.gm = GridMap(self.gm.height, self.gm.width, diag=self.gm.diag, diag_safety=self.gm.diag_safety)
            self._costmap = CostMap.zeros(self.gm.height, self.gm.width)
            self.gm.set_costmap(self._costmap)
        self.states = []
        self.gens = []
        self.step_counts = [0]*max(1, len(self.axes))
        self.anim_running = False
        if self._ani is not None:
            self._ani.event_source.stop(); self._ani = None
        self._redraw_all_images()
        self.label_status.config(text="Reset")
        self._place_logo()
        self._update_info_labels()

    # logo
    def _fig_px(self):
        return max(1, self.canvas_widget.winfo_width()), max(1, self.canvas_widget.winfo_height())

    def _axes_bboxes_px(self):
        cw, ch = self._fig_px()
        boxes = []
        for ax in self.axes:
            b = ax.get_position()
            boxes.append((int(b.x0*cw), int(b.y0*ch), int(b.x1*cw), int(b.y1*ch)))
        return boxes

    def _load_logo_array(self):
        if not os.path.exists(LOGO_PATH): return None
        arr = mpimg.imread(LOGO_PATH)
        if arr.dtype != np.uint8: arr = (arr * 255).astype(np.uint8)
        return arr

    def _resize_logo_by_width(self, arr, w_target):
        if Image is None:
            return arr
        w_target = int(max(1, min(w_target, LOGO_MAX_W)))
        ar = arr.shape[1] / arr.shape[0]
        h_target = max(1, int(w_target / ar))
        return np.asarray(Image.fromarray(arr).resize((w_target, h_target), Image.LANCZOS))

    def _place_logo(self):
        for im in list(self.fig.images):
            if getattr(im, "_is_logo", False):
                im.remove()

        arr = self._logo_arr if (self._logo_arr is not None) else self._load_logo_array()
        if arr is None:
            self._logo_arr = None
            self.canvas.draw_idle()
            return
        self._logo_arr = arr

        cw, ch = self._fig_px()
        boxes = self._axes_bboxes_px()
        if not boxes:
            self.canvas.draw_idle()
            return

        right_edge  = max(b[2] for b in boxes)
        bottom_edge = min(b[1] for b in boxes)
        right_free  = max(0, cw - right_edge  - LOGO_MARGIN)
        bottom_free = max(0, bottom_edge      - LOGO_MARGIN)
        if right_free <= 0 or bottom_free <= 0:
            self.canvas.draw_idle(); return

        ar = arr.shape[1] / arr.shape[0]
        w_by_h = bottom_free * ar
        w_try = min(LOGO_MAX_W, right_free, w_by_h)
        if w_try <= 1:
            self.canvas.draw_idle(); return

        logo = self._resize_logo_by_width(arr, w_try)
        lh, lw = logo.shape[0], logo.shape[1]
        xo = cw - lw - LOGO_MARGIN
        yo = LOGO_MARGIN
        artist = self.fig.figimage(logo, xo=xo, yo=yo, zorder=5)
        artist._is_logo = True
        self.canvas.draw_idle()

    # statics
    def _compute_stats_from_path(self, path):
        if not path or len(path) < 2:
            return dict(n1=0, n2=0, others=0.0, cost=0.0, path=(len(path) if path else 0))

        n1 = 0
        n2 = 0
        for (r0,c0), (r1,c1) in zip(path[:-1], path[1:]):
            dr, dc = abs(r1-r0), abs(c1-c0)
            if dr+dc == 1: n1 += 1
            elif dr == 1 and dc == 1: n2 += 1
            else:
                n1 += (dr + dc)

        others = 0.0
        if self._costmap is not None and getattr(self._costmap, "cost", None) is not None and \
           self._costmap.cost.shape == (self.gm.height, self.gm.width):
            for (r,c) in path[1:]:
                others += float(self._costmap.cost[r, c])

        total = n1 + (math.sqrt(2) * n2) + others
        return dict(n1=n1, n2=n2, others=others, cost=total, path=len(path))

    def _format_info_line(self, i):
        sqrt2_symbol = "√2"
        state = self.states[i] if (self.states and i < len(self.states)) else None
        path = (state.get("path") if (state and isinstance(state, dict)) else None) or []
        S = self._compute_stats_from_path(path)
        
        # Step in real time
        steps_now = self.step_counts[i] if (self.step_counts and i < len(self.step_counts)) else 0


        segs = []
        # steps → path → cost → detail
        if self.show_steps.get():
            segs.append(f"steps={steps_now}")
        if self.show_path.get():
            segs.append(f"path(hop)={S['path']}")
        if self.show_cost.get():
            segs.append(f"cost={S['cost']:.3f}")
        if self.show_detail.get():
            segs.append(f"1={S['n1']}, {sqrt2_symbol}={S['n2']}, others={S['others']:.3f}")

        return " | ".join(segs) if segs else "—"

    def _build_info_labels(self):
        for t in getattr(self, "info_texts", []):
            try: t.remove()
            except Exception: pass
        self.info_texts = []
        for _ in self.axes:
            t = self.fig.text(0, 0, "",
                              transform=self.fig.transFigure,
                              ha='center', va='top', fontsize=10,
                              bbox=dict(facecolor='white', alpha=0.8, edgecolor='none'), zorder=12)
            self.info_texts.append(t)
        self._place_info_labels()
        self._update_info_labels()

    def _place_info_labels(self):
        if not self.axes or not self.info_texts: return
        cw = max(1, self.canvas_widget.winfo_width())
        ch = max(1, self.canvas_widget.winfo_height())
        for ax, t in zip(self.axes, self.info_texts):
            box = ax.get_position()
            off_px = 12 
            off = off_px / ch
            x = box.x0 + box.width*0.5
            y = max(0.001, box.y0 - off)
            t.set_position((x, y))
        self.canvas.draw_idle()

    def _update_info_labels(self):
        if not self.info_texts:
            return
        for i, t in enumerate(self.info_texts[:len(self.axes)]):
            t.set_text(self._format_info_line(i))
        self._place_info_labels()

    # Costmap Editor 
    class _CostmapEditor(tk.Toplevel):
        def __init__(self, master, gm, costmap: CostMap, on_apply):
            super().__init__(master)
            self.title("Custom Costmap Editor")
            self.gm = gm
            self.cm = costmap.copy()
            self.on_apply = on_apply

            bar = ttk.Frame(self); bar.pack(side=tk.TOP, fill=tk.X)
            ttk.Label(bar, text=f"Size: {gm.width}×{gm.height}").pack(side=tk.LEFT, padx=8)

            ttk.Label(bar, text="Cost:").pack(side=tk.LEFT, padx=(12,4))
            self.mode_value = tk.DoubleVar(value=1.0)
            self._val_label = ttk.Label(bar, width=8, anchor='w', text="1.00")
            self._val_label.pack(side=tk.LEFT)

            self._slider = ttk.Scale(
                bar, from_=0.0, to=5.0, orient='horizontal',
                variable=self.mode_value,
                command=lambda _evt=None: self._on_slider_changed()
            )
            self._slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)

            self.eraser = tk.BooleanVar(value=False)
            def _on_eraser():
                if self.eraser.get():
                    self.mode_value.set(0.0)
                    self._slider.state(['disabled'])
                    self._val_label.config(text="0.00")
                else:
                    self._slider.state(['!disabled'])
                    self._on_slider_changed()
            ttk.Checkbutton(bar, text="Eraser(=0)", variable=self.eraser, command=_on_eraser).pack(side=tk.LEFT, padx=10)

            ttk.Button(bar, text="Apply",  command=self._apply).pack(side=tk.RIGHT, padx=6)
            ttk.Button(bar, text="Cancel", command=self.destroy).pack(side=tk.RIGHT, padx=2)
            ttk.Button(bar, text="Clear All", command=lambda:(self.cm.clear(), self._redraw())).pack(side=tk.RIGHT, padx=2)

            self.canvas = tk.Canvas(self, width=100, height=800, bg="#ffffff", highlightthickness=0)
            self.canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
            self.canvas.bind("<Configure>", self._on_resize)
            self.canvas.bind("<Button-1>", self._on_down)
            self.canvas.bind("<B1-Motion>", self._on_drag)
            self.canvas.bind("<ButtonRelease-1>", self._on_up)

            self.cell_px = 24; self.margin = 1
            self.ox = 0; self.oy = 0
            self._dragging = False; self._last_rc = None

            self._redraw()

        def _on_resize(self, ev):
            H, W = self.gm.height, self.gm.width
            grid_w = W*(self.cell_px+self.margin)+self.margin
            grid_h = H*(self.cell_px+self.margin)+self.margin
            cw = self.canvas.winfo_width(); ch = self.canvas.winfo_height()
            self.ox = max(0, (cw-grid_w)//2); self.oy = max(0, (ch-grid_h)//2)
            self._redraw()

        def _rc_from_xy(self, x, y):
            gx = x - self.ox - self.margin
            gy = y - self.oy - self.margin
            if gx < 0 or gy < 0: return None
            cell = self.cell_px + self.margin
            c = gx // cell
            r = gy // cell
            H, W = self.gm.height, self.gm.width
            if c >= W or r >= H: return None
            return int(r), int(c)

        def _redraw(self):
            self.canvas.delete("all")
            H, W = self.gm.height, self.gm.width

            # background
            for r in range(H):
                for c in range(W):
                    v = self.gm.grid[r, c]
                    fill = "#1e1e1e" if v == 1 else "#f5f5f5"
                    sx = self.ox + self.margin + c*(self.cell_px+self.margin)
                    sy = self.oy + self.margin + r*(self.cell_px+self.margin)
                    ex = sx + self.cell_px; ey = sy + self.cell_px
                    self.canvas.create_rectangle(sx, sy, ex, ey, fill=fill, outline="")

            # start/goal
            if self.gm.start is not None:
                rs, cs = self.gm.start
                sx = self.ox + self.margin + cs*(self.cell_px+self.margin)
                sy = self.oy + self.margin + rs*(self.cell_px+self.margin)
                ex = sx + self.cell_px; ey = sy + self.cell_px
                self.canvas.create_rectangle(sx, sy, ex, ey, fill="#32cd32", outline="")
            if self.gm.goal is not None:
                rg, cg = self.gm.goal
                sx = self.ox + self.margin + cg*(self.cell_px+self.margin)
                sy = self.oy + self.margin + rg*(self.cell_px+self.margin)
                ex = sx + self.cell_px; ey = sy + self.cell_px
                self.canvas.create_rectangle(sx, sy, ex, ey, fill="#dc143c", outline="")

            # cost overlay
            def _hex_from_value(v: float):
                MAXV = 5.0
                t = max(0.0, min(1.0, v / MAXV)) ** 0.6
                def hex_to_rgb(h): h=h.lstrip('#'); return tuple(int(h[i:i+2],16) for i in (0,2,4))
                def rgb_to_hex(r,g,b): return '#%02x%02x%02x' % (int(r),int(g),int(b))
                c0 = hex_to_rgb("#EEE6F7"); c1 = hex_to_rgb("#6A1B9A")
                r = c0[0] + (c1[0]-c0[0]) * t
                g = c0[1] + (c1[1]-c0[1]) * t
                b = c0[2] + (c1[2]-c0[2]) * t
                return rgb_to_hex(r, g, b)

            for r in range(H):
                for c in range(W):
                    if self.gm.grid[r, c] == 1:
                        continue
                    v = float(self.cm.value_at(r, c))
                    if v <= 1e-9:
                        continue
                    sx = self.ox + self.margin + c*(self.cell_px+self.margin)
                    sy = self.oy + self.margin + r*(self.cell_px+self.margin)
                    ex = sx + self.cell_px; ey = sy + self.cell_px
                    self.canvas.create_rectangle(sx, sy, ex, ey, fill=_hex_from_value(v), outline="")

            # grid line
            for x in range(0, W*(self.cell_px+self.margin)+1, self.cell_px+self.margin):
                xx = self.ox + x
                self.canvas.create_line(xx, self.oy, xx, self.oy + H*(self.cell_px+self.margin), fill="#c8c8c8", width=1)
            for y in range(0, H*(self.cell_px+self.margin)+1, self.cell_px+self.margin):
                yy = self.oy + y
                self.canvas.create_line(self.ox, yy, self.ox + W*(self.cell_px+self.margin), yy, fill="#c8c8c8", width=1)

        def _on_down(self, ev):
            rc = self._rc_from_xy(ev.x, ev.y)
            if not rc: return
            self._dragging = True; self._last_rc = rc
            self._apply_one(rc); self._redraw()

        def _on_drag(self, ev):
            if not self._dragging: return
            rc = self._rc_from_xy(ev.x, ev.y)
            if not rc or rc == self._last_rc: return
            r0,c0 = self._last_rc; r1,c1 = rc
            dr, dc = abs(r1-r0), abs(c1-c0)
            sr = 1 if r0 < r1 else -1; sc = 1 if c0 < c1 else -1
            err = dr - dc; r, c = r0, c0
            while True:
                self._apply_one((r,c))
                if r==r1 and c==c1: break
                e2 = 2*err
                if e2 > -dc: err -= dc; r += sr
                if e2 <  dr: err += dr; c += sc
            self._last_rc = rc; self._redraw()

        def _on_up(self, ev):
            self._dragging = False; self._last_rc = None

        def _on_slider_changed(self):
            v = float(self.mode_value.get())
            self._val_label.config(text=f"{v:.2f}")

        def _apply_one(self, rc):
            r, c = rc
            if self.gm.grid[r, c] == 1:
                return
            v = 0.0 if self.eraser.get() else float(self.mode_value.get())
            self.cm.set_value(r, c, v)

        def _apply(self):
            if callable(self.on_apply): self.on_apply(self.cm)
            self.destroy()

def main():
    app = CompareApp(H=5, W=5)
    app.mainloop()

if __name__ == "__main__":
    main()
