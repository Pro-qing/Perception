#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import math

# 确保脚本目录在 Python 路径中 (rosrun 兼容)
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QTabWidget, QLabel, QPushButton, QFormLayout, 
                             QSpinBox, QDoubleSpinBox, QCheckBox, QScrollArea, 
                             QMessageBox, QTextEdit, QLineEdit, QGroupBox, QGridLayout, QComboBox)
from PyQt5.QtGui import QPainter, QColor, QPen, QPolygonF, QFont
from PyQt5.QtCore import Qt, QPointF, pyqtSignal, QObject

from param_path import *

# 使用 ruamel.yaml 保留原文件的所有注释和缩进格式
from ruamel.yaml import YAML
from ruamel.yaml.compat import StringIO
from ruamel.yaml.comments import CommentedMap, CommentedSeq

# ================= ROS 相关导入 =================
HAS_ROS = False
try:
    import rospy
    import std_msgs.msg
    from sensor_msgs.msg import PointCloud2, LaserScan
    import sensor_msgs.point_cloud2 as pc2
    HAS_ROS = True
except ImportError:
    print("未检测到 rospy 或 sensor_msgs，点云显示功能将被禁用。")

# =====================================================================
# 参数显示映射表
# =====================================================================
SAFE_BASE_PARAMS = {
    "max_longitudinal_scale": "前后方向最大缩放比例",
    "min_longitudinal_scale": "前后方向最小缩放比例",
    "max_lateral_scale": "左右方向最大缩放比例",
    "min_lateral_scale": "左右方向最小缩放比例",
    "longitudinal_sensitivity": "纵向灵敏度 (前后方向)",
    "lateral_sensitivity": "横向灵敏度 (左右方向)",
    "reference_speed": "参考速度 (m/s)"
}

SAFE_BOX_TYPES = {
    "exigencyrect_": "急停框 (前进)",
    "reverse_exigencyrect_": "急停框 (后退)",
    "slowrect_": "减速框 (前进)",
    "reverse_slowrect_": "减速框 (后退)"
}

CALIB_GROUPS = {
    "main":   {"name": "主雷达 (预留)", "raw_topic": "", "calib_topic": ""},
    "top":    {"name": "补盲雷达", "raw_topic": "/points_mid", "calib_topic": "/points_mid_calibration"},
    "left":   {"name": "左前单线", "raw_topic": "/scan_left", "calib_topic": "/points_left_calibration"},
    "right":  {"name": "右前单线", "raw_topic": "/scan_right", "calib_topic": "/points_right_calibration"},
    "bleft":  {"name": "左后单线", "raw_topic": "/scan_bleft", "calib_topic": "/points_bleft_calibration"},
    "bright": {"name": "右后单线", "raw_topic": "/scan_bright", "calib_topic": "/points_bright_calibration"}
}

def safe_float(val, default=0.0):
    if val is None: return default
    try: return float(val)
    except (ValueError, TypeError): return default

def create_flow_point(x, y):
    pt = CommentedMap()
    pt['x'] = round(x, 2); pt['y'] = round(y, 2)
    pt.fa.set_flow_style()
    return pt

def apply_tf_3d(points_3d, tx, ty, tz, yaw, pitch, roll):
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    R11, R12, R13 = cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr
    R21, R22, R23 = sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr
    R31, R32, R33 = -sp, cp*sr, cp*cr
    res = []
    for x, y, z in points_3d:
        res.append((R11*x + R12*y + R13*z + tx, R21*x + R22*y + R23*z + ty, R31*x + R32*y + R33*z + tz))
    return res

# =====================================================================
# UI 样式库
# =====================================================================
GLOBAL_STYLE = """
QWidget { font-family: "Segoe UI", Arial, sans-serif; font-size: 14px; color: #333; background-color: #F5F7FA; }
QTabWidget::pane { border: 1px solid #E0E0E0; background-color: #FFFFFF; border-radius: 8px; margin-top: -1px; }
QTabBar::tab { background-color: #E0E0E0; color: #666; border-top-left-radius: 6px; border-top-right-radius: 6px; padding: 10px 20px; margin-right: 2px; font-weight: bold; }
QTabBar::tab:selected { background-color: #FFFFFF; color: #1976D2; border: 1px solid #E0E0E0; border-bottom: 2px solid #1976D2; }
QGroupBox { background-color: #FFFFFF; border: 1px solid #D6D6D6; border-radius: 6px; margin-top: 18px; padding-top: 15px; }
QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; left: 15px; padding: 0 5px; color: #1976D2; font-weight: bold; font-size: 15px; }
QPushButton { border-radius: 5px; padding: 8px 15px; font-weight: bold; border: none; }
QPushButton:hover { opacity: 0.8; }
QPushButton:pressed { background-color: rgba(0,0,0,0.1); }
QDoubleSpinBox, QSpinBox, QLineEdit, QComboBox { border: 1px solid #BDBDBD; border-radius: 4px; padding: 5px; background-color: #FAFAFA; }
QDoubleSpinBox:focus, QSpinBox:focus, QLineEdit:focus, QComboBox:focus { border: 1px solid #1976D2; background-color: #FFFFFF; }
QScrollArea { border: none; background-color: transparent; }
QScrollArea > QWidget > QWidget { background-color: transparent; }
"""

# =====================================================================
# ROS 监听与发布
# =====================================================================
class ROSListener(QObject):
    ref_cloud_updated = pyqtSignal(list)
    sensor_data_updated = pyqtSignal(str, list)

    def __init__(self):
        super().__init__()
        self.current_tfs = {k: (0,0,0,0,0,0) for k in CALIB_GROUPS.keys()}
        self.pubs = {}
        if not HAS_ROS: return

        rospy.Subscriber("/points_16", PointCloud2, self.ref_callback, queue_size=1)
        for key, config in CALIB_GROUPS.items():
            if not config['raw_topic']: continue
            self.pubs[key] = rospy.Publisher(config['calib_topic'], PointCloud2, queue_size=1)
            if "scan" in config['raw_topic']:
                rospy.Subscriber(config['raw_topic'], LaserScan, lambda msg, k=key: self.scan_callback(k, msg), queue_size=1)
            else:
                rospy.Subscriber(config['raw_topic'], PointCloud2, lambda msg, k=key: self.pc2_callback(k, msg), queue_size=1)

    def update_tf(self, key, tf_tuple): self.current_tfs[key] = tf_tuple

    def publish_transformed_cloud(self, key, points_3d, stamp):
        if key not in self.pubs: return
        header = std_msgs.msg.Header()
        header.stamp = stamp; header.frame_id = "velodyne"
        cloud_msg = pc2.create_cloud_xyz32(header, points_3d)
        self.pubs[key].publish(cloud_msg)

    def ref_callback(self, msg):
        pts = []
        try:
            for i, p in enumerate(pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)):
                if i % 4 == 0 and -20 < p[0] < 20 and -20 < p[1] < 20: pts.append((p[0], p[1]))
            self.ref_cloud_updated.emit(pts)
        except: pass

    def pc2_callback(self, key, msg):
        pts_3d = []
        try:
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                if -15.0 < p[0] < 15.0 and -15.0 < p[1] < 15.0: pts_3d.append((p[0], p[1], p[2]))
            t3d = apply_tf_3d(pts_3d, *self.current_tfs[key])
            self.publish_transformed_cloud(key, t3d, msg.header.stamp)
            ui_pts = [(p[0], p[1]) for i, p in enumerate(t3d) if i % 3 == 0]
            self.sensor_data_updated.emit(key, ui_pts)
        except: pass

    def scan_callback(self, key, msg):
        pts_3d = []
        try:
            angle = msg.angle_min
            for r in msg.ranges:
                if msg.range_min < r < msg.range_max and r < 15.0:
                    pts_3d.append((r * math.cos(angle), r * math.sin(angle), 0.0))
                angle += msg.angle_increment
            t3d = apply_tf_3d(pts_3d, *self.current_tfs[key])
            self.publish_transformed_cloud(key, t3d, msg.header.stamp)
            ui_pts = [(p[0], p[1]) for p in t3d]
            self.sensor_data_updated.emit(key, ui_pts)
        except: pass

def draw_velodyne_origin(painter, cx, cy):
    painter.setPen(QPen(QColor(255, 80, 80, 150), 1, Qt.DashLine))
    painter.drawLine(int(cx), 0, int(cx), int(cy*2)) 
    painter.setPen(QPen(QColor(80, 255, 80, 150), 1, Qt.DashLine))
    painter.drawLine(0, int(cy), int(cx*2), int(cy)) 

    painter.setPen(QPen(QColor(200, 200, 200))); painter.setFont(QFont("Microsoft YaHei", 8))
    painter.drawText(int(cx) + 5, 15, "X(前)"); painter.drawText(5, int(cy) - 5, "Y(左)")

    painter.setBrush(QColor(255, 255, 255, 50))
    painter.setPen(Qt.NoPen)
    painter.drawEllipse(QPointF(cx, cy), 10, 10)
    painter.setBrush(QColor(255, 255, 255))
    painter.drawEllipse(QPointF(cx, cy), 4, 4)
    painter.setPen(QPen(QColor(255, 255, 255)))
    painter.setFont(QFont("Arial", 9, QFont.Bold))
    painter.drawText(int(cx) + 12, int(cy) - 12, "velodyne")

# =====================================================================
# 画板: 车辆轮廓与安全框
# =====================================================================
class VehiclePreviewWidget(QWidget):
    def __init__(self, color_r=0, color_g=255, color_b=255):
        super().__init__()
        self.points, self.ref_points, self.cloud_points = [], [], []
        self.setMinimumSize(320, 320)
        self.fill_color = QColor(color_r, color_g, color_b, 60)
        self.line_color = QColor(color_r, color_g, color_b)

    def set_points(self, points): self.points = points; self.update()
    def set_reference_points(self, points): self.ref_points = points; self.update()
    def set_cloud_points(self, points): self.cloud_points = points; self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        try:
            painter.setRenderHint(QPainter.Antialiasing)
            painter.fillRect(self.rect(), QColor("#121212"))
            w, h = self.width(), self.height(); cx, cy = w / 2, h / 2

            max_range = 1.0 
            all_pts = (self.points if isinstance(self.points, list) else []) + (self.ref_points if isinstance(self.ref_points, list) else [])
            for p in all_pts:
                if isinstance(p, dict):
                    max_range = max(max_range, abs(safe_float(p.get('x'))), abs(safe_float(p.get('y'))))
            scale = (min(w, h) / 2.0) / (max_range * 1.25) if max_range > 0 else 1.0

            if self.cloud_points:
                painter.setPen(QPen(QColor(255, 255, 255, 140), 2))
                cloud_poly = QPolygonF()
                for cx_pt, cy_pt in self.cloud_points: cloud_poly.append(QPointF(cx - cy_pt * scale, cy - cx_pt * scale))
                painter.drawPoints(cloud_poly)

            draw_velodyne_origin(painter, cx, cy)

            def build_poly(pt_list):
                poly = QPolygonF(); valid = []
                for i, p in enumerate(pt_list):
                    if isinstance(p, dict) and 'x' in p and 'y' in p:
                        rx, ry = safe_float(p['x']), safe_float(p['y'])
                        pt = QPointF(cx - ry * scale, cy - rx * scale)
                        poly.append(pt); valid.append((pt, i))
                return poly, valid

            if self.ref_points:
                ref_poly, _ = build_poly(self.ref_points)
                painter.setPen(QPen(QColor(0, 255, 255, 150), 2)); painter.setBrush(QColor(0, 255, 255, 30))
                painter.drawPolygon(ref_poly)

            if self.points and isinstance(self.points, list):
                main_poly, valid_pts = build_poly(self.points)
                painter.setPen(QPen(self.line_color, 2)); painter.setBrush(self.fill_color)
                painter.drawPolygon(main_poly)
                painter.setFont(QFont("Arial", 9, QFont.Bold))
                for pt, idx in valid_pts:
                    painter.setBrush(QColor(255, 255, 0)); painter.setPen(Qt.NoPen)
                    painter.drawEllipse(pt, 5, 5)
                    painter.setPen(QPen(QColor(255, 255, 255))); painter.drawText(int(pt.x()) + 8, int(pt.y()) - 8, f"P{idx+1}")
        except: pass

# =====================================================================
# 专属画板: 雷达标定实时预览
# =====================================================================
class CalibrationPreviewWidget(QWidget):
    def __init__(self, title):
        super().__init__()
        self.title = title
        self.ref_cloud = []; self.sensor_cloud = []; self.tf_params = (0,0,0,0,0,0) 
        self.setMinimumSize(350, 350)

    def set_ref_cloud(self, pts_2d): self.ref_cloud = pts_2d; self.update()
    def set_sensor_cloud(self, pts_2d): self.sensor_cloud = pts_2d; self.update()
    def update_tf(self, tx, ty, tz, yaw, pitch, roll): self.tf_params = (tx, ty, tz, yaw, pitch, roll); self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        try:
            painter.setRenderHint(QPainter.Antialiasing)
            painter.fillRect(self.rect(), QColor("#121212"))
            w, h = self.width(), self.height(); cx, cy = w / 2, h / 2
            scale = min(w, h) / 30.0 

            draw_velodyne_origin(painter, cx, cy)

            if self.ref_cloud:
                painter.setPen(QPen(QColor(255, 255, 255, 80), 2))
                poly = QPolygonF()
                for px, py in self.ref_cloud: poly.append(QPointF(cx - py * scale, cy - px * scale))
                painter.drawPoints(poly)

            if self.sensor_cloud:
                painter.setPen(QPen(QColor(0, 255, 0, 200), 3))
                t_poly = QPolygonF()
                for px, py in self.sensor_cloud: t_poly.append(QPointF(cx - py * scale, cy - px * scale))
                painter.drawPoints(t_poly)

            painter.setPen(QPen(QColor(255, 255, 0))); painter.setFont(QFont("Arial", 10, QFont.Bold))
            painter.drawText(10, 20, self.title)
        except: pass

# =====================================================================
# 统一动态多边形编辑器
# =====================================================================
class PolygonEditor(QWidget):
    def __init__(self, init_points, raw_key, yaml_data_source, hidden_text_edit, preview_widget):
        super().__init__()
        self.points = [{'x': safe_float(p.get('x')), 'y': safe_float(p.get('y'))} for p in init_points]
        self.raw_key = raw_key; self.yaml_data_source = yaml_data_source
        self.hidden_text_edit = hidden_text_edit; self.preview = preview_widget
        self.layout = QVBoxLayout(self); self.layout.setContentsMargins(0,0,0,0)
        
        g = QGroupBox("整体外扩/缩小"); gl = QGridLayout(g)
        self.sf, self.sb, self.sl, self.sr = [QDoubleSpinBox() for _ in range(4)]
        for s in (self.sf, self.sb, self.sl, self.sr): s.setRange(-10,10); s.setSingleStep(0.05)
        gl.addWidget(QLabel("前:"),0,0); gl.addWidget(self.sf,0,1); gl.addWidget(QLabel("后:"),0,2); gl.addWidget(self.sb,0,3)
        gl.addWidget(QLabel("左:"),1,0); gl.addWidget(self.sl,1,1); gl.addWidget(QLabel("右:"),1,2); gl.addWidget(self.sr,1,3)
        btn = QPushButton("应用"); btn.setStyleSheet("background-color: #673AB7; color: white;"); btn.clicked.connect(self.apply_bulk)
        gl.addWidget(btn,0,4,2,1); self.layout.addWidget(g)

        self.scroll = QScrollArea(); self.scroll.setWidgetResizable(True)
        self.p_cont = QWidget(); self.p_lay = QGridLayout(self.p_cont)
        self.p_lay.setContentsMargins(5, 5, 5, 5); self.p_lay.setHorizontalSpacing(10)
        self.scroll.setWidget(self.p_cont); self.layout.addWidget(self.scroll)
        
        self.add_btn = QPushButton("+ 添加点位"); self.add_btn.setStyleSheet("background-color: #2196F3; color: white;")
        self.add_btn.clicked.connect(self.add_point); self.layout.addWidget(self.add_btn)
        
        self.spin_pairs = []; self.render_points()

    def apply_bulk(self):
        df, db, dl, dr = self.sf.value(), self.sb.value(), self.sl.value(), self.sr.value()
        for sx, sy in self.spin_pairs:
            x, y = sx.value(), sy.value()
            if x>0.01: x+=df 
            elif x<-0.01: x-=db 
            if y>0.01: y+=dl 
            elif y<-0.01: y-=dr
            sx.setValue(x); sy.setValue(y)
        self.sf.setValue(0); self.sb.setValue(0); self.sl.setValue(0); self.sr.setValue(0)

    def render_points(self):
        for i in reversed(range(self.p_lay.count())): 
            w = self.p_lay.itemAt(i).widget(); 
            if w: w.setParent(None)
        self.spin_pairs.clear()
        
        for i, pt in enumerate(self.points):
            lbl = QLabel(f"P{i+1}:"); lbl.setStyleSheet("color: #D32F2F; font-weight: bold; font-size: 15px;")
            sx = QDoubleSpinBox(); sx.setRange(-50.0, 50.0); sx.setDecimals(2); sx.setValue(pt['x']); sx.setSingleStep(0.05)
            sy = QDoubleSpinBox(); sy.setRange(-50.0, 50.0); sy.setDecimals(2); sy.setValue(pt['y']); sy.setSingleStep(0.05)
            db = QPushButton("X"); db.setFixedSize(30,30)
            db.setStyleSheet("background-color:#E53935;color:white;border-radius:15px;"); db.clicked.connect(lambda _, idx=i: self.rm(idx))
            sx.valueChanged.connect(self.up); sy.valueChanged.connect(self.up)
            
            self.p_lay.addWidget(lbl, i, 0); self.p_lay.addWidget(QLabel("X:"), i, 1); self.p_lay.addWidget(sx, i, 2)
            self.p_lay.addWidget(QLabel("Y:"), i, 3); self.p_lay.addWidget(sy, i, 4); self.p_lay.addWidget(db, i, 5)
            self.spin_pairs.append((sx, sy))
        self.out()

    def add_point(self): self.points.append(self.points[-1].copy() if self.points else {'x':0.0,'y':0.0}); self.render_points()
    def rm(self, idx): 
        if len(self.points)>1: self.points.pop(idx); self.render_points()
        else: QMessageBox.warning(self, "警告", "至少需要保留一个点！")
    def up(self):
        for i, (sx, sy) in enumerate(self.spin_pairs): self.points[i]['x'], self.points[i]['y'] = round(sx.value(),2), round(sy.value(),2)
        self.out()
    def out(self):
        self.preview.set_points(self.points)
        pts = [create_flow_point(p['x'], p['y']) for p in self.points]
        self.yaml_data_source[self.raw_key] = pts
        b = StringIO(); YAML().dump(pts, b); self.hidden_text_edit.setPlainText(b.getvalue())

# =====================================================================
# 主界面
# =====================================================================
class ConfigEditorGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.safe_yaml_path = SAFE_OBSTACLE_YAML
        self.calib_yaml_path = LIDAR_CALIBRATION_YAML
        
        # ---- perception pipeline 参数文件路径 (从 param_path.py 导入) ----
        self.downsample_yaml_path = DOWNSAMPLE_YAML
        self.ground_yaml_path = GROUND_YAML
        self.charge_yaml_path = CHARGE_YAML
        self.euclidean_yaml_path = EUCLIDEAN_CLUSTER_YAML
        self.shape_yaml_path = SHAPE_YAML
        
        self.yaml_parser = YAML(); self.yaml_parser.preserve_quotes = True
        self.safe_yaml_data = None; self.calib_yaml_data = None

        # ---- perception pipeline 数据 ----
        self.downsample_yaml_data = None; self.downsample_widgets = {}
        self.ground_yaml_data = None; self.ground_widgets = {}
        self.charge_yaml_data = None; self.charge_widgets = {}
        self.euclidean_yaml_data = None; self.euclidean_widgets = {}
        self.shape_yaml_data = None; self.shape_widgets = {}

        self.previews_safe = []
        self.previews_downsample_poly = []
        self.calib_previews = {} 

        if HAS_ROS:
            try:
                rospy.init_node('config_editor_node', anonymous=True, disable_signals=True)
                self.ros_listener = ROSListener()
                self.ros_listener.ref_cloud_updated.connect(self.dispatch_ref_cloud)
                self.ros_listener.sensor_data_updated.connect(self.dispatch_sensor_cloud)
            except Exception as e: print("ROS 节点启动失败:", e)

        self.init_ui()

    def dispatch_ref_cloud(self, points):
        for p in self.previews_safe + self.previews_downsample_poly:
            try: p.set_cloud_points(points)
            except: pass
        for p in self.calib_previews.values():
            try: p.set_ref_cloud(points)
            except: pass

    def dispatch_sensor_cloud(self, key, points_2d):
        if key in self.calib_previews:
            try: self.calib_previews[key].set_sensor_cloud(points_2d)
            except: pass

    def init_ui(self):
        self.setWindowTitle("Lidar Filtering 可视化调参工具 (属性点升级版)")
        self.resize(1300, 950)
        self.setStyleSheet(GLOBAL_STYLE)
        
        layout = QVBoxLayout(self)
        self.tabs = QTabWidget()
        
        self.tab_safe  = QWidget(); self.setup_safe_tab();  self.tabs.addTab(self.tab_safe, "安全检测框设置")
        self.tab_calib = QWidget(); self.setup_calib_tab(); self.tabs.addTab(self.tab_calib, "雷达标定配置")

        # ---- perception pipeline 参数编辑标签页 ----
        self.tab_downsample = QWidget(); self.setup_generic_yaml_tab(
            self.tab_downsample, "降采样参数", self.downsample_yaml_path,
            "downsample_yaml_data", "downsample_widgets",
            self.load_downsample_yaml, self.save_downsample_yaml)
        self.tabs.addTab(self.tab_downsample, "降采样参数")

        self.tab_ground = QWidget(); self.setup_generic_yaml_tab(
            self.tab_ground, "地面分割参数", self.ground_yaml_path,
            "ground_yaml_data", "ground_widgets",
            self.load_ground_yaml, self.save_ground_yaml)
        self.tabs.addTab(self.tab_ground, "地面分割")

        self.tab_charge = QWidget(); self.setup_generic_yaml_tab(
            self.tab_charge, "充电桩参数", self.charge_yaml_path,
            "charge_yaml_data", "charge_widgets",
            self.load_charge_yaml, self.save_charge_yaml)
        self.tabs.addTab(self.tab_charge, "充电桩过滤")

        self.tab_euclidean = QWidget(); self.setup_generic_yaml_tab(
            self.tab_euclidean, "欧式聚类参数", self.euclidean_yaml_path,
            "euclidean_yaml_data", "euclidean_widgets",
            self.load_euclidean_yaml, self.save_euclidean_yaml)
        self.tabs.addTab(self.tab_euclidean, "欧式聚类")

        self.tab_shape = QWidget(); self.setup_generic_yaml_tab(
            self.tab_shape, "形状估计参数", self.shape_yaml_path,
            "shape_yaml_data", "shape_widgets",
            self.load_shape_yaml, self.save_shape_yaml)
        self.tabs.addTab(self.tab_shape, "形状估计")
        
        layout.addWidget(self.tabs)

    def save_generic_yaml(self, widgets_dict, data_source, path):  # kept for safe_obstacle
        for k, w in widgets_dict.items():
            if isinstance(w, QCheckBox): data_source[k] = w.isChecked()
            elif isinstance(w, QDoubleSpinBox): data_source[k] = w.value()
            elif isinstance(w, QTextEdit):
                try:
                    text_content = w.toPlainText().strip()
                    # 特殊处理 enable 和 mode_mapping - 直接解析值
                    if k in ["enable", "mode_mapping"]:
                        # 如果是列表或字典形式，直接解析
                        if text_content.startswith("[") or text_content.startswith("{"):
                            data_source[k] = YAML().load(text_content)
                        else:
                            return False
                    else:
                        # 其他 QTextEdit 直接作为 YAML 内容解析
                        data_source[k] = YAML().load(text_content)
                except Exception as e:
                    print(f"解析 {k} 失败: {e}")
                    return False
        with open(path, 'w', encoding='utf-8') as f: self.yaml_parser.dump(data_source, f)
        return True

    # ========================== 安全框 ==========================
    def setup_safe_tab(self):
        l = QVBoxLayout(self.tab_safe)
        h = QHBoxLayout()
        br = QPushButton("🔄 重新读取")
        br.setStyleSheet("background-color: #757575; color: white;")
        br.clicked.connect(self.load_safe_yaml)
        bs = QPushButton("💾 保存安全框修改")
        bs.setStyleSheet("background-color: #FF9800; color: white;")
        bs.clicked.connect(self.save_safe_yaml)
        h.addWidget(br); h.addWidget(bs); l.addLayout(h)

        add_group = QGroupBox("道路属性点管理")
        add_layout = QHBoxLayout(add_group)
        self.new_attr_combo = QComboBox()
        self.new_attr_combo.addItems([str(i) for i in range(1, 21)])
        btn_add_attr = QPushButton("➕ 生成该属性安全套件")
        btn_add_attr.setStyleSheet("background-color: #2196F3; color: white;")
        btn_add_attr.clicked.connect(self.add_road_attribute)
        add_layout.addWidget(QLabel("选择道路属性点:")); add_layout.addWidget(self.new_attr_combo)
        add_layout.addWidget(btn_add_attr); add_layout.addStretch()
        l.addWidget(add_group)

        self.scroll_s = QScrollArea(); self.scroll_s.setWidgetResizable(True)
        self.safe_form_widget = QWidget(); self.safe_form_layout = QVBoxLayout(self.safe_form_widget)
        self.scroll_s.setWidget(self.safe_form_widget); l.addWidget(self.scroll_s)
        
        self.load_safe_yaml()

    def clean_yaml_file(self, path):
        """ 清理 yaml 文件中因为删除导致的多余空行 """
        try:
            with open(path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
            clean_lines = [line for line in lines if line.strip() != ""]
            with open(path, 'w', encoding='utf-8') as f:
                f.writelines(clean_lines)
        except: pass

    def load_safe_yaml(self):
        if not os.path.exists(self.safe_yaml_path): return
        with open(self.safe_yaml_path, 'r', encoding='utf-8') as f: 
            self.safe_yaml_data = self.yaml_parser.load(f)
            
        for i in reversed(range(self.safe_form_layout.count())):
            w = self.safe_form_layout.itemAt(i).widget()
            if w: w.setParent(None)
            
        self.previews_safe = []; self.safe_widgets = {}
        ref = []
        
        # 读取全局 enable 和 mode_mapping
        global_enable = self.safe_yaml_data.get('enable', [])
        global_mode_mapping = self.safe_yaml_data.get('mode_mapping', {})
        
        base_group = QGroupBox("安全框通用基础参数")
        base_layout = QFormLayout(base_group)
        for k, lbl in SAFE_BASE_PARAMS.items():
            if k not in self.safe_yaml_data: continue
            val = self.safe_yaml_data[k]
            label = QLabel(lbl); label.setStyleSheet("font-weight: bold; margin-top: 5px;")
            w = QDoubleSpinBox(); w.setRange(-100, 100); w.setDecimals(2)
            try:
                w.setValue(float(val))
            except:
                w.setValue(0.0)
            self.safe_widgets[k] = w
            base_layout.addRow(label, w)
        self.safe_form_layout.addWidget(base_group)

        attributes_map = {}
        for key in list(self.safe_yaml_data.keys()):
            if not isinstance(self.safe_yaml_data[key], list): continue
            attr_suffix = None
            for prefix in SAFE_BOX_TYPES.keys():
                if key.startswith(prefix):
                    attr_suffix = key.replace(prefix, "")
                    break
            if attr_suffix:
                if attr_suffix not in attributes_map: attributes_map[attr_suffix] = []
                attributes_map[attr_suffix].append(key)

        for suffix, keys in sorted(attributes_map.items(), key=lambda x: int(x[0]) if x[0].isdigit() else 0):
            group = QGroupBox(f"📌 道路属性点套件: [ {suffix} ]")
            group.setStyleSheet("QGroupBox { border: 2px solid #1976D2; border-radius: 8px; margin-top: 20px; } QGroupBox::title { color: #1976D2; font-size: 16px; }")
            g_layout = QVBoxLayout(group)
            
            # ============ 套件启用控制区 ============
            control_layout = QHBoxLayout()
            enable_cb = QCheckBox(f"启用套件 [{suffix}]")
            enable_cb.setStyleSheet("font-weight: bold; color: #1976D2;")
            
            # 检查套件是否在 enable 列表中（需要规范化类型比较）
            is_enabled = False
            suffix_int = int(suffix) if suffix.isdigit() else suffix
            for enabled_item in global_enable:
                if isinstance(enabled_item, (int, float)):
                    if int(enabled_item) == suffix_int:
                        is_enabled = True
                        break
                else:
                    if str(enabled_item) == str(suffix):
                        is_enabled = True
                        break
            enable_cb.setChecked(is_enabled)
            
            mode_mapping_label = QLabel("模式映射:")
            mode_mapping_edit = QLineEdit()
            mode_mapping_val = global_mode_mapping.get(suffix, [])
            try:
                b = StringIO()
                YAML().dump(mode_mapping_val, b)
                yaml_str = b.getvalue().strip()
                mode_mapping_edit.setText(yaml_str)
            except:
                mode_mapping_edit.setText(str(mode_mapping_val))
            mode_mapping_edit.setMaximumWidth(200)
            
            # 保存这些widget引用供保存时使用
            self.safe_widgets[f"_enable_{suffix}"] = enable_cb
            self.safe_widgets[f"_mode_mapping_{suffix}"] = mode_mapping_edit
            
            control_layout.addWidget(enable_cb)
            control_layout.addWidget(mode_mapping_label)
            control_layout.addWidget(mode_mapping_edit)
            control_layout.addStretch()
            g_layout.addLayout(control_layout)
            
            g_layout.addSpacing(10)
            
            # ============ 删除按钮 ============
            del_btn = QPushButton(f"🗑️ 删除属性套件 [{suffix}]")
            del_btn.setStyleSheet("background-color: #E53935; color: white;")
            del_btn.clicked.connect(lambda chk, s=suffix, ks=keys: self.delete_road_attribute(s, ks))
            g_layout.addWidget(del_btn, alignment=Qt.AlignRight)

            # ============ 多边形编辑标签页 ============
            sub_tabs = QTabWidget()
            sub_tabs.setStyleSheet("QTabBar::tab { background-color: #E3F2FD; color: #1976D2; } QTabBar::tab:selected { background-color: #BBDEFB; font-weight: bold; border-bottom: 2px solid #1976D2; }")

            for prefix, desc in SAFE_BOX_TYPES.items():
                expected_key = f"{prefix}{suffix}"
                if expected_key in keys:
                    val = self.safe_yaml_data[expected_key]
                    page = QWidget(); pl = QHBoxLayout(page)
                    t = QTextEdit(); t.hide(); self.safe_widgets[expected_key] = t
                    pv = VehiclePreviewWidget(255, 150, 0); pv.set_reference_points(ref); self.previews_safe.append(pv)
                    ed = PolygonEditor(val, expected_key, self.safe_yaml_data, t, pv)
                    pl.addWidget(ed, stretch=1); pl.addWidget(pv, stretch=1)
                    sub_tabs.addTab(page, desc)

            g_layout.addWidget(sub_tabs)
            self.safe_form_layout.addWidget(group)

    def add_road_attribute(self):
        attr = self.new_attr_combo.currentText()
        test_key = f"exigencyrect_{attr}"
        if test_key in self.safe_yaml_data:
            return QMessageBox.warning(self, "警告", f"属性套件 '{attr}' 已经存在！")

        default_box = [
            create_flow_point(1.10, -0.30), create_flow_point(1.0, -0.70),
            create_flow_point(0.05, -0.70), create_flow_point(0.05, -0.80),
            create_flow_point(-1.7, -0.80), create_flow_point(-1.7, 0.75),
            create_flow_point(0.05, 0.75), create_flow_point(0.05, 0.66),
            create_flow_point(1.0, 0.66), create_flow_point(1.10, 0.30)
        ]

        for prefix in SAFE_BOX_TYPES.keys():
            new_key = f"{prefix}{attr}"; src_key = f"{prefix}1"
            if src_key in self.safe_yaml_data:
                self.safe_yaml_data[new_key] = [create_flow_point(p['x'], p['y']) for p in self.safe_yaml_data[src_key]]
            else:
                self.safe_yaml_data[new_key] = [create_flow_point(p['x'], p['y']) for p in default_box]

        # 自动添加到 enable 列表中
        if 'enable' not in self.safe_yaml_data:
            enable_list = CommentedSeq()
            enable_list.fa.set_flow_style()
            self.safe_yaml_data['enable'] = enable_list
        
        # 使用整数类型
        attr_num = int(attr) if attr.isdigit() else attr
        if attr_num not in self.safe_yaml_data['enable']:
            self.safe_yaml_data['enable'].append(attr_num)

        # 自动在 mode_mapping 中添加映射
        if 'mode_mapping' not in self.safe_yaml_data:
            self.safe_yaml_data['mode_mapping'] = {}
        if attr not in self.safe_yaml_data['mode_mapping']:
            # 默认模式映射为该套件号本身
            self.safe_yaml_data['mode_mapping'][attr] = [attr_num] if isinstance(attr_num, int) else [attr]

        self.save_safe_yaml()
        self.load_safe_yaml()

    def delete_road_attribute(self, suffix, keys_to_delete):
        reply = QMessageBox.question(self, "确认删除", f"确定要删除道路属性套件 '{suffix}' 吗？", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            # 暴力清除字典并重建，打破 CommentedMap 的幽灵缓存
            new_data = CommentedMap()
            for k, v in self.safe_yaml_data.items():
                if k not in keys_to_delete:
                    new_data[k] = v
            
            # 从 enable 列表中移除该套件
            if 'enable' in new_data:
                suffix_int = int(suffix) if suffix.isdigit() else suffix
                # 过滤掉与 suffix 匹配的项（无论是整数还是浮点数）
                filtered_enable = []
                for item in new_data['enable']:
                    if isinstance(item, (int, float)):
                        if int(item) != suffix_int:
                            filtered_enable.append(item)
                    else:
                        if str(item) != str(suffix):
                            filtered_enable.append(item)
                
                # 保持流式样式
                enable_list = CommentedSeq(filtered_enable)
                enable_list.fa.set_flow_style()
                new_data['enable'] = enable_list
            
            # 从 mode_mapping 中移除该套件
            if 'mode_mapping' in new_data:
                if suffix in new_data['mode_mapping']:
                    del new_data['mode_mapping'][suffix]
            
            self.safe_yaml_data = new_data
            
            with open(self.safe_yaml_path, 'w', encoding='utf-8') as f:
                self.yaml_parser.dump(self.safe_yaml_data, f)
                
            self.clean_yaml_file(self.safe_yaml_path)
            self.load_safe_yaml()

    def save_safe_yaml(self):
        # 先保存基础参数和多边形数据
        for k, w in self.safe_widgets.items():
            if k.startswith("_enable_") or k.startswith("_mode_mapping_"):
                # 这些在下面单独处理
                continue
            elif isinstance(w, QCheckBox):
                self.safe_yaml_data[k] = w.isChecked()
            elif isinstance(w, QDoubleSpinBox):
                self.safe_yaml_data[k] = w.value()
            elif isinstance(w, QTextEdit):
                try:
                    text_content = w.toPlainText().strip()
                    if text_content.startswith("[") or text_content.startswith("{"):
                        self.safe_yaml_data[k] = YAML().load(text_content)
                    else:
                        self.safe_yaml_data[k] = YAML().load(text_content)
                except Exception as e:
                    print(f"解析 {k} 失败: {e}")
                    return
        
        # 处理每个套件的启用状态和模式映射
        new_enable = []
        new_mode_mapping = {}
        
        for k, w in self.safe_widgets.items():
            if k.startswith("_enable_"):
                suffix = k.replace("_enable_", "")
                if w.isChecked():
                    # 使用整数，不要用浮点数
                    if suffix.isdigit():
                        new_enable.append(int(suffix))
                    else:
                        new_enable.append(suffix)
            elif k.startswith("_mode_mapping_"):
                suffix = k.replace("_mode_mapping_", "")
                try:
                    mapping_text = w.text().strip()
                    if mapping_text:
                        mapping_val = YAML().load(mapping_text)
                        new_mode_mapping[suffix] = mapping_val
                except Exception as e:
                    print(f"解析模式映射 [{suffix}] 失败: {e}")
                    return
        
        # 更新全局的 enable 和 mode_mapping（保持流式样式）
        enable_list = CommentedSeq(new_enable)
        enable_list.fa.set_flow_style()
        self.safe_yaml_data['enable'] = enable_list
        self.safe_yaml_data['mode_mapping'] = new_mode_mapping
        
        # 保存到文件
        try:
            with open(self.safe_yaml_path, 'w', encoding='utf-8') as f:
                self.yaml_parser.dump(self.safe_yaml_data, f)
            QMessageBox.information(self, "提示", "安全检测框保存成功！")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"保存失败: {e}")

    # ========================== 3. 雷达标定 ==========================
    def setup_calib_tab(self):
        l = QVBoxLayout(self.tab_calib)
        h = QHBoxLayout(); br = QPushButton("🔄 重新读取标定"); br.clicked.connect(self.load_calib_yaml); br.setStyleSheet("background-color: #757575; color: white;")
        bs = QPushButton("🎯 保存雷达标定"); bs.clicked.connect(self.save_calib_yaml); bs.setStyleSheet("background-color: #E91E63; color: white;")
        h.addWidget(br); h.addWidget(bs); l.addLayout(h)
        self.scroll_calib = QScrollArea(); self.scroll_calib.setWidgetResizable(True)
        self.calib_form_widget = QWidget(); self.calib_form_layout = QVBoxLayout(self.calib_form_widget)
        self.scroll_calib.setWidget(self.calib_form_widget); l.addWidget(self.scroll_calib)
        self.load_calib_yaml()

    def load_calib_yaml(self):
        if not os.path.exists(self.calib_yaml_path): return
        with open(self.calib_yaml_path, 'r', encoding='utf-8') as f: 
            self.calib_yaml_data = self.yaml_parser.load(f)
        for i in reversed(range(self.calib_form_layout.count())):
            w = self.calib_form_layout.itemAt(i).widget()
            if w: w.setParent(None)
            
        self.calib_widgets = {}; self.calib_previews.clear()

        # 支持两种 YAML 结构: calibration/main: {x, y, z, roll, pitch, yaw} 或 tf_calibration/main_x: val
        cal_data = None
        if 'calibration' in self.calib_yaml_data:
            cal_data = self.calib_yaml_data['calibration']
        elif 'tf_calibration' in self.calib_yaml_data:
            cal_data = self.calib_yaml_data['tf_calibration']
        if cal_data is None:
            return

        use_nested = 'calibration' in self.calib_yaml_data and isinstance(cal_data.get('main'), dict)

        for prefix, config in CALIB_GROUPS.items():
            group = QGroupBox(f"{config['name']} -> 发布话题: {config['calib_topic']}")
            hl = QHBoxLayout(group)
            
            gl = QGridLayout()
            params = ['x', 'y', 'z', 'yaw', 'pitch', 'roll']
            spins = {}
            for i, p in enumerate(params):
                if use_nested:
                    sensor_data = cal_data.get(prefix, {})
                    val = sensor_data.get(p, 0.0) if isinstance(sensor_data, dict) else 0.0
                else:
                    val = cal_data.get(f"{prefix}_{p}", 0.0)
                sb = QDoubleSpinBox()
                sb.setRange(-3.14159*2 if p in ['yaw','pitch','roll'] else -10.0, 3.14159*2 if p in ['yaw','pitch','roll'] else 10.0)
                sb.setDecimals(4); sb.setSingleStep(0.01); sb.setValue(float(val) if val is not None else 0.0)
                self.calib_widgets[f"{prefix}_{p}"] = sb; spins[p] = sb
                gl.addWidget(QLabel(f"{p.upper()}:"), i//2, (i%2)*2)
                gl.addWidget(sb, i//2, (i%2)*2 + 1)
            
            w_left = QWidget(); w_left.setLayout(gl); hl.addWidget(w_left, stretch=1)

            if prefix != "main":
                pv = CalibrationPreviewWidget(f"实时校准: {prefix}")
                self.calib_previews[prefix] = pv
                hl.addWidget(pv, stretch=1)
                
                def make_update_func(k, p_spins):
                    def update_tf():
                        tx, ty, tz = p_spins['x'].value(), p_spins['y'].value(), p_spins['z'].value()
                        yw, pt, rl = p_spins['yaw'].value(), p_spins['pitch'].value(), p_spins['roll'].value()
                        if HAS_ROS: self.ros_listener.update_tf(k, (tx, ty, tz, yw, pt, rl))
                    return update_tf
                
                up_func = make_update_func(prefix, spins)
                for s in spins.values(): s.valueChanged.connect(up_func)
                up_func()

            self.calib_form_layout.addWidget(group)

    def save_calib_yaml(self):
        if not self.calib_yaml_data: return
        cal_data = None
        use_nested = False
        if 'calibration' in self.calib_yaml_data:
            cal_data = self.calib_yaml_data['calibration']
            use_nested = isinstance(cal_data.get('main'), dict)
        elif 'tf_calibration' in self.calib_yaml_data:
            cal_data = self.calib_yaml_data['tf_calibration']
        if cal_data is None: return

        for k, w in self.calib_widgets.items():
            prefix, param = k.rsplit('_', 1)
            if use_nested:
                if prefix not in cal_data or not isinstance(cal_data[prefix], dict):
                    continue
                cal_data[prefix][param] = w.value()
            else:
                cal_data[k] = w.value()
        try:
            with open(self.calib_yaml_path, 'w', encoding='utf-8') as f: self.yaml_parser.dump(self.calib_yaml_data, f)
            QMessageBox.information(self, "提示", "雷达标定参数保存成功！")
        except Exception as e: QMessageBox.critical(self, "错误", f"保存失败: {e}")

    # =====================================================================
    # 通用 YAML 参数编辑器 (用于 perception pipeline 参数)
    # =====================================================================
    # 层级参数描述映射表: key = "父key/子key", value = 中文描述
    GENERIC_PARAM_LABELS = {
        # ---- downsample.yaml ----
        "input_topic": "输入话题",
        "output_topic": "输出话题",
        "voxel_grid/leaf_size_x": "体素X尺寸 (米)",
        "voxel_grid/leaf_size_y": "体素Y尺寸 (米)",
        "voxel_grid/leaf_size_z": "体素Z尺寸 (米)",
        "voxel_grid/min_points_per_voxel": "每体素最小点数",
        "voxel_grid/downsample_all_data": "降采样所有字段",
        "body_filter/enable": "车身过滤使能",
        "body_filter/min_z": "车身最小高度 (米)",
        "body_filter/max_z": "车身最大高度 (米)",
        "body_filter/polygon": "车身轮廓多边形",
        "body_filter/publish_marker": "发布车身Marker",
        "body_filter/marker_topic": "Marker话题",
        "body_filter/marker_color_r": "Marker颜色R",
        "body_filter/marker_color_g": "Marker颜色G",
        "body_filter/marker_color_b": "Marker颜色B",
        "body_filter/marker_color_a": "Marker颜色A",
        "crop_box/enable": "裁剪框使能",
        "crop_box/min_x": "裁剪最小X (米)",
        "crop_box/max_x": "裁剪最大X (米)",
        "crop_box/min_y": "裁剪最小Y (米)",
        "crop_box/max_y": "裁剪最大Y (米)",
        "crop_box/min_z": "裁剪最小Z (米)",
        "crop_box/max_z": "裁剪最大Z (米)",
        "crop_box/negative": "裁剪反转",
        "height_filter/enable": "高度过滤使能",
        "height_filter/min_height": "最小高度 (米)",
        "height_filter/max_height": "最大高度 (米)",
        # ---- ground.yaml ----
        "ransac/max_iterations": "RANSAC最大迭代",
        "ransac/distance_threshold": "距离阈值 (米)",
        "ransac/probability": "成功概率",
        "ransac/eps_angle": "法向量夹角 (度)",
        "ransac/optimize_coefficients": "优化模型系数",
        "ransac/use_perpendicular": "垂直平面模型",
        "ground/max_height": "地面最大高度 (米)",
        "ground/min_height": "地面最小高度 (米)",
        "iterative/enable": "迭代拟合使能",
        "iterative/max_iterations": "迭代最大次数",
        "iterative/height_threshold": "高度阈值 (米)",
        "pre_filter/enable": "预过滤使能",
        "pre_filter/min_z": "预过滤最小Z (米)",
        "pre_filter/max_z": "预过滤最大Z (米)",
        # ---- charge.yaml ----
        "charge/enable": "充电桩过滤使能",
        "charge/length": "充电桩长度 (米)",
        "charge/wide": "充电桩宽度 (米)",
        "charge/high": "充电桩高度 (米)",
        "charge/error": "充电桩误差 (米)",
        # ---- euclidean_cluster.yaml ----
        "cluster_tolerance": "聚类容差 (米)",
        "min_cluster_size": "最小簇点数",
        "max_cluster_size": "最大簇点数",
        "kdtree_eps": "KD-Tree搜索精度",
        "obstacle_filter/enable": "障碍物过滤使能",
        "obstacle_filter/min_height": "障碍物最小高度 (米)",
        "obstacle_filter/max_height": "障碍物最大高度 (米)",
        "obstacle_filter/min_width": "障碍物最小宽度 (米)",
        "obstacle_filter/max_width": "障碍物最大宽度 (米)",
        "obstacle_filter/min_length": "障碍物最小长度 (米)",
        "obstacle_filter/max_length": "障碍物最大长度 (米)",
        "obstacle_filter/max_distance": "障碍物最大距离 (米)",
        # ---- shape.yaml ----
        "debug": "调试模式",
        "min_cluster_size_for_obb": "OBB最小簇点数",
        "use_pca": "使用PCA方法",
        "default_label": "默认标签",
        "classification/car/min_length": "轿车-最小长度",
        "classification/car/max_length": "轿车-最大长度",
        "classification/car/min_width": "轿车-最小宽度",
        "classification/car/max_width": "轿车-最大宽度",
        "classification/car/min_height": "轿车-最小高度",
        "classification/car/max_height": "轿车-最大高度",
        "classification/truck/min_length": "卡车-最小长度",
        "classification/truck/max_length": "卡车-最大长度",
        "classification/truck/min_width": "卡车-最小宽度",
        "classification/truck/max_width": "卡车-最大宽度",
        "classification/truck/min_height": "卡车-最小高度",
        "classification/truck/max_height": "卡车-最大高度",
        "classification/bus/min_length": "公交-最小长度",
        "classification/bus/max_length": "公交-最大长度",
        "classification/bus/min_width": "公交-最小宽度",
        "classification/bus/max_width": "公交-最大宽度",
        "classification/bus/min_height": "公交-最小高度",
        "classification/bus/max_height": "公交-最大高度",
        "classification/bicycle/min_length": "自行车-最小长度",
        "classification/bicycle/max_length": "自行车-最大长度",
        "classification/bicycle/min_width": "自行车-最小宽度",
        "classification/bicycle/max_width": "自行车-最大宽度",
        "classification/bicycle/min_height": "自行车-最小高度",
        "classification/bicycle/max_height": "自行车-最大高度",
        "classification/person/min_length": "行人-最小长度",
        "classification/person/max_length": "行人-最大长度",
        "classification/person/min_width": "行人-最小宽度",
        "classification/person/max_width": "行人-最大宽度",
        "classification/person/min_height": "行人-最小高度",
        "classification/person/max_height": "行人-最大高度",
        "classification/box/min_length": "箱子-最小长度",
        "classification/box/max_length": "箱子-最大长度",
        "classification/box/min_width": "箱子-最小宽度",
        "classification/box/max_width": "箱子-最大宽度",
        "classification/box/min_height": "箱子-最小高度",
        "classification/box/max_height": "箱子-最大高度",
    }

    def _walk_yaml(self, data, prefix=""):
        """递归遍历 YAML 数据，生成 (full_key, value, label, depth) 列表"""
        items = []
        if isinstance(data, CommentedMap):
            for key in data:
                full_key = f"{prefix}/{key}" if prefix else str(key)
                val = data[key]
                if isinstance(val, (CommentedMap, dict)):
                    # 展开为分组
                    group_label = self.GENERIC_PARAM_LABELS.get(full_key, str(key))
                    items.append(("__group__", full_key, group_label, 0))
                    items.extend(self._walk_yaml(val, full_key))
                elif isinstance(val, (list, CommentedSeq)):
                    # 列表类型：用 QTextEdit 展示
                    label = self.GENERIC_PARAM_LABELS.get(full_key, str(key))
                    items.append((full_key, val, label, 0))
                else:
                    label = self.GENERIC_PARAM_LABELS.get(full_key, str(key))
                    items.append((full_key, val, label, 0))
        return items

    def setup_generic_yaml_tab(self, tab, title, yaml_path, data_attr, widgets_attr, load_func, save_func):
        """通用 YAML 参数编辑标签页设置"""
        l = QVBoxLayout(tab)
        h = QHBoxLayout()
        br = QPushButton("🔄 重新读取")
        br.setStyleSheet("background-color: #757575; color: white;")
        br.clicked.connect(load_func)
        bs = QPushButton(f"💾 保存{title}")
        bs.setStyleSheet("background-color: #4CAF50; color: white;")
        bs.clicked.connect(save_func)
        h.addWidget(br); h.addWidget(bs); l.addLayout(h)

        scroll = QScrollArea(); scroll.setWidgetResizable(True)
        form_widget = QWidget()
        form_layout = QVBoxLayout(form_widget)
        scroll.setWidget(form_widget)
        l.addWidget(scroll)

        # 保存引用
        setattr(tab, '_form_layout', form_layout)

        # 触发加载
        load_func()

    def _add_widget_pair(self, target_layout, lbl, w):
        """安全地添加 label+widget 到布局，兼容 QFormLayout 和 QVBoxLayout"""
        if isinstance(target_layout, QFormLayout):
            target_layout.addRow(lbl, w)
        else:
            # QVBoxLayout: 创建一个临时 QWidget 包裹 QFormLayout
            row_widget = QWidget()
            row_form = QFormLayout(row_widget)
            row_form.setContentsMargins(0, 0, 0, 0)
            row_form.addRow(lbl, w)
            target_layout.addWidget(row_widget)

    def _load_generic_yaml(self, yaml_path, data_attr, widgets_attr, tab):
        """通用 YAML 加载逻辑"""
        if not os.path.exists(yaml_path):
            return
        yaml_parser = YAML()
        yaml_parser.preserve_quotes = True
        with open(yaml_path, 'r', encoding='utf-8') as f:
            data = yaml_parser.load(f)
        setattr(self, data_attr, data)

        form_layout = tab._form_layout
        # 清空旧控件
        for i in reversed(range(form_layout.count())):
            w = form_layout.itemAt(i).widget()
            if w: w.setParent(None)

        widgets = {}
        setattr(self, widgets_attr, widgets)

        # 创建默认的顶层 QFormLayout 容器 (处理没有分组的顶层参数)
        default_container = QWidget()
        default_form = QFormLayout(default_container)
        form_layout.addWidget(default_container)
        setattr(tab, '_current_group_layout', default_form)

        items = self._walk_yaml(data)
        for item in items:
            full_key, val, label_text, depth = item

            if full_key == "__group__":
                # 分组标题
                group = QGroupBox(f"📁 {label_text}")
                group.setStyleSheet("QGroupBox { border: 2px solid #4CAF50; border-radius: 6px; margin-top: 20px; } QGroupBox::title { color: #4CAF50; font-size: 15px; }")
                form_layout.addWidget(group)
                # 创建 QFormLayout 放入 group
                gf = QFormLayout(group)
                setattr(tab, '_current_group_layout', gf)
                continue

            target_layout = getattr(tab, '_current_group_layout', default_form)

            if isinstance(val, bool):
                lbl = QLabel(label_text); lbl.setStyleSheet("font-weight: bold; margin-top: 5px;")
                w = QCheckBox(); w.setChecked(val)
                widgets[full_key] = w
                self._add_widget_pair(target_layout, lbl, w)
            elif isinstance(val, (int, float)):
                lbl = QLabel(label_text); lbl.setStyleSheet("font-weight: bold; margin-top: 5px;")
                w = QDoubleSpinBox(); w.setRange(-100000, 100000); w.setDecimals(4)
                try: w.setValue(float(val))
                except: w.setValue(0.0)
                widgets[full_key] = w
                self._add_widget_pair(target_layout, lbl, w)
            elif isinstance(val, (list, CommentedSeq)):
                # 检查是否为多边形点位列表 (list of {x, y} dicts)
                is_polygon = (len(val) > 0 and isinstance(val[0], (CommentedMap, dict))
                              and 'x' in val[0] and 'y' in val[0])
                if is_polygon:
                    # 使用 PolygonEditor + VehiclePreviewWidget
                    poly_group = QGroupBox(f"📐 {label_text}")
                    poly_hl = QHBoxLayout(poly_group)
                    t = QTextEdit(); t.hide()
                    pv = VehiclePreviewWidget(0, 200, 255)
                    self.previews_downsample_poly.append(pv)
                    ed = PolygonEditor(list(val), full_key, data, t, pv)
                    poly_hl.addWidget(ed, stretch=1)
                    poly_hl.addWidget(pv, stretch=1)
                    widgets[full_key] = t
                    target_layout.addWidget(poly_group)
                else:
                    lbl = QLabel(label_text); lbl.setStyleSheet("font-weight: bold; margin-top: 5px; color: #E65100;")
                    w = QTextEdit(); w.setMaximumHeight(120)
                    try:
                        b = StringIO(); YAML().dump(val, b); w.setPlainText(b.getvalue())
                    except: w.setPlainText(str(val))
                    widgets[full_key] = w
                    self._add_widget_pair(target_layout, lbl, w)
            elif isinstance(val, str):
                lbl = QLabel(label_text); lbl.setStyleSheet("font-weight: bold; margin-top: 5px;")
                w = QLineEdit(); w.setText(str(val))
                widgets[full_key] = w
                self._add_widget_pair(target_layout, lbl, w)
            elif val is None:
                lbl = QLabel(f"{label_text} (null)"); lbl.setStyleSheet("font-weight: bold; margin-top: 5px; color: #999;")
                w = QLineEdit(); w.setPlaceholderText("null")
                widgets[full_key] = w
                self._add_widget_pair(target_layout, lbl, w)

    def _save_generic_yaml(self, yaml_path, data_attr, widgets_attr, title):
        """通用 YAML 保存逻辑"""
        data = getattr(self, data_attr)
        widgets = getattr(self, widgets_attr)
        if data is None:
            return

        for full_key, w in widgets.items():
            # 通过路径定位到 YAML 数据中的值
            keys = full_key.split('/')
            target = data
            for k in keys[:-1]:
                if k in target:
                    target = target[k]
                else:
                    break
            final_key = keys[-1]

            if isinstance(w, QCheckBox):
                target[final_key] = w.isChecked()
            elif isinstance(w, QDoubleSpinBox):
                target[final_key] = w.value()
            elif isinstance(w, QTextEdit):
                try:
                    text = w.toPlainText().strip()
                    target[final_key] = YAML().load(text)
                except Exception as e:
                    print(f"解析 {full_key} 失败: {e}")
            elif isinstance(w, QLineEdit):
                text = w.text().strip()
                # 尝试保持原类型
                orig = target.get(final_key)
                if isinstance(orig, str) or orig is None:
                    target[final_key] = text
                else:
                    try:
                        target[final_key] = YAML().load(text)
                    except:
                        target[final_key] = text

        try:
            yaml_parser = YAML()
            yaml_parser.preserve_quotes = True
            with open(yaml_path, 'w', encoding='utf-8') as f:
                yaml_parser.dump(data, f)
            QMessageBox.information(self, "提示", f"{title}保存成功！")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"保存失败: {e}")

    # ---- downsample.yaml ----
    def load_downsample_yaml(self):
        self._load_generic_yaml(self.downsample_yaml_path, "downsample_yaml_data", "downsample_widgets", self.tab_downsample)

    def save_downsample_yaml(self):
        self._save_generic_yaml(self.downsample_yaml_path, "downsample_yaml_data", "downsample_widgets", "降采样参数")

    # ---- ground.yaml ----
    def load_ground_yaml(self):
        self._load_generic_yaml(self.ground_yaml_path, "ground_yaml_data", "ground_widgets", self.tab_ground)

    def save_ground_yaml(self):
        self._save_generic_yaml(self.ground_yaml_path, "ground_yaml_data", "ground_widgets", "地面分割参数")

    # ---- charge.yaml ----
    def load_charge_yaml(self):
        self._load_generic_yaml(self.charge_yaml_path, "charge_yaml_data", "charge_widgets", self.tab_charge)

    def save_charge_yaml(self):
        self._save_generic_yaml(self.charge_yaml_path, "charge_yaml_data", "charge_widgets", "充电桩参数")

    # ---- euclidean_cluster.yaml ----
    def load_euclidean_yaml(self):
        self._load_generic_yaml(self.euclidean_yaml_path, "euclidean_yaml_data", "euclidean_widgets", self.tab_euclidean)

    def save_euclidean_yaml(self):
        self._save_generic_yaml(self.euclidean_yaml_path, "euclidean_yaml_data", "euclidean_widgets", "欧式聚类参数")

    # ---- shape.yaml ----
    def load_shape_yaml(self):
        self._load_generic_yaml(self.shape_yaml_path, "shape_yaml_data", "shape_widgets", self.tab_shape)

    def save_shape_yaml(self):
        self._save_generic_yaml(self.shape_yaml_path, "shape_yaml_data", "shape_widgets", "形状估计参数")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = ConfigEditorGUI()
    gui.show()
    if HAS_ROS: app.aboutToQuit.connect(rospy.signal_shutdown)
    sys.exit(app.exec_())