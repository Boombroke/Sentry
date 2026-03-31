#!/usr/bin/env python3
# pyright: basic, reportAttributeAccessIssue=false, reportArgumentType=false

import os
import pathlib
import sys
import threading
from glob import glob
import re
import subprocess
from typing import Callable

import matplotlib.image as mpimg
import numpy as np
import serial
import serial.tools.list_ports
import yaml
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure
from PyQt5 import QtCore, QtGui, QtWidgets

from protocol import (
    crc16, pack_with_crc, unpack_packet, packet_size_for_header,
    ImuPacket, StatusPacket, HpPacket, NavPacket,
    STM32_TO_ROS_PACKETS, ROS_TO_STM32_PACKETS, PACKETS,
)

try:
    import rclpy
    from rclpy.node import Node
    HAS_ROS = True
except ImportError:
    HAS_ROS = False


EXPECTED_NODES = [
    ('rm_serial_driver', '串口驱动'),
    ('controller_server', 'Nav2 控制器'),
    ('planner_server', 'Nav2 规划器'),
    ('bt_navigator', 'Nav2 导航'),
    ('sentry_behavior_server', '决策行为树'),
    ('loam_interface', '定位接口'),
    ('sensor_scan_generation', '扫描生成'),
    ('velocity_smoother', '速度平滑'),
]

EXPECTED_TOPICS = [
    ('serial/gimbal_joint_state', 10.0, '云台姿态'),
    ('/cmd_vel', 10.0, '速度指令'),
    ('odometry', 10.0, '里程计'),
    ('obstacle_scan', 5.0, '障碍物扫描'),
    ('terrain_map', 2.0, '地形图'),
    ('referee/game_status', 1.0, '比赛状态'),
    ('referee/robot_status', 1.0, '机器人状态'),
    ('referee/all_robot_hp', 0.5, '全队血量'),
]

EXPECTED_TF = [
    ('map', 'odom', '重定位'),
    ('odom', 'chassis', '里程计'),
    ('chassis', 'gimbal_yaw', '云台'),
]

SCRIPTS_DIR = pathlib.Path(__file__).resolve().parent.parent / 'scripts'

FIX_SCRIPTS = {
    'ros_env': 'fix_ros_env.sh',
    'nav2_deps': 'fix_nav2_deps.sh',
    'serial_perm': 'fix_serial_permission.sh',
    'build': 'fix_build.sh',
    'setup_all': 'setup_env.sh',
}

REMEDIES = {
    'no_ros': {
        'desc': 'ROS2 Humble 未安装或未 source',
        'hint': '执行: source /opt/ros/humble/setup.bash\n或安装: bash src/scripts/fix_ros_env.sh',
        'script': 'fix_ros_env.sh',
    },
    'no_serial_device': {
        'desc': '未检测到串口设备',
        'hint': '1. 检查 USB 线是否连接\n2. 执行: bash src/scripts/fix_serial_permission.sh',
        'script': 'fix_serial_permission.sh',
    },
    'node_missing': {
        'desc': '节点未运行',
        'hint': '启动对应的 launch 文件，或检查是否已编译',
        'script': 'fix_build.sh',
    },
    'topic_no_publisher': {
        'desc': 'Topic 无发布者',
        'hint': '检查对应节点是否启动，或重新编译后启动',
        'script': 'fix_build.sh',
    },
    'topic_low_freq': {
        'desc': 'Topic 频率偏低',
        'hint': '检查传感器连接或节点负载',
        'script': None,
    },
    'tf_broken': {
        'desc': 'TF 链路断开',
        'hint': '检查定位节点和串口驱动是否正常运行',
        'script': None,
    },
    'nav2_missing': {
        'desc': 'Nav2 节点缺失',
        'hint': '安装 Nav2: bash src/scripts/fix_nav2_deps.sh',
        'script': 'fix_nav2_deps.sh',
    },
    'build_needed': {
        'desc': '工作空间未编译或需要重新编译',
        'hint': '执行: bash src/scripts/fix_build.sh',
        'script': 'fix_build.sh',
    },
}


class SerialReaderThread(QtCore.QThread):
    nav_packet_received = QtCore.pyqtSignal(float, float, float, bool)
    serial_error = QtCore.pyqtSignal(str)

    def __init__(self, serial_obj: serial.Serial, serial_lock: threading.Lock):
        super().__init__()
        self._serial = serial_obj
        self._serial_lock = serial_lock
        self._running = True
        self._buffer = bytearray()

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        while self._running:
            try:
                with self._serial_lock:
                    if self._serial and self._serial.is_open:
                        chunk = self._serial.read(256)
                    else:
                        break
                if chunk:
                    self._buffer.extend(chunk)
                    self._parse_buffer()
                else:
                    self.msleep(5)
            except Exception as exc:
                self.serial_error.emit(f"读取错误: {exc}")
                self.msleep(30)

    def _parse_buffer(self) -> None:
        while len(self._buffer) >= 3:
            hdr = self._buffer[0]
            pkt_size = packet_size_for_header(hdr)
            if pkt_size == 0:
                del self._buffer[0]
                continue
            if len(self._buffer) < pkt_size:
                break
            raw = bytes(self._buffer[:pkt_size])
            del self._buffer[:pkt_size]
            result = unpack_packet(raw)
            if result is None:
                continue
            pkt, crc_ok = result
            if isinstance(pkt, NavPacket) and NavPacket in ROS_TO_STM32_PACKETS and NavPacket.HEADER in PACKETS:
                self.nav_packet_received.emit(pkt.vel_x, pkt.vel_y, pkt.vel_w, crc_ok)


class SerialMockTab(QtWidgets.QWidget):
    GAME_PROGRESS_LABELS = {
        'game_progress': [
            (0, '0 未开始'),
            (1, '1 准备阶段'),
            (2, '2 裁判系统自检'),
            (3, '3 五秒倒计时'),
            (4, '4 比赛中'),
            (5, '5 结算'),
        ]
    }

    SPECIAL_FIELDS = {
        'game_progress': 'combo',
        'team_colour': 'radio',
        'rfid_base': 'checkbox',
    }

    FIELD_LABEL_OVERRIDES = {
        'game_progress': '比赛阶段',
        'stage_remain_time': '剩余时间',
        'current_hp': '当前血量',
        'projectile_allowance_17mm': '17mm 弹丸额度',
        'team_colour': '队伍颜色',
        'rfid_base': '基地 RFID 激活',
    }

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)

        self.serial_port: serial.Serial | None = None
        self.serial_lock = threading.Lock()
        self.reader_thread: SerialReaderThread | None = None

        self.packet_tabs: dict[type, dict] = {}
        self.nav_rx_count = 0
        self.error_count = 0

        self._status_callback: Callable[[str], None] | None = None

        self._build_ui()
        self._connect_signals()
        self.refresh_ports()
        self.update_status_bar()
        self._set_connection_state(False)

    def set_status_callback(self, callback) -> None:
        self._status_callback = callback
        self.update_status_bar()

    def _build_ui(self) -> None:
        main_layout = QtWidgets.QVBoxLayout(self)

        top_bar = QtWidgets.QHBoxLayout()
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setMinimumWidth(180)
        self.refresh_port_button = QtWidgets.QPushButton('Refresh')

        self.baud_combo = QtWidgets.QComboBox()
        for b in ['9600', '115200', '460800', '921600']:
            self.baud_combo.addItem(b)
        self.baud_combo.setCurrentText('115200')

        self.connect_button = QtWidgets.QPushButton('Connect')
        self.conn_indicator = QtWidgets.QLabel('● Disconnected')
        self.conn_indicator.setAlignment(QtCore.Qt.AlignCenter)
        self.conn_indicator.setMinimumWidth(140)

        top_bar.addWidget(QtWidgets.QLabel('Serial Port'))
        top_bar.addWidget(self.port_combo)
        top_bar.addWidget(self.refresh_port_button)
        top_bar.addSpacing(12)
        top_bar.addWidget(QtWidgets.QLabel('Baud Rate'))
        top_bar.addWidget(self.baud_combo)
        top_bar.addSpacing(12)
        top_bar.addWidget(self.connect_button)
        top_bar.addStretch(1)
        top_bar.addWidget(self.conn_indicator)
        main_layout.addLayout(top_bar)

        self.tabs = QtWidgets.QTabWidget()
        send_packet_classes = [
            pkt_class
            for pkt_class in (ImuPacket, StatusPacket, HpPacket)
            if pkt_class in STM32_TO_ROS_PACKETS and pkt_class.HEADER in PACKETS
        ]
        for pkt_class in send_packet_classes:
            tab_name = f'{pkt_class.__name__.replace("Packet", "")} 0x{pkt_class.HEADER:02X}'
            tab_widget, enable_cb, controls, interval_spin = self._build_packet_tab(pkt_class)
            timer = QtCore.QTimer(self)
            self.tabs.addTab(tab_widget, tab_name)
            self.packet_tabs[pkt_class] = {
                'enable': enable_cb,
                'controls': controls,
                'interval': interval_spin,
                'timer': timer,
                'count': 0,
            }
        main_layout.addWidget(self.tabs, stretch=1)

        rx_group = QtWidgets.QGroupBox('接收显示 (ROS → STM32)')
        rx_layout = QtWidgets.QGridLayout(rx_group)
        self.nav_vel_x_label = QtWidgets.QLabel('0.000 m/s')
        self.nav_vel_y_label = QtWidgets.QLabel('0.000 m/s')
        self.nav_vel_w_label = QtWidgets.QLabel('0.000 rad/s')
        self.nav_crc_label = QtWidgets.QLabel('N/A')

        rx_layout.addWidget(QtWidgets.QLabel('vel_x'), 0, 0)
        rx_layout.addWidget(self.nav_vel_x_label, 0, 1)
        rx_layout.addWidget(QtWidgets.QLabel('vel_y'), 0, 2)
        rx_layout.addWidget(self.nav_vel_y_label, 0, 3)
        rx_layout.addWidget(QtWidgets.QLabel('vel_w'), 1, 0)
        rx_layout.addWidget(self.nav_vel_w_label, 1, 1)
        rx_layout.addWidget(QtWidgets.QLabel('CRC'), 1, 2)
        rx_layout.addWidget(self.nav_crc_label, 1, 3)

        main_layout.addWidget(rx_group)

    def _build_packet_tab(self, pkt_class) -> tuple[QtWidgets.QWidget, QtWidgets.QCheckBox, dict[str, object], QtWidgets.QSpinBox]:
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QGridLayout(tab)

        enable_cb = QtWidgets.QCheckBox('Enable')
        enable_cb.setChecked(True)
        layout.addWidget(enable_cb, 0, 0, 1, 2)

        controls: dict[str, object] = {}
        for i, meta in enumerate(pkt_class.FIELDS_META, start=1):
            field_name = meta['name']
            field_type = meta['type']
            field_label = self.FIELD_LABEL_OVERRIDES.get(field_name, meta.get('desc', field_name))
            layout.addWidget(QtWidgets.QLabel(field_label), i, 0)

            special_type = self.SPECIAL_FIELDS.get(field_name)
            if special_type == 'combo' and field_name in self.GAME_PROGRESS_LABELS:
                combo = QtWidgets.QComboBox()
                for val, text in self.GAME_PROGRESS_LABELS[field_name]:
                    combo.addItem(text, val)
                combo.setCurrentIndex(4)
                layout.addWidget(combo, i, 1)
                controls[field_name] = combo
                continue

            if special_type == 'radio' and field_name == 'team_colour':
                red_radio = QtWidgets.QRadioButton('Red (1)')
                blue_radio = QtWidgets.QRadioButton('Blue (0)')
                red_radio.setChecked(True)
                team_layout = QtWidgets.QHBoxLayout()
                team_layout.addWidget(red_radio)
                team_layout.addWidget(blue_radio)
                team_layout.addStretch(1)
                team_widget = QtWidgets.QWidget()
                team_widget.setLayout(team_layout)
                layout.addWidget(team_widget, i, 1)
                controls[field_name] = (red_radio, blue_radio)
                continue

            if special_type == 'checkbox' and field_name == 'rfid_base':
                checkbox = QtWidgets.QCheckBox('基地 RFID 激活')
                checkbox.setChecked(False)
                layout.addWidget(checkbox, i, 1)
                controls[field_name] = checkbox
                continue

            if field_type == 'float':
                slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
                slider.setRange(-314, 314)
                spin = QtWidgets.QDoubleSpinBox()
                spin.setRange(-3.14, 3.14)
                spin.setSingleStep(0.01)
                spin.setDecimals(2)
                spin.setValue(0.0)
                self._bind_slider_spin(slider, spin)
                layout.addWidget(slider, i, 1)
                layout.addWidget(spin, i, 2)
                controls[field_name] = spin
            elif field_type == 'uint16':
                spin = QtWidgets.QSpinBox()
                spin.setRange(0, 65535)
                spin.setValue(self._default_uint16_value(field_name))
                if field_name == 'stage_remain_time':
                    spin.setSuffix(' s')
                layout.addWidget(spin, i, 1)
                controls[field_name] = spin
            elif field_type == 'uint8':
                spin = QtWidgets.QSpinBox()
                spin.setRange(0, 255)
                spin.setValue(0)
                layout.addWidget(spin, i, 1)
                controls[field_name] = spin

        interval_spin = QtWidgets.QSpinBox()
        interval_spin.setRange(1, 10000)
        default_interval = max(1, 1000 // pkt_class.FREQUENCY_HZ) if pkt_class.FREQUENCY_HZ <= 50 else 20
        interval_spin.setValue(default_interval)
        interval_spin.setSuffix(' ms')
        layout.addWidget(QtWidgets.QLabel('发送周期'), len(pkt_class.FIELDS_META) + 1, 0)
        layout.addWidget(interval_spin, len(pkt_class.FIELDS_META) + 1, 1)
        layout.setColumnStretch(1, 1)
        return tab, enable_cb, controls, interval_spin

    def _default_uint16_value(self, field_name: str) -> int:
        if field_name == 'stage_remain_time':
            return 300
        if 'projectile' in field_name:
            return 300
        if 'hp' in field_name:
            return 600
        return 0

    def _connect_signals(self) -> None:
        self.refresh_port_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.on_connect_button_clicked)

        for pkt_class, tab_info in self.packet_tabs.items():
            tab_info['timer'].timeout.connect(lambda pkt_class=pkt_class: self._send_packet_type(pkt_class))
            tab_info['enable'].toggled.connect(self.update_timer_states)
            tab_info['interval'].valueChanged.connect(self.update_timer_states)

    def _bind_slider_spin(self, slider: QtWidgets.QSlider, spin: QtWidgets.QDoubleSpinBox) -> None:
        slider.valueChanged.connect(lambda v: spin.setValue(v / 100.0))
        spin.valueChanged.connect(lambda v: slider.setValue(int(round(v * 100))))

    def refresh_ports(self) -> None:
        current = self.port_combo.currentText()
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        if not ports:
            self.port_combo.addItem('')
            return
        for p in ports:
            self.port_combo.addItem(p.device)
        idx = self.port_combo.findText(current)
        if idx >= 0:
            self.port_combo.setCurrentIndex(idx)

    def on_connect_button_clicked(self) -> None:
        if self.serial_port and self.serial_port.is_open:
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self) -> None:
        port = self.port_combo.currentText().strip()
        if not port:
            self._increase_error('未选择串口')
            return
        try:
            baud = int(self.baud_combo.currentText())
            self.serial_port = serial.Serial(port=port, baudrate=baud, timeout=0.05)
            self.reader_thread = SerialReaderThread(self.serial_port, self.serial_lock)
            self.reader_thread.nav_packet_received.connect(self.on_nav_packet_received)
            self.reader_thread.serial_error.connect(self._increase_error)
            self.reader_thread.start()
            self._set_connection_state(True)
            self.update_timer_states()
        except Exception as exc:
            self.serial_port = None
            self._increase_error(f'连接失败: {exc}')
            self._set_connection_state(False)

    def disconnect_serial(self) -> None:
        for tab_info in self.packet_tabs.values():
            tab_info['timer'].stop()

        if self.reader_thread:
            self.reader_thread.stop()
            self.reader_thread.wait(500)
            self.reader_thread = None

        if self.serial_port:
            try:
                if self.serial_port.is_open:
                    self.serial_port.close()
            except Exception as exc:
                self._increase_error(f'断开异常: {exc}')
            self.serial_port = None

        self._set_connection_state(False)

    def _set_connection_state(self, connected: bool) -> None:
        if connected:
            self.conn_indicator.setText('● Connected')
            self.conn_indicator.setStyleSheet('color: #34d399; font-weight: 600;')
            self.connect_button.setText('Disconnect')
        else:
            self.conn_indicator.setText('● Disconnected')
            self.conn_indicator.setStyleSheet('color: #ef4444; font-weight: 600;')
            self.connect_button.setText('Connect')

    def update_timer_states(self) -> None:
        connected = self.serial_port is not None and self.serial_port.is_open
        for tab_info in self.packet_tabs.values():
            self._set_timer(
                tab_info['timer'],
                connected and tab_info['enable'].isChecked(),
                tab_info['interval'].value(),
            )

    def _set_timer(self, timer: QtCore.QTimer, enabled: bool, interval_ms: int) -> None:
        if enabled:
            timer.start(max(1, interval_ms))
        else:
            timer.stop()

    def _send_packet(self, packet: bytes) -> bool:
        if not self.serial_port or not self.serial_port.is_open:
            return False
        try:
            with self.serial_lock:
                written = self.serial_port.write(packet)
            if written != len(packet):
                self._increase_error('串口发送长度异常')
                return False
            return True
        except Exception as exc:
            self._increase_error(f'发送失败: {exc}')
            return False

    def _send_packet_type(self, pkt_class) -> None:
        tab_info = self.packet_tabs[pkt_class]
        kwargs = {}
        for meta in pkt_class.FIELDS_META:
            field_name = meta['name']
            field_type = meta['type']
            control = tab_info['controls'][field_name]
            if isinstance(control, QtWidgets.QComboBox):
                kwargs[field_name] = int(control.currentData())
            elif isinstance(control, tuple):
                kwargs[field_name] = 1 if control[0].isChecked() else 0
            elif isinstance(control, QtWidgets.QCheckBox):
                kwargs[field_name] = 1 if control.isChecked() else 0
            elif field_type == 'float':
                kwargs[field_name] = float(control.value())
            else:
                kwargs[field_name] = int(control.value())

        packet = pack_with_crc(pkt_class(**kwargs))
        if int.from_bytes(packet[-2:], 'little') != crc16(packet[:-2]):
            self._increase_error('打包CRC异常')
            return
        if self._send_packet(packet):
            tab_info['count'] += 1
            self.update_status_bar()

    @QtCore.pyqtSlot(float, float, float, bool)
    def on_nav_packet_received(self, vel_x: float, vel_y: float, vel_w: float, crc_ok: bool) -> None:
        self.nav_vel_x_label.setText(f'{vel_x:.3f} m/s')
        self.nav_vel_y_label.setText(f'{vel_y:.3f} m/s')
        self.nav_vel_w_label.setText(f'{vel_w:.3f} rad/s')
        if crc_ok:
            self.nav_crc_label.setText('OK')
            self.nav_crc_label.setStyleSheet('color: #34d399; font-weight: 600;')
        else:
            self.nav_crc_label.setText('Error')
            self.nav_crc_label.setStyleSheet('color: #ef4444; font-weight: 600;')
            self.error_count += 1
        self.nav_rx_count += 1
        self.update_status_bar()

    def _increase_error(self, msg: str) -> None:
        self.error_count += 1
        self.update_status_bar()
        if self.window() and isinstance(self.window(), QtWidgets.QMainWindow):
            self.window().statusBar().showMessage(msg, 3000)

    def update_status_bar(self) -> None:
        parts = [f'{pkt_class.__name__.replace("Packet", "")}: {info["count"]}' for pkt_class, info in self.packet_tabs.items()]
        parts.append(f'Nav RX: {self.nav_rx_count}')
        parts.append(f'Errors: {self.error_count}')
        status_text = ' | '.join(parts)
        if self._status_callback:
            self._status_callback(status_text)

    def shutdown(self) -> None:
        self.disconnect_serial()


class MapPickerTab(QtWidgets.QWidget):
    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.map_image: np.ndarray | None = None
        self.resolution: float | None = None
        self.origin: list[float] | None = None
        self.map_extent: list[float] | None = None
        self.marked_points: list[tuple[float, float]] = []
        self.marker_artists: list[object] = []
        self.yaml_path = ''

        self._build_ui()

    def _build_ui(self) -> None:
        main_layout = QtWidgets.QVBoxLayout(self)

        top_layout = QtWidgets.QHBoxLayout()
        self.select_map_button = QtWidgets.QPushButton('选择地图')
        self.clear_mark_button = QtWidgets.QPushButton('清除标记')
        self.map_path_label = QtWidgets.QLabel('地图文件: 未选择')
        self.map_path_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)

        top_layout.addWidget(self.select_map_button)
        top_layout.addWidget(self.clear_mark_button)
        top_layout.addWidget(self.map_path_label, stretch=1)
        main_layout.addLayout(top_layout)

        content_layout = QtWidgets.QHBoxLayout()

        self.figure = Figure(facecolor='#1e1e2e')
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.ax = self.figure.add_subplot(111)
        self._apply_axes_style()
        self.toolbar = NavigationToolbar2QT(self.canvas, self)

        canvas_layout = QtWidgets.QVBoxLayout()
        canvas_layout.addWidget(self.toolbar)
        canvas_layout.addWidget(self.canvas, stretch=1)

        left_container = QtWidgets.QWidget()
        left_container.setLayout(canvas_layout)
        content_layout.addWidget(left_container, stretch=4)

        right_panel = QtWidgets.QGroupBox('坐标列表')
        right_layout = QtWidgets.QVBoxLayout(right_panel)
        self.coord_list = QtWidgets.QListWidget()
        self.copy_selected_button = QtWidgets.QPushButton('复制选中')
        self.clear_list_button = QtWidgets.QPushButton('清空列表')

        right_layout.addWidget(self.coord_list, stretch=1)
        right_layout.addWidget(self.copy_selected_button)
        right_layout.addWidget(self.clear_list_button)
        content_layout.addWidget(right_panel, stretch=1)

        main_layout.addLayout(content_layout, stretch=1)

        self.select_map_button.clicked.connect(self.select_map_file)
        self.clear_mark_button.clicked.connect(self.clear_markers)
        self.copy_selected_button.clicked.connect(self.copy_selected_coordinate)
        self.clear_list_button.clicked.connect(self.clear_markers)
        self.canvas.mpl_connect('button_press_event', self.on_canvas_click)

        self._draw_empty_hint()

    def _apply_axes_style(self) -> None:
        self.ax.set_facecolor('#1e1e2e')
        self.ax.tick_params(colors='#d0d0d0')
        for spine in self.ax.spines.values():
            spine.set_color('#8087a2')
        self.ax.xaxis.label.set_color('#d0d0d0')
        self.ax.yaxis.label.set_color('#d0d0d0')
        self.ax.title.set_color('#d0d0d0')

    def _draw_empty_hint(self) -> None:
        self.ax.clear()
        self._apply_axes_style()
        self.ax.set_title('Map Visualization')
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.text(0.5, 0.5, '请选择地图 YAML 文件', transform=self.ax.transAxes, ha='center', va='center', color='#d0d0d0')
        self.ax.grid(True, linestyle='--', alpha=0.3, color='#5a5f73')
        self.canvas.draw_idle()

    def select_map_file(self) -> None:
        yaml_path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            '选择地图 YAML',
            '',
            'YAML Files (*.yaml *.yml);;All Files (*)',
        )
        if not yaml_path:
            return
        self.load_map(yaml_path)

    def load_map(self, yaml_path: str) -> None:
        try:
            with open(yaml_path, 'r', encoding='utf-8') as file:
                map_data = yaml.safe_load(file)
            image_field = map_data['image']
            image_path = image_field if os.path.isabs(image_field) else os.path.join(os.path.dirname(yaml_path), image_field)
            image = mpimg.imread(image_path)
            if image.ndim == 3:
                image = image[:, :, 0]

            resolution = float(map_data['resolution'])
            origin = map_data['origin']
            if not isinstance(origin, list) or len(origin) < 2:
                raise ValueError('origin 字段格式错误')

            self.map_image = np.array(image)
            self.resolution = resolution
            self.origin = origin
            self.yaml_path = yaml_path
            self.map_path_label.setText(f'地图文件: {yaml_path}')

            self.clear_markers()
            self._redraw_base_map()
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, '地图加载失败', f'无法加载地图:\n{exc}')

    def _redraw_base_map(self) -> None:
        if self.map_image is None or self.resolution is None or self.origin is None:
            self._draw_empty_hint()
            return

        height, width = self.map_image.shape
        self.map_extent = [
            self.origin[0],
            self.origin[0] + width * self.resolution,
            self.origin[1],
            self.origin[1] + height * self.resolution,
        ]

        self.ax.clear()
        self._apply_axes_style()
        self.ax.imshow(np.flipud(self.map_image), cmap='gray', origin='lower', extent=self.map_extent)
        self.ax.quiver(self.origin[0], self.origin[1], 1, 0, scale=5, color='red', label='X-axis')
        self.ax.quiver(self.origin[0], self.origin[1], 0, 1, scale=5, color='blue', label='Y-axis')
        self.ax.scatter(0, 0, c='red', marker='o', label='Origin')
        self.ax.text(0, 0, '(0,0)', color='red', fontsize=10, ha='right', va='bottom')
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_title('Map Visualization')
        
        legend = self.ax.legend(facecolor='#1f2438', edgecolor='#3f4666')
        for text in legend.get_texts():
            text.set_color('#d0d0d0')
            
        self.ax.grid(True, linestyle='--', alpha=0.3, color='#5a5f73')

        self.marker_artists.clear()
        for x, y in self.marked_points:
            scatter = self.ax.scatter(x, y, c='#34d399', marker='x')
            text = self.ax.text(x, y, f'({x:.2f},{y:.2f})', color='#34d399', fontsize=8, ha='left', va='bottom')
            self.marker_artists.extend([scatter, text])

        self.canvas.draw_idle()

    def on_canvas_click(self, event) -> None:
        if self.map_image is None:
            return
        if event.inaxes != self.ax or event.button != 1 or event.xdata is None or event.ydata is None:
            return

        x = float(event.xdata)
        y = float(event.ydata)
        self.marked_points.append((x, y))
        self.coord_list.addItem(f'({x:.2f}, {y:.2f})')

        scatter = self.ax.scatter(x, y, c='#34d399', marker='x')
        text = self.ax.text(x, y, f'({x:.2f},{y:.2f})', color='#34d399', fontsize=8, ha='left', va='bottom')
        self.marker_artists.extend([scatter, text])
        self.canvas.draw_idle()

    def clear_markers(self) -> None:
        self.marked_points.clear()
        self.coord_list.clear()
        if self.map_image is not None:
            self._redraw_base_map()

    def copy_selected_coordinate(self) -> None:
        item = self.coord_list.currentItem()
        if item is None:
            return
        QtWidgets.QApplication.clipboard().setText(item.text())
        if self.window() and isinstance(self.window(), QtWidgets.QMainWindow):
            self.window().statusBar().showMessage(f'已复制坐标: {item.text()}', 2000)


class CheckWorker(QtCore.QThread):
    result_ready = QtCore.pyqtSignal(dict)

    def __init__(self, detailed_topic_check: bool = False, parent: QtCore.QObject | None = None) -> None:
        super().__init__(parent)
        self.detailed_topic_check = detailed_topic_check

    def run(self) -> None:
        results: dict[str, list[dict[str, str]]] = {}
        results['serial'] = self._check_serial_devices()
        if HAS_ROS:
            results['nodes'] = self._check_nodes()
            results['topics'] = self._check_topics()
            results['tf'] = self._check_tf()
        else:
            no_ros_item = {
                'name': 'ROS 环境',
                'status': 'error',
                'detail': '未检测到 ROS2 环境',
                'remedy_key': 'no_ros',
            }
            results['nodes'] = [no_ros_item]
            results['topics'] = [no_ros_item]
            results['tf'] = [no_ros_item]
        self.result_ready.emit(results)

    def _run_command(self, cmd: list[str], timeout: int) -> tuple[bool, str, str]:
        if not HAS_ROS:
            return False, '', '需要 ROS 环境'
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
            return result.returncode == 0, result.stdout, result.stderr
        except subprocess.TimeoutExpired:
            return False, '', f'命令超时: {" ".join(cmd)}'
        except FileNotFoundError:
            return False, '', '未找到 ros2 命令'
        except Exception as exc:
            return False, '', f'执行失败: {exc}'

    def _check_serial_devices(self) -> list[dict[str, str]]:
        devices = {port.device for port in serial.tools.list_ports.comports()}
        devices.update(glob('/dev/ttyACM*'))
        devices.update(glob('/dev/ttyUSB*'))
        if not devices:
            return [{
                'name': '串口设备',
                'status': 'error',
                'detail': '未检测到 /dev/ttyACM* 或 /dev/ttyUSB*',
                'remedy_key': 'no_serial_device',
            }]
        rows = []
        for dev in sorted(devices):
            rows.append({'name': dev, 'status': 'ok', 'detail': '已检测到', 'remedy_key': ''})
        return rows

    def _check_nodes(self) -> list[dict[str, str]]:
        ok, out, err = self._run_command(['ros2', 'node', 'list'], timeout=3)
        if not ok:
            detail = err.strip() or '无法获取节点列表'
            remedy_key = 'no_ros' if '需要 ROS 环境' in detail or '未找到 ros2 命令' in detail else 'build_needed'
            return [{'name': 'ROS 节点列表', 'status': 'error', 'detail': detail, 'remedy_key': remedy_key}]
        node_list = [line.strip() for line in out.splitlines() if line.strip()]
        rows = []
        for node_key, node_desc in EXPECTED_NODES:
            found = any(node_key in node_name for node_name in node_list)
            remedy_key = ''
            if not found:
                remedy_key = 'nav2_missing' if node_key in {'controller_server', 'planner_server', 'bt_navigator'} else 'node_missing'
            rows.append({
                'name': node_desc,
                'status': 'ok' if found else 'error',
                'detail': f'{node_key} 运行中' if found else f'{node_key} 未检测到',
                'remedy_key': remedy_key,
            })
        return rows

    def _parse_publisher_count(self, text: str) -> int:
        match = re.search(r'Publisher count:\s*(\d+)', text)
        if not match:
            return 0
        return int(match.group(1))

    def _parse_topic_rate(self, text: str) -> float | None:
        match = re.search(r'average rate:\s*([0-9.]+)', text)
        if not match:
            return None
        try:
            return float(match.group(1))
        except ValueError:
            return None

    def _check_topics(self) -> list[dict[str, str]]:
        rows: list[dict[str, str]] = []
        for topic_name, min_rate, topic_desc in EXPECTED_TOPICS:
            ok, out, err = self._run_command(['ros2', 'topic', 'info', topic_name], timeout=3)
            if not ok:
                rows.append({
                    'name': topic_desc,
                    'status': 'error',
                    'detail': err.strip() or f'{topic_name} 查询失败',
                    'remedy_key': 'topic_no_publisher',
                })
                continue

            publishers = self._parse_publisher_count(out)
            if publishers <= 0:
                rows.append({
                    'name': topic_desc,
                    'status': 'error',
                    'detail': f'{topic_name}: 无发布者',
                    'remedy_key': 'topic_no_publisher',
                })
                continue

            if not self.detailed_topic_check:
                rows.append({'name': topic_desc, 'status': 'ok', 'detail': f'{topic_name}: {publishers} 个发布者', 'remedy_key': ''})
                continue

            hz_ok, hz_out, hz_err = self._run_command(['ros2', 'topic', 'hz', topic_name, '--window', '5'], timeout=6)
            if not hz_ok:
                rows.append({
                    'name': topic_desc,
                    'status': 'warn',
                    'detail': f'{topic_name}: 频率检测失败 ({hz_err.strip() or "timeout"})',
                    'remedy_key': 'topic_low_freq',
                })
                continue

            avg_rate = self._parse_topic_rate(hz_out)
            if avg_rate is None:
                rows.append({
                    'name': topic_desc,
                    'status': 'warn',
                    'detail': f'{topic_name}: 无法解析频率',
                    'remedy_key': 'topic_low_freq',
                })
                continue
            if avg_rate < min_rate:
                rows.append({
                    'name': topic_desc,
                    'status': 'warn',
                    'detail': f'{topic_name}: {avg_rate:.2f}Hz (< {min_rate:.1f}Hz)',
                    'remedy_key': 'topic_low_freq',
                })
            else:
                rows.append({'name': topic_desc, 'status': 'ok', 'detail': f'{topic_name}: {avg_rate:.2f}Hz', 'remedy_key': ''})
        return rows

    def _check_tf(self) -> list[dict[str, str]]:
        rows: list[dict[str, str]] = []
        for parent_frame, child_frame, tf_desc in EXPECTED_TF:
            ok, out, err = self._run_command(
                ['ros2', 'run', 'tf2_ros', 'tf2_echo', parent_frame, child_frame, '--timeout', '2'],
                timeout=4,
            )
            has_transform = ('At time' in out) or ('Translation' in out)
            if ok and has_transform:
                rows.append({'name': tf_desc, 'status': 'ok', 'detail': f'{parent_frame} → {child_frame} 正常', 'remedy_key': ''})
            else:
                detail = err.strip() or f'{parent_frame} → {child_frame} 未检测到'
                rows.append({'name': tf_desc, 'status': 'error', 'detail': detail, 'remedy_key': 'tf_broken'})
        return rows


class CollapsibleSection(QtWidgets.QWidget):
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.is_expanded = True
        
        self.main_layout = QtWidgets.QVBoxLayout(self)
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        self.main_layout.setSpacing(0)
        
        self.header = QtWidgets.QFrame()
        self.header.setStyleSheet(
            'background: #1f2438; border: 1px solid #3a3f57; border-radius: 6px; padding: 6px 10px;'
        )
        self.header.setCursor(QtCore.Qt.PointingHandCursor)
        self.header.mousePressEvent = self.toggle_content
        
        header_layout = QtWidgets.QHBoxLayout(self.header)
        header_layout.setContentsMargins(0, 0, 0, 0)
        
        self.arrow_label = QtWidgets.QLabel('▼')
        self.arrow_label.setStyleSheet('color: #4f8cff; font-weight: bold; border: none; background: transparent;')
        
        self.title_label = QtWidgets.QLabel(title)
        self.title_label.setStyleSheet('color: #e5e7eb; font-weight: bold; border: none; background: transparent;')
        
        self.summary_label = QtWidgets.QLabel('<span style="color: #94a3b8;">⏳</span>')
        self.summary_label.setStyleSheet('border: none; background: transparent;')
        
        header_layout.addWidget(self.arrow_label)
        header_layout.addSpacing(8)
        header_layout.addWidget(self.title_label)
        header_layout.addStretch(1)
        header_layout.addWidget(self.summary_label)
        
        self.main_layout.addWidget(self.header)
        
        self.content_area = QtWidgets.QWidget()
        self.content_layout = QtWidgets.QGridLayout(self.content_area)
        self.content_layout.setContentsMargins(10, 10, 10, 10)
        
        self.main_layout.addWidget(self.content_area)
        
    def toggle_content(self, event):
        self.is_expanded = not self.is_expanded
        self.content_area.setVisible(self.is_expanded)
        self.arrow_label.setText('▼' if self.is_expanded else '▶')
        
    def set_summary(self, ok: int, warn: int, err: int, total: int, pending: int = 0):
        if pending > 0 or total == 0:
            self.summary_label.setText('<span style="color: #94a3b8;">⏳</span>')
        elif err > 0:
            self.summary_label.setText(f'<span style="color: #e5e7eb;">{ok}/{total}</span> <span style="color: #ef4444;">❌ {err}</span>')
        elif warn > 0:
            self.summary_label.setText(f'<span style="color: #e5e7eb;">{ok}/{total}</span> <span style="color: #fbbf24;">⚠️ {warn}</span>')
        else:
            self.summary_label.setText(f'<span style="color: #34d399;">{total}/{total} ✅</span>')


class ConnectivityTab(QtWidgets.QWidget):
    STATUS_META = {
        'ok': ('✅', '#34d399'),
        'warn': ('⚠️', '#fbbf24'),
        'error': ('❌', '#ef4444'),
        'pending': ('⏳', '#94a3b8'),
    }

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self._worker: CheckWorker | None = None
        self._sections: dict[str, CollapsibleSection] = {}
        self._group_layouts: dict[str, QtWidgets.QGridLayout] = {}
        self._build_ui()
        self._auto_timer = QtCore.QTimer(self)
        self._auto_timer.timeout.connect(self.run_checks)
        self._apply_auto_refresh_state()
        self.run_checks()

    def _build_ui(self) -> None:
        main_layout = QtWidgets.QVBoxLayout(self)

        top_bar = QtWidgets.QHBoxLayout()
        self.refresh_button = QtWidgets.QPushButton('立即刷新')
        self.setup_all_button = QtWidgets.QPushButton('一键修复')
        self.auto_refresh_checkbox = QtWidgets.QCheckBox('自动刷新')
        self.auto_refresh_checkbox.setChecked(True)
        self.interval_spinbox = QtWidgets.QSpinBox()
        self.interval_spinbox.setRange(1, 60)
        self.interval_spinbox.setValue(5)
        self.interval_spinbox.setSuffix(' s')
        self.detail_hz_checkbox = QtWidgets.QCheckBox('详细频率检测')
        self.detail_hz_checkbox.setChecked(False)

        self.mode_label = QtWidgets.QLabel()
        if HAS_ROS:
            self.mode_label.setText('模式: ROS ✓')
            self.mode_label.setStyleSheet('color: #34d399; font-weight: 600;')
        else:
            self.mode_label.setText('模式: 基础')
            self.mode_label.setStyleSheet('color: #fbbf24; font-weight: 600;')

        top_bar.addWidget(self.refresh_button)
        top_bar.addSpacing(8)
        top_bar.addWidget(self.setup_all_button)
        top_bar.addSpacing(8)
        top_bar.addWidget(self.auto_refresh_checkbox)
        top_bar.addWidget(self.interval_spinbox)
        top_bar.addSpacing(12)
        top_bar.addWidget(self.detail_hz_checkbox)
        top_bar.addStretch(1)
        top_bar.addWidget(self.mode_label)
        main_layout.addLayout(top_bar)

        self.scroll_area = QtWidgets.QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setFrameShape(QtWidgets.QFrame.NoFrame)
        content_widget = QtWidgets.QWidget()
        content_layout = QtWidgets.QVBoxLayout(content_widget)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(10)

        self._create_group(content_layout, 'serial', '串口设备')
        self._create_group(content_layout, 'nodes', '关键节点')
        self._create_group(content_layout, 'topics', 'Topic 状态')
        self._create_group(content_layout, 'tf', 'TF 链路')
        content_layout.addStretch(1)

        self.scroll_area.setWidget(content_widget)
        main_layout.addWidget(self.scroll_area, stretch=1)

        self.summary_label = QtWidgets.QLabel('总览: 等待检测')
        self.summary_label.setStyleSheet('color: #cbd5e1; font-weight: 600; padding: 4px 2px;')
        main_layout.addWidget(self.summary_label)

        self.refresh_button.clicked.connect(self.run_checks)
        self.setup_all_button.clicked.connect(lambda: self._run_fix_script(FIX_SCRIPTS['setup_all']))
        self.auto_refresh_checkbox.toggled.connect(self._apply_auto_refresh_state)
        self.interval_spinbox.valueChanged.connect(self._apply_auto_refresh_state)

    def _create_group(self, parent_layout: QtWidgets.QVBoxLayout, key: str, title: str) -> None:
        section = CollapsibleSection(title, self)
        self._sections[key] = section
        group_layout = section.content_layout
        group_layout.setColumnStretch(0, 0)
        group_layout.setColumnStretch(1, 2)
        group_layout.setColumnStretch(2, 3)
        group_layout.setColumnStretch(3, 3)
        group_layout.setColumnStretch(4, 0)
        self._group_layouts[key] = group_layout
        parent_layout.addWidget(section)

    def _clear_layout(self, layout: QtWidgets.QGridLayout) -> None:
        while layout.count() > 0:
            item = layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()

    def _set_group_items(self, group: str, items: list[dict[str, str]]) -> None:
        layout = self._group_layouts[group]
        self._clear_layout(layout)
        for row, item in enumerate(items):
            icon, color = self.STATUS_META.get(item['status'], self.STATUS_META['pending'])
            icon_label = QtWidgets.QLabel(icon)
            icon_label.setStyleSheet(f'color: {color}; font-weight: 700;')

            name_label = QtWidgets.QLabel(item['name'])
            name_label.setStyleSheet('color: #e2e8f0;')

            detail_label = QtWidgets.QLabel(item['detail'])
            detail_label.setWordWrap(True)
            detail_label.setStyleSheet('color: #94a3b8;')

            layout.addWidget(icon_label, row, 0)
            layout.addWidget(name_label, row, 1)
            layout.addWidget(detail_label, row, 2)

            remedy_key = item.get('remedy_key', '')
            remedy = REMEDIES.get(remedy_key)
            if item['status'] in {'warn', 'error'} and remedy is not None:
                hint_text = remedy.get('hint', '').splitlines()[0]
                hint_label = QtWidgets.QLabel(hint_text)
                hint_label.setWordWrap(True)
                hint_label.setStyleSheet('color: #94a3b8; font-size: 11px; font-style: italic;')
                layout.addWidget(hint_label, row, 3)

                script_name = remedy.get('script')
                if script_name:
                    fix_btn = QtWidgets.QPushButton('修复')
                    fix_btn.setStyleSheet(
                        'QPushButton { background: #1e3a5f; color: #7dd3fc; border: 1px solid #2563eb; '
                        'border-radius: 4px; padding: 2px 8px; font-size: 12px; }'
                        'QPushButton:hover { background: #1e40af; }'
                    )
                    fix_btn.clicked.connect(lambda _, script_name=script_name: self._run_fix_script(script_name))
                    layout.addWidget(fix_btn, row, 4)

        section = self._sections[group]
        ok = sum(1 for i in items if i['status'] == 'ok')
        warn = sum(1 for i in items if i['status'] == 'warn')
        err = sum(1 for i in items if i['status'] == 'error')
        pending = sum(1 for i in items if i['status'] == 'pending')
        total = ok + warn + err + pending
        section.set_summary(ok, warn, err, total, pending)

    def _run_fix_script(self, script_name: str) -> None:
        script_path = SCRIPTS_DIR / script_name
        if not script_path.exists():
            QtWidgets.QMessageBox.warning(self, '脚本不存在', f'未找到: {script_path}')
            return
        for term_cmd in [
            ['gnome-terminal', '--', 'bash', str(script_path)],
            ['xterm', '-e', f'bash {script_path}; read -p "按回车关闭..."'],
            ['x-terminal-emulator', '-e', f'bash {script_path}'],
        ]:
            try:
                subprocess.Popen(term_cmd)
                return
            except FileNotFoundError:
                continue
        QtWidgets.QMessageBox.information(
            self,
            '请手动执行',
            f'未找到终端模拟器，请手动执行:\n\nbash {script_path}',
        )

    def _set_checking_state(self) -> None:
        self._set_group_items('serial', [{'name': '串口设备', 'status': 'pending', 'detail': '检测中...', 'remedy_key': ''}])
        if HAS_ROS:
            self._set_group_items('nodes', [{'name': '关键节点', 'status': 'pending', 'detail': '检测中...', 'remedy_key': ''}])
            self._set_group_items('topics', [{'name': 'Topic 状态', 'status': 'pending', 'detail': '检测中...', 'remedy_key': ''}])
            self._set_group_items('tf', [{'name': 'TF 链路', 'status': 'pending', 'detail': '检测中...', 'remedy_key': ''}])
        else:
            placeholder = [{'name': 'ROS 检测', 'status': 'pending', 'detail': '需要 ROS 环境', 'remedy_key': ''}]
            self._set_group_items('nodes', placeholder)
            self._set_group_items('topics', placeholder)
            self._set_group_items('tf', placeholder)
        self.summary_label.setText('总览: 检测中...')

    def _apply_auto_refresh_state(self) -> None:
        if self.auto_refresh_checkbox.isChecked():
            self._auto_timer.start(self.interval_spinbox.value() * 1000)
        else:
            self._auto_timer.stop()

    def run_checks(self) -> None:
        if self._worker is not None and self._worker.isRunning():
            return
        self._set_checking_state()
        self._worker = CheckWorker(detailed_topic_check=self.detail_hz_checkbox.isChecked(), parent=self)
        self._worker.result_ready.connect(self._on_results_ready)
        self._worker.finished.connect(self._on_worker_finished)
        self._worker.start()

    @QtCore.pyqtSlot(dict)
    def _on_results_ready(self, results: dict) -> None:
        self._set_group_items('serial', results.get('serial', [{'name': '串口设备', 'status': 'error', 'detail': '检测失败', 'remedy_key': 'build_needed'}]))
        self._set_group_items('nodes', results.get('nodes', [{'name': '关键节点', 'status': 'error', 'detail': '检测失败', 'remedy_key': 'build_needed'}]))
        self._set_group_items('topics', results.get('topics', [{'name': 'Topic 状态', 'status': 'error', 'detail': '检测失败', 'remedy_key': 'build_needed'}]))
        self._set_group_items('tf', results.get('tf', [{'name': 'TF 链路', 'status': 'error', 'detail': '检测失败', 'remedy_key': 'build_needed'}]))
        self._update_summary(results)

    @QtCore.pyqtSlot()
    def _on_worker_finished(self) -> None:
        self._worker = None

    def _update_summary(self, results: dict) -> None:
        ok_count = 0
        warn_count = 0
        error_count = 0
        for group_items in results.values():
            for item in group_items:
                if item['status'] == 'ok':
                    ok_count += 1
                elif item['status'] == 'warn':
                    warn_count += 1
                elif item['status'] == 'error':
                    error_count += 1
        total = ok_count + warn_count + error_count
        summary = f'总览: {ok_count}/{total} 正常' if total > 0 else '总览: 无可用检测项'
        if warn_count > 0:
            summary += f'  ⚠️ {warn_count} 警告'
        if error_count > 0:
            summary += f'  ❌ {error_count} 异常'
        self.summary_label.setText(summary)


class SentryToolboxWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle('Sentry Toolbox')
        self.resize(1100, 800)

        self._apply_dark_theme()

        self.tabs = QtWidgets.QTabWidget()
        self.setCentralWidget(self.tabs)

        self.serial_tab = SerialMockTab(self)
        self.map_tab = MapPickerTab(self)
        self.connectivity_tab = ConnectivityTab(self)

        self.tabs.addTab(self.serial_tab, '串口 Mock')
        self.tabs.addTab(self.map_tab, '地图坐标拾取')
        self.tabs.addTab(self.connectivity_tab, '连通性检测')

        self.status_label = QtWidgets.QLabel('Ready')
        self.statusBar().addPermanentWidget(self.status_label, 1)
        self.serial_tab.set_status_callback(self.status_label.setText)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self.serial_tab.shutdown()
        super().closeEvent(event)

    def _apply_dark_theme(self) -> None:
        app = QtWidgets.QApplication.instance()
        if app is not None:
            palette = QtGui.QPalette()
            palette.setColor(QtGui.QPalette.Window, QtGui.QColor('#11131f'))
            palette.setColor(QtGui.QPalette.WindowText, QtGui.QColor('#f8fafc'))
            palette.setColor(QtGui.QPalette.Base, QtGui.QColor('#1a1d2e'))
            palette.setColor(QtGui.QPalette.AlternateBase, QtGui.QColor('#1f2438'))
            palette.setColor(QtGui.QPalette.ToolTipBase, QtGui.QColor('#1a1d2e'))
            palette.setColor(QtGui.QPalette.ToolTipText, QtGui.QColor('#f8fafc'))
            palette.setColor(QtGui.QPalette.Text, QtGui.QColor('#f8fafc'))
            palette.setColor(QtGui.QPalette.Button, QtGui.QColor('#2a2f45'))
            palette.setColor(QtGui.QPalette.ButtonText, QtGui.QColor('#e5e7eb'))
            palette.setColor(QtGui.QPalette.BrightText, QtGui.QColor('#ef4444'))
            palette.setColor(QtGui.QPalette.Highlight, QtGui.QColor('#4f8cff'))
            palette.setColor(QtGui.QPalette.HighlightedText, QtGui.QColor('#ffffff'))
            app.setPalette(palette)

        self.setStyleSheet('''
            QMainWindow {
                background-color: #11131f;
            }
            QWidget {
                font-size: 13px;
                color: #f8fafc;
            }
            QTabWidget::pane {
                border: 1px solid #3a3f57;
                background: #1a1d2e;
                border-radius: 4px;
            }
            QTabBar::tab {
                background: #171a28;
                color: #cbd5e1;
                border: 1px solid #3a3f57;
                padding: 6px 12px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                margin-right: 2px;
            }
            QTabBar::tab:selected {
                background: #1a1d2e;
                color: #f8fafc;
                border-bottom-color: #1a1d2e;
            }
            QTabBar::tab:hover:!selected {
                background: #1f2438;
            }
            QGroupBox {
                background-color: #1a1d2e;
                border: 1px solid #3a3f57;
                border-radius: 8px;
                margin-top: 12px;
                padding-top: 16px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 10px;
                padding: 0 4px;
                color: #e5e7eb;
            }
            QPushButton {
                background-color: #2a2f45;
                color: #e5e7eb;
                border: 1px solid #3f4666;
                border-radius: 4px;
                padding: 5px 12px;
            }
            QPushButton:hover {
                background-color: #343b56;
            }
            QPushButton:pressed {
                background-color: #1f2438;
            }
            QSpinBox, QDoubleSpinBox, QComboBox, QLineEdit {
                background-color: #1f2438;
                color: #f8fafc;
                border: 1px solid #3f4666;
                border-radius: 4px;
                padding: 3px 6px;
            }
            QComboBox::drop-down {
                border: none;
            }
            QComboBox QAbstractItemView {
                background-color: #1f2438;
                color: #f8fafc;
                selection-background-color: #4f8cff;
            }
            QSlider::groove:horizontal {
                border: 1px solid #3f4666;
                height: 6px;
                background: #1f2438;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #4f8cff;
                border: 1px solid #4f8cff;
                width: 14px;
                margin: -4px 0;
                border-radius: 7px;
            }
            QLabel {
                background: transparent;
                color: #f8fafc;
            }
            QCheckBox, QRadioButton {
                background: transparent;
                color: #f8fafc;
            }
            QListWidget {
                background-color: #1f2438;
                color: #f8fafc;
                border: 1px solid #3f4666;
                border-radius: 4px;
            }
            QListWidget::item:selected {
                background-color: #4f8cff;
                color: #ffffff;
            }
            QStatusBar {
                background-color: #171a28;
                color: #cbd5e1;
                border-top: 1px solid #32374e;
            }
        ''')


def main() -> int:
    app = QtWidgets.QApplication(sys.argv)
    window = SentryToolboxWindow()
    window.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
