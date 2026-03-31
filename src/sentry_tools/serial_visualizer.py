#!/usr/bin/env python3
# pyright: basic, reportMissingTypeStubs=false, reportAttributeAccessIssue=false, reportUnknownMemberType=false, reportUnknownVariableType=false, reportUnknownArgumentType=false, reportUnknownParameterType=false, reportUnannotatedClassAttribute=false, reportUnusedCallResult=false, reportImplicitStringConcatenation=false, reportImplicitOverride=false, reportMissingParameterType=false

import collections
import sys
import threading
import time

import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from PyQt5 import QtCore, QtGui, QtWidgets
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from rm_interfaces.msg import GameRobotHP, GameStatus, RobotStatus
from sensor_msgs.msg import JointState


GAME_PROGRESS_LABELS = [
    '未开始',
    '准备阶段',
    '裁判系统自检',
    '五秒倒计时',
    '比赛中',
    '结算',
]

WINDOW_SECONDS = 10.0
UI_INTERVAL_MS = 50
ROBOT_HP_MAX = 600
OUTPOST_HP_MAX = 1500
BASE_HP_MAX = 1500


class SharedData:
    def __init__(self) -> None:
        self.lock = threading.Lock()

        self.gimbal_t = collections.deque(maxlen=200)
        self.gimbal_pitch = collections.deque(maxlen=200)
        self.gimbal_yaw = collections.deque(maxlen=200)

        self.cmd_t = collections.deque(maxlen=200)
        self.cmd_vx = collections.deque(maxlen=200)
        self.cmd_vy = collections.deque(maxlen=200)
        self.cmd_vw = collections.deque(maxlen=200)

        self.game_progress = 0
        self.stage_remain_time = 0
        self.current_hp = 0
        self.projectile_allowance = 0

        self.team_hp = {
            '1号': 0,
            '2号': 0,
            '3号': 0,
            '4号': 0,
            '7号': 0,
            '前哨': 0,
            '基地': 0,
        }

        self.cmd_count = 0
        self.latest_vx = 0.0
        self.latest_vy = 0.0
        self.latest_vw = 0.0

        self.last_rx = {
            'gimbal': 0.0,
            'cmd_vel': 0.0,
            'game': 0.0,
            'robot': 0.0,
            'hp': 0.0,
        }


class SerialVisualizerNode(Node):
    def __init__(self, shared: SharedData) -> None:
        super().__init__('serial_visualizer')
        self.shared = shared

        self.create_subscription(JointState, 'serial/gimbal_joint_state', self._on_gimbal, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(GameStatus, 'referee/game_status', self._on_game_status, 10)
        self.create_subscription(RobotStatus, 'referee/robot_status', self._on_robot_status, 10)
        self.create_subscription(GameRobotHP, 'referee/all_robot_hp', self._on_all_robot_hp, 10)

    def _on_gimbal(self, msg: JointState) -> None:
        now = time.monotonic()
        pitch = msg.position[0] if len(msg.position) > 0 else 0.0
        yaw = msg.position[1] if len(msg.position) > 1 else 0.0
        with self.shared.lock:
            self.shared.gimbal_t.append(now)
            self.shared.gimbal_pitch.append(float(pitch))
            self.shared.gimbal_yaw.append(float(yaw))
            self.shared.last_rx['gimbal'] = now

    def _on_cmd_vel(self, msg: Twist) -> None:
        now = time.monotonic()
        with self.shared.lock:
            self.shared.cmd_t.append(now)
            self.shared.cmd_vx.append(float(msg.linear.x))
            self.shared.cmd_vy.append(float(msg.linear.y))
            self.shared.cmd_vw.append(float(msg.angular.z))
            self.shared.latest_vx = float(msg.linear.x)
            self.shared.latest_vy = float(msg.linear.y)
            self.shared.latest_vw = float(msg.angular.z)
            self.shared.cmd_count += 1
            self.shared.last_rx['cmd_vel'] = now

    def _on_game_status(self, msg: GameStatus) -> None:
        now = time.monotonic()
        with self.shared.lock:
            self.shared.game_progress = int(msg.game_progress)
            self.shared.stage_remain_time = int(msg.stage_remain_time)
            self.shared.last_rx['game'] = now

    def _on_robot_status(self, msg: RobotStatus) -> None:
        now = time.monotonic()
        with self.shared.lock:
            self.shared.current_hp = int(msg.current_hp)
            self.shared.projectile_allowance = int(msg.projectile_allowance_17mm)
            self.shared.last_rx['robot'] = now

    def _on_all_robot_hp(self, msg: GameRobotHP) -> None:
        now = time.monotonic()
        with self.shared.lock:
            self.shared.team_hp['1号'] = int(msg.ally_1_robot_hp)
            self.shared.team_hp['2号'] = int(msg.ally_2_robot_hp)
            self.shared.team_hp['3号'] = int(msg.ally_3_robot_hp)
            self.shared.team_hp['4号'] = int(msg.ally_4_robot_hp)
            self.shared.team_hp['7号'] = int(msg.ally_7_robot_hp)
            self.shared.team_hp['前哨'] = int(msg.ally_outpost_hp)
            self.shared.team_hp['基地'] = int(msg.ally_base_hp)
            self.shared.last_rx['hp'] = now


class RosSpinThread(threading.Thread):
    def __init__(self, executor: SingleThreadedExecutor) -> None:
        super().__init__(daemon=True)
        self.executor = executor

    def run(self) -> None:
        self.executor.spin()


class PlotCanvas(FigureCanvasQTAgg):
    def __init__(self, title: str) -> None:
        fig = Figure(figsize=(6, 3), facecolor='#1e1e2e')
        self.ax = fig.add_subplot(111)
        super().__init__(fig)
        self.title = title
        self._apply_axes_style()

    def _apply_axes_style(self) -> None:
        self.ax.set_facecolor('#1e1e2e')
        self.ax.grid(True, color='#5a5f73', linestyle='--', linewidth=0.5, alpha=0.45)
        self.ax.tick_params(colors='#d0d0d0')
        for spine in self.ax.spines.values():
            spine.set_color('#8087a2')
        self.ax.set_title(self.title, color='#e6e6e6', fontsize=11)
        self.ax.set_xlabel('时间 (s)', color='#d0d0d0')

    def update_plot(self, x_data, curves, labels, colors, y_label: str) -> None:
        self.ax.clear()
        self._apply_axes_style()
        self.ax.set_ylabel(y_label, color='#d0d0d0')

        has_points = False
        y_min = float('inf')
        y_max = float('-inf')
        for y_data, label, color in zip(curves, labels, colors):
            if x_data and y_data:
                self.ax.plot(x_data, y_data, color=color, linewidth=1.8, label=label)
                local_min = min(y_data)
                local_max = max(y_data)
                y_min = min(y_min, local_min)
                y_max = max(y_max, local_max)
                has_points = True

        self.ax.set_xlim(-WINDOW_SECONDS, 0.0)
        if has_points:
            span = max(1e-3, y_max - y_min)
            pad = max(0.05, span * 0.2)
            self.ax.set_ylim(y_min - pad, y_max + pad)
            self.ax.legend(loc='upper left', facecolor='#2a2f45', edgecolor='#444a61', fontsize=9)
        else:
            self.ax.set_ylim(-1.0, 1.0)

        self.draw_idle()


class DashboardPanel(QtWidgets.QGroupBox):
    def __init__(self, title: str) -> None:
        super().__init__(title)
        self.setStyleSheet(
            'QGroupBox { color: #e5e7eb; border: 1px solid #3a3f57; border-radius: 8px; margin-top: 10px; }'
            'QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }'
        )


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, shared: SharedData) -> None:
        super().__init__()
        self.shared = shared
        self.last_ui_tick = time.monotonic()
        self.ui_hz = 0.0
        self.setWindowTitle('Serial Visualizer - Sentry Nav')
        self.resize(1400, 900)

        plt.style.use('dark_background')

        self._setup_ui()
        self._setup_timer()

    def _setup_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root_layout = QtWidgets.QVBoxLayout(central)
        root_layout.setContentsMargins(12, 12, 12, 8)
        root_layout.setSpacing(10)

        body_layout = QtWidgets.QHBoxLayout()
        body_layout.setSpacing(10)
        root_layout.addLayout(body_layout, stretch=1)

        left_widget = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left_widget)
        left_layout.setSpacing(10)
        left_layout.setContentsMargins(0, 0, 0, 0)

        gimbal_box = DashboardPanel('云台姿态')
        gimbal_layout = QtWidgets.QVBoxLayout(gimbal_box)
        self.gimbal_plot = PlotCanvas('Pitch / Yaw 实时曲线 (最近10s)')
        gimbal_layout.addWidget(self.gimbal_plot)

        cmd_box = DashboardPanel('导航速度')
        cmd_layout = QtWidgets.QVBoxLayout(cmd_box)
        self.cmd_plot = PlotCanvas('Vx / Vy / Vw 实时曲线 (最近10s)')
        cmd_layout.addWidget(self.cmd_plot)

        left_layout.addWidget(gimbal_box, stretch=1)
        left_layout.addWidget(cmd_box, stretch=1)

        right_widget = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right_widget)
        right_layout.setSpacing(10)
        right_layout.setContentsMargins(0, 0, 0, 0)

        game_box = DashboardPanel('比赛状态')
        game_layout = QtWidgets.QVBoxLayout(game_box)
        self.game_stage_label = QtWidgets.QLabel('阶段: 未开始')
        self.game_stage_label.setStyleSheet('font-size: 18px; font-weight: 700; color: #dbeafe;')
        self.game_remain_label = QtWidgets.QLabel('剩余: 0s')
        self.game_remain_label.setStyleSheet('font-size: 22px; font-weight: 800; color: #f8fafc;')
        self.game_progress_bar = QtWidgets.QProgressBar()
        self.game_progress_bar.setRange(0, 420)
        self.game_progress_bar.setValue(0)
        self.game_progress_bar.setFormat('%v/%m s')
        self.game_progress_bar.setStyleSheet(
            'QProgressBar { border: 1px solid #3f4666; border-radius: 6px; background: #1f2438; color: #d9e2ff; text-align: center; }'
            'QProgressBar::chunk { border-radius: 6px; background-color: #4f8cff; }'
        )
        game_layout.addWidget(self.game_stage_label)
        game_layout.addWidget(self.game_remain_label)
        game_layout.addWidget(self.game_progress_bar)

        robot_box = DashboardPanel('机器人状态')
        robot_layout = QtWidgets.QVBoxLayout(robot_box)
        hp_row = QtWidgets.QHBoxLayout()
        hp_text = QtWidgets.QLabel('血量:')
        hp_text.setStyleSheet('font-size: 16px; font-weight: 600;')
        self.hp_value_label = QtWidgets.QLabel('0')
        self.hp_value_label.setStyleSheet('font-size: 24px; font-weight: 800; color: #f8fafc;')
        hp_row.addWidget(hp_text)
        hp_row.addStretch(1)
        hp_row.addWidget(self.hp_value_label)
        self.hp_bar = QtWidgets.QProgressBar()
        self.hp_bar.setRange(0, ROBOT_HP_MAX)
        self.hp_bar.setValue(0)
        self.hp_bar.setFormat('%v/%m')
        self.ammo_label = QtWidgets.QLabel('弹量: 0 发')
        self.ammo_label.setStyleSheet('font-size: 24px; font-weight: 800; color: #7dd3fc;')
        robot_layout.addLayout(hp_row)
        robot_layout.addWidget(self.hp_bar)
        robot_layout.addWidget(self.ammo_label)

        tx_box = DashboardPanel('发送数据 (ROS→电控)')
        tx_layout = QtWidgets.QGridLayout(tx_box)
        tx_layout.setSpacing(6)

        vel_labels = [('vel_x', 'm/s'), ('vel_y', 'm/s'), ('vel_w', 'rad/s')]
        self.tx_value_labels = {}
        for i, (name, unit) in enumerate(vel_labels):
            name_lbl = QtWidgets.QLabel(f'{name}:')
            name_lbl.setStyleSheet('font-size: 14px; font-weight: 600;')
            val_lbl = QtWidgets.QLabel('0.000')
            val_lbl.setStyleSheet('font-size: 20px; font-weight: 800; color: #f8fafc; font-family: monospace;')
            unit_lbl = QtWidgets.QLabel(unit)
            unit_lbl.setStyleSheet('font-size: 12px; color: #94a3b8;')
            tx_layout.addWidget(name_lbl, i, 0)
            tx_layout.addWidget(val_lbl, i, 1)
            tx_layout.addWidget(unit_lbl, i, 2)
            self.tx_value_labels[name] = val_lbl

        self.tx_freq_label = QtWidgets.QLabel('频率: — Hz')
        self.tx_freq_label.setStyleSheet('font-size: 13px; color: #7dd3fc;')
        self.tx_count_label = QtWidgets.QLabel('计数: 0')
        self.tx_count_label.setStyleSheet('font-size: 13px; color: #94a3b8;')
        tx_footer = QtWidgets.QHBoxLayout()
        tx_footer.addWidget(self.tx_freq_label)
        tx_footer.addStretch(1)
        tx_footer.addWidget(self.tx_count_label)
        tx_layout.addLayout(tx_footer, len(vel_labels), 0, 1, 3)

        team_box = DashboardPanel('全队血量')
        team_layout = QtWidgets.QVBoxLayout(team_box)
        team_layout.setSpacing(6)
        self.team_bar_widgets = {}
        team_defs = [
            ('1号', ROBOT_HP_MAX),
            ('2号', ROBOT_HP_MAX),
            ('3号', ROBOT_HP_MAX),
            ('4号', ROBOT_HP_MAX),
            ('7号', ROBOT_HP_MAX),
            ('前哨', OUTPOST_HP_MAX),
            ('基地', BASE_HP_MAX),
        ]
        for label, max_value in team_defs:
            row = QtWidgets.QHBoxLayout()
            name = QtWidgets.QLabel(label)
            name.setFixedWidth(36)
            bar = QtWidgets.QProgressBar()
            bar.setRange(0, max_value)
            bar.setValue(0)
            bar.setFormat('%v/%m')
            bar.setStyleSheet(
                'QProgressBar { border: 1px solid #3f4666; border-radius: 5px; background: #1f2438; color: #d9e2ff; text-align: center; min-height: 18px; }'
                'QProgressBar::chunk { border-radius: 5px; background-color: #34d399; }'
            )
            value = QtWidgets.QLabel('0')
            value.setFixedWidth(46)
            value.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            row.addWidget(name)
            row.addWidget(bar, stretch=1)
            row.addWidget(value)
            team_layout.addLayout(row)
            self.team_bar_widgets[label] = (bar, value)

        right_layout.addWidget(game_box)
        right_layout.addWidget(robot_box)
        right_layout.addWidget(tx_box)
        right_layout.addWidget(team_box, stretch=1)

        body_layout.addWidget(left_widget, stretch=3)
        body_layout.addWidget(right_widget, stretch=2)

        self.status_label = QtWidgets.QLabel('Topics: gimbal ✗ cmd_vel ✗ game ✗ robot ✗ hp ✗   20Hz')
        self.status_label.setStyleSheet('padding: 4px 8px; color: #cbd5e1; background: #171a28; border: 1px solid #32374e; border-radius: 6px;')
        root_layout.addWidget(self.status_label)

        self._set_hp_bar_color('#ef4444')

    def _setup_timer(self) -> None:
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(UI_INTERVAL_MS)
        self.timer.timeout.connect(self._on_ui_tick)
        self.timer.start()

    def _set_hp_bar_color(self, color: str) -> None:
        self.hp_bar.setStyleSheet(
            'QProgressBar { border: 1px solid #3f4666; border-radius: 6px; background: #1f2438; color: #e2e8f0; text-align: center; min-height: 22px; font-size: 13px; }'
            f'QProgressBar::chunk {{ border-radius: 6px; background-color: {color}; }}'
        )

    @staticmethod
    def _extract_window(time_data, value_data, now_ts: float):
        if not time_data or not value_data:
            return [], []
        cutoff = now_ts - WINDOW_SECONDS
        rel_t = []
        rel_v = []
        for t, v in zip(time_data, value_data):
            if t >= cutoff:
                rel_t.append(t - now_ts)
                rel_v.append(v)
        return rel_t, rel_v

    def _on_ui_tick(self) -> None:
        now = time.monotonic()
        dt = max(1e-3, now - self.last_ui_tick)
        inst_hz = 1.0 / dt
        self.ui_hz = self.ui_hz * 0.85 + inst_hz * 0.15 if self.ui_hz > 0 else inst_hz
        self.last_ui_tick = now

        with self.shared.lock:
            gimbal_t = list(self.shared.gimbal_t)
            gimbal_pitch = list(self.shared.gimbal_pitch)
            gimbal_yaw = list(self.shared.gimbal_yaw)

            cmd_t = list(self.shared.cmd_t)
            cmd_vx = list(self.shared.cmd_vx)
            cmd_vy = list(self.shared.cmd_vy)
            cmd_vw = list(self.shared.cmd_vw)

            latest_vx = self.shared.latest_vx
            latest_vy = self.shared.latest_vy
            latest_vw = self.shared.latest_vw
            cmd_count = self.shared.cmd_count

            game_progress = self.shared.game_progress
            stage_remain_time = self.shared.stage_remain_time
            current_hp = self.shared.current_hp
            projectile_allowance = self.shared.projectile_allowance
            team_hp = dict(self.shared.team_hp)
            last_rx = dict(self.shared.last_rx)

        x_g, y_pitch = self._extract_window(gimbal_t, gimbal_pitch, now)
        _, y_yaw = self._extract_window(gimbal_t, gimbal_yaw, now)
        self.gimbal_plot.update_plot(
            x_g,
            [y_pitch, y_yaw],
            ['pitch', 'yaw'],
            ['#4f8cff', '#ff9f43'],
            '角度 (rad)',
        )

        x_c, y_vx = self._extract_window(cmd_t, cmd_vx, now)
        _, y_vy = self._extract_window(cmd_t, cmd_vy, now)
        _, y_vw = self._extract_window(cmd_t, cmd_vw, now)
        self.cmd_plot.update_plot(
            x_c,
            [y_vx, y_vy, y_vw],
            ['vx', 'vy', 'vw'],
            ['#4f8cff', '#ff9f43', '#22c55e'],
            '速度',
        )

        stage_name = GAME_PROGRESS_LABELS[game_progress] if 0 <= game_progress < len(GAME_PROGRESS_LABELS) else f'未知({game_progress})'
        self.game_stage_label.setText(f'阶段: {stage_name}')
        remain = max(0, int(stage_remain_time))
        self.game_remain_label.setText(f'剩余: {remain}s')
        self.game_progress_bar.setValue(min(self.game_progress_bar.maximum(), remain))

        hp = max(0, min(ROBOT_HP_MAX, int(current_hp)))
        self.hp_value_label.setText(str(hp))
        self.hp_bar.setValue(hp)
        hp_pct = hp / float(ROBOT_HP_MAX)
        if hp_pct > 0.6:
            self._set_hp_bar_color('#22c55e')
        elif hp_pct > 0.3:
            self._set_hp_bar_color('#f59e0b')
        else:
            self._set_hp_bar_color('#ef4444')
        self.ammo_label.setText(f'弹量: {max(0, int(projectile_allowance))} 发')

        for key, (bar, value_label) in self.team_bar_widgets.items():
            value = max(0, int(team_hp.get(key, 0)))
            value = min(value, bar.maximum())
            bar.setValue(value)
            value_label.setText(str(value))

        self.tx_value_labels['vel_x'].setText(f'{latest_vx:+.3f}')
        self.tx_value_labels['vel_y'].setText(f'{latest_vy:+.3f}')
        self.tx_value_labels['vel_w'].setText(f'{latest_vw:+.3f}')
        self.tx_count_label.setText(f'计数: {cmd_count}')

        cmd_hz = 0.0
        cutoff = now - 2.0
        recent_cmd = sum(1 for t in cmd_t if t >= cutoff)
        cmd_hz = recent_cmd / 2.0
        self.tx_freq_label.setText(f'频率: {cmd_hz:.1f} Hz')

        for name, lbl in self.tx_value_labels.items():
            val = latest_vx if name == 'vel_x' else (latest_vy if name == 'vel_y' else latest_vw)
            if abs(val) > 0.01:
                lbl.setStyleSheet('font-size: 20px; font-weight: 800; color: #4f8cff; font-family: monospace;')
            else:
                lbl.setStyleSheet('font-size: 20px; font-weight: 800; color: #f8fafc; font-family: monospace;')

        active_s = 1.0
        def mark(topic_key: str) -> str:
            return '✓' if now - last_rx.get(topic_key, 0.0) < active_s else '✗'

        self.status_label.setText(
            f'Topics: gimbal {mark("gimbal")} cmd_vel {mark("cmd_vel")} game {mark("game")} robot {mark("robot")} hp {mark("hp")}   {self.ui_hz:>4.1f}Hz'
        )


def apply_dark_palette(app: QtWidgets.QApplication) -> None:
    palette = QtGui.QPalette()
    palette.setColor(QtGui.QPalette.Window, QtGui.QColor(24, 26, 34))
    palette.setColor(QtGui.QPalette.WindowText, QtGui.QColor(235, 238, 245))
    palette.setColor(QtGui.QPalette.Base, QtGui.QColor(30, 32, 46))
    palette.setColor(QtGui.QPalette.AlternateBase, QtGui.QColor(39, 42, 58))
    palette.setColor(QtGui.QPalette.ToolTipBase, QtGui.QColor(240, 240, 240))
    palette.setColor(QtGui.QPalette.ToolTipText, QtGui.QColor(30, 30, 30))
    palette.setColor(QtGui.QPalette.Text, QtGui.QColor(235, 238, 245))
    palette.setColor(QtGui.QPalette.Button, QtGui.QColor(36, 39, 52))
    palette.setColor(QtGui.QPalette.ButtonText, QtGui.QColor(233, 237, 245))
    palette.setColor(QtGui.QPalette.BrightText, QtGui.QColor(255, 100, 100))
    palette.setColor(QtGui.QPalette.Highlight, QtGui.QColor(79, 140, 255))
    palette.setColor(QtGui.QPalette.HighlightedText, QtGui.QColor(255, 255, 255))
    app.setPalette(palette)
    app.setStyleSheet(
        'QWidget { font-size: 13px; }'
        'QLabel { color: #e5e7eb; }'
        'QMainWindow { background-color: #181a22; }'
    )


def main() -> None:
    app = QtWidgets.QApplication(sys.argv)
    apply_dark_palette(app)

    rclpy.init()
    shared = SharedData()
    node = SerialVisualizerNode(shared)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = RosSpinThread(executor)
    spin_thread.start()

    window = MainWindow(shared)
    window.show()

    def shutdown() -> None:
        if window.timer.isActive():
            window.timer.stop()
        executor.shutdown(timeout_sec=0.5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)

    app.aboutToQuit.connect(shutdown)
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
