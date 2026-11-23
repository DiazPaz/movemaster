import os
import platform


def _linux_release():
    """Lee el identificador y versión de Linux sin dependencias externas."""
    values = {}
    try:
        with open('/etc/os-release', encoding='utf-8') as release_file:
            for line in release_file:
                if '=' in line and not line.lstrip().startswith('#'):
                    key, value = line.strip().split('=', 1)
                    values[key] = value.strip('\"\'')
    except OSError:
        pass
    return values.get('ID', '').lower(), values.get('VERSION_ID', '')


def configure_qt_platform():
    """Aplica solo el workaround conocido; Qt elige en otros entornos."""
    system_name = platform.system()
    distro, version = _linux_release() if system_name == 'Linux' else ('', '')
    if 'QT_QPA_PLATFORM' in os.environ:
        chosen = os.environ['QT_QPA_PLATFORM']
        reason = 'configuración externa'
    elif system_name == 'Linux' and distro == 'ubuntu' and version == '26.04' and os.environ.get('DISPLAY'):
        # Ubuntu 26.04 de desarrollo: evita la ruta gráfica que provocó
        # el fallo GLib-GIO / xsettings. En sesión Wayland usa XWayland.
        os.environ['QT_QPA_PLATFORM'] = 'xcb'
        chosen = 'xcb'
        reason = 'compatibilidad Ubuntu 26.04'
    else:
        chosen = '(automático)'
        reason = 'selección nativa de Qt'
    print(f'[SYSTEM] {system_name} {distro} {version} | '
          f'arquitectura={platform.machine()} | '
          f'sesión={os.environ.get("XDG_SESSION_TYPE", "desconocida")}')
    print(f'[QT] Plataforma: {chosen} ({reason})')


# La plataforma gráfica DEBE configurarse antes de importar PySide6.
configure_qt_platform()

import sys
import time
import math
import random
import re
from pathlib import Path
import serial
import cv2
import numpy as np
from PySide6.QtCore import Qt, QEvent, QTimer, QThread, Signal, QPointF, QLocale, QElapsedTimer, QRect
from PySide6.QtGui import QDoubleValidator, QImage, QPixmap, QPainter, QPen, QColor, QFont, QFontMetrics
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QTabWidget, QLineEdit, QPushButton, QLabel, QListWidget, QDialog, QDialogButtonBox, QGroupBox, QMessageBox, QSlider, QStackedLayout, QStackedWidget, QSizePolicy, QFrame

def numeric_validator(minimum, maximum, decimals, parent):
    """Accept decimals, but never scientific notation or thousands separators."""
    validator = QDoubleValidator(minimum, maximum, decimals, parent)
    validator.setNotation(QDoubleValidator.Notation.StandardNotation)
    locale = QLocale.c()
    locale.setNumberOptions(QLocale.NumberOption.RejectGroupSeparator)
    validator.setLocale(locale)
    return validator


class HMIInput(QLineEdit):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.editing = False
        self.setReadOnly(True)
        self.setProperty('editing', False)

    def start_editing(self):
        self.editing = True
        self.setReadOnly(False)
        self.setProperty('editing', True)
        self.selectAll()
        self.update_style()

    def stop_editing(self):
        self.editing = False
        self.setReadOnly(True)
        self.setProperty('editing', False)
        self.deselect()
        self.update_style()

    def focusOutEvent(self, event):
        # Al navegar a otro control, la caja vuelve a ser solo lectura.
        if self.editing:
            self.stop_editing()
        super().focusOutEvent(event)

    def keyPressEvent(self, event):
        # Aunque un evento escape al filtro global, no se permite modificar
        # el numero hasta entrar al modo de edicion con Enter.
        if not self.editing:
            event.ignore()
            return
        super().keyPressEvent(event)

    def toggle_editing(self):
        if self.editing:
            self.stop_editing()
        else:
            self.start_editing()

    def update_style(self):
        self.style().unpolish(self)
        self.style().polish(self)
        self.update()

class CRTDisplay(QLabel):

    def __init__(self, source_widget, parent=None):
        super().__init__(parent)
        self.source_widget = source_widget
        self.distortion = -0.05
        self.vignette_strength = 0.75
        self.scanline_spacing = 4
        self.scanline_brightness = 0.72
        # CRT animation: drifting horizontal scanlines and soft moving glow.
        self.scanline_offset = 0
        self.scanline_position = 0.0
        self.scanline_speed = 4.0       # pixels per frame (~80 px/s at 20 FPS)
        self.scanline_height = 12.0     # glow half-width in pixels
        self.scanline_intensity = 0.12  # maximum added brightness
        self.flicker_enabled = True
        self.map_x = None
        self.map_y = None
        self.last_size = None
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setStyleSheet('background-color: black;')
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_crt)
        self.timer.start(50)

    def create_distortion_map(self, width, height):
        x, y = np.meshgrid(np.linspace(-1.0, 1.0, width), np.linspace(-1.0, 1.0, height))
        r2 = x * x + y * y
        factor = 1.0 + self.distortion * r2
        source_x = x / factor
        source_y = y / factor
        self.map_x = ((source_x + 1.0) * 0.5 * (width - 1)).astype(np.float32)
        self.map_y = ((source_y + 1.0) * 0.5 * (height - 1)).astype(np.float32)

    def update_crt(self):
        if not self.source_widget.isVisible():
            return
        width = self.source_widget.width()
        height = self.source_widget.height()
        if width <= 1 or height <= 1:
            return
        pixmap = self.source_widget.grab()
        image = pixmap.toImage().convertToFormat(QImage.Format.Format_RGBA8888)
        width = image.width()
        height = image.height()
        if width == 0 or height == 0:
            return
        bytes_per_line = image.bytesPerLine()
        buffer = np.frombuffer(image.bits(), dtype=np.uint8, count=image.sizeInBytes())
        frame = buffer.reshape((height, bytes_per_line // 4, 4))
        frame = frame[:, :width, :].copy()
        current_size = (width, height)
        if self.last_size != current_size:
            self.create_distortion_map(width, height)
            self.last_size = current_size
        frame = cv2.remap(frame, self.map_x, self.map_y, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0, 255))
        mask = np.zeros((height, width), dtype=np.uint8)
        margin_x = int(width * 0.025)
        margin_y = int(height * 0.035)
        radius = int(min(width, height) * 0.07)
        cv2.rectangle(mask, (margin_x + radius, margin_y), (width - margin_x - radius, height - margin_y), 255, -1)
        cv2.rectangle(mask, (margin_x, margin_y + radius), (width - margin_x, height - margin_y - radius), 255, -1)
        corners = [(margin_x + radius, margin_y + radius), (width - margin_x - radius, margin_y + radius), (margin_x + radius, height - margin_y - radius), (width - margin_x - radius, height - margin_y - radius)]
        for center in corners:
            cv2.circle(mask, center, radius, 255, -1)
        mask_float = mask.astype(np.float32) / 255.0
        frame[:, :, :3] = (frame[:, :, :3] * mask_float[:, :, None]).astype(np.uint8)
        frame[:, :, 3] = 255
        xv = np.linspace(-1.0, 1.0, width)
        yv = np.linspace(-1.0, 1.0, height)
        xx, yy = np.meshgrid(xv, yv)
        radial_distance = np.sqrt(xx ** 2 + yy ** 2)
        vignette = 1.0 - self.vignette_strength * np.clip(radial_distance - 0.3, 0.0, 1.0)
        vignette = np.clip(vignette, 0.25, 1.0)
        frame[:, :, :3] = (frame[:, :, :3] * vignette[:, :, None]).astype(np.uint8)
        # Moving dark scanlines: shift the raster phase one pixel per frame.
        self.scanline_offset = (self.scanline_offset + 1) % max(1, self.scanline_spacing)
        for y in range(self.scanline_offset, height, self.scanline_spacing):
            frame[y:y + 1, :, :3] = (
                frame[y:y + 1, :, :3].astype(np.float32) * self.scanline_brightness
            ).astype(np.uint8)

        # Soft luminous scanline, continuously scanning top to bottom.
        # Vectorized per-row brightness avoids costly per-pixel Python loops.
        self.scanline_position = (self.scanline_position + self.scanline_speed) % height
        row_index = np.arange(height, dtype=np.float32)
        distance = np.abs(row_index - self.scanline_position)
        # Smoothly wrap the glow around the top/bottom edge.
        distance = np.minimum(distance, height - distance)
        glow = np.clip(1.0 - distance / max(1.0, self.scanline_height), 0.0, 1.0)
        row_gain = 1.0 + glow * self.scanline_intensity
        frame[:, :, :3] = np.clip(
            frame[:, :, :3].astype(np.float32) * row_gain[:, None, None], 0, 255
        ).astype(np.uint8)

        frame[:, :, 0] = (frame[:, :, 0] * 0.85).astype(np.uint8)
        frame[:, :, 1] = np.clip(frame[:, :, 1] * 1.08, 0, 255).astype(np.uint8)
        frame[:, :, 2] = np.clip(frame[:, :, 2] * 1.03, 0, 255).astype(np.uint8)
        if self.flicker_enabled:
            brightness = random.uniform(0.97, 1.015)
            frame[:, :, :3] = np.clip(frame[:, :, :3].astype(np.float32) * brightness, 0, 255).astype(np.uint8)
        frame = np.ascontiguousarray(frame)
        output_image = QImage(frame.data, width, height, frame.strides[0], QImage.Format.Format_RGBA8888).copy()
        output_pixmap = QPixmap.fromImage(output_image)
        output_pixmap = output_pixmap.scaled(self.size(), Qt.AspectRatioMode.IgnoreAspectRatio, Qt.TransformationMode.SmoothTransformation)
        self.setPixmap(output_pixmap)


def find_esp32_serial_port():
    """Detecta el USB de la ESP32; HMI_SERIAL_PORT tiene prioridad."""
    override = os.environ.get('HMI_SERIAL_PORT', '').strip()
    if override:
        print(f'[SERIAL] Puerto configurado: {override}')
        return override

    # Primero buscar por descriptores USB conocidos del ESP32 o bridges
    # CH340/CP210x/FTDI. Si hay varios, se puede especificar el puerto.
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    keywords = ('esp32', 'silicon labs', 'cp210', 'ch340', 'ch341',
                'wch', 'usb serial', 'usb-serial', 'ftdi')
    for port in ports:
        descriptor = ' '.join(str(part or '') for part in
                              (port.description, port.manufacturer, port.product)).lower()
        if any(word in descriptor for word in keywords):
            print(f'[SERIAL] Detectado: {port.device} ({port.description})')
            return port.device

    # Alternativa para dispositivos USB sin descriptor identificable.
    candidates = [port.device for port in ports
                  if port.device.startswith(('/dev/ttyUSB', '/dev/ttyACM'))]
    if len(candidates) == 1:
        print(f'[SERIAL] Puerto USB único: {candidates[0]}')
        return candidates[0]
    if len(candidates) > 1:
        print(f'[SERIAL] Varios puertos disponibles: {candidates}. '
              'Define HMI_SERIAL_PORT para elegir la ESP32.')
    else:
        print('[SERIAL] No se detectó ESP32 USB. '
              'Puedes definir HMI_SERIAL_PORT para elegir el puerto.')
    return None

class PendantFrameDecoder:
    """Framing 0xA5 + 8 payload bytes; CRC8 over first seven bytes."""
    SYNC = 0xA5

    def __init__(self):
        self.buffer = bytearray()

    @staticmethod
    def crc8(data):
        crc = 0
        for byte in data:
            crc ^= byte
            for _ in range(8):
                crc = (((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)) & 0xFF
        return crc

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 9:
            sync_at = self.buffer.find(self.SYNC)
            if sync_at < 0:
                self.buffer.clear()
                break
            if sync_at:
                del self.buffer[:sync_at]
            if len(self.buffer) < 9:
                break
            payload = self.buffer[1:9]
            if self.crc8(payload[:7]) != payload[7]:
                del self.buffer[0]  # Resync even if payload contains 0xA5.
                continue
            del self.buffer[:9]
            x = int.from_bytes(payload[3:5], 'little', signed=True)
            y = int.from_bytes(payload[5:7], 'little', signed=True)
            joint_x = payload[2] >> 4
            joint_y = payload[2] & 0x0F
            if not (-1000 <= x <= 1000 and -1000 <= y <= 1000):
                continue
            if joint_x not in (0, 4, 5) or joint_y not in (0, 1, 2, 3):
                continue
            frames.append((payload[0], bool(payload[1] & 1),
                           bool(payload[1] & 2), joint_x, joint_y, x, y))
        if len(self.buffer) > 128:
            self.buffer = self.buffer[-8:]
        return frames


class SerialReceiver(QThread):
    freedrive_received = Signal(bool)
    joint_selected = Signal(int)
    joystick_received = Signal(int, int, int, int, bool)
    connection_status = Signal(str)
    gripper_received = Signal(bool)
    button_received = Signal(int)
    konami_ack = Signal(bool)
    konami_input = Signal(str)

    def __init__(self, port=None, baudrate=115200, parent=None):
        super().__init__(parent)
        self.port = port
        self.baudrate = baudrate
        self.running = True
        self.serial_port = None
        self._command_lock = __import__('threading').Lock()
        self._commands = []

    def send_command(self, command):
        with self._command_lock:
            self._commands.append(command)

    def run(self):
        if not self.port:
            self.connection_status.emit('ESP32 no detectada: HMI sin conexion Serial')
            return
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.connection_status.emit(f'ESP32 connected: {self.port}')
        except serial.SerialException as error:
            self.connection_status.emit(f'ESP32 serial error: {error}')
            return
        decoder = PendantFrameDecoder()
        last_manual = last_gripper = last_x = last_y = None
        last_seq = None
        konami_requested = False
        konami_confirmed = False
        text_buffer = bytearray()
        while self.running:
            try:
                with self._command_lock:
                    commands, self._commands = self._commands, []
                for command in commands:
                    if command == 'KONAMI,1':
                        konami_requested = True
                        konami_confirmed = False
                        text_buffer.clear()
                    elif command == 'KONAMI,0':
                        konami_requested = False
                        konami_confirmed = False
                        text_buffer.clear()
                        decoder = PendantFrameDecoder()
                        last_seq = None
                    self.serial_port.write((command + '\n').encode('ascii'))
                data = self.serial_port.read(128)
                if not data:
                    continue
                if konami_requested:
                    # Se descarta cualquier residuo de la trama binaria al cambiar de modo.
                    # Las respuestas de Konami son texto ASCII delimitado por newline.
                    for byte in data:
                        if byte == 10:
                            line = bytes(text_buffer).decode('ascii', errors='ignore').strip()
                            text_buffer.clear()
                            if line == 'KONAMI,1':
                                konami_confirmed = True
                                self.konami_ack.emit(True)
                            elif line == 'KONAMI,DENIED':
                                self.konami_ack.emit(False)
                            elif line.startswith('K,') and konami_confirmed:
                                token = line[2:]
                                if token in ('LEFT', 'RIGHT', 'UP', 'DOWN', 'CENTER', 'JUMP'):
                                    self.konami_input.emit(token)
                        elif 32 <= byte <= 126:
                            if len(text_buffer) < 64:
                                text_buffer.append(byte)
                            else:
                                text_buffer.clear()
                    continue
                for seq, manual, gripper, jx, jy, x, y in decoder.feed(data):
                    # Ignore repeated sequence numbers, but tolerate wraparound.
                    if seq == last_seq:
                        continue
                    last_seq = seq
                    if manual != last_manual:
                        self.freedrive_received.emit(manual)
                        last_manual = manual
                    if gripper != last_gripper:
                        self.gripper_received.emit(gripper)
                        last_gripper = gripper
                    if jx != last_x and jx:
                        self.joint_selected.emit(jx)
                    if jy != last_y and jy:
                        self.joint_selected.emit(jy)
                        # Binary format does not include button events.
                        # Infer J1/J2 from selection changes only.
                        if jy in (1, 2) and last_y is not None:
                            self.button_received.emit(jy)
                    last_x, last_y = jx, jy
                    self.joystick_received.emit(jx, jy, x, y, manual)
            except (serial.SerialException, OSError) as error:
                self.connection_status.emit(f'ESP32 serial error: {error}')
                break
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except serial.SerialException:
                pass

    def stop(self):
        self.running = False
        self.wait(1000)

class GripperIndicator(QWidget):
    """Small animated, non-interactive two-finger gripper diagram."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(116, 72)
        self.closed = False
        self.opening = 1.0
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.animate)

    def set_closed(self, closed):
        self.closed = bool(closed)
        if not self.timer.isActive():
            self.timer.start(25)

    def animate(self):
        goal = 0.0 if self.closed else 1.0
        delta = goal - self.opening
        if abs(delta) <= 0.025:
            self.opening = goal
            self.timer.stop()
        else:
            self.opening += max(-0.13, min(0.13, delta))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        green = QColor('#65ff72')
        painter.setPen(QPen(green, 3))
        cx, cy = (self.width() / 2, 29)
        gap = 9 + 23 * self.opening
        painter.drawRect(int(cx - 11), 2, 22, 18)
        painter.drawLine(QPointF(cx, 20), QPointF(cx, 30))
        for sign in (-1, 1):
            x = cx + sign * gap
            painter.drawLine(QPointF(x, 28), QPointF(x, 62))
            painter.drawLine(QPointF(x, 62), QPointF(x - sign * 12, 62))
        painter.end()

class JointControl(QWidget):
    """Synchronised robot feedback and waypoint input, 800x480 friendly."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.freedrive_enabled = False
        self.gripper_enabled = False
        self.selected_joint_y = 1
        self.selected_joint_x = 4
        self.joint_inputs = []
        self.joint_sliders = []
        self.joint_value_labels = []
        self.joint_selector_labels = {}
        self.current_positions = [180.0] * 5

        root = QVBoxLayout(self)
        root.setContentsMargins(2, 0, 2, 0)
        root.setSpacing(5)

        # Normal mode: editable fields on the left, live read-only sliders on right.
        self.input_widget = QWidget()
        normal_row = QHBoxLayout(self.input_widget)
        normal_row.setContentsMargins(0, 0, 0, 0)
        normal_row.setSpacing(6)

        entry_group = QGroupBox('TARGET JOINT ANGLES')
        entry_layout = QFormLayout(entry_group)
        entry_layout.setContentsMargins(7, 18, 7, 5)
        entry_layout.setVerticalSpacing(5)
        for i in range(5):
            field = HMIInput()
            field.setText('180.0')
            field.setValidator(numeric_validator(0.0, 360.0, 2, field))
            field.setFixedHeight(34)
            self.joint_inputs.append(field)
            entry_layout.addRow(f'J{i + 1}:', field)
        normal_row.addWidget(entry_group, 1)
        normal_row.addWidget(self.build_position_panel(self, normal=True), 1)
        root.addWidget(self.input_widget, 1)

        # Freedrive: selectors and gripper LEFT; position sliders RIGHT.
        self.slider_widget = QWidget()
        free_row = QHBoxLayout(self.slider_widget)
        free_row.setContentsMargins(0, 0, 0, 0)
        free_row.setSpacing(6)

        left = QGroupBox('JOINT SELECTION / GRIPPER')
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(7, 15, 7, 5)
        left_layout.setSpacing(3)

        joint_columns = QHBoxLayout()
        joint_columns.setSpacing(16)
        for title, ids in (('Y', (1, 2, 3)), ('X', (4, 5))):
            col = QVBoxLayout()
            col.setSpacing(3)
            col.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignHCenter)
            title_label = QLabel(title)
            title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            title_label.setFixedHeight(16)
            col.addWidget(title_label)
            for jid in ids:
                indicator = QLabel(f'J{jid}')
                indicator.setAlignment(Qt.AlignmentFlag.AlignCenter)
                indicator.setFixedSize(42, 28)
                col.addWidget(indicator, alignment=Qt.AlignmentFlag.AlignCenter)
                self.joint_selector_labels[jid] = indicator
            joint_columns.addLayout(col, 1)
        left_layout.addLayout(joint_columns)
        left_layout.addStretch(1)

        # Dedicated gripper row BELOW the joint indicators; never overlaps.
        gripper_line = QFrame()
        gripper_line.setFrameShape(QFrame.Shape.HLine)
        gripper_line.setStyleSheet('color: #47d85a;')
        left_layout.addWidget(gripper_line)
        gripper_title = QLabel('GRIPPER')
        gripper_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        gripper_title.setFixedHeight(18)
        left_layout.addWidget(gripper_title)
        gripper_row = QHBoxLayout()
        gripper_row.setSpacing(5)
        self.gripper_indicator = GripperIndicator()
        self.gripper_indicator.setFixedSize(87, 46)
        self.gripper_status = QLabel('OPEN')
        self.gripper_status.setStyleSheet('font-size: 12px; font-weight: bold;')
        self.gripper_status.setAlignment(Qt.AlignmentFlag.AlignVCenter)
        gripper_row.addStretch(1)
        gripper_row.addWidget(self.gripper_indicator)
        gripper_row.addWidget(self.gripper_status)
        gripper_row.addStretch(1)
        left_layout.addLayout(gripper_row)
        free_row.addWidget(left, 1)
        free_row.addWidget(self.build_position_panel(self, normal=False), 1)
        root.addWidget(self.slider_widget, 1)
        self.slider_widget.hide()

        self.freedrive_button = QPushButton('FREEDRIVE: OFF')
        self.freedrive_button.setEnabled(False)
        self.freedrive_button.setFixedHeight(34)
        root.addWidget(self.freedrive_button)
        self.update_joint_selector()

    def build_position_panel(self, parent, normal=False):
        group = QGroupBox('CURRENT POSITIONS')
        col = QVBoxLayout(group)
        col.setContentsMargins(6, 15, 6, 5)
        col.setSpacing(4)
        sliders = []
        labels = []
        for i in range(5):
            row = QHBoxLayout()
            row.setSpacing(3)
            name = QLabel(f'J{i + 1}')
            name.setFixedWidth(22)
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(0, 360)
            slider.setValue(180)
            slider.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            slider.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
            slider.setFixedHeight(24)
            value = QLabel('180.0°')
            value.setFixedWidth(55)
            value.setStyleSheet('font-size: 12px;')
            row.addWidget(name)
            row.addWidget(slider, 1)
            row.addWidget(value)
            col.addLayout(row)
            sliders.append(slider)
            labels.append(value)
        col.addStretch(1)
        if normal:
            self.normal_sliders = sliders
            self.normal_value_labels = labels
        else:
            self.joint_sliders = sliders
            self.joint_value_labels = labels
        return group

    def set_freedrive(self, enabled):
        self.freedrive_enabled = bool(enabled)
        self.input_widget.setVisible(not enabled)
        self.slider_widget.setVisible(bool(enabled))
        self.freedrive_button.setText('FREEDRIVE: ON' if enabled else 'FREEDRIVE: OFF')
        if not enabled:
            self.sync_input_fields(self.current_positions)

    def toggle_freedrive(self):
        self.set_freedrive(not self.freedrive_enabled)

    def set_selected_joint(self, joint_id):
        if 1 <= joint_id <= 3:
            self.selected_joint_y = joint_id
        elif 4 <= joint_id <= 5:
            self.selected_joint_x = joint_id
        else:
            return
        self.update_joint_selector()

    def update_joint_selector(self):
        for jid, label in self.joint_selector_labels.items():
            active = jid in (self.selected_joint_y, self.selected_joint_x)
            label.setStyleSheet(
                'background:#65ff72;color:#000;border:2px solid #d0ffd5;'
                'font-weight:bold;font-size:14px;'
                if active else
                'background:#020502;color:#65ff72;border:2px solid #47d85a;'
                'font-weight:bold;font-size:14px;'
            )

    def set_gripper(self, enabled):
        self.gripper_enabled = bool(enabled)
        self.gripper_indicator.set_closed(enabled)
        self.gripper_status.setText('CLOSED' if enabled else 'OPEN')

    def sync_input_fields(self, values):
        for field, angle in zip(self.joint_inputs, values):
            # Do not overwrite a deliberate manual edit currently in progress.
            if not field.editing:
                field.setText(f'{angle:.1f}')

    def update_joint_positions(self, values):
        self.current_positions = list(values[:5])
        for i, angle in enumerate(self.current_positions):
            for slider, label in (
                (self.joint_sliders[i], self.joint_value_labels[i]),
                (self.normal_sliders[i], self.normal_value_labels[i]),
            ):
                slider.setValue(int(round(angle)))
                label.setText(f'{angle:.1f}°')
        self.sync_input_fields(self.current_positions)

    def get_joint_values(self):
        values = []
        for field in self.joint_inputs:
            value = field.text().strip()
            if not value:
                raise ValueError('All joints require an angle.')
            values.append(float(value))
        return values

class WaypointDialog(QDialog):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.values = None
        self.setWindowTitle('Add Waypoint')
        self.resize(800, 480)
        self.canvas = QWidget(self)
        self.canvas.setFixedSize(800, 480)
        self.screen = QWidget(self.canvas)
        layout = QVBoxLayout(self.screen)
        layout.setContentsMargins(8, 5, 8, 5)
        layout.setSpacing(3)
        title = QLabel('WAYPOINT POSITION')
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setObjectName('dialogTitle')
        layout.addWidget(title)
        self.joint_control = JointControl()
        layout.addWidget(self.joint_control)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.save_waypoint)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)
        # Same CRT overlay used by the main window, capturing the dialog content.
        dialog_outer = QVBoxLayout(self)
        dialog_outer.setContentsMargins(0, 0, 0, 0)
        dialog_outer.addWidget(self.canvas, 0, Qt.AlignmentFlag.AlignCenter)
        overlay = QStackedLayout(self.canvas)
        overlay.setContentsMargins(0, 0, 0, 0)
        overlay.setStackingMode(QStackedLayout.StackingMode.StackAll)
        self.crt_display = CRTDisplay(self.screen)
        overlay.addWidget(self.screen)
        overlay.addWidget(self.crt_display)
        overlay.setCurrentWidget(self.crt_display)
        self.crt_display.raise_()
        self.joint_control.joint_inputs[0].setFocus()

    def save_waypoint(self):
        # In Freedrive, capture the canonical robot state, not a local copy.
        if self.joint_control.freedrive_enabled:
            robot_hmi = self.parent()
            if robot_hmi is None:
                QMessageBox.warning(self, 'Robot State', 'Global joint state unavailable.')
                return
            self.values = list(robot_hmi.joint_values)
            self.accept()
            return
        try:
            self.values = self.joint_control.get_joint_values()
        except ValueError as error:
            QMessageBox.warning(self, 'Invalid Input', str(error))
            return
        self.accept()

    def get_values(self):
        return self.values

    def showEvent(self, event):
        super().showEvent(event)
        self.crt_display.raise_()
        QTimer.singleShot(80, self.crt_display.update_crt)
        if not getattr(self, '_checked_initial_screen', False):
            self._checked_initial_screen = True
            # El diálogo también ocupa toda la pantalla sin decoraciones.
            QTimer.singleShot(0, self.showFullScreen)

    def _fullscreen_if_small_screen(self):
        screen = self.screen()
        if screen is not None and screen.geometry().size().width() == 800 and screen.geometry().size().height() == 480:
            self.showFullScreen()

    def changeEvent(self, event):
        super().changeEvent(event)
        if event.type() == QEvent.Type.WindowStateChange and self.isMaximized():
            screen = self.screen()
            if screen is not None and screen.geometry().width() == 800 and screen.geometry().height() == 480:
                QTimer.singleShot(0, self.showFullScreen)

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key.Key_F11:
            self.showNormal() if self.isFullScreen() else self.showFullScreen()
            return
        focused = QApplication.focusWidget()
        if isinstance(focused, HMIInput):
            if key in (Qt.Key.Key_Return, Qt.Key.Key_Enter):
                focused.toggle_editing()
                return
            if key == Qt.Key.Key_Escape and focused.editing:
                focused.stop_editing()
                return
            if focused.editing:
                super().keyPressEvent(event)
                return
        if key == Qt.Key.Key_Down:
            self.focusNextChild()
            return
        if key == Qt.Key.Key_Up:
            self.focusPreviousChild()
            return
        if key == Qt.Key.Key_Escape:
            self.reject()
            return
        if key in (Qt.Key.Key_Return, Qt.Key.Key_Enter):
            if isinstance(focused, QPushButton):
                focused.click()
                return
        super().keyPressEvent(event)


class CreditsExplorer(QWidget):
    """Escena 2D desplazable dentro del CRT principal, sin motores ni comandos robot."""

    # Cambia estos cuatro nombres para personalizar las etiquetas bajo los retratos.
    CREATOR_NAMES = [
        "DiazPaz",
        "LivingCheerios",
        "kaeeruu",
        "Eugenio Nava",
    ]

    SCREEN_W = 800
    SCREEN_H = 480
    WORLD_W = 2480
    DOOR_X = 2180
    DOOR_W = 120
    DOOR_H = 180
    DOOR_FPS = 12
    DOOR_LOOPS = 9  # Despues de reproducir frames_door_open una vez
    PLAYER_W = 100
    PLAYER_H = 100
    FLOOR_Y = 393
    WALK_SPEED = 270.0          # px/s (doble de v36)
    RUN_SPEED = 540.0           # px/s (mantiene carrera a 2x caminar)
    CROUCH_WALK_SPEED = 135.0   # px/s, movimiento agachado
    DOUBLE_TAP_SECONDS = 0.32  # Segundo toque de la misma flecha activa carrera
    JUMP_SPEED = -780.0         # px/s; Y positivo hacia abajo
    GRAVITY = 1200.0             # px/s^2, salto 2.5 veces mas alto que v38
    FAST_FALL_GRAVITY = 3200.0  # px/s^2, caida acelerada al mantener ABAJO
    FAST_FALL_MIN_SPEED = 600.0 # px/s, hace que la bajada empiece inmediatamente
    LANDING_SHAKE_SECONDS = 0.22  # Vibracion breve al aterrizar
    LANDING_SHAKE_PIXELS = 5.0   # Desplazamiento maximo en pantalla
    GREEN = QColor('#65ff72')

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(self.SCREEN_W, self.SCREEN_H)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.keys = set()
        self.project_dir = Path(__file__).resolve().parent / 'assets'
        self.animations = {
            'idle': self.load_animation('frames_idle', 7),
            'walk': self.load_animation('frames_walk', 6),
            'jump': self.load_animation('frames_jump', 1),
            'crouch': self.load_animation('frames_crouch', 1),
            'crouch_walk': self.load_animation('frames_crouch_walk', 6),
            'lookup': self.load_animation('frames_lookup', 1),
            'gotosleep': self.load_animation('frames_gotosleep', 6),
            'sleep': self.load_animation('frames_sleep', 10),
        }
        self.creator_images = self.load_creator_images()
        self.door_image = self.load_door_image()
        self.door_open_animation = self.load_door_frames("frames_door_open")
        self.door_animation = self.load_door_frames("frames_door")
        self.on_door_complete = None
        self.timer = QTimer(self)
        self.timer.setInterval(16)
        self.timer.timeout.connect(self.tick)
        self.clock = QElapsedTimer()
        self.reset_scene()

    def load_animation(self, folder, count):
        """Invierte luminosidad: negro original -> verde, blanco -> negro.
        Conserva alpha y textura, como la intro del HMI.
        """
        result = []
        for number in range(1, count + 1):
            path = self.project_dir / folder / f'{number}.png'
            raw = QImage(str(path))
            if raw.isNull():
                print(f'[CREDITS] Sprite no encontrado: {path}')
                result.append(QPixmap())
                continue
            raw = raw.convertToFormat(QImage.Format.Format_RGBA8888)
            w, h = raw.width(), raw.height()
            rgba = np.frombuffer(raw.constBits(), dtype=np.uint8).reshape(
                h, raw.bytesPerLine())[:, :w * 4].reshape(h, w, 4).copy()
            rgb = rgba[:, :, :3].astype(np.uint16)
            intensity = 255 - ((77 * rgb[:, :, 0] + 150 * rgb[:, :, 1] + 29 * rgb[:, :, 2]) >> 8)
            green = np.array([101, 255, 114], dtype=np.uint16)
            rgba[:, :, :3] = ((intensity[:, :, None] * green) // 255).astype(np.uint8)
            img = QImage(rgba.data, w, h, rgba.strides[0], QImage.Format.Format_RGBA8888).copy()
            result.append(QPixmap.fromImage(img).scaled(
                self.PLAYER_W, self.PLAYER_H, Qt.AspectRatioMode.IgnoreAspectRatio,
                Qt.TransformationMode.FastTransformation))
        return result

    def load_creator_images(self):
        """Carga los cuatro retratos con filtro verde sin invertir luminosidad."""
        found = []
        for number in (1, 2, 3, 4):
            path = self.project_dir / 'portraits' / f'{number}.png'
            found.append(self.load_green_pixelart(path, 90, 120))
        return found

    def load_green_pixelart(self, path, width, height):
        """Convierte luminosidad directamente a verde (sin invertir), conservando alpha."""
        raw = QImage(str(path))
        if raw.isNull():
            return QPixmap()
        raw = raw.convertToFormat(QImage.Format.Format_RGBA8888)
        w, h = raw.width(), raw.height()
        rgba = np.frombuffer(raw.constBits(), dtype=np.uint8).reshape(
            h, raw.bytesPerLine())[:, :w * 4].reshape(h, w, 4).copy()
        rgb = rgba[:, :, :3].astype(np.uint16)
        # Puerta: blanco original -> verde; negro original -> negro.
        # No invertir, a diferencia de los sprites del personaje.
        intensity = ((77 * rgb[:, :, 0] + 150 * rgb[:, :, 1] + 29 * rgb[:, :, 2]) >> 8)
        green = np.array([101, 255, 114], dtype=np.uint16)
        rgba[:, :, :3] = ((intensity[:, :, None] * green) // 255).astype(np.uint8)
        img = QImage(rgba.data, w, h, rgba.strides[0], QImage.Format.Format_RGBA8888).copy()
        return QPixmap.fromImage(img).scaled(
            width, height,
            Qt.AspectRatioMode.IgnoreAspectRatio,
            Qt.TransformationMode.FastTransformation)

    def load_door_image(self):
        """Busca la imagen de la puerta en door/ (prioriza 1.png) y le aplica el filtro verde."""
        folder = self.project_dir / 'door'
        choices = [folder / '1.png'] + sorted(p for p in folder.glob('*.png') if p.name != '1.png')
        for path in choices:
            image = self.load_green_pixelart(path, self.DOOR_W, self.DOOR_H)
            if not image.isNull():
                return image
        print(f'[CREDITS] No se encontro imagen de puerta en {folder}')
        return QPixmap()

    def load_door_frames(self, folder):
        """Carga cuatro frames de la puerta, les aplica el filtro verde y los escala 3x."""
        images = []
        for n in range(1, 5):
            path = self.project_dir / folder / f'{n}.png'
            img = self.load_green_pixelart(path, self.DOOR_W, self.DOOR_H)
            if img.isNull():
                print(f'[CREDITS] Falta {folder}/{n}.png')
            images.append(img)
        return images

    def reset_scene(self):
        self.keys.clear()
        self.mode = "play"
        self.door_elapsed = 0.0
        self.door_start_x = 0.0
        self.last_direction_press = {}
        self.running_direction = None
        self.jump_held = set()
        self.x = 60.0
        self.y = float(self.FLOOR_Y - self.PLAYER_H)
        self.vy = 0.0
        self.grounded = True
        self.shake_remaining = 0.0
        self.facing_right = True
        self.looking_up = False
        self.animation_elapsed = 0.0
        self.inactive_elapsed = 0.0
        self.sleep_phase = None  # None, 'gotosleep' o 'sleep'
        self.sleep_elapsed = 0.0
        self.camera_x = 0.0
        self.clock.invalidate()
        self.update()

    def begin(self):
        self.timer.stop()
        self.reset_scene()
        self.clock.start()
        self.timer.start()

    def end(self):
        self.timer.stop()
        self.keys.clear()
        self.running_direction = None
        self.jump_held.clear()

    def near_door(self):
        player_center = self.x + self.PLAYER_W / 2
        door_center = self.DOOR_X + self.DOOR_W / 2
        return self.grounded and abs(player_center - door_center) <= 85

    def enter_door(self):
        if self.mode != 'play' or not self.near_door():
            return
        self.mode = 'door'
        self.door_elapsed = 0.0
        self.door_start_x = self.x
        self.animation_elapsed = 0.0
        self.sleep_phase = None
        self.looking_up = False
        self.vy = 0.0
        self.keys.clear()
        self.running_direction = None
        self.jump_held.clear()
        self.update()

    def wake_up(self):
        """Cancela el sueño inmediatamente al recibir una acción del jugador."""
        if self.sleep_phase is not None:
            self.sleep_phase = None
            self.sleep_elapsed = 0.0
            self.animation_elapsed = 0.0
        self.inactive_elapsed = 0.0

    def press(self, key):
        """Se invoca solo al PRESIONAR una tecla física, no con auto-repeat.

        El movimiento se mantiene mediante self.keys hasta recibir KeyRelease.
        """
        if self.mode not in ('play', 'door'):
            return
        if key in (Qt.Key.Key_Left, Qt.Key.Key_Right, Qt.Key.Key_Up,
                   Qt.Key.Key_Down, Qt.Key.Key_Space):
            self.wake_up()
        if key in (Qt.Key.Key_Return, Qt.Key.Key_Enter):
            if self.mode == 'play':
                self.enter_door()
            return
        if key == Qt.Key.Key_R and self.mode == 'play':
            self.reset_scene()
            self.clock.restart()
            return

        if key in (Qt.Key.Key_Left, Qt.Key.Key_Right):
            if key not in self.keys:
                now = time.monotonic()
                last = self.last_direction_press.get(key, float('-inf'))
                if now - last <= self.DOUBLE_TAP_SECONDS:
                    self.running_direction = key
                else:
                    self.running_direction = None
                self.last_direction_press[key] = now
            # Los dos sentidos no deben competir entre sí.
            opposite = Qt.Key.Key_Left if key == Qt.Key.Key_Right else Qt.Key.Key_Right
            self.keys.discard(opposite)

        self.keys.add(key)
        if self.mode == 'play' and key in (Qt.Key.Key_Up, Qt.Key.Key_Space, Qt.Key.Key_Return, Qt.Key.Key_Enter):
            if self.grounded and key not in self.jump_held:
                self.vy = self.JUMP_SPEED
                self.grounded = False
                self.animation_elapsed = 0.0
            self.jump_held.add(key)

    def release(self, key):
        self.keys.discard(key)
        self.jump_held.discard(key)
        if key == self.running_direction:
            self.running_direction = None

    def clear_input(self):
        self.keys.clear()
        self.jump_held.clear()
        self.running_direction = None

    def tick(self):
        dt = min(self.clock.restart() / 1000.0, 0.05) if self.clock.isValid() else 0.016
        if self.mode == 'door':
            self.door_elapsed += dt
            open_seconds = len(self.door_open_animation) / self.DOOR_FPS
            loops_seconds = self.DOOR_LOOPS * len(self.door_animation) / self.DOOR_FPS
            total = open_seconds + loops_seconds
            progress = min(1.0, self.door_elapsed / total)
            target = self.DOOR_X + (self.DOOR_W - self.PLAYER_W) / 2
            smooth = progress * progress * (3.0 - 2.0 * progress)
            self.x = self.door_start_x + (target - self.door_start_x) * smooth
            # El desplazamiento hacia la puerta sigue siendo automatico;
            # las entradas solo cambian los fotogramas del personaje.
            self.animation_elapsed += dt
            if target != self.door_start_x:
                self.facing_right = target > self.door_start_x
            self.y = float(self.FLOOR_Y - self.PLAYER_H)
            self.camera_x = max(0.0, min(float(self.WORLD_W - self.SCREEN_W), self.x - 280.0))
            if progress >= 1.0:
                self.mode = 'black'
                self.black_elapsed = 0.0
            self.update()
            return
        if self.mode == 'black':
            self.black_elapsed += dt
            if self.black_elapsed >= 0.35:
                self.mode = 'complete'
                self.timer.stop()
                if self.on_door_complete:
                    QTimer.singleShot(0, self.on_door_complete)
            self.update()
            return
        if self.mode != 'play':
            return
        # Inactividad: dormir despues de 10 s sin acciones ni movimiento.
        active_input = bool(self.keys.intersection((
            Qt.Key.Key_Left, Qt.Key.Key_Right, Qt.Key.Key_Up,
            Qt.Key.Key_Down, Qt.Key.Key_Space))) or self.looking_up
        if active_input or not self.grounded:
            self.wake_up()
        else:
            self.inactive_elapsed += dt
            if self.sleep_phase is None and self.inactive_elapsed >= 10.0:
                self.sleep_phase = 'gotosleep'
                self.sleep_elapsed = 0.0
                self.animation_elapsed = 0.0
            elif self.sleep_phase is not None:
                self.sleep_elapsed += dt
                if (self.sleep_phase == 'gotosleep' and
                        self.sleep_elapsed >= len(self.animations['gotosleep']) / 12.0):
                    self.sleep_phase = 'sleep'
                    self.sleep_elapsed = 0.0
        # Reducir progresivamente el temblor de aterrizaje.
        self.shake_remaining = max(0.0, self.shake_remaining - dt)
        direction = int(Qt.Key.Key_Right in self.keys) - int(Qt.Key.Key_Left in self.keys)
        crouching = Qt.Key.Key_Down in self.keys and self.grounded
        moving_key = Qt.Key.Key_Right if direction > 0 else Qt.Key.Key_Left
        running = direction != 0 and self.running_direction == moving_key
        # Movimiento horizontal independiente del salto: permite mantener
        # derecha/izquierda antes O despues de comenzar a saltar.
        speed = (self.CROUCH_WALK_SPEED if crouching else
                 self.RUN_SPEED if running else self.WALK_SPEED)
        if direction:
            self.facing_right = direction > 0
        self.x = max(0.0, min(float(self.WORLD_W - self.PLAYER_W), self.x + direction * speed * dt))
        if not self.grounded:
            fast_fall = Qt.Key.Key_Down in self.keys
            if fast_fall:
                # La tecla ABAJO cancela el ascenso y fuerza una caida rapida.
                self.vy = max(self.vy, self.FAST_FALL_MIN_SPEED)
                self.vy += self.FAST_FALL_GRAVITY * dt
            else:
                self.vy += self.GRAVITY * dt
            self.y += self.vy * dt
            floor_position = self.FLOOR_Y - self.PLAYER_H
            if self.y >= floor_position:
                # Registrar el impacto antes de detener la velocidad vertical.
                impact_speed = self.vy
                self.y = float(floor_position)
                self.vy = 0.0
                self.grounded = True
                if impact_speed > 0 and fast_fall:
                    self.shake_remaining = self.LANDING_SHAKE_SECONDS
        self.camera_x = max(0.0, min(float(self.WORLD_W - self.SCREEN_W), self.x - 280.0))
        self.animation_elapsed += dt
        self.update()

    def current_sprite(self):
        if self.mode == 'door':
            # Durante la absorcion, se permite cambiar la animacion sin
            # alterar la trayectoria automatica hacia el centro de la puerta.
            direction = int(Qt.Key.Key_Right in self.keys) - int(Qt.Key.Key_Left in self.keys)
            moving_key = Qt.Key.Key_Right if direction > 0 else Qt.Key.Key_Left
            running = direction != 0 and self.running_direction == moving_key
            crouching = Qt.Key.Key_Down in self.keys
            if crouching and direction:
                frames, fps = self.animations['crouch_walk'], 12
            elif crouching:
                frames, fps = self.animations['crouch'], 1
            elif self.looking_up or Qt.Key.Key_Up in self.keys:
                frames, fps = self.animations['lookup'], 1
            elif direction:
                # Animar caminata/carrera solo mientras se mantiene una direccion.
                frames, fps = self.animations['walk'], 24 if running else 12
            else:
                # La puerta sigue atrayendo al personaje, pero sin entradas
                # de movimiento no se reproduce la animacion de caminar.
                return self.animations['idle'][0]
            return frames[int(self.animation_elapsed * fps) % len(frames)]

        direction = int(Qt.Key.Key_Right in self.keys) - int(Qt.Key.Key_Left in self.keys)
        crouching = Qt.Key.Key_Down in self.keys and self.grounded
        moving_key = Qt.Key.Key_Right if direction > 0 else Qt.Key.Key_Left
        running = direction != 0 and self.running_direction == moving_key
        if self.sleep_phase == 'gotosleep':
            frames, fps = self.animations['gotosleep'], 12
            index = min(len(frames) - 1, int(self.sleep_elapsed * fps))
            return frames[index]
        elif self.sleep_phase == 'sleep':
            frames, fps = self.animations['sleep'], 12
            return frames[int(self.sleep_elapsed * fps) % len(frames)]
        if self.looking_up and self.grounded:
            frames, fps = self.animations['lookup'], 1
        elif not self.grounded and Qt.Key.Key_Down in self.keys:
            frames, fps = self.animations['crouch'], 1
        elif not self.grounded:
            frames, fps = self.animations['jump'], 1
        elif crouching and direction:
            frames, fps = self.animations['crouch_walk'], 12
        elif crouching:
            frames, fps = self.animations['crouch'], 1
        elif direction:
            frames, fps = self.animations['walk'], 24 if running else 12
        else:
            frames, fps = self.animations['idle'], 12
        return frames[int(self.animation_elapsed * fps) % len(frames)]

    def paintEvent(self, event):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor('#020502'))
        if self.mode in ('black', 'complete'):
            p.end()
            return
        # Vibracion de camara: se dibuja toda la escena desplazada unos
        # pixeles; el fondo queda pintado para evitar bordes sin limpiar.
        if self.shake_remaining > 0.0:
            decay = self.shake_remaining / self.LANDING_SHAKE_SECONDS
            amplitude = self.LANDING_SHAKE_PIXELS * decay
            p.translate(random.uniform(-amplitude, amplitude),
                        random.uniform(-amplitude, amplitude))
        p.setPen(self.GREEN)
        offset = self.camera_x
        p.setFont(QFont('Courier New', 19, QFont.Weight.Bold))
        p.drawText(QRect(round(-offset), 118, self.SCREEN_W, 42),
                   Qt.AlignmentFlag.AlignCenter, 'Created by Practicabras')

        # Cuatro retratos de 90x120 (imagen 30x40 ampliada x3).
        # El escenario avanza con la camara hasta el final del mundo.
        p.setFont(QFont('Courier New', 12))
        for idx, world_x in enumerate((990, 1220, 1450, 1680)):
            x = round(world_x - offset)
            if x < -100 or x > 820:
                continue
            box = QRect(x, 160, 90, 120)
            pic = self.creator_images[idx]
            if not pic.isNull():
                p.drawPixmap(box, pic)
            else:
                p.setPen(QPen(self.GREEN, 1))
                p.drawRect(box)
                p.drawLine(x, 160, x + 90, 280)
                p.drawLine(x + 90, 160, x, 280)
            p.setPen(self.GREEN)
            # Etiquetas centradas respecto al retrato, con espacio suficiente
            # hasta el siguiente (230 px) y fuente ajustable para nombres largos.
            label_text = str(self.CREATOR_NAMES[idx])
            label_rect = QRect(x - 60, 288, 210, 44)
            label_font = QFont('Courier New', 12)
            label_font.setBold(True)
            for point_size in range(12, 7, -1):
                label_font.setPointSize(point_size)
                if QFontMetrics(label_font).horizontalAdvance(label_text) <= 202:
                    break
            p.setFont(label_font)
            p.drawText(label_rect,
                       Qt.AlignmentFlag.AlignCenter | Qt.TextFlag.TextWordWrap,
                       label_text)
            p.setFont(QFont('Courier New', 12))

        # Puerta al final del recorrido, despues de los tres retratos.
        door_screen_x = round(self.DOOR_X - offset)
        door_rect = QRect(door_screen_x, self.FLOOR_Y - self.DOOR_H,
                          self.DOOR_W, self.DOOR_H)
        if self.mode == 'door':
            elapsed_frames = int(self.door_elapsed * self.DOOR_FPS)
            open_count = len(self.door_open_animation)
            if elapsed_frames < open_count:
                # Apertura: una sola pasada, 4 frames a 12 FPS.
                door_pic = self.door_open_animation[elapsed_frames]
            else:
                # Portal activo: cuatro frames, nueve ciclos a 12 FPS.
                loop_frame = min(elapsed_frames - open_count,
                                 self.DOOR_LOOPS * len(self.door_animation) - 1)
                door_pic = self.door_animation[loop_frame % len(self.door_animation)]
        else:
            door_pic = self.door_image
        p.setPen(self.GREEN)
        p.setFont(QFont('Courier New', 14, QFont.Weight.Bold))
        p.drawText(QRect(door_screen_x - 30, self.FLOOR_Y - self.DOOR_H - 58,
                         self.DOOR_W + 60, 24), Qt.AlignmentFlag.AlignCenter,
                   'EXIT')
        if not door_pic.isNull():
            p.drawPixmap(door_rect, door_pic)
        else:
            p.setPen(QPen(self.GREEN, 2))
            p.drawRect(door_rect)
            p.drawText(door_rect, Qt.AlignmentFlag.AlignCenter, 'DOOR')
        if self.mode == 'play' and self.near_door():
            p.setPen(self.GREEN)
            p.setFont(QFont('Courier New', 13, QFont.Weight.Bold))
            p.drawText(QRect(door_screen_x - 35, self.FLOOR_Y - self.DOOR_H - 35,
                             self.DOOR_W + 70, 25), Qt.AlignmentFlag.AlignCenter,
                       'ENTER TO OPEN')

        # Suelo visible, con limites invisibles en ambos extremos.
        # La colision horizontal se mantiene en tick() mediante clamp.
        p.setPen(QPen(self.GREEN, 2))
        p.drawLine(0, self.FLOOR_Y, self.SCREEN_W, self.FLOOR_Y)

        sprite = self.current_sprite()
        character_x = round(self.x - offset)
        if not sprite.isNull():
            if not self.facing_right:
                p.save()
                p.translate(character_x + self.PLAYER_W, 0)
                p.scale(-1.0, 1.0)
                p.drawPixmap(0, round(self.y), sprite)
                p.restore()
            else:
                p.drawPixmap(character_x, round(self.y), sprite)
        else:
            p.setPen(QPen(self.GREEN, 2))
            p.drawRect(QRect(character_x, round(self.y), self.PLAYER_W, self.PLAYER_H))

        p.end()


class CRTPowerOnTransition(QWidget):
    """Efecto de apagado/encendido: contraer imagen a una linea y expandir HMI.

    Se pinta dentro de la misma area 800x480 y el CRTDisplay exterior
    sigue aplicando scanlines, distorsion y vignette.
    """

    def __init__(self, intro_pixmap, hmi_pixmap, parent=None):
        super().__init__(parent)
        self.setFixedSize(800, 480)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.intro_pixmap = intro_pixmap
        self.hmi_pixmap = hmi_pixmap
        self.elapsed = QElapsedTimer()
        self.duration_ms = 950
        self.finished_callback = None
        self.timer = QTimer(self)
        self.timer.setInterval(16)
        self.timer.timeout.connect(self.advance)

    def start(self, callback):
        self.finished_callback = callback
        self.elapsed.start()
        self.timer.start()
        self.update()

    def advance(self):
        if self.elapsed.elapsed() >= self.duration_ms:
            self.timer.stop()
            if self.finished_callback:
                callback = self.finished_callback
                self.finished_callback = None
                callback()
            return
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(0, 0, 0))
        t = min(self.elapsed.elapsed() / self.duration_ms, 1.0) if self.elapsed.isValid() else 0.0
        # 0..0.33: la intro se comprime verticalmente
        # 0.33..0.49: linea luminosa se extingue
        # 0.49..0.59: negro completo
        # 0.59..0.90: la imagen del HMI se expande
        # 0.90..1: imagen estable antes de entregar el control
        if t < 0.33:
            f = t / 0.33
            height = max(2, round(480 * (1.0 - f) ** 2.5))
            source = self.intro_pixmap
            opacity = 1.0
        elif t < 0.49:
            height = 2
            source = self.intro_pixmap
            opacity = max(0.0, 1.0 - (t - 0.33) / 0.16)
        elif t < 0.59:
            height = 0
            source = self.hmi_pixmap
            opacity = 0.0
        elif t < 0.90:
            f = (t - 0.59) / 0.31
            height = max(2, round(480 * (1.0 - (1.0 - f) ** 2.5)))
            source = self.hmi_pixmap
            opacity = min(1.0, f * 2.0 + 0.15)
        else:
            height = 480
            source = self.hmi_pixmap
            opacity = 1.0

        if height and source and not source.isNull():
            top = (480 - height) // 2
            painter.setOpacity(opacity)
            painter.drawPixmap(QRect(0, top, 800, height), source)
            painter.setOpacity(1.0)
            # Fulgor verde/blanco en la linea de colapso/apertura
            if height < 120 and opacity > 0:
                glow = min(1.0, opacity * (1.0 - height / 140.0))
                painter.fillRect(QRect(35, 236, 730, 8), QColor(101, 255, 114, int(38 * glow)))
                painter.fillRect(QRect(60, 239, 680, 2), QColor(212, 255, 218, int(210 * glow)))
        painter.end()


class RobotHMI(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle('Robot HMI')
        self.resize(800, 480)
        self._auto_fullscreen_pending = False
        # Secret: arrows, physical J2, physical J1, keyboard ENTER.
        self.pong_sequence = ['UP', 'UP', 'DOWN', 'DOWN', 'LEFT', 'RIGHT', 'LEFT', 'RIGHT', 'J2', 'J1', 'ENTER']
        self.pong_sequence_offline = ['UP', 'UP', 'DOWN', 'DOWN', 'LEFT', 'RIGHT', 'LEFT', 'RIGHT', 'MINUS', 'PLUS', 'ENTER']
        self.secret_using_esp32 = None
        self.pong_progress = 0
        self.pong_last_event = 0.0
        self.pong_active = False
        self.arcade_active = False
        self.konami_pending = False
        self.konami_input_last = time.monotonic()
        self.konami_request_started = 0.0
        self.konami_direction = None
        # Teclas físicas pulsadas: evita conteos múltiples sin alterar la navegación.
        self.pressed_keys = set()
        self.intro_active = True
        self.intro_started = False
        self.waypoints = []
        self.editing_step_index = None  # None means ADD; integer means EDIT
        self.gripper_enabled = False
        self.active_waypoint_dialog = None
        self.joint_values = [180.0, 180.0, 180.0, 180.0, 180.0]
        # Estimacion visual: la trama de pendant NO transporta angulos reales.
        self._jog_last_x = time.monotonic()
        self._jog_last_y = time.monotonic()
        self._jog_last_frame = time.monotonic()
        self._jog_x = 4
        self._jog_y = 1

        root = QWidget()
        self.setCentralWidget(root)
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)
        self.canvas = QWidget(root)
        self.canvas.setFixedSize(800, 480)
        root_layout.addWidget(self.canvas, 0, Qt.AlignmentFlag.AlignCenter)
        self.stack = QStackedLayout(self.canvas)
        self.stack.setStackingMode(QStackedLayout.StackingMode.StackAll)
        self.hmi_widget = QWidget()
        main_layout = QVBoxLayout(self.hmi_widget)
        main_layout.setContentsMargins(8, 6, 8, 5)
        main_layout.setSpacing(3)
        header = QLabel('MOVEMASTER II CONTROL SYSTEM')
        header.setAlignment(Qt.AlignmentFlag.AlignCenter)
        header.setObjectName('header')
        main_layout.addWidget(header)
        self.tabs = QTabWidget()
        self.tabs.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.tabs.tabBar().setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.configuration_tab = self.create_configuration_tab()
        self.program_tab = self.create_program_tab()
        self.movement_tab = self.create_movement_tab()
        self.tabs.addTab(self.configuration_tab, 'CONFIGURATION')
        self.tabs.addTab(self.program_tab, 'PROGRAM')
        self.tabs.addTab(self.movement_tab, 'MOVEMENT')
        main_layout.addWidget(self.tabs)
        keyboard_help = QLabel('[ + / - ] TAB     [ ↑ / ↓ ] SELECT     [ ← / → ] COLUMN     [ ENTER ] EDIT / CONFIRM')
        keyboard_help.setAlignment(Qt.AlignmentFlag.AlignCenter)
        keyboard_help.setObjectName('keyboardHelp')
        main_layout.addWidget(keyboard_help)
        # A page stack inside the existing CRT pipeline: same 800x480 CRT for HMI and Pong.
        self.content_pages = QStackedWidget()
        self.content_pages.addWidget(self.hmi_widget)
        self.goat_game = CreditsExplorer()
        self.goat_game.on_door_complete = self.return_from_door
        self.content_pages.addWidget(self.goat_game)
        # Intro animada: 17 PNG + 10 tiempos extra del ultimo fotograma.
        # Se renderiza a traves del MISMO CRTDisplay que el HMI y Pong.
        self.intro_label = QLabel()
        self.intro_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.intro_label.setFixedSize(800, 480)
        self.intro_label.setStyleSheet('background-color: black;')
        self.intro_label.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.intro_frames = []
        frames_dir = Path(__file__).resolve().parent / 'assets' / 'frames'
        for number in range(1, 18):
            image_path = frames_dir / f'{number}.png'
            image = QImage(str(image_path))
            if image.isNull():
                print(f'[INTRO] No se pudo leer: {image_path}')
                self.intro_frames = []
                break
            # Intro monocromatica verde CRT (#65ff72), como los controles del HMI.
            # Primero se invierte la luminosidad: negro original -> verde brillante,
            # blanco original -> negro. Se conservan degradados y transparencia.
            image = image.convertToFormat(QImage.Format.Format_RGBA8888)
            width, height = image.width(), image.height()
            rgba = np.frombuffer(image.constBits(), dtype=np.uint8).reshape(height, image.bytesPerLine())[:, :width * 4].reshape(height, width, 4).copy()
            rgb = rgba[:, :, :3].astype(np.uint16)
            # Luminancia del negativo de los RGB; valores entre 0 y 255.
            light = 255 - ((77 * rgb[:, :, 0] + 150 * rgb[:, :, 1] + 29 * rgb[:, :, 2]) >> 8)
            green = np.array([0x65, 0xFF, 0x72], dtype=np.uint16)
            rgba[:, :, :3] = ((light[:, :, None] * green[None, None, :]) // 255).astype(np.uint8)
            image = QImage(rgba.data, width, height, rgba.strides[0], QImage.Format.Format_RGBA8888).copy()
            pixmap = QPixmap.fromImage(image)
            self.intro_frames.append(pixmap.scaled(
                800, 480,
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            ))
        self.content_pages.addWidget(self.intro_label)
        self.intro_elapsed = QElapsedTimer()
        self.intro_timer = QTimer(self)
        self.intro_timer.setInterval(15)  # el reloj transcurrido determina los 12 FPS
        self.intro_timer.timeout.connect(self.update_intro)
        if self.intro_frames:
            self.intro_label.setPixmap(self.intro_frames[0])
            self.content_pages.setCurrentWidget(self.intro_label)
        else:
            print('[INTRO] Faltan imagenes. Se mostrara el HMI directamente.')
            self.intro_active = False
            self.content_pages.setCurrentWidget(self.hmi_widget)
        self.crt_display = CRTDisplay(self.content_pages)
        self.stack.addWidget(self.content_pages)
        self.stack.addWidget(self.crt_display)
        self.stack.setCurrentWidget(self.crt_display)
        self.crt_display.raise_()
        QApplication.instance().installEventFilter(self)
        self.speed_input.setFocus()
        self.serial_receiver = SerialReceiver(port=find_esp32_serial_port(), baudrate=115200, parent=self)
        self.serial_receiver.freedrive_received.connect(self.on_freedrive_received)
        self.serial_receiver.joint_selected.connect(self.on_joint_selected)
        self.serial_receiver.joystick_received.connect(self.on_joystick_received)
        self.serial_receiver.gripper_received.connect(self.on_gripper_received)
        self.serial_receiver.button_received.connect(self.on_button_received)
        self.serial_receiver.konami_ack.connect(self.on_konami_ack)
        self.serial_receiver.konami_input.connect(self.on_konami_input)
        self.konami_watchdog = QTimer(self)
        self.konami_watchdog.setInterval(80)
        self.konami_watchdog.timeout.connect(self.check_konami_input)
        self.serial_receiver.connection_status.connect(print)
        self.serial_receiver.start()

    def showEvent(self, event):
        super().showEvent(event)
        if hasattr(self, 'crt_display'):
            self.crt_display.raise_()
            QTimer.singleShot(100, self.crt_display.update_crt)
        if self.intro_active and not self.intro_started:
            self.intro_started = True
            QTimer.singleShot(0, self.start_intro)
        # An 800x480 monitor has no room for desktop window decorations.
        # Enter actual fullscreen the first time this window is displayed there.
        if not getattr(self, '_checked_initial_screen', False):
            self._checked_initial_screen = True
            QTimer.singleShot(0, self._fullscreen_if_small_screen)

    def start_intro(self):
        """Comienza la intro una unica vez, cuando la ventana ya es visible."""
        if not self.intro_active or not self.intro_frames:
            return
        self.intro_elapsed.start()
        self.intro_timer.start()
        self.update_intro()

    def update_intro(self):
        """12 FPS. 1-16 duran 1 frame; 17 dura 11 frames (1+10)."""
        if not self.intro_active:
            return
        # 27 intervalos a 12 FPS = 2.25 s en total.
        # indice 0..16: imagenes 1..17; indice 17..26: imagen 17.
        frame_slot = (self.intro_elapsed.elapsed() * 12) // 1000
        if frame_slot >= 27:
            self.finish_intro()
            return
        image_index = min(int(frame_slot), 16)
        if getattr(self, '_intro_last_frame', -1) != image_index:
            self._intro_last_frame = image_index
            self.intro_label.setPixmap(self.intro_frames[image_index])

    def finish_intro(self):
        """Despues del ultimo fotograma, iniciar la transicion CRT."""
        self.intro_timer.stop()
        # Mantener bloqueada la navegacion hasta terminar el efecto.
        intro_pixmap = self.intro_frames[-1]
        # grab() funciona aunque el widget aun no sea la pagina visible.
        hmi_pixmap = self.hmi_widget.grab()
        self.power_transition = CRTPowerOnTransition(intro_pixmap, hmi_pixmap)
        self.content_pages.addWidget(self.power_transition)
        self.content_pages.setCurrentWidget(self.power_transition)
        self.power_transition.start(self.complete_intro_transition)

    def complete_intro_transition(self):
        """Mostrar el HMI interactivo tras el encendido del CRT."""
        self.content_pages.setCurrentWidget(self.hmi_widget)
        self.intro_active = False
        self.pressed_keys.clear()
        self.pong_progress = 0
        self.focus_first_widget()
        self.crt_display.update_crt()
        # Liberar la pagina temporal cuando Qt vuelva al bucle de eventos.
        transition = self.power_transition
        self.power_transition = None
        self.content_pages.removeWidget(transition)
        transition.deleteLater()

    def _fullscreen_if_small_screen(self):
        screen = self.screen()
        if screen is not None and screen.geometry().width() == 800 and screen.geometry().height() == 480:
            self.showFullScreen()

    def changeEvent(self, event):
        super().changeEvent(event)
        if event.type() == QEvent.Type.WindowStateChange and self.isMaximized():
            screen = self.screen()
            if screen is not None and screen.geometry().width() == 800 and screen.geometry().height() == 480:
                QTimer.singleShot(0, self.showFullScreen)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, 'crt_display'):
            self.crt_display.raise_()

    def create_configuration_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        speed_group = QGroupBox('MOVEMENT SPEED')
        speed_layout = QFormLayout()
        self.speed_input = HMIInput()
        self.speed_input.setPlaceholderText('0 - 100 %')
        self.speed_input.setValidator(numeric_validator(0, 100, 1, self.speed_input))
        speed_layout.addRow('Robot Speed:', self.speed_input)
        speed_group.setLayout(speed_layout)
        layout.addWidget(speed_group)
        home_group = QGroupBox('HOME CALIBRATION')
        home_layout = QVBoxLayout()
        self.home_status = QLabel('Home: NOT CALIBRATED')
        self.home_button = QPushButton('CALIBRATE HOME')
        self.home_button.clicked.connect(self.calibrate_home)
        home_layout.addWidget(self.home_status)
        home_layout.addWidget(self.home_button)
        home_group.setLayout(home_layout)
        layout.addWidget(home_group)
        tcp_group = QGroupBox('TCP CONFIGURATION')
        tcp_layout = QFormLayout()
        self.tcp_distance_input = HMIInput()
        self.tcp_distance_input.setPlaceholderText('Distance [mm]')
        self.tcp_distance_input.setValidator(numeric_validator(0, 1000, 2, self.tcp_distance_input))
        self.tcp_apply_button = QPushButton('APPLY TCP')
        self.tcp_apply_button.clicked.connect(self.apply_tcp)
        tcp_layout.addRow('Faceplate -> TCP:', self.tcp_distance_input)
        tcp_layout.addRow('', self.tcp_apply_button)
        tcp_group.setLayout(tcp_layout)
        layout.addWidget(tcp_group)
        layout.addStretch()
        return tab

    def create_program_tab(self):
        # Both pages live inside the same PROGRAM tab and the same CRT surface.
        tab = QWidget()
        tab_layout = QVBoxLayout(tab)
        tab_layout.setContentsMargins(3, 3, 3, 3)
        self.program_stack = QStackedWidget()
        tab_layout.addWidget(self.program_stack)

        # Main program page.
        program_page = QWidget()
        layout = QHBoxLayout(program_page)
        layout.setContentsMargins(3, 3, 3, 3)
        sequence_group = QGroupBox('PROGRAM SEQUENCE')
        sequence_layout = QVBoxLayout()
        self.sequence_list = QListWidget()
        self.sequence_list.addItem('START')
        self.sequence_list.addItem('FINISH')
        self.sequence_list.setCurrentRow(0)
        self.sequence_list.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.sequence_list.setSelectionMode(QListWidget.SelectionMode.SingleSelection)
        sequence_layout.addWidget(self.sequence_list)
        sequence_group.setLayout(sequence_layout)
        layout.addWidget(sequence_group, 2)
        controls_group = QGroupBox('PROGRAM CONTROLS')
        controls_layout = QVBoxLayout()
        self.add_waypoint_button = QPushButton('ADD WAYPOINT')
        self.end_effector_button = QPushButton('END EFFECTOR')
        self.edit_step_button = QPushButton('EDIT')
        self.wait_button = QPushButton('WAIT')
        self.remove_waypoint_button = QPushButton('REMOVE WAYPOINT')
        self.clear_program_button = QPushButton('CLEAR PROGRAM')
        self.run_program_button = QPushButton('RUN PROGRAM')
        self.program_buttons = [
            self.add_waypoint_button, self.end_effector_button, self.edit_step_button,
            self.wait_button, self.remove_waypoint_button,
            self.clear_program_button, self.run_program_button
        ]
        self.add_waypoint_button.clicked.connect(self.add_waypoint)
        self.end_effector_button.clicked.connect(self.open_end_effector)
        self.edit_step_button.clicked.connect(self.edit_selected_step)
        self.wait_button.clicked.connect(self.open_wait)
        self.remove_waypoint_button.clicked.connect(self.remove_waypoint)
        self.clear_program_button.clicked.connect(self.clear_program)
        self.run_program_button.clicked.connect(self.run_program)
        controls_layout.setSpacing(3)
        for button in self.program_buttons:
            button.setFixedHeight(31)
        controls_layout.addWidget(self.add_waypoint_button)
        controls_layout.addWidget(self.end_effector_button)
        controls_layout.addWidget(self.edit_step_button)
        controls_layout.addWidget(self.wait_button)
        controls_layout.addWidget(self.remove_waypoint_button)
        controls_layout.addWidget(self.clear_program_button)
        controls_layout.addStretch()
        controls_layout.addWidget(self.run_program_button)
        controls_group.setLayout(controls_layout)
        layout.addWidget(controls_group, 1)
        self.program_stack.addWidget(program_page)

        # Inline waypoint editor: the main window / CRT never changes.
        waypoint_page = QWidget()
        editor_layout = QVBoxLayout(waypoint_page)
        editor_layout.setContentsMargins(3, 1, 3, 1)
        editor_layout.setSpacing(3)
        title = QLabel('ADD WAYPOINT  |  ROBOT POSITION')
        self.waypoint_editor_title = title
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setObjectName('dialogTitle')
        title.setFixedHeight(26)
        editor_layout.addWidget(title)
        self.waypoint_joint_control = JointControl()
        editor_layout.addWidget(self.waypoint_joint_control, 1)
        actions = QHBoxLayout()
        self.save_waypoint_button = QPushButton('SAVE WAYPOINT')
        self.cancel_waypoint_button = QPushButton('CANCEL / BACK')
        self.save_waypoint_button.setFixedHeight(34)
        self.cancel_waypoint_button.setFixedHeight(34)
        self.save_waypoint_button.clicked.connect(self.save_waypoint_inline)
        self.cancel_waypoint_button.clicked.connect(self.cancel_waypoint_inline)
        actions.addWidget(self.save_waypoint_button)
        actions.addWidget(self.cancel_waypoint_button)
        editor_layout.addLayout(actions)
        self.program_stack.addWidget(waypoint_page)
        # Inline end effector editor (same tab, same CRT).
        effector_page = QWidget()
        effector_layout = QVBoxLayout(effector_page)
        effector_layout.setContentsMargins(12, 4, 12, 8)
        effector_layout.setSpacing(6)
        effector_title = QLabel('END EFFECTOR  |  GRIPPER STATE')
        effector_title.setObjectName('dialogTitle')
        effector_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        effector_title.setFixedHeight(30)
        effector_layout.addWidget(effector_title)

        state_group = QGroupBox('PROGRAMMED GRIPPER POSITION')
        state_layout = QVBoxLayout(state_group)
        state_layout.setSpacing(10)
        self.end_effector_open_button = QPushButton('OPEN')
        self.end_effector_closed_button = QPushButton('CLOSED')
        for button in (self.end_effector_open_button, self.end_effector_closed_button):
            button.setCheckable(True)
            button.setFixedHeight(42)
            state_layout.addWidget(button)
        self.end_effector_open_button.clicked.connect(lambda: self.select_end_effector_state(False))
        self.end_effector_closed_button.clicked.connect(lambda: self.select_end_effector_state(True))
        self.end_effector_preview = QLabel('SELECT THE STATE TO ADD TO THE PROGRAM')
        self.end_effector_preview.setAlignment(Qt.AlignmentFlag.AlignCenter)
        state_layout.addWidget(self.end_effector_preview)
        effector_layout.addWidget(state_group, 1)

        effector_actions = QHBoxLayout()
        self.save_end_effector_button = QPushButton('ADD TO SEQUENCE')
        self.cancel_end_effector_button = QPushButton('CANCEL / BACK')
        self.save_end_effector_button.setFixedHeight(34)
        self.cancel_end_effector_button.setFixedHeight(34)
        self.save_end_effector_button.clicked.connect(self.save_end_effector)
        self.cancel_end_effector_button.clicked.connect(self.cancel_end_effector)
        effector_actions.addWidget(self.save_end_effector_button)
        effector_actions.addWidget(self.cancel_end_effector_button)
        effector_layout.addLayout(effector_actions)
        self.program_stack.addWidget(effector_page)
        self.end_effector_closed = None

        # WAIT editor: this page shares PROGRAM's CRT surface.
        wait_page = QWidget()
        wait_layout = QVBoxLayout(wait_page)
        wait_layout.setContentsMargins(14, 8, 14, 8)
        wait_title = QLabel('WAIT  |  DELAY IN SECONDS')
        wait_title.setObjectName('dialogTitle')
        wait_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        wait_title.setFixedHeight(30)
        wait_layout.addWidget(wait_title)
        wait_group = QGroupBox('WAIT DURATION')
        wait_group_layout = QFormLayout(wait_group)
        self.wait_seconds_input = HMIInput()
        self.wait_seconds_input.setValidator(numeric_validator(0.0, 86400.0, 3, self.wait_seconds_input))
        self.wait_seconds_input.setText('1.0')
        self.wait_seconds_input.setPlaceholderText('Seconds (0 - 86400)')
        wait_group_layout.addRow('Seconds:', self.wait_seconds_input)
        wait_layout.addWidget(wait_group, 1)
        wait_actions = QHBoxLayout()
        self.save_wait_button = QPushButton('ADD TO SEQUENCE')
        self.cancel_wait_button = QPushButton('CANCEL / BACK')
        self.save_wait_button.setFixedHeight(34)
        self.cancel_wait_button.setFixedHeight(34)
        self.save_wait_button.clicked.connect(self.save_wait)
        self.cancel_wait_button.clicked.connect(self.cancel_wait)
        wait_actions.addWidget(self.save_wait_button)
        wait_actions.addWidget(self.cancel_wait_button)
        wait_layout.addLayout(wait_actions)
        self.program_stack.addWidget(wait_page)  # index 3
        self.program_stack.setCurrentIndex(0)
        return tab

    def waypoint_editor_active(self):
        return self.program_stack.currentIndex() == 1

    def end_effector_editor_active(self):
        return self.program_stack.currentIndex() == 2

    def wait_editor_active(self):
        return self.program_stack.currentIndex() == 3

    def selected_step_index(self):
        # START is row 0; FINISH is last row and neither is editable.
        row = self.sequence_list.currentRow()
        return row - 1 if 1 <= row <= len(self.waypoints) else None

    def edit_selected_step(self):
        index = self.selected_step_index()
        if index is None:
            QMessageBox.information(self, 'Edit Step', 'Select a waypoint, gripper or WAIT step in PROGRAM SEQUENCE.')
            return
        step = self.waypoints[index]
        self.editing_step_index = index
        if step['type'] == 'waypoint':
            self.add_waypoint()
            self.editing_step_index = index
            # Edit should use the saved waypoint angles rather than the live robot state.
            control = self.waypoint_joint_control
            control.set_freedrive(False)
            control.update_joint_positions(step['joints'])
            self.save_waypoint_button.setText('SAVE CHANGES')
            self.waypoint_editor_title.setText('EDIT WAYPOINT  |  JOINT POSITIONS')
            control.joint_inputs[0].setFocus()
        elif step['type'] == 'gripper':
            self.open_end_effector()
            self.editing_step_index = index
            self.select_end_effector_state(bool(step['closed']))
            self.save_end_effector_button.setText('SAVE CHANGES')
            self.end_effector_open_button.setFocus()
        elif step['type'] == 'wait':
            self.open_wait()
            self.editing_step_index = index
            self.wait_seconds_input.setText(f"{step['seconds']:g}")
            self.save_wait_button.setText('SAVE CHANGES')
            self.wait_seconds_input.setFocus()

    def open_wait(self):
        self.editing_step_index = None
        self.wait_seconds_input.setText('1.0')
        self.save_wait_button.setText('ADD TO SEQUENCE')
        self.program_stack.setCurrentIndex(3)
        self.wait_seconds_input.setFocus()

    def save_wait(self):
        try:
            value = float(self.wait_seconds_input.text().strip())
            import math
            if not math.isfinite(value) or not 0 <= value <= 86400:
                raise ValueError
        except ValueError:
            QMessageBox.warning(self, 'Invalid Wait', 'Enter a duration between 0 and 86400 seconds.')
            return
        step = {'type': 'wait', 'seconds': value}
        if self.editing_step_index is None:
            self.waypoints.append(step)
        else:
            self.waypoints[self.editing_step_index] = step
        self.refresh_waypoints()
        self.cancel_wait()

    def cancel_wait(self):
        self.editing_step_index = None
        self.program_stack.setCurrentIndex(0)
        self.sequence_list.setFocus()

    def open_end_effector(self):
        self.editing_step_index = None
        self.save_end_effector_button.setText('ADD TO SEQUENCE')
        # Start with no selection: adding a step requires an explicit choice.
        self.select_end_effector_state(None)
        self.program_stack.setCurrentIndex(2)
        self.end_effector_open_button.setFocus()

    def select_end_effector_state(self, closed):
        self.end_effector_closed = closed
        self.end_effector_open_button.setChecked(closed is False)
        self.end_effector_closed_button.setChecked(closed is True)
        if closed is None:
            text = 'SELECT OPEN OR CLOSED'
        else:
            text = 'GRIPPER WILL BE ' + ('CLOSED' if closed else 'OPEN')
        self.end_effector_preview.setText(text)

    def save_end_effector(self):
        if self.end_effector_closed is None:
            QMessageBox.warning(self, 'Select Gripper State', 'Choose OPEN or CLOSED before adding the step.')
            return
        # This stores a PROGRAM COMMAND, not the live physical gripper state.
        step = {'type': 'gripper', 'closed': self.end_effector_closed}
        if self.editing_step_index is None:
            self.waypoints.append(step)
        else:
            self.waypoints[self.editing_step_index] = step
        self.refresh_waypoints()
        self.cancel_end_effector()

    def cancel_end_effector(self):
        self.editing_step_index = None
        self.program_stack.setCurrentIndex(0)
        self.sequence_list.setFocus()

    def create_movement_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        group = QGroupBox('MANUAL JOINT CONTROL')
        group_layout = QVBoxLayout()
        group_layout.setContentsMargins(4, 6, 4, 6)
        group_layout.setSpacing(3)
        self.movement_joint_control = JointControl()
        group_layout.addWidget(self.movement_joint_control)
        self.move_robot_button = QPushButton('MOVE ROBOT')
        self.move_robot_button.setFixedHeight(34)
        self.move_robot_button.clicked.connect(self.move_robot)
        group_layout.addWidget(self.move_robot_button)
        self.move_robot_button.setVisible(not self.movement_joint_control.freedrive_enabled)
        group.setLayout(group_layout)
        layout.addWidget(group)
        layout.addStretch()
        return tab

    def calibrate_home(self):
        self.home_status.setText('Home: CALIBRATED')
        print('HOME CALIBRATED')

    def apply_tcp(self):
        text = self.tcp_distance_input.text().strip()
        if text == '':
            QMessageBox.warning(self, 'Invalid TCP', 'Enter a TCP distance.')
            return
        distance = float(text)
        print(f'TCP DISTANCE = {distance} mm')

    def move_robot(self):
        # In Freedrive, physical controls produce the position changes.
        if self.movement_joint_control.freedrive_enabled:
            return
        try:
            values = self.movement_joint_control.get_joint_values()
            if any(not (0.0 <= value <= 360.0) for value in values):
                raise ValueError('Joint angles must be between 0 and 360 degrees.')
        except ValueError as error:
            QMessageBox.warning(self, 'Invalid Input', str(error))
            return

        # Exit text editing before synchronizing the freshly applied values.
        for field in self.movement_joint_control.joint_inputs:
            if field.editing:
                field.stop_editing()

        # Text control uses the same canonical state update as Freedrive.
        self.set_robot_positions(values)
        print('MOVE ROBOT (HMI position update):', self.joint_values)

    def add_waypoint(self):
        self.editing_step_index = None
        self.save_waypoint_button.setText('SAVE WAYPOINT')
        self.waypoint_editor_title.setText('ADD WAYPOINT  |  ROBOT POSITION')
        # Populate the embedded editor from the single canonical robot state.
        control = self.waypoint_joint_control
        for field in control.joint_inputs:
            if field.editing:
                field.stop_editing()
        control.update_joint_positions(self.joint_values)
        control.set_selected_joint(self.movement_joint_control.selected_joint_y)
        control.set_selected_joint(self.movement_joint_control.selected_joint_x)
        control.set_gripper(self.gripper_enabled)
        control.set_freedrive(self.movement_joint_control.freedrive_enabled)
        self.program_stack.setCurrentIndex(1)
        if control.freedrive_enabled:
            self.save_waypoint_button.setFocus()
        else:
            control.joint_inputs[0].setFocus()

    def save_waypoint_inline(self):
        control = self.waypoint_joint_control
        if control.freedrive_enabled and self.editing_step_index is None:
            # Freedrive always snapshots the latest CAN/Serial robot positions.
            values = list(self.joint_values)
        else:
            try:
                values = control.get_joint_values()
            except ValueError as error:
                QMessageBox.warning(self, 'Invalid Input', str(error))
                return
            if any(not 0 <= value <= 360 for value in values):
                QMessageBox.warning(self, 'Invalid Input', 'Joint angles must be 0–360 degrees.')
                return
        step = {'type': 'waypoint', 'joints': list(values)}
        if self.editing_step_index is None:
            self.waypoints.append(step)
        else:
            self.waypoints[self.editing_step_index] = step
        self.refresh_waypoints()
        self.cancel_waypoint_inline()

    def cancel_waypoint_inline(self):
        self.editing_step_index = None
        for field in self.waypoint_joint_control.joint_inputs:
            if field.editing:
                field.stop_editing()
        self.program_stack.setCurrentIndex(0)
        self.sequence_list.setFocus()

    def remove_waypoint(self):
        row = self.sequence_list.currentRow()
        if row <= 0:
            return
        if row == self.sequence_list.count() - 1:
            return
        self.waypoints.pop(row - 1)
        self.refresh_waypoints()

    def clear_program(self):
        self.waypoints.clear()
        self.refresh_waypoints()

    def refresh_waypoints(self):
        previous_row = self.sequence_list.currentRow()
        self.sequence_list.clear()
        self.sequence_list.addItem('START')
        waypoint_number = 0
        for step_number, step in enumerate(self.waypoints, 1):
            if step['type'] == 'waypoint':
                waypoint_number += 1
                joints = step['joints']
                self.sequence_list.addItem(
                    f'{step_number}. WAYPOINT {waypoint_number} | '
                    + '  '.join(f'J{i+1}={value:.1f}°' for i, value in enumerate(joints))
                )
            elif step['type'] == 'gripper':
                state = 'CLOSED' if step['closed'] else 'OPEN'
                self.sequence_list.addItem(f'{step_number}. END EFFECTOR | GRIPPER: {state}')
            elif step['type'] == 'wait':
                self.sequence_list.addItem(f"{step_number}. WAIT | {step['seconds']:g} s")
        self.sequence_list.addItem('FINISH')
        if previous_row < 0:
            previous_row = 0
        previous_row = min(previous_row, self.sequence_list.count() - 1)
        self.sequence_list.setCurrentRow(previous_row)

    def run_program(self):
        if not self.waypoints:
            QMessageBox.warning(self, 'Empty Program', 'Add at least one waypoint.')
            return
        print('PROGRAM START')
        for i, step in enumerate(self.waypoints, 1):
            if step['type'] == 'waypoint':
                print(f'STEP {i} | WAYPOINT:', step['joints'])
            elif step['type'] == 'gripper':
                print(f'STEP {i} | GRIPPER:', 'CLOSED' if step['closed'] else 'OPEN')
            elif step['type'] == 'wait':
                print(f"STEP {i} | WAIT: {step['seconds']:g} seconds")
        print('PROGRAM FINISH')

    def on_gripper_received(self, enabled):
        self.gripper_enabled = bool(enabled)
        self.movement_joint_control.set_gripper(enabled)
        self.waypoint_joint_control.set_gripper(enabled)

    def on_button_received(self, joint_id):
        # Real hardware button press, independent of Freedrive and navigation.
        # JOINT,<id>,<angle> messages do NOT count as button presses.
        if joint_id in (1, 2):
            self.record_secret_input(f'J{joint_id}')

    def esp32_connected(self):
        """Comprueba que el receptor Serial este funcionando con un puerto abierto."""
        receiver = getattr(self, 'serial_receiver', None)
        port = getattr(receiver, 'serial_port', None)
        return bool(receiver is not None and receiver.isRunning()
                    and port is not None and getattr(port, 'is_open', False))

    def record_secret_input(self, token):
        """Detecta la combinacion sin consumir las teclas de navegacion.

        Con ESP32: flechas + J2 + J1 + Enter.
        Sin ESP32: flechas + menos + mas + Enter.
        """
        if self.arcade_active or self.konami_pending:
            return False

        connected = self.esp32_connected()
        if connected != self.secret_using_esp32:
            self.secret_using_esp32 = connected
            self.pong_progress = 0

        now = time.monotonic()
        if now - self.pong_last_event > 12.0:
            self.pong_progress = 0

        sequence = self.pong_sequence if connected else self.pong_sequence_offline
        progress = self.pong_progress
        self.pong_last_event = now

        if token == sequence[progress]:
            self.pong_progress += 1
            if self.pong_progress == len(sequence):
                self.pong_progress = 0
                self.open_arcade()
                return True
            return False

        self.pong_progress = 1 if token == sequence[0] else 0
        return False

    def activate_arcade(self):
        """Abre la escena al recibir ACK o en modo offline sin ESP32."""
        self.konami_pending = False
        self.arcade_active = True
        self.pong_active = False
        self.pressed_keys.clear()
        self.goat_game.begin()
        self.content_pages.setCurrentWidget(self.goat_game)
        self.konami_direction = None
        self.konami_input_last = time.monotonic()
        if self.esp32_connected():
            self.konami_watchdog.start()
        else:
            self.konami_watchdog.stop()

    def open_arcade(self):
        """Solicita Konami a la ESP32 o abre la escena en modo teclado."""
        if self.arcade_active or self.konami_pending:
            return
        if not self.esp32_connected():
            self.activate_arcade()
            return
        self.konami_pending = True
        self.konami_request_started = time.monotonic()
        self.konami_watchdog.start()
        self.serial_receiver.send_command('KONAMI,1')

    def on_konami_ack(self, enabled):
        if not self.konami_pending:
            return
        if not enabled:
            self.konami_pending = False
            self.konami_watchdog.stop()
            print('[KONAMI] La ESP32 rechazo el cambio de modo')
            return
        self.activate_arcade()

    def on_konami_input(self, token):
        if not self.arcade_active or self.goat_game.mode not in ('play', 'door'):
            return
        self.konami_input_last = time.monotonic()
        directions = {
            'LEFT': Qt.Key.Key_Left, 'RIGHT': Qt.Key.Key_Right,
            'UP': None, 'DOWN': Qt.Key.Key_Down, 'CENTER': None,
        }
        if token == 'JUMP':
            # Pulsacion de un boton (no repeticiones por mantenerlo).
            self.goat_game.press(Qt.Key.Key_Space)
            self.goat_game.release(Qt.Key.Key_Space)
            return
        new_key = directions.get(token)
        if self.konami_direction is not None and self.konami_direction != new_key:
            self.goat_game.release(self.konami_direction)
        if new_key is not None and new_key != self.konami_direction:
            self.goat_game.press(new_key)
        self.konami_direction = new_key
        self.goat_game.looking_up = (token == 'UP')
        self.goat_game.update()

    def check_konami_input(self):
        if self.konami_pending and time.monotonic() - self.konami_request_started > 2.0:
            self.konami_pending = False
            self.konami_watchdog.stop()
            self.serial_receiver.send_command('KONAMI,0')
            print('[KONAMI] No se recibio confirmacion de la ESP32')
        if self.arcade_active and time.monotonic() - self.konami_input_last > 0.18:
            if self.konami_direction is not None:
                self.goat_game.release(self.konami_direction)
            self.konami_direction = None
            self.goat_game.looking_up = False
            self.goat_game.update()

    def return_from_door(self):
        """La escena ya esta negra; repetir apertura CRT antes de regresar."""
        if not self.arcade_active:
            return
        self.goat_game.end()
        black = QPixmap(800, 480)
        black.fill(QColor(0, 0, 0))
        hmi_image = self.hmi_widget.grab()
        self.power_transition = CRTPowerOnTransition(black, hmi_image)
        self.content_pages.addWidget(self.power_transition)
        self.content_pages.setCurrentWidget(self.power_transition)
        self.power_transition.start(self.complete_door_return)

    def complete_door_return(self):
        transition = self.power_transition
        self.power_transition = None
        self.close_arcade()
        self.content_pages.removeWidget(transition)
        transition.deleteLater()
        self.crt_display.update_crt()

    def close_arcade(self):
        self.konami_watchdog.stop()
        self.konami_pending = False
        self.konami_direction = None
        self.goat_game.looking_up = False
        if self.esp32_connected():
            self.serial_receiver.send_command('KONAMI,0')
        self.goat_game.end()
        self.arcade_active = False
        self.pong_active = False
        self.pong_progress = 0
        self.pressed_keys.clear()
        self.content_pages.setCurrentWidget(self.hmi_widget)
        self.focus_first_widget()

    def open_pong(self):
        self.open_arcade()

    def close_pong(self):
        self.close_arcade()

    def on_freedrive_received(self, enabled):
        if enabled:
            controls = [self.movement_joint_control]
            if not (self.waypoint_editor_active() and self.editing_step_index is not None):
                controls.append(self.waypoint_joint_control)
            for control in controls:
                for field in control.joint_inputs:
                    if field.editing:
                        field.stop_editing()
                control.sync_input_fields(self.joint_values)
        self.movement_joint_control.set_freedrive(enabled)
        if not (self.waypoint_editor_active() and self.editing_step_index is not None):
            self.waypoint_joint_control.set_freedrive(enabled)
        self.move_robot_button.setVisible(not enabled)
        self.update_robot_position(self.joint_values)
        if self.tabs.currentIndex() == 1 and self.waypoint_editor_active() and self.editing_step_index is None:
            (self.save_waypoint_button if enabled else self.waypoint_joint_control.joint_inputs[0]).setFocus()

    def on_joint_selected(self, joint_id):
        if 1 <= joint_id <= 5:
            self.movement_joint_control.set_selected_joint(joint_id)
            self.waypoint_joint_control.set_selected_joint(joint_id)

    def on_joystick_received(self, joint_x, joint_y, x, y, manual):
        """Mantiene la estimacion angular antigua SOLO para el HMI.

        Las tramas no incluyen angulos ni encoder feedback. El backend de
        motores debe ser la fuente de posicion real cuando se integre.
        """
        now = time.monotonic()
        self._jog_last_frame = now
        if joint_x:
            self._jog_x = joint_x
        if joint_y:
            self._jog_y = joint_y
        if not manual:
            self._jog_last_x = self._jog_last_y = now
            return

        def step(joint, normalized, previous):
            # Se conserva el esquema de la version anterior:
            # zona muerta ADC=300; incrementos de 1 grado y retardo 20..250ms.
            # El valor de joystick serializado esta cuantizado a -1000..1000.
            if joint == 0 or not 1 <= joint <= 5:
                return previous
            magnitude = abs(normalized) * 2048.0 / 1000.0
            if magnitude <= 300:
                return previous
            delay_ms = 250 - (min(magnitude, 2048) - 300) * 230 / (2048 - 300)
            if (now - previous) * 1000.0 < delay_ms:
                return previous
            current = list(self.joint_values)
            index = joint - 1
            angle = max(0.0, min(360.0, current[index] + (1 if normalized > 0 else -1)))
            if angle != current[index]:
                current[index] = angle
                self.set_robot_positions(current)
            return now

        self._jog_last_x = step(self._jog_x, x, self._jog_last_x)
        self._jog_last_y = step(self._jog_y, y, self._jog_last_y)

    def on_joint_received(self, joint_id, angle):
        index = joint_id - 1
        if not 0 <= index < 5:
            return
        # Update the one shared state rather than any JointControl copy.
        new_positions = list(self.joint_values)
        new_positions[index] = float(angle)
        self.set_robot_positions(new_positions)
        self.movement_joint_control.set_selected_joint(joint_id)
        self.waypoint_joint_control.set_selected_joint(joint_id)

    def set_robot_positions(self, positions):
        """Single source of truth for all five joint positions.

        Every update from CAN/Serial and text control must pass through here.
        The two JointControl widgets only display this state.
        """
        if len(positions) != 5:
            raise ValueError('Expected exactly five joint positions.')
        new_values = [float(value) for value in positions]
        if any(not (0.0 <= value <= 360.0) for value in new_values):
            raise ValueError('Joint angles must be between 0 and 360 degrees.')

        self.joint_values = new_values
        self.movement_joint_control.update_joint_positions(self.joint_values)
        if not (self.waypoint_editor_active() and self.editing_step_index is not None):
            self.waypoint_joint_control.update_joint_positions(self.joint_values)

    def update_robot_position(self, joint_values):
        """Backward-compatible entry point; uses the shared state."""
        self.set_robot_positions(joint_values)

    def next_tab(self):
        index = (self.tabs.currentIndex() + 1) % self.tabs.count()
        self.tabs.setCurrentIndex(index)
        self.focus_first_widget()

    def previous_tab(self):
        index = (self.tabs.currentIndex() - 1) % self.tabs.count()
        self.tabs.setCurrentIndex(index)
        self.focus_first_widget()

    def focus_first_widget(self):
        index = self.tabs.currentIndex()
        if index == 0:
            self.speed_input.setFocus()
        elif index == 1:
            if self.wait_editor_active():
                self.wait_seconds_input.setFocus()
            elif self.end_effector_editor_active():
                self.end_effector_open_button.setFocus()
            elif self.waypoint_editor_active():
                self.save_waypoint_button.setFocus() if self.waypoint_joint_control.freedrive_enabled else self.waypoint_joint_control.joint_inputs[0].setFocus()
            else:
                self.sequence_list.setFocus()
                if self.sequence_list.currentRow() < 0:
                    self.sequence_list.setCurrentRow(0)
        elif index == 2:
            self.movement_joint_control.joint_inputs[0].setFocus()

    def eventFilter(self, obj, event):
        # Este filtro está instalado una sola vez en QApplication.
        # Liberar la tecla NO interrumpe el procesamiento normal de Qt.
        if event.type() == QEvent.Type.KeyRelease:
            if not event.isAutoRepeat():
                self.pressed_keys.discard(event.key())
                if self.arcade_active:
                    self.goat_game.release(event.key())
            if self.arcade_active:
                return True
            return super().eventFilter(obj, event)

        # Evita teclas atascadas después de perder el foco de la aplicación.
        if event.type() in (QEvent.Type.ApplicationDeactivate, QEvent.Type.WindowDeactivate):
            self.pressed_keys.clear()
            if self.arcade_active:
                self.goat_game.clear_input()
            return super().eventFilter(obj, event)

        if event.type() != QEvent.Type.KeyPress:
            return super().eventFilter(obj, event)
        # No navegar, editar ni activar Pong durante la pantalla de arranque.
        if self.intro_active:
            return True
        if QApplication.activeWindow() is not self:
            return super().eventFilter(obj, event)

        key = event.key()
        tracked_keys = {
            Qt.Key.Key_Up, Qt.Key.Key_Down,
            Qt.Key.Key_Left, Qt.Key.Key_Right,
            Qt.Key.Key_Return, Qt.Key.Key_Enter,
            Qt.Key.Key_Minus, Qt.Key.Key_Plus,
        }
        focused_for_secret = QApplication.focusWidget()
        editing_text = (
            isinstance(focused_for_secret, HMIInput)
            and focused_for_secret.editing
        )

        # Solo una pulsación física se cuenta para la secuencia y para
        # navegación con flechas. No bloquea el primer evento.
        if key in tracked_keys and not editing_text and not self.arcade_active:
            if event.isAutoRepeat() or key in self.pressed_keys:
                return True
            self.pressed_keys.add(key)

        # Controles de la escena: movimiento y Enter frente a la puerta.
        if self.arcade_active:
            if self.goat_game.mode not in ('play', 'door'):
                return True
            if key == Qt.Key.Key_F11:
                if not event.isAutoRepeat():
                    self.showNormal() if self.isFullScreen() else self.showFullScreen()
            elif key in (Qt.Key.Key_Left, Qt.Key.Key_Right,
                         Qt.Key.Key_Down, Qt.Key.Key_Up, Qt.Key.Key_Space,
                         Qt.Key.Key_Return, Qt.Key.Key_Enter, Qt.Key.Key_R):
                if not event.isAutoRepeat() and key not in self.goat_game.keys:
                    self.goat_game.press(key)
            return True

        # Observador pasivo: las flechas mantienen su navegación habitual.
        # J2 y J1 deben proceder de eventos Serial BUTTON,J2 y BUTTON,J1.
        # Solo se consume Enter cuando abre el menu arcade.
        if not editing_text and (key in tracked_keys or
                                 (key == Qt.Key.Key_Equal and
                                  event.modifiers() & Qt.KeyboardModifier.ShiftModifier)):
            secret_keys = {
                Qt.Key.Key_Up: 'UP',
                Qt.Key.Key_Down: 'DOWN',
                Qt.Key.Key_Left: 'LEFT',
                Qt.Key.Key_Right: 'RIGHT',
                Qt.Key.Key_Return: 'ENTER',
                Qt.Key.Key_Enter: 'ENTER',
                Qt.Key.Key_Minus: 'MINUS',
                Qt.Key.Key_Plus: 'PLUS',
            }
            token = ('PLUS' if key == Qt.Key.Key_Equal else secret_keys[key])
            if not event.isAutoRepeat() and self.record_secret_input(token):
                return True

        if key == Qt.Key.Key_F11:
            if self.isFullScreen():
                self.showNormal()
            else:
                self.showFullScreen()
            return True
        if key == Qt.Key.Key_Escape and self.tabs.currentIndex() == 1 and self.wait_editor_active():
            if self.wait_seconds_input.editing:
                self.wait_seconds_input.stop_editing()
            else:
                self.cancel_wait()
            return True
        if key == Qt.Key.Key_Escape and self.tabs.currentIndex() == 1 and self.end_effector_editor_active():
            self.cancel_end_effector()
            return True
        if key == Qt.Key.Key_Escape and self.tabs.currentIndex() == 1 and self.waypoint_editor_active():
            focused_editor = QApplication.focusWidget()
            if isinstance(focused_editor, HMIInput) and focused_editor.editing:
                focused_editor.stop_editing()
            else:
                self.cancel_waypoint_inline()
            return True
        if key == Qt.Key.Key_Escape and self.isFullScreen():
            self.showNormal()
            return True
        focused = QApplication.focusWidget()
        if isinstance(focused, HMIInput):
            if key in (Qt.Key.Key_Return, Qt.Key.Key_Enter):
                focused.toggle_editing()
                return True
            if key == Qt.Key.Key_Escape and focused.editing:
                focused.stop_editing()
                return True
            if focused.editing:
                return super().eventFilter(obj, event)
        if key == Qt.Key.Key_Plus:
            self.next_tab()
            return True
        if key == Qt.Key.Key_Equal and event.modifiers() & Qt.KeyboardModifier.ShiftModifier:
            self.next_tab()
            return True
        if key == Qt.Key.Key_Minus:
            self.previous_tab()
            return True
        if self.tabs.currentIndex() == 1 and self.wait_editor_active():
            controls = (self.wait_seconds_input, self.save_wait_button, self.cancel_wait_button)
            if focused in controls and key in (Qt.Key.Key_Up, Qt.Key.Key_Down, Qt.Key.Key_Left, Qt.Key.Key_Right):
                idx = controls.index(focused)
                controls[(idx + (1 if key in (Qt.Key.Key_Down, Qt.Key.Key_Right) else -1)) % len(controls)].setFocus()
                return True
        if self.tabs.currentIndex() == 1 and self.end_effector_editor_active():
            # Keyboard controls remain operational even with the cursor hidden.
            buttons = (self.end_effector_open_button, self.end_effector_closed_button,
                       self.save_end_effector_button, self.cancel_end_effector_button)
            if focused in buttons and key in (Qt.Key.Key_Up, Qt.Key.Key_Down, Qt.Key.Key_Left, Qt.Key.Key_Right):
                idx = buttons.index(focused)
                move = -1 if key in (Qt.Key.Key_Up, Qt.Key.Key_Left) else 1
                buttons[(idx + move) % len(buttons)].setFocus()
                return True
        if self.tabs.currentIndex() == 1 and self.waypoint_editor_active():
            if key in (Qt.Key.Key_Left, Qt.Key.Key_Right):
                if focused is self.save_waypoint_button:
                    self.cancel_waypoint_button.setFocus()
                    return True
                if focused is self.cancel_waypoint_button:
                    self.save_waypoint_button.setFocus()
                    return True
            if key == Qt.Key.Key_Down:
                self.focusNextChild()
                return True
            if key == Qt.Key.Key_Up:
                self.focusPreviousChild()
                return True
        if self.tabs.currentIndex() == 1 and self.program_stack.currentIndex() == 0:
            if focused is self.sequence_list or focused is self.sequence_list.viewport():
                # Mover explicitamente el renglon seleccionado, sin depender
                # del comportamiento predeterminado del QListWidget/viewport.
                if key in (Qt.Key.Key_Up, Qt.Key.Key_Down):
                    row = self.sequence_list.currentRow()
                    if row < 0:
                        row = 0
                    direction = -1 if key == Qt.Key.Key_Up else 1
                    next_row = max(0, min(self.sequence_list.count() - 1, row + direction))
                    self.sequence_list.setCurrentRow(next_row)
                    self.sequence_list.scrollToItem(self.sequence_list.item(next_row))
                    return True
                if key == Qt.Key.Key_Right:
                    self.edit_step_button.setFocus()
                    return True
                if key == Qt.Key.Key_Left:
                    return True
                if key in (Qt.Key.Key_Return, Qt.Key.Key_Enter):
                    if self.selected_step_index() is not None:
                        self.edit_selected_step()
                    return True
            if focused in self.program_buttons:
                current_index = self.program_buttons.index(focused)
                if key == Qt.Key.Key_Down:
                    next_index = (current_index + 1) % len(self.program_buttons)
                    self.program_buttons[next_index].setFocus()
                    return True
                if key == Qt.Key.Key_Up:
                    previous_index = (current_index - 1) % len(self.program_buttons)
                    self.program_buttons[previous_index].setFocus()
                    return True
                if key == Qt.Key.Key_Left:
                    self.sequence_list.setFocus()
                    if self.sequence_list.currentRow() < 0:
                        self.sequence_list.setCurrentRow(0)
                    return True
                if key == Qt.Key.Key_Right:
                    return True
                if key in (Qt.Key.Key_Return, Qt.Key.Key_Enter):
                    focused.click()
                    return True
        if isinstance(focused, QListWidget):
            if key in (Qt.Key.Key_Up, Qt.Key.Key_Down):
                return super().eventFilter(obj, event)
        if key == Qt.Key.Key_Down:
            self.focusNextChild()
            return True
        if key == Qt.Key.Key_Up:
            self.focusPreviousChild()
            return True
        if key in (Qt.Key.Key_Return, Qt.Key.Key_Enter):
            if isinstance(focused, QPushButton):
                focused.click()
                return True
        return super().eventFilter(obj, event)

    def closeEvent(self, event):
        if hasattr(self, 'serial_receiver') and (getattr(self, 'arcade_active', False) or getattr(self, 'konami_pending', False)):
            self.serial_receiver.send_command('KONAMI,0')
        if getattr(self, 'arcade_active', False):
            self.goat_game.end()
        if hasattr(self, 'serial_receiver'):
            self.serial_receiver.stop()
        super().closeEvent(event)
RETRO_STYLE = '\nQMainWindow {\n    background-color: #000000;\n}\nQWidget {\n    background-color: #020502;\n    color: #65ff72;\n    font-family: "Courier New";\n    font-size: 16px;\n}\nQLabel#header {\n    font-size: 26px;\n    font-weight: bold;\n    color: #90ff99;\n    padding: 3px;\n}\nQLabel#dialogTitle {\n    font-size: 20px;\n    font-weight: bold;\n}\nQLabel#keyboardHelp {\n    font-size: 13px;\n    color: #4fc95c;\n    padding: 4px;\n}\nQGroupBox {\n    border: 2px solid #47d85a;\n    margin-top: 10px;\n    padding: 5px;\n    font-weight: bold;\n}\nQGroupBox::title {\n    subcontrol-origin: margin;\n    left: 10px;\n    padding: 0px 5px;\n}\n/* ======================================================\n   INPUTS\n   ====================================================== */\nQLineEdit {\n    background-color: #000200;\n    color: #75ff82;\n    border: 2px solid #47d85a;\n    padding: 3px 7px;\n    font-size: 15px;\n}\n/* Selected but locked */\nQLineEdit:focus {\n    background-color: #071007;\n    color: #75ff82;\n    border: 3px solid #d0ffd5;\n}\n/* Edit mode */\nQLineEdit[editing="true"] {\n    background-color: #123518;\n    color: #ffffff;\n    border: 3px solid #65ff72;\n}\n/* ======================================================\n   BUTTONS\n   ====================================================== */\nQPushButton {\n    background-color: #061006;\n    color: #65ff72;\n    border: 2px solid #47d85a;\n    padding: 3px 7px;\n    font-size: 15px;\n}\nQPushButton:focus {\n    background-color: #102615;\n    border: 3px solid #d0ffd5;\n}\nQPushButton:hover {\n    background-color: #102615;\n}\nQPushButton:checked {\n    background-color: #65ff72;\n    color: #000000;\n    border: 3px solid #d0ffd5;\n}\nQPushButton:pressed {\n    background-color: #65ff72;\n    color: #000000;\n}\n/* ======================================================\n   TABS\n   ====================================================== */\nQTabWidget::pane {\n    border: 2px solid #47d85a;\n}\nQTabBar::tab {\n    background-color: #061006;\n    color: #65ff72;\n    border: 2px solid #47d85a;\n    padding: 10px 24px;\n}\nQTabBar::tab:selected {\n    background-color: #47d85a;\n    color: #000000;\n}\n/* ======================================================\n   LIST\n   ====================================================== */\nQListWidget {\n    background-color: #000200;\n    color: #65ff72;\n    border: 2px solid #47d85a;\n    padding: 5px;\n}\nQListWidget:focus {\n    border: 3px solid #d0ffd5;\n}\nQListWidget::item {\n    padding: 8px;\n}\nQListWidget::item:selected {\n    background-color: #47d85a;\n    color: #000000;\n}\n/* ======================================================\n   SLIDERS\n   ====================================================== */\nQSlider::groove:horizontal {\n    height: 8px;\n    background: #061006;\n    border: 1px solid #47d85a;\n}\nQSlider::handle:horizontal {\n    width: 16px;\n    background: #65ff72;\n    margin: -5px 0px;\n}\n'
def find_hmi_screen(app):
    """Selecciona el monitor de 800x480; si no existe, usa el principal."""
    for screen in app.screens():
        size = screen.geometry().size()
        if size.width() == 800 and size.height() == 480:
            return screen
    return app.primaryScreen() or (app.screens()[0] if app.screens() else None)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyleSheet(RETRO_STYLE)
    # Oculta el puntero en todas las ventanas y diálogos Qt del HMI.
    app.setOverrideCursor(Qt.CursorShape.BlankCursor)

    target_screen = find_hmi_screen(app)
    window = RobotHMI()

    if target_screen is not None:
        geometry = target_screen.geometry()
        print(f'[DISPLAY] Monitor seleccionado: {target_screen.name()} '
              f'({geometry.width()}x{geometry.height()})')
        # Crear el manejador nativo sin mostrar la ventana todavía:
        # así la intro se inicia ya en el monitor seleccionado.
        window.winId()
        window.windowHandle().setScreen(target_screen)
        window.move(geometry.topLeft())
    else:
        print('[DISPLAY] No se detectaron monitores; Qt elegirá la pantalla.')

    window.showFullScreen()
    sys.exit(app.exec())
