#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
室内多传感器寻敌系统 - 作战地图工具
===============================

提供作战地图显示和标绘功能，支持在地图上进行标绘操作

功能特性：
- 支持导入地图图片（PNG、JPG等格式）
- 支持自由绘制线条，多种颜色和线宽
- 支持标记敌人位置（红色圆圈）
- 支持标记人质位置（蓝色圆圈）
- 支持清除和撤销操作
- 支持保存标绘结果

作者: 郭晋鹏团队
版本: 2.0.0
"""

from PyQt5.QtWidgets import (QWidget, QPushButton, QColorDialog, QSlider, QLabel, 
                            QHBoxLayout, QVBoxLayout, QFileDialog, QMessageBox, 
                            QButtonGroup, QFrame, QSizePolicy)
from PyQt5.QtCore import Qt, QPoint, QRect, QSize
from PyQt5.QtGui import QPainter, QPen, QColor, QPixmap, QPainterPath, QImage, QBrush, QIcon, QFont

import cv2
import numpy as np
import sys
import os

# 确保可以从任何位置导入
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

class MapDrawingArea(QWidget):
    """
    地图绘图区域类
    
    提供一个可以进行地图标绘的区域，支持多种绘制工具和操作
    
    主要属性:
        drawing: 是否正在绘制
        last_point: 上一个绘制点
        current_point: 当前绘制点
        pixmap: 绘制的图像
        temp_pixmap: 临时绘制的图像
        history: 历史记录，用于撤销操作
        scale_factor: 缩放因子
        
    主要方法:
        loadMap: 加载地图图片
        setDrawingTool: 设置绘图工具
        setDrawingColor: 设置绘图颜色
        setLineWidth: 设置线宽
        undo: 撤销操作
        clear: 清除所有绘制
        getImage: 获取绘制后的图像
        zoomIn: 放大地图
        zoomOut: 缩小地图
    """
    
    # 绘图工具枚举
    TOOL_PEN = 0      # 自由绘制
    TOOL_ENEMY = 1    # 标记敌人
    TOOL_HOSTAGE = 2  # 标记人质
    TOOL_ERASER = 3   # 橡皮擦
    
    # 符号尺寸
    SYMBOL_SIZE = 20
    
    # 缩放设置
    MIN_SCALE = 0.1   # 最小缩放比例
    MAX_SCALE = 5.0   # 最大缩放比例
    ZOOM_FACTOR = 1.2 # 每次缩放的比例
    
    def __init__(self, parent=None):
        """初始化地图绘图区域"""
        super().__init__(parent)
        
        # 初始化绘图属性
        self.drawing = False
        self.last_point = QPoint()
        self.current_point = QPoint()
        self.pixmap = QPixmap()
        self.temp_pixmap = QPixmap()
        self.original_pixmap = QPixmap()  # 保存原始图像
        self.history = []
        self.has_map = False
        
        # 缩放和平移属性
        self.scale_factor = 1.0
        self.offset = QPoint(0, 0)
        self.panning = False
        self.pan_start_pos = QPoint()
        
        # 绘图设置
        self.drawing_tool = self.TOOL_PEN
        self.pen_color = QColor(0, 255, 0)  # 默认绿色
        self.pen_width = 3
        
        # 敌人和人质标记颜色
        self.enemy_color = QColor(255, 0, 0)    # 红色
        self.hostage_color = QColor(0, 0, 255)  # 蓝色
        
        # 橡皮擦设置
        self.eraser_size = 20
        
        # 设置鼠标跟踪
        self.setMouseTracking(True)
        
        # 设置焦点策略，使其能接收键盘事件
        self.setFocusPolicy(Qt.StrongFocus)
        
        # 创建空白图像
        self.createEmptyMap()
        
    def createEmptyMap(self):
        """创建空白地图"""
        self.pixmap = QPixmap(800, 600)
        self.pixmap.fill(Qt.white)
        self.history = [self.pixmap.copy()]
        self.has_map = False
        self.update()
        
    def loadMap(self, file_path=None):
        """
        加载地图图片
        
        参数:
            file_path: 图片文件路径，如果为None则弹出文件选择对话框
        
        返回:
            bool: 是否成功加载地图
        """
        if file_path is None:
            file_path, _ = QFileDialog.getOpenFileName(
                self, '导入地图', './', 
                "图像文件 (*.jpg *.jpeg *.png *.bmp)"
            )
            
        if not file_path:
            return False
            
        try:
            # 加载原始图片
            original_pixmap = QPixmap(file_path)
            if original_pixmap.isNull():
                QMessageBox.warning(self, "错误", "无法加载地图文件")
                return False
            
            # 保存原始图片
            self.original_pixmap = original_pixmap.copy()
            
            # 重置缩放和偏移
            self.scale_factor = 1.0
            self.offset = QPoint(0, 0)
            
            # 适配图像大小到窗口
            self.fitToView()
            
            # 保存初始状态到历史记录
            self.history = [self.pixmap.copy()]
            self.has_map = True
            
            # 更新界面
            self.update()
            return True
            
        except Exception as e:
            QMessageBox.warning(self, "错误", f"加载地图失败: {str(e)}")
            return False
            
    def fitToView(self):
        """适配图像大小到当前视图"""
        if self.original_pixmap.isNull():
            return
            
        # 获取父窗口大小（如果有）
        parent_size = self.parentWidget().size() if self.parentWidget() else self.size()
        
        # 计算适合的缩放比例
        width_ratio = (parent_size.width() - 40) / self.original_pixmap.width()
        height_ratio = (parent_size.height() - 40) / self.original_pixmap.height()
        
        # 选择较小的比例，确保图像完全可见
        self.scale_factor = min(width_ratio, height_ratio, 1.0)
        
        # 缩放图像
        self.updateScaledPixmap()
        
    def updateScaledPixmap(self):
        """根据当前缩放因子更新缩放后的图像"""
        if self.original_pixmap.isNull():
            return
            
        # 计算缩放后的尺寸
        new_width = int(self.original_pixmap.width() * self.scale_factor)
        new_height = int(self.original_pixmap.height() * self.scale_factor)
        
        # 缩放图像
        self.pixmap = self.original_pixmap.scaled(
            new_width, new_height, 
            Qt.KeepAspectRatio, 
            Qt.SmoothTransformation
        )
        
        # 设置最小尺寸
        self.setMinimumSize(new_width, new_height)
        
        # 更新界面
        self.update()
        
    def zoomIn(self):
        """放大地图"""
        if self.scale_factor < self.MAX_SCALE:
            self.scale_factor *= self.ZOOM_FACTOR
            self.updateScaledPixmap()
            
    def zoomOut(self):
        """缩小地图"""
        if self.scale_factor > self.MIN_SCALE:
            self.scale_factor /= self.ZOOM_FACTOR
            self.updateScaledPixmap()
            
    def setDrawingTool(self, tool):
        """设置绘图工具"""
        self.drawing_tool = tool
        
    def setDrawingColor(self, color):
        """设置绘图颜色"""
        self.pen_color = color
        
    def setLineWidth(self, width):
        """设置线宽"""
        self.pen_width = width
        
    def undo(self):
        """撤销上一步操作"""
        if len(self.history) > 1:
            self.history.pop()
            self.pixmap = self.history[-1].copy()
            self.update()
            
    def clear(self):
        """清除所有绘制，恢复到初始状态"""
        if self.history:
            self.pixmap = self.history[0].copy()
            self.history = [self.pixmap.copy()]
            self.update()
            
    def getImage(self):
        """获取绘制后的图像（QPixmap格式）"""
        return self.pixmap.copy()
    
    def saveImage(self, filename=None):
        """
        保存绘制后的图像到文件
        
        参数:
            filename: 文件路径，如果为None则弹出文件选择对话框
            
        返回:
            bool: 是否成功保存
        """
        if filename is None:
            filename, _ = QFileDialog.getSaveFileName(
                self, "保存地图标绘", "./", 
                "PNG图像 (*.png);;JPEG图像 (*.jpg);;所有文件 (*.*)"
            )
            
        if filename:
            return self.pixmap.save(filename)
        return False
        
    def paintEvent(self, event):
        """绘制事件处理"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setRenderHint(QPainter.SmoothPixmapTransform)
        
        # 绘制背景
        painter.fillRect(self.rect(), QColor(40, 40, 40))
        
        # 绘制底图（居中显示）
        if not self.pixmap.isNull():
            # 计算居中位置
            x = (self.width() - self.pixmap.width()) // 2 + self.offset.x()
            y = (self.height() - self.pixmap.height()) // 2 + self.offset.y()
            painter.drawPixmap(x, y, self.pixmap)
            
        # 绘制临时内容（如拖动中的形状）
        if self.drawing and not self.temp_pixmap.isNull():
            x = (self.width() - self.temp_pixmap.width()) // 2 + self.offset.x()
            y = (self.height() - self.temp_pixmap.height()) // 2 + self.offset.y()
            painter.drawPixmap(x, y, self.temp_pixmap)
            
    def wheelEvent(self, event):
        """鼠标滚轮事件处理"""
        if self.has_map:
            # 获取滚轮方向
            delta = event.angleDelta().y()
            
            # 根据滚轮方向缩放
            if delta > 0:
                self.zoomIn()  # 放大
            else:
                self.zoomOut()  # 缩小
                
            # 阻止事件传播
            event.accept()
            
    def mousePressEvent(self, event):
        """鼠标按下事件处理"""
        if event.button() == Qt.LeftButton:
            # 计算图像坐标系中的点击位置
            img_pos = self._mapToImagePos(event.pos())
            
            # 检查点击是否在图像范围内
            if self._isPointInImage(img_pos):
                if event.modifiers() & Qt.ControlModifier:
                    # 按住Ctrl键进行平移
                    self.panning = True
                    self.pan_start_pos = event.pos()
                else:
                    # 正常绘制
                    self.drawing = True
                    self.last_point = img_pos
                    
                    # 对于敌人和人质标记，直接在点击位置绘制符号
                    if self.drawing_tool == self.TOOL_ENEMY:
                        self._drawSymbol(img_pos, self.enemy_color)
                        
                    elif self.drawing_tool == self.TOOL_HOSTAGE:
                        self._drawSymbol(img_pos, self.hostage_color)
                
    def mouseMoveEvent(self, event):
        """鼠标移动事件处理"""
        if self.panning:
            # 平移地图
            delta = event.pos() - self.pan_start_pos
            self.offset += delta
            self.pan_start_pos = event.pos()
            self.update()
        elif self.drawing:
            # 计算图像坐标系中的当前位置
            img_pos = self._mapToImagePos(event.pos())
            
            # 检查是否在图像范围内
            if self._isPointInImage(img_pos):
                self.current_point = img_pos
                
                if self.drawing_tool == self.TOOL_PEN:
                    # 自由绘制，直接在pixmap上绘制
                    painter = QPainter(self.pixmap)
                    painter.setPen(QPen(self.pen_color, self.pen_width, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
                    painter.drawLine(self.last_point, self.current_point)
                    painter.end()
                    
                    # 更新上一个点
                    self.last_point = self.current_point
                    
                elif self.drawing_tool == self.TOOL_ERASER:
                    # 橡皮擦，绘制白色圆形
                    painter = QPainter(self.pixmap)
                    painter.setPen(Qt.NoPen)
                    painter.setBrush(QBrush(Qt.white))
                    painter.drawEllipse(self.current_point, self.eraser_size, self.eraser_size)
                    painter.end()
                    
                    # 更新上一个点
                    self.last_point = self.current_point
                    
                # 更新界面
                self.update()
            
    def mouseReleaseEvent(self, event):
        """鼠标释放事件处理"""
        if event.button() == Qt.LeftButton:
            if self.panning:
                self.panning = False
            elif self.drawing:
                self.drawing = False
                
                # 保存到历史记录
                self.history.append(self.pixmap.copy())
                
            # 更新界面
            self.update()
            
    def _mapToImagePos(self, pos):
        """将窗口坐标映射到图像坐标"""
        # 计算图像左上角在窗口中的位置
        x_offset = (self.width() - self.pixmap.width()) // 2 + self.offset.x()
        y_offset = (self.height() - self.pixmap.height()) // 2 + self.offset.y()
        
        # 计算相对于图像左上角的坐标
        return QPoint(pos.x() - x_offset, pos.y() - y_offset)
        
    def _isPointInImage(self, pos):
        """检查点是否在图像范围内"""
        return (0 <= pos.x() < self.pixmap.width() and 
                0 <= pos.y() < self.pixmap.height())
            
    def _drawSymbol(self, position, color):
        """
        绘制符号（敌人或人质标记）
        
        参数:
            position: 符号位置
            color: 符号颜色
        """
        try:
            painter = QPainter(self.pixmap)
            painter.setRenderHint(QPainter.Antialiasing)  # 抗锯齿
            
            # 设置画笔
            pen = QPen(color, 3, Qt.SolidLine)
            painter.setPen(pen)
            
            # 设置画刷 - 半透明填充
            brush = QBrush(QColor(color.red(), color.green(), color.blue(), 80))
            painter.setBrush(brush)
            
            # 符号尺寸
            size = self.SYMBOL_SIZE
            
            # 绘制外圆
            painter.drawEllipse(position, size, size)
            
            # 绘制内圆
            painter.setBrush(QBrush(QColor(color.red(), color.green(), color.blue(), 150)))
            painter.drawEllipse(position, int(size/2), int(size/2))
            
            # 绘制十字线 - 确保使用整数坐标
            painter.setPen(QPen(Qt.white, 2, Qt.SolidLine))
            x1 = int(position.x() - size/2)
            y1 = int(position.y())
            x2 = int(position.x() + size/2)
            y2 = int(position.y())
            painter.drawLine(x1, y1, x2, y2)
            
            x1 = int(position.x())
            y1 = int(position.y() - size/2)
            x2 = int(position.x())
            y2 = int(position.y() + size/2)
            painter.drawLine(x1, y1, x2, y2)
            
            # 添加文字标签
            if color == self.enemy_color:
                text = "敌"
            elif color == self.hostage_color:
                text = "质"
            else:
                text = ""
                
            if text:
                painter.setPen(Qt.white)
                font = QFont("Arial", 10)
                font.setBold(True)
                painter.setFont(font)
                painter.drawText(int(position.x() - 7), int(position.y() + 5), text)
            
            painter.end()
            
            # 保存到历史记录
            self.history.append(self.pixmap.copy())
            
            # 更新界面
            self.update()
            
        except Exception as e:
            print(f"绘制符号错误: {e}")
            # 如果出错，确保painter被结束
            if painter.isActive():
                painter.end()


class BattleMapToolbar(QWidget):
    """
    作战地图工具栏类
    
    提供地图操作、绘图工具选择、颜色选择等功能
    
    主要属性:
        map_area: 关联的地图绘图区域
        
    主要方法:
        setupUI: 设置用户界面
        connectSignals: 连接信号和槽
    """
    
    def __init__(self, map_area, parent=None):
        """
        初始化作战地图工具栏
        
        参数:
            map_area: 关联的地图绘图区域
            parent: 父窗口
        """
        super().__init__(parent)
        self.map_area = map_area
        self.setupUI()
        self.connectSignals()
        
    def setupUI(self):
        """设置用户界面"""
        # 主布局
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(5)
        
        # 地图操作布局
        map_layout = QHBoxLayout()
        
        # 导入地图按钮
        self.import_map_btn = QPushButton("📷 导入地图")
        self.import_map_btn.setFixedSize(100, 30)
        
        # 清除地图按钮
        self.clear_map_btn = QPushButton("🗑️ 清除地图")
        self.clear_map_btn.setFixedSize(100, 30)
        
        # 保存地图按钮
        self.save_map_btn = QPushButton("💾 保存地图")
        self.save_map_btn.setFixedSize(100, 30)
        
        # 撤销按钮
        self.undo_btn = QPushButton("↩️ 撤销")
        self.undo_btn.setFixedSize(80, 30)
        
        # 将按钮添加到地图操作布局
        map_layout.addWidget(self.import_map_btn)
        map_layout.addWidget(self.clear_map_btn)
        map_layout.addWidget(self.save_map_btn)
        map_layout.addWidget(self.undo_btn)
        map_layout.addStretch()
        
        # 工具选择布局
        tools_layout = QHBoxLayout()
        
        # 创建工具按钮组
        self.tools_group = QButtonGroup(self)
        
        # 笔刷工具按钮
        self.pen_btn = QPushButton("✏️ 笔刷")
        self.pen_btn.setCheckable(True)
        self.pen_btn.setChecked(True)
        self.pen_btn.setFixedSize(80, 30)
        self.tools_group.addButton(self.pen_btn, MapDrawingArea.TOOL_PEN)
        
        # 标记敌人按钮
        self.enemy_btn = QPushButton("🔴 敌人")
        self.enemy_btn.setCheckable(True)
        self.enemy_btn.setFixedSize(80, 30)
        self.tools_group.addButton(self.enemy_btn, MapDrawingArea.TOOL_ENEMY)
        
        # 标记人质按钮
        self.hostage_btn = QPushButton("🔵 人质")
        self.hostage_btn.setCheckable(True)
        self.hostage_btn.setFixedSize(80, 30)
        self.tools_group.addButton(self.hostage_btn, MapDrawingArea.TOOL_HOSTAGE)
        
        # 橡皮擦按钮
        self.eraser_btn = QPushButton("🧽 橡皮擦")
        self.eraser_btn.setCheckable(True)
        self.eraser_btn.setFixedSize(80, 30)
        self.tools_group.addButton(self.eraser_btn, MapDrawingArea.TOOL_ERASER)
        
        # 将按钮添加到工具布局
        tools_layout.addWidget(self.pen_btn)
        tools_layout.addWidget(self.enemy_btn)
        tools_layout.addWidget(self.hostage_btn)
        tools_layout.addWidget(self.eraser_btn)
        
        # 添加分隔符
        separator = QFrame()
        separator.setFrameShape(QFrame.VLine)
        separator.setFrameShadow(QFrame.Sunken)
        separator.setStyleSheet("background-color: rgba(150, 150, 150, 0.5);")
        separator.setFixedSize(1, 25)
        tools_layout.addWidget(separator)
        
        # 缩放按钮
        self.zoom_in_btn = QPushButton("🔍+")
        self.zoom_in_btn.setFixedSize(40, 30)
        self.zoom_in_btn.setToolTip("放大地图")
        
        self.zoom_out_btn = QPushButton("🔍-")
        self.zoom_out_btn.setFixedSize(40, 30)
        self.zoom_out_btn.setToolTip("缩小地图")
        
        self.fit_view_btn = QPushButton("🔍⟲")
        self.fit_view_btn.setFixedSize(40, 30)
        self.fit_view_btn.setToolTip("适应窗口大小")
        
        tools_layout.addWidget(self.zoom_in_btn)
        tools_layout.addWidget(self.zoom_out_btn)
        tools_layout.addWidget(self.fit_view_btn)
        tools_layout.addStretch()
        
        # 颜色和线宽布局
        settings_layout = QHBoxLayout()
        
        # 颜色选择按钮
        self.color_btn = QPushButton("🎨 颜色")
        self.color_btn.setFixedSize(80, 30)
        
        # 颜色预览
        self.color_preview = QLabel()
        self.color_preview.setFixedSize(30, 30)
        self.color_preview.setStyleSheet("background-color: #00FF00; border: 1px solid black;")
        
        # 线宽标签
        self.width_label = QLabel("线宽:")
        self.width_label.setStyleSheet("color: white;")
        
        # 线宽滑动条
        self.width_slider = QSlider(Qt.Horizontal)
        self.width_slider.setMinimum(1)
        self.width_slider.setMaximum(20)
        self.width_slider.setValue(3)
        self.width_slider.setFixedWidth(100)
        
        # 线宽数值显示
        self.width_value = QLabel("3")
        self.width_value.setStyleSheet("color: white;")
        self.width_value.setFixedWidth(20)
        
        # 将控件添加到设置布局
        settings_layout.addWidget(self.color_btn)
        settings_layout.addWidget(self.color_preview)
        settings_layout.addWidget(self.width_label)
        settings_layout.addWidget(self.width_slider)
        settings_layout.addWidget(self.width_value)
        settings_layout.addStretch()
        
        # 将所有布局添加到主布局
        main_layout.addLayout(map_layout)
        main_layout.addLayout(tools_layout)
        main_layout.addLayout(settings_layout)
        
        # 设置样式
        self.setStyleSheet("""
            QPushButton {
                background: rgba(60, 60, 60, 0.8);
                color: white;
                font-size: 12px;
                border: 1px solid rgba(100, 100, 100, 0.8);
                border-radius: 5px;
            }
            QPushButton:hover {
                background: rgba(80, 80, 80, 1.0);
                border: 1px solid rgba(150, 150, 150, 1.0);
            }
            QPushButton:checked {
                background: rgba(0, 120, 215, 0.8);
                border: 1px solid rgba(0, 150, 255, 1.0);
            }
            QLabel {
                color: white;
                font-size: 12px;
            }
            QSlider::groove:horizontal {
                height: 8px;
                background: rgba(80, 80, 80, 0.8);
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: rgba(0, 120, 215, 0.8);
                border: 1px solid rgba(0, 150, 255, 1.0);
                width: 18px;
                margin: -5px 0;
                border-radius: 9px;
            }
        """)
        
        # 设置工具提示
        self.import_map_btn.setToolTip("导入地图图片文件")
        self.clear_map_btn.setToolTip("清除当前标绘内容")
        self.save_map_btn.setToolTip("保存标绘结果")
        self.undo_btn.setToolTip("撤销上一步操作")
        self.pen_btn.setToolTip("自由绘制工具")
        self.enemy_btn.setToolTip("标记敌人位置")
        self.hostage_btn.setToolTip("标记人质位置")
        self.eraser_btn.setToolTip("橡皮擦工具")
        self.color_btn.setToolTip("选择绘制颜色")
        
    def connectSignals(self):
        """连接信号和槽"""
        # 地图操作按钮信号 - 这些信号将由BattleMapWidget处理
        # self.import_map_btn.clicked.connect(self.map_area.loadMap)
        # self.clear_map_btn.clicked.connect(self.map_area.clear)
        # self.save_map_btn.clicked.connect(self.map_area.saveImage)
        self.undo_btn.clicked.connect(self.map_area.undo)
        
        # 工具按钮组信号
        self.tools_group.buttonClicked.connect(self._onToolButtonClicked)
        
        # 颜色按钮信号
        self.color_btn.clicked.connect(self._onColorButtonClicked)
        
        # 线宽滑动条信号
        self.width_slider.valueChanged.connect(self._onWidthChanged)
        
        # 缩放按钮信号
        self.zoom_in_btn.clicked.connect(self.map_area.zoomIn)
        self.zoom_out_btn.clicked.connect(self.map_area.zoomOut)
        self.fit_view_btn.clicked.connect(self.map_area.fitToView)
        
    def _onToolButtonClicked(self, button):
        """工具按钮点击处理"""
        tool_id = self.tools_group.id(button)
        self.map_area.setDrawingTool(tool_id)
        
    def _onColorButtonClicked(self):
        """颜色按钮点击处理"""
        color = QColorDialog.getColor(self.map_area.pen_color, self, "选择颜色")
        if color.isValid():
            self.map_area.setDrawingColor(color)
            self.color_preview.setStyleSheet(f"background-color: {color.name()}; border: 1px solid black;")
            
    def _onWidthChanged(self, value):
        """线宽改变处理"""
        self.map_area.setLineWidth(value)
        self.width_value.setText(str(value))


class BattleMapWidget(QWidget):
    """
    作战地图组件类
    
    集成地图显示和工具栏，提供完整的作战地图功能
    
    主要属性:
        map_area: 地图绘图区域
        toolbar: 工具栏
        
    主要方法:
        setupUI: 设置用户界面
        loadMap: 加载地图
        saveMap: 保存地图
    """
    
    def __init__(self, parent=None):
        """
        初始化作战地图组件
        
        参数:
            parent: 父窗口
        """
        super().__init__(parent)
        self.setupUI()
        
    def setupUI(self):
        """设置用户界面"""
        # 主布局
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # 创建标题区域
        title_frame = QFrame()
        title_frame.setFixedHeight(50)
        title_frame.setStyleSheet("background: rgba(30, 30, 30, 0.7);")
        title_layout = QHBoxLayout(title_frame)
        title_layout.setContentsMargins(20, 5, 20, 5)
        
        # 标题标签
        title_label = QLabel("作战地图标绘系统")
        title_label.setStyleSheet("""
            color: #FFD700;
            font-size: 18px;
            font-weight: bold;
        """)
        
        # 状态标签
        self.status_label = QLabel("准备就绪")
        self.status_label.setStyleSheet("""
            color: #00FF00;
            font-size: 12px;
        """)
        
        title_layout.addWidget(title_label)
        title_layout.addStretch()
        title_layout.addWidget(self.status_label)
        
        # 创建地图绘图区域
        self.map_area = MapDrawingArea()
        
        # 创建工具栏
        self.toolbar = BattleMapToolbar(self.map_area)
        
        # 创建滚动区域容器
        map_container = QFrame()
        map_container.setStyleSheet("background: rgba(40, 40, 40, 0.5);")
        map_container_layout = QVBoxLayout(map_container)
        map_container_layout.setContentsMargins(10, 10, 10, 10)
        map_container_layout.addWidget(self.map_area)
        
        # 将工具栏和地图区域添加到主布局
        main_layout.addWidget(title_frame)
        main_layout.addWidget(self.toolbar)
        main_layout.addWidget(map_container, 1)  # 1表示拉伸因子
        
        # 连接信号
        self.toolbar.import_map_btn.clicked.connect(self._onMapLoaded)
        self.toolbar.save_map_btn.clicked.connect(self._onMapSaved)
        self.toolbar.clear_map_btn.clicked.connect(self._onMapCleared)
        
    def _onMapLoaded(self):
        """地图加载回调"""
        if self.map_area.loadMap():
            self.status_label.setText("地图已加载")
            self.status_label.setStyleSheet("color: #00FF00; font-size: 12px;")
        
    def _onMapSaved(self):
        """地图保存回调"""
        if self.map_area.saveImage():
            self.status_label.setText("地图已保存")
            self.status_label.setStyleSheet("color: #00FF00; font-size: 12px;")
            
    def _onMapCleared(self):
        """地图清除回调"""
        self.map_area.clear()
        self.status_label.setText("地图已清除")
        self.status_label.setStyleSheet("color: #FFA500; font-size: 12px;")
        
    def loadMap(self, file_path=None):
        """
        加载地图
        
        参数:
            file_path: 图片文件路径，如果为None则弹出文件选择对话框
            
        返回:
            bool: 是否成功加载地图
        """
        return self.map_area.loadMap(file_path)
        
    def saveMap(self, file_path=None):
        """
        保存地图
        
        参数:
            file_path: 文件路径，如果为None则弹出文件选择对话框
            
        返回:
            bool: 是否成功保存
        """
        return self.map_area.saveImage(file_path) 