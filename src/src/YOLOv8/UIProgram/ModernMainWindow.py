#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
室内多传感器寻敌系统主程序界面
========================================

现代化主窗口程序，提供完整的人脸检测功能

功能特性：
- 支持图片、视频、摄像头、批量检测
- 现代化UI设计，半透明背景效果
- 支持窗口自由调整大小和全屏模式
- 批量检测支持图片浏览功能（上一张/下一张）
- 实时检测结果显示和性能统计
- 智能路径查找，支持多种启动方式

界面布局：
- 左侧侧边栏：主要功能按钮
- 右侧显示区域：堆叠页面切换
  - 主页：系统介绍
  - 检测页面：检测功能和结果显示

检测功能：
- 图片检测：支持jpg、png、jpeg、bmp格式
- 视频检测：支持mp4、avi、mov、mkv格式
- 摄像头检测：实时检测和显示
- 批量检测：文件夹批量处理，支持图片浏览

窗口控制：
- F11键：切换全屏/窗口模式
- ESC键：全屏时退出全屏
- 支持拖拽移动和调整大小

作者: 郭晋鹏团队
版本: 2.0.0
"""

import sys
import os
import time
import cv2
import numpy as np
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PIL import ImageFont
from ultralytics import YOLO

# 确保可以从任何位置导入
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

# 导入检测模块
import detect_tools as tools
import Config

# 导入作战地图模块
from UIProgram.BattleMapTools import BattleMapWidget
# 导入目标导航模块
from UIProgram.NavigationTools import NavigationWidget


class ModernMainWindow(QMainWindow):
    """
    现代化主窗口类
    
    继承自QMainWindow，提供完整的人脸检测功能界面。
    包含检测模型加载、UI创建、事件处理等功能。
    
    主要属性：
        model: YOLO检测模型
        batch_images: 批量检测图片列表
        current_batch_index: 当前显示的图片索引
        is_fullscreen: 全屏状态标志
        
    主要方法：
        initDetection(): 初始化检测模型和变量
        setupUI(): 创建用户界面
        open_img(): 图片检测
        video_show(): 视频检测
        camera_show(): 摄像头检测
        batch_detect(): 批量检测
    """
    
    def __init__(self, parent=None):
        """
        初始化主窗口
        
        参数：
            parent: 父窗口，默认为None
        """
        super().__init__(parent)
        self.initDetection()  # 初始化检测功能
        self.setupUI()  # 设置用户界面
        
        # 窗口拖拽相关变量
        self.dragging = False
        self.drag_position = QPoint()
    
    def initDetection(self):
        """初始化检测相关变量"""
        # 检测结果显示尺寸
        self.show_width = 500
        self.show_height = 350
        
        # 当前处理的文件路径
        self.org_path = None
        self.org_img = None
        self.draw_img = None
        
        # 摄像头相关
        self.is_camera_open = False
        self.cap = None
        
        # 批量检测相关
        self.batch_images = []  # 批量检测的图片列表
        self.current_batch_index = 0  # 当前显示的图片索引
        self.batch_results = {}  # 批量检测结果缓存
        
        # 加载检测模型
        try:
            # 智能路径查找 - 支持从根目录或UIProgram目录启动
            def find_file(filename):
                """智能查找文件路径"""
                # 尝试多个可能的路径
                possible_paths = [
                    filename,  # 当前目录
                    os.path.join('..', filename),  # 上级目录
                    os.path.join(os.path.dirname(os.path.dirname(__file__)), filename),  # 项目根目录
                ]
                
                for path in possible_paths:
                    if os.path.exists(path):
                        return path
                return filename  # 如果都找不到，返回原文件名
            
            # 修正模型路径
            model_path = find_file(Config.model_path)
            
            self.model = YOLO(model_path, task='detect')
            self.model(np.zeros((48, 48, 3)))  # 预先加载推理模型
            
            # 修正字体路径
            font_path = find_file(os.path.join('Font', 'platech.ttf'))
            
            self.fontC = ImageFont.truetype(font_path, 25, 0)
            self.colors = tools.Colors()
            print("模型加载成功")
        except Exception as e:
            print(f"模型加载失败: {e}")
            self.model = None
            self.fontC = None
        
        # 检测结果
        self.results = None
        self.location_list = []
        self.cls_list = []
        self.conf_list = []
        
        # 定时器
        self.timer_camera = QTimer()
        self.timer_camera.timeout.connect(self.open_frame)
        
    def setupUI(self):
        """设置用户界面"""
        # 设置窗口属性 - 支持自由调整大小和全屏
        self.setWindowTitle("室内环境多传感器寻敌系统")
        self.setMinimumSize(700, 500)  # 设置最小尺寸（稍微缩减）
        self.resize(1000, 700)  # 初始尺寸（稍微缩减）
        self.setWindowFlags(Qt.FramelessWindowHint)
        
        # 全屏状态标志
        self.is_fullscreen = False
        
        # 创建中央组件和主布局 - 按照UI文件的方式
        self.central_widget = QWidget(self)
        self.central_widget.setStyleSheet("background: transparent;")
        self.setCentralWidget(self.central_widget)
        
        self.main_layout = QHBoxLayout(self.central_widget)
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        self.main_layout.setSpacing(0)
        
        # 创建侧边栏
        self.createSidebar()
        
        # 创建主工作区
        self.createMainWorkspace()
        
        # 设置样式
        self.setStyles()
        
        # 窗口居中显示
        self.centerWindow()
        
    def createSidebar(self):
        """创建侧边栏"""
        # 创建左侧侧边栏 - 按照UI文件的方式
        self.sidebar = QFrame()
        self.sidebar.setObjectName("sidebar")
        self.sidebar.setFixedWidth(100)
        
        self.sidebar_layout = QVBoxLayout(self.sidebar)
        self.sidebar_layout.setContentsMargins(10, 50, 10, 10)
        self.sidebar_layout.setSpacing(30)
        
        # 创建主功能按钮 - 按照UI文件的方式
        self.target_detection_btn = QPushButton("目标识别")
        self.target_detection_btn.setObjectName("sidebarButton")
        self.target_detection_btn.setFixedHeight(30)
        self.target_detection_btn.setToolTip("目标识别功能")
        
        self.map_btn = QPushButton("作战地图")
        self.map_btn.setObjectName("sidebarButton")
        self.map_btn.setFixedHeight(30)
        self.map_btn.setToolTip("作战地图标绘功能")
        
        self.navigation_btn = QPushButton("目标导航")
        self.navigation_btn.setObjectName("sidebarButton")
        self.navigation_btn.setFixedHeight(30)
        self.navigation_btn.setToolTip("目标导航功能")
        
        # 添加到布局
        self.sidebar_layout.addWidget(self.target_detection_btn)
        self.sidebar_layout.addWidget(self.map_btn)
        self.sidebar_layout.addWidget(self.navigation_btn)
        self.sidebar_layout.addStretch()
        
        # 创建垂直布局容器并添加到主布局
        left_container = QWidget()
        left_container.setStyleSheet("background: transparent;")
        left_layout = QVBoxLayout(left_container)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(0)
        left_layout.addWidget(self.sidebar)
        
        self.main_layout.addWidget(left_container)
        
    def createMainWorkspace(self):
        """创建主工作区"""
        # 创建右侧显示区域 - 按照UI文件的方式
        self.display_area = QFrame()
        self.display_area.setObjectName("displayArea")
        
        self.display_layout = QVBoxLayout(self.display_area)
        self.display_layout.setContentsMargins(0, 0, 0, 0)
        self.display_layout.setSpacing(0)
        
        # 创建堆叠窗口部件 - 按照UI文件的方式
        self.stacked_widget = QStackedWidget()
        self.stacked_widget.setStyleSheet("background: transparent;")
        self.display_layout.addWidget(self.stacked_widget)
        
        # 创建主页页面
        self.createHomePage()
        
        # 创建识别页面
        self.createDetectionPage()
        
        # 创建作战地图页面
        self.createBattleMapPage()
        
        # 创建目标导航页面
        self.createNavigationPage()
        
        # 创建关闭按钮和调整大小控件
        self.createCloseButton()
        self.createResizeGrip()
        
        # 连接信号
        self.connectSignals()
        
        self.main_layout.addWidget(self.display_area)
        
    def createHomePage(self):
        """创建主页页面"""
        self.home_page = QWidget()
        self.home_page.setStyleSheet("background: transparent;")
        
        home_layout = QVBoxLayout(self.home_page)
        home_layout.setContentsMargins(50, 50, 50, 50)
        home_layout.setSpacing(0)
        
        # 系统名称标签 - 按照UI文件的样式
        self.system_name_label = QLabel("室内环境多传感器寻敌系统")
        self.system_name_label.setObjectName("systemNameLabel")
        self.system_name_label.setAlignment(Qt.AlignCenter)
        
        # 添加弹性空间和标签
        home_layout.addStretch(2)
        home_layout.addWidget(self.system_name_label)
        home_layout.addStretch(3)
        
        self.stacked_widget.addWidget(self.home_page)
        
    def createDetectionPage(self):
        """创建识别页面"""
        self.detection_page = QWidget()
        self.detection_page.setStyleSheet("background: transparent;")
        
        detection_layout = QVBoxLayout(self.detection_page)
        detection_layout.setContentsMargins(20, 20, 20, 20)
        detection_layout.setSpacing(10)
        
        # 页面标题
        title_label = QLabel("目标识别模式")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("""
            QLabel {
                color: #FFD700;
                font-size: 18px;
                font-weight: bold;
                background: rgba(0, 0, 0, 0.5);
                border-radius: 10px;
                padding: 10px;
                border: 2px solid rgba(255, 215, 0, 0.6);
            }
        """)
        detection_layout.addWidget(title_label)
        
        # 功能按钮区域
        button_layout = QHBoxLayout()
        
        # 图片检测按钮
        self.image_btn = QPushButton("📷 图片检测")
        self.image_btn.setFixedSize(120, 40)
        self.image_btn.setStyleSheet("""
            QPushButton {
                background: rgba(76, 175, 80, 0.8);
                color: white;
                font-size: 12px;
                font-weight: bold;
                border: 2px solid rgba(255, 255, 255, 0.6);
                border-radius: 8px;
            }
            QPushButton:hover {
                background: rgba(76, 175, 80, 1.0);
                border: 2px solid rgba(255, 215, 0, 1.0);
            }
        """)
        
        # 视频检测按钮
        self.video_btn = QPushButton("🎥 视频检测")
        self.video_btn.setFixedSize(120, 40)
        self.video_btn.setStyleSheet("""
            QPushButton {
                background: rgba(33, 150, 243, 0.8);
                color: white;
                font-size: 12px;
                font-weight: bold;
                border: 2px solid rgba(255, 255, 255, 0.6);
                border-radius: 8px;
            }
            QPushButton:hover {
                background: rgba(33, 150, 243, 1.0);
                border: 2px solid rgba(255, 215, 0, 1.0);
            }
        """)
        
        # 摄像头检测按钮
        self.camera_btn = QPushButton("📹 摄像头检测")
        self.camera_btn.setFixedSize(120, 40)
        self.camera_btn.setStyleSheet("""
            QPushButton {
                background: rgba(255, 152, 0, 0.8);
                color: white;
                font-size: 12px;
                font-weight: bold;
                border: 2px solid rgba(255, 255, 255, 0.6);
                border-radius: 8px;
            }
            QPushButton:hover {
                background: rgba(255, 152, 0, 1.0);
                border: 2px solid rgba(255, 215, 0, 1.0);
            }
        """)
        
        # 批量检测按钮
        self.batch_btn = QPushButton("📁 批量检测")
        self.batch_btn.setFixedSize(120, 40)
        self.batch_btn.setStyleSheet("""
            QPushButton {
                background: rgba(156, 39, 176, 0.8);
                color: white;
                font-size: 12px;
                font-weight: bold;
                border: 2px solid rgba(255, 255, 255, 0.6);
                border-radius: 8px;
            }
            QPushButton:hover {
                background: rgba(156, 39, 176, 1.0);
                border: 2px solid rgba(255, 215, 0, 1.0);
            }
        """)
        
        button_layout.addWidget(self.image_btn)
        button_layout.addWidget(self.video_btn)
        button_layout.addWidget(self.camera_btn)
        button_layout.addWidget(self.batch_btn)
        button_layout.addStretch()
        
        detection_layout.addLayout(button_layout)
        
        # 显示区域 - 设置为可伸缩
        self.display_label = QLabel("请选择检测模式")
        self.display_label.setAlignment(Qt.AlignCenter)
        self.display_label.setMinimumSize(400, 300)
        self.display_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.display_label.setScaledContents(False)  # 保持图像比例
        self.display_label.setStyleSheet("""
            QLabel {
                background: rgba(255, 255, 255, 0.1);
                border: 2px dashed rgba(255, 255, 255, 0.5);
                border-radius: 10px;
                color: white;
                font-size: 14px;
            }
        """)
        detection_layout.addWidget(self.display_label)
        
        # 批量检测导航按钮区域
        self.batch_nav_layout = QHBoxLayout()
        
        # 上一张按钮
        self.prev_btn = QPushButton("⬅ 上一张")
        self.prev_btn.setFixedSize(80, 30)
        self.prev_btn.setEnabled(False)  # 默认禁用
        self.prev_btn.setStyleSheet("""
            QPushButton {
                background: rgba(96, 96, 96, 0.8);
                color: white;
                font-size: 10px;
                font-weight: bold;
                border: 2px solid rgba(255, 255, 255, 0.6);
                border-radius: 5px;
            }
            QPushButton:enabled {
                background: rgba(63, 81, 181, 0.8);
            }
            QPushButton:enabled:hover {
                background: rgba(63, 81, 181, 1.0);
                border: 2px solid rgba(255, 215, 0, 1.0);
            }
        """)
        
        # 图片索引标签
        self.batch_index_label = QLabel("")
        self.batch_index_label.setAlignment(Qt.AlignCenter)
        self.batch_index_label.setStyleSheet("""
            QLabel {
                color: white;
                font-size: 10px;
                background: rgba(0, 0, 0, 0.5);
                padding: 5px;
                border-radius: 5px;
            }
        """)
        
        # 下一张按钮
        self.next_btn = QPushButton("下一张 ➡")
        self.next_btn.setFixedSize(80, 30)
        self.next_btn.setEnabled(False)  # 默认禁用
        self.next_btn.setStyleSheet("""
            QPushButton {
                background: rgba(96, 96, 96, 0.8);
                color: white;
                font-size: 10px;
                font-weight: bold;
                border: 2px solid rgba(255, 255, 255, 0.6);
                border-radius: 5px;
            }
            QPushButton:enabled {
                background: rgba(63, 81, 181, 0.8);
            }
            QPushButton:enabled:hover {
                background: rgba(63, 81, 181, 1.0);
                border: 2px solid rgba(255, 215, 0, 1.0);
            }
        """)
        
        self.batch_nav_layout.addStretch()
        self.batch_nav_layout.addWidget(self.prev_btn)
        self.batch_nav_layout.addWidget(self.batch_index_label)
        self.batch_nav_layout.addWidget(self.next_btn)
        self.batch_nav_layout.addStretch()
        
        # 默认隐藏导航按钮
        self.batch_nav_widget = QWidget()
        self.batch_nav_widget.setLayout(self.batch_nav_layout)
        self.batch_nav_widget.setVisible(False)
        detection_layout.addWidget(self.batch_nav_widget)
        
        # 信息显示区域
        info_layout = QHBoxLayout()
        
        # 检测结果
        self.result_label = QLabel("检测结果：待检测")
        self.result_label.setStyleSheet("""
            QLabel {
                color: #00FF00;
                font-size: 12px;
                background: rgba(0, 0, 0, 0.5);
                padding: 5px;
                border-radius: 5px;
            }
        """)
        
        # 检测时间
        self.time_label = QLabel("检测时间：0ms")
        self.time_label.setStyleSheet("""
            QLabel {
                color: #FFD700;
                font-size: 12px;
                background: rgba(0, 0, 0, 0.5);
                padding: 5px;
                border-radius: 5px;
            }
        """)
        
        info_layout.addWidget(self.result_label)
        info_layout.addWidget(self.time_label)
        info_layout.addStretch()
        
        detection_layout.addLayout(info_layout)
        
        self.stacked_widget.addWidget(self.detection_page)
        
    def createBattleMapPage(self):
        """创建作战地图页面"""
        # 创建作战地图组件
        self.battle_map_widget = BattleMapWidget()
        
        # 添加到堆叠窗口
        self.stacked_widget.addWidget(self.battle_map_widget)
        
    def showBattleMapPage(self):
        """显示作战地图页面"""
        self.stacked_widget.setCurrentWidget(self.battle_map_widget)
        
    def createNavigationPage(self):
        """创建目标导航页面"""
        # 创建导航组件
        self.navigation_widget = NavigationWidget()
        
        # 添加到堆叠窗口
        self.stacked_widget.addWidget(self.navigation_widget)
        
    def showNavigationPage(self):
        """显示目标导航页面"""
        self.stacked_widget.setCurrentWidget(self.navigation_widget)
        
    def connectSignals(self):
        """连接信号"""
        # 侧边栏按钮信号
        self.target_detection_btn.clicked.connect(self.showDetectionPage)
        self.map_btn.clicked.connect(self.showBattleMapPage)  # 连接到作战地图页面
        self.navigation_btn.clicked.connect(self.showNavigationPage)  # 连接到目标导航页面
        
        # 检测功能按钮信号
        self.image_btn.clicked.connect(self.open_img)
        self.video_btn.clicked.connect(self.video_show)
        self.camera_btn.clicked.connect(self.camera_show)
        self.batch_btn.clicked.connect(self.batch_detect)
        
        # 批量检测导航按钮信号
        self.prev_btn.clicked.connect(self.show_prev_image)
        self.next_btn.clicked.connect(self.show_next_image)
        
    def showHomePage(self):
        """显示主页"""
        self.stacked_widget.setCurrentWidget(self.home_page)
        
    def showDetectionPage(self):
        """显示识别页面"""
        self.stacked_widget.setCurrentWidget(self.detection_page)
        
    def createCloseButton(self):
        """创建关闭按钮和全屏按钮"""
        # 全屏/还原按钮
        self.fullscreen_button = QPushButton("□", self)
        self.fullscreen_button.setFixedSize(40, 40)
        self.fullscreen_button.setObjectName("fullscreenButton")
        self.fullscreen_button.clicked.connect(self.toggleFullscreen)
        
        # 关闭按钮
        self.close_button = QPushButton("×", self)
        self.close_button.setFixedSize(40, 40)
        self.close_button.setObjectName("closeButton")
        self.close_button.clicked.connect(self.close)
        
    def createResizeGrip(self):
        """创建调整大小控件"""
        # 添加右下角的调整大小控件
        self.resize_grip = QSizeGrip(self)
        self.resize_grip.setFixedSize(20, 20)
        self.resize_grip.setStyleSheet("""
            QSizeGrip {
                background: rgba(255, 255, 255, 0.3);
                border: none;
            }
        """)
        
    def resizeEvent(self, event):
        """窗口大小改变事件"""
        if event:
            super().resizeEvent(event)
        # 调整全屏按钮和关闭按钮位置
        self.fullscreen_button.move(self.width() - 100, 10)
        self.close_button.move(self.width() - 50, 10)
        # 调整调整大小控件位置（全屏时隐藏）
        if hasattr(self, 'resize_grip'):
            if self.is_fullscreen:
                self.resize_grip.hide()
            else:
                self.resize_grip.show()
                self.resize_grip.move(self.width() - 20, self.height() - 20)
        
    def mousePressEvent(self, event):
        """鼠标按下事件 - 用于窗口拖拽"""
        if event.button() == Qt.LeftButton:
            self.dragging = True
            self.drag_position = event.globalPos() - self.frameGeometry().topLeft()
            event.accept()
            
    def mouseMoveEvent(self, event):
        """鼠标移动事件 - 用于窗口拖拽"""
        if event.buttons() == Qt.LeftButton and self.dragging:
            self.move(event.globalPos() - self.drag_position)
            event.accept()
            
    def mouseReleaseEvent(self, event):
        """鼠标释放事件 - 结束窗口拖拽"""
        self.dragging = False
        
    def keyPressEvent(self, event):
        """按键事件处理"""
        if event.key() == Qt.Key_F11:
            self.toggleFullscreen()
        elif event.key() == Qt.Key_Escape and self.is_fullscreen:
            self.toggleFullscreen()
        else:
            super().keyPressEvent(event)
            
    def toggleFullscreen(self):
        """切换全屏状态"""
        if self.is_fullscreen:
            # 退出全屏
            self.showNormal()
            self.is_fullscreen = False
            self.fullscreen_button.setText("□")
            # 恢复窗口边框
            self.setWindowFlags(Qt.FramelessWindowHint)
            self.show()
        else:
            # 进入全屏
            self.showFullScreen()
            self.is_fullscreen = True
            self.fullscreen_button.setText("⊡")
            # 隐藏调整大小控件
            if hasattr(self, 'resize_grip'):
                self.resize_grip.hide()
                
        # 更新按钮位置
        self.resizeEvent(None)
        
    def setStyles(self):
        """设置样式表"""
        # 按照UI文件的方式设置样式
        style_sheet = """
            ModernMainWindow {
                background-image: url(UIProgram/ui_imgs/icons/R6.png);
                background-repeat: no-repeat;
                background-position: center;
                background-attachment: fixed;
            }
            
            #sidebar {
                background: rgba(0, 0, 0, 0.4);
                border-right: 2px solid rgba(255, 255, 255, 0.3);
            }
            
            #sidebarButton {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, 
                    stop:0 rgba(255, 69, 0, 0.8), stop:1 rgba(220, 20, 60, 0.8));
                color: #FFFFFF;
                font-size: 12px;
                font-weight: bold;
                font-family: 'Arial Black', sans-serif;
                border: 2px solid rgba(255, 255, 255, 0.6);
                border-radius: 10px;
            }
            
            #sidebarButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, 
                    stop:0 rgba(255, 99, 71, 1.0), stop:1 rgba(255, 69, 0, 1.0));
                border: 2px solid rgba(255, 215, 0, 1.0);
            }
            
            #displayArea {
                background: transparent;
            }
            
            #systemNameLabel {
                color: #FF0000;
                font-size: 24px;
                font-weight: bold;
                font-family: 'Impact', 'Arial Black', sans-serif;

                background: rgba(0, 0, 0, 0.5);
                border-radius: 15px;
                padding: 30px;
                border: 2px solid rgba(255, 0, 0, 0.8);
            }
            
            #fullscreenButton {
                background: rgba(30, 144, 255, 0.8);
                color: white;
                font-size: 16px;
                font-weight: bold;
                border: 2px solid rgba(255, 255, 255, 0.6);
                border-radius: 20px;
            }
            
            #fullscreenButton:hover {
                background: rgba(65, 105, 225, 1.0);
                border: 2px solid rgba(255, 215, 0, 1.0);
            }
            
            #fullscreenButton:pressed {
                background: rgba(25, 25, 112, 1.0);
            }
            
            #closeButton {
                background: rgba(220, 20, 60, 0.8);
                color: white;
                font-size: 20px;
                font-weight: bold;
                border: 2px solid rgba(255, 255, 255, 0.6);
                border-radius: 20px;
            }
            
            #closeButton:hover {
                background: rgba(255, 69, 0, 1.0);
                border: 2px solid rgba(255, 215, 0, 1.0);
            }
            
            #closeButton:pressed {
                background: rgba(139, 0, 0, 1.0);
            }
        """
        
        self.setStyleSheet(style_sheet)
        
    def centerWindow(self):
        """窗口居中显示"""
        screen = QApplication.desktop().screenGeometry()
        x = (screen.width() - self.width()) // 2
        y = (screen.height() - self.height()) // 2
        self.move(x, y)
    
    # =============== 检测功能方法 ===============
    
    def open_img(self):
        """打开图片并进行检测"""
        if self.model is None:
            self.display_label.setText("模型未加载，无法进行检测")
            return
            
        # 关闭摄像头
        if self.cap:
            self.video_stop()
            
        # 隐藏批量导航按钮
        self.batch_nav_widget.setVisible(False)
            
        # 选择图片文件
        file_path, _ = QFileDialog.getOpenFileName(
            self, '选择图片', './', 
            "Image files (*.jpg *.jpeg *.png *.bmp)"
        )
        
        if not file_path:
            return
            
        try:
            self.org_path = file_path
            self.org_img = tools.img_cvread(self.org_path)
            
            # 进行检测
            t1 = time.time()
            self.results = self.model(self.org_path)[0]
            t2 = time.time()
            
            # 处理检测结果
            if len(self.results.boxes) > 0:
                self.location_list = self.results.boxes.xyxy.tolist()
                self.location_list = [list(map(int, e)) for e in self.location_list]
                self.cls_list = [int(i) for i in self.results.boxes.cls.tolist()]
                self.conf_list = ['%.2f%%' % (each*100) for each in self.results.boxes.conf.tolist()]
                
                # 绘制检测结果
                now_img = self.results.plot()
                self.draw_img = now_img
                
                # 显示结果
                self.display_detection_result(now_img)
                
                # 更新信息
                take_time = (t2 - t1) * 1000
                self.result_label.setText(f"检测结果：发现 {len(self.cls_list)} 个目标")
                self.time_label.setText(f"检测时间：{take_time:.1f}ms")
                
            else:
                # 没有检测到目标
                self.display_detection_result(self.org_img)
                self.result_label.setText("检测结果：未发现目标")
                take_time = (t2 - t1) * 1000
                self.time_label.setText(f"检测时间：{take_time:.1f}ms")
                
        except Exception as e:
            self.display_label.setText(f"检测失败：{str(e)}")
            print(f"检测错误: {e}")
    
    def video_show(self):
        """视频检测"""
        if self.model is None:
            self.display_label.setText("模型未加载，无法进行检测")
            return
            
        # 关闭摄像头
        if self.cap:
            self.video_stop()
            
        # 隐藏批量导航按钮
        self.batch_nav_widget.setVisible(False)
            
        # 选择视频文件
        file_path, _ = QFileDialog.getOpenFileName(
            self, '选择视频', './', 
            "Video files (*.mp4 *.avi *.mov *.mkv)"
        )
        
        if not file_path:
            return
            
        try:
            self.cap = cv2.VideoCapture(file_path)
            self.timer_camera.start(30)  # 30ms间隔
            self.result_label.setText("检测结果：视频检测中...")
            
        except Exception as e:
            self.display_label.setText(f"视频打开失败：{str(e)}")
    
    def camera_show(self):
        """摄像头检测"""
        if self.model is None:
            self.display_label.setText("模型未加载，无法进行检测")
            return
            
        # 隐藏批量导航按钮
        self.batch_nav_widget.setVisible(False)
            
        if not self.is_camera_open:
            try:
                self.cap = cv2.VideoCapture(0)
                if self.cap.isOpened():
                    self.is_camera_open = True
                    self.timer_camera.start(30)
                    self.result_label.setText("检测结果：摄像头检测中...")
                else:
                    self.display_label.setText("无法打开摄像头")
            except Exception as e:
                self.display_label.setText(f"摄像头打开失败：{str(e)}")
        else:
            self.video_stop()
    
    def batch_detect(self):
        """批量检测"""
        if self.model is None:
            self.display_label.setText("模型未加载，无法进行检测")
            return
            
        # 关闭摄像头
        if self.cap:
            self.video_stop()
            
        # 选择文件夹
        directory = QFileDialog.getExistingDirectory(self, "选择图片文件夹", "./")
        if not directory:
            return
            
        try:
            img_suffix = ['jpg', 'png', 'jpeg', 'bmp']
            self.batch_images = []
            
            # 收集所有图片文件
            for file_name in os.listdir(directory):
                if file_name.split('.')[-1].lower() in img_suffix:
                    self.batch_images.append(os.path.join(directory, file_name))
            
            if not self.batch_images:
                self.display_label.setText("选择的文件夹中没有图片文件")
                self.batch_nav_widget.setVisible(False)
                return
            
            # 排序文件列表
            self.batch_images.sort()
            
            # 重置批量检测状态
            self.current_batch_index = 0
            self.batch_results = {}
            
            # 显示导航按钮
            self.batch_nav_widget.setVisible(True)
            
            # 进行批量检测
            total_targets = 0
            for i, img_path in enumerate(self.batch_images):
                results = self.model(img_path)[0]
                self.batch_results[i] = results
                if len(results.boxes) > 0:
                    total_targets += len(results.boxes)
                    
            # 显示第一张图片
            self.show_batch_image(0)
            
            self.result_label.setText(f"批量检测完成：处理 {len(self.batch_images)} 张图片，发现 {total_targets} 个目标")
            
        except Exception as e:
            self.display_label.setText(f"批量检测失败：{str(e)}")
            self.batch_nav_widget.setVisible(False)
    
    def show_batch_image(self, index):
        """显示批量检测中的指定图片"""
        if not self.batch_images or index < 0 or index >= len(self.batch_images):
            return
            
        try:
            # 更新当前索引
            self.current_batch_index = index
            
            # 获取图片路径和检测结果
            img_path = self.batch_images[index]
            results = self.batch_results.get(index)
            
            # 更新索引标签
            self.batch_index_label.setText(f"{index + 1}/{len(self.batch_images)}")
            
            # 更新按钮状态
            self.prev_btn.setEnabled(index > 0)
            self.next_btn.setEnabled(index < len(self.batch_images) - 1)
            
            # 显示图片
            if results and len(results.boxes) > 0:
                # 有检测结果，显示标注图片
                now_img = results.plot()
                self.display_detection_result(now_img)
                target_count = len(results.boxes)
                self.time_label.setText(f"当前图片：发现 {target_count} 个目标")
            else:
                # 无检测结果，显示原图
                img = tools.img_cvread(img_path)
                self.display_detection_result(img)
                self.time_label.setText("当前图片：未发现目标")
                
        except Exception as e:
            print(f"显示批量图片错误: {e}")
            
    def show_prev_image(self):
        """显示上一张图片"""
        if self.batch_images and self.current_batch_index > 0:
            self.show_batch_image(self.current_batch_index - 1)
            
    def show_next_image(self):
        """显示下一张图片"""
        if self.batch_images and self.current_batch_index < len(self.batch_images) - 1:
            self.show_batch_image(self.current_batch_index + 1)
    
    def open_frame(self):
        """处理摄像头/视频帧"""
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                try:
                    # 进行检测
                    results = self.model(frame)[0]
                    
                    if len(results.boxes) > 0:
                        frame = results.plot()
                        target_count = len(results.boxes)
                        self.result_label.setText(f"检测结果：发现 {target_count} 个目标")
                    else:
                        self.result_label.setText("检测结果：未发现目标")
                    
                    # 显示帧
                    self.display_detection_result(frame)
                    
                except Exception as e:
                    print(f"帧处理错误: {e}")
            else:
                self.video_stop()
    
    def video_stop(self):
        """停止视频/摄像头"""
        if self.timer_camera.isActive():
            self.timer_camera.stop()
        if self.cap:
            self.cap.release()
            self.cap = None
        self.is_camera_open = False
        self.result_label.setText("检测结果：已停止")
    
    def display_detection_result(self, img):
        """显示检测结果图像"""
        try:
            # 获取原始图像尺寸
            img_height, img_width = img.shape[:2]
            
            # 获取显示区域的实际尺寸（减去边距）
            label_size = self.display_label.size()
            display_width = max(label_size.width() - 20, 400)  # 减去边距，最小400
            display_height = max(label_size.height() - 20, 300)  # 减去边距，最小300
            
            # 计算缩放比例以适应显示区域，保持图像比例
            scale_w = display_width / img_width
            scale_h = display_height / img_height
            scale = min(scale_w, scale_h)
            
            new_width = int(img_width * scale)
            new_height = int(img_height * scale)
            
            # 缩放图像
            resized_img = cv2.resize(img, (new_width, new_height))
            
            # 转换为Qt格式并显示
            pix_img = tools.cvimg_to_qpiximg(resized_img)
            self.display_label.setPixmap(pix_img)
            
        except Exception as e:
            print(f"图像显示错误: {e}")
            self.display_label.setText("图像显示失败")


if __name__ == '__main__':
    # 设置高DPI支持（必须在创建QApplication之前）
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    
    app = QApplication(sys.argv)
    
    window = ModernMainWindow()
    window.show()
    
    sys.exit(app.exec_()) 