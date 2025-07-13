# -*- coding: utf-8 -*-
"""
室内多传感器寻敌系统启动画面
===============================

启动画面模块，提供优雅的应用程序启动体验

功能特性：
- 显示系统名称和版本信息
- 模拟加载进度动画
- 动态状态提示文字
- 自动进度更新
- 支持半透明背景效果
- 智能图片路径查找

窗口尺寸: 250x175 (紧凑型设计)
显示时长: 约5秒 (100个进度步骤，每步50ms)

作者: 郭晋鹏团队
版本: 2.0.0
"""

from PyQt5.QtWidgets import QSplashScreen, QLabel, QProgressBar, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QPropertyAnimation, QEasingCurve
from PyQt5.QtGui import QPixmap, QPainter, QLinearGradient, QBrush, QPen, QFont, QColor
import os

class SplashScreen(QSplashScreen):
    """
    启动画面窗口类
    
    继承自QSplashScreen，提供应用程序启动时的欢迎界面。
    包含进度条、状态文字、系统信息等元素。
    
    信号：
        splashFinished: 启动完成信号，用于通知主程序继续执行
        
    属性：
        current_progress (int): 当前加载进度 (0-100)
        progress_timer (QTimer): 进度更新定时器
    """
    
    # 启动完成信号
    splashFinished = pyqtSignal()
    
    def __init__(self, parent=None):
        """
        初始化启动画面
        
        参数：
            parent: 父窗口，默认为None
        """
        super().__init__(parent)
        self.current_progress = 0  # 初始化进度为0
        self.setupUI()  # 设置用户界面
        self.startAnimation()  # 启动加载动画
    
    def setupUI(self):
        """
        设置用户界面
        
        创建启动画面的所有UI元素，包括：
        - 系统标题和版本信息
        - 状态提示标签
        - 进度条
        - 背景图片和视觉效果
        """
        # 设置窗口属性 - 缩小到一半
        self.setFixedSize(250, 175)
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
        self.setAttribute(Qt.WA_TranslucentBackground)
        
        # 创建主widget和布局
        main_widget = QWidget()
        layout = QVBoxLayout(main_widget)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(5)
        
        # 标题标签
        self.title_label = QLabel("室内多传感器寻敌系统")
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setStyleSheet("""
            QLabel {
                color: white;
                font-size: 14px;
                font-weight: bold;
                background: transparent;
            }
        """)
        layout.addWidget(self.title_label)
        
        # 版本标签
        self.version_label = QLabel("版本 2.0.0")
        self.version_label.setAlignment(Qt.AlignCenter)
        self.version_label.setStyleSheet("""
            QLabel {
                color: rgba(255, 255, 255, 180);
                font-size: 10px;
                background: transparent;
            }
        """)
        layout.addWidget(self.version_label)
        
        # 添加弹性空间
        layout.addStretch()
        
        # 状态标签
        self.status_label = QLabel("正在初始化...")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("""
            QLabel {
                color: rgba(255, 255, 255, 200);
                font-size: 9px;
                background: transparent;
            }
        """)
        layout.addWidget(self.status_label)
        
        # 进度条
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setFixedHeight(4)
        self.progress_bar.setStyleSheet("""
            QProgressBar {
                background-color: rgba(255, 255, 255, 30);
                border: 1px solid rgba(255, 255, 255, 50);
                border-radius: 2px;
                text-align: center;
            }
            
            QProgressBar::chunk {
                background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 0,
                           stop: 0 #4CAF50, stop: 1 #2196F3);
                border-radius: 1px;
            }
        """)
        layout.addWidget(self.progress_bar)
        
        # 创建背景图片 - 调整为新尺寸
        background_pixmap = QPixmap(250, 175)
        background_pixmap.fill(Qt.transparent)
        
        painter = QPainter(background_pixmap)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setRenderHint(QPainter.SmoothPixmapTransform)
        
        # 加载loading.png作为背景并缩放填充
        # 智能路径查找
        def find_image_path(image_name):
            possible_paths = [
                image_name,
                os.path.join("UIProgram", image_name),
                os.path.join("..", "UIProgram", image_name) if os.path.basename(os.getcwd()) == "UIProgram" else os.path.join("UIProgram", image_name)
            ]
            for path in possible_paths:
                if os.path.exists(path):
                    return path
            return image_name
            
        loading_bg_path = find_image_path("ui_imgs/icons/loading.png")
        if os.path.exists(loading_bg_path):
            loading_image = QPixmap(loading_bg_path)
            if not loading_image.isNull():
                # 按比例缩放并填充整个区域
                scaled_image = loading_image.scaled(250, 175, Qt.KeepAspectRatioByExpanding, Qt.SmoothTransformation)
                painter.drawPixmap(0, 0, 250, 175, scaled_image)
        
        # 绘制半透明遮罩层，使文字更清晰
        painter.setBrush(QBrush(QColor(0, 0, 0, 80)))
        painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(0, 0, 250, 175, 8, 8)
        
        # 绘制边框
        painter.setPen(QPen(QColor(116, 185, 255), 1))
        painter.setBrush(Qt.NoBrush)
        painter.drawRoundedRect(1, 1, 248, 173, 8, 8)
        
        painter.end()
        self.setPixmap(background_pixmap)
        
        # 将主widget设置为layout
        main_widget.setParent(self)
        main_widget.setGeometry(0, 0, 250, 175)
        main_widget.setStyleSheet("background: transparent;")
        
    def startAnimation(self):
        """
        启动动画效果
        
        创建并启动进度更新定时器，每50毫秒更新一次进度。
        整个启动过程约持续5秒钟。
        """
        # 进度更新定时器
        self.progress_timer = QTimer()
        self.progress_timer.timeout.connect(self.updateProgress)
        self.progress_timer.start(50)  # 50ms更新一次
        
    def updateProgress(self):
        """
        更新加载进度
        
        定时器回调函数，负责：
        - 更新进度条数值
        - 根据进度显示不同的状态文字
        - 检查是否完成加载并触发结束信号
        """
        self.current_progress += 1
        
        # 更新进度条
        self.progress_bar.setValue(self.current_progress)
        
        # 更新状态文本
        if self.current_progress < 20:
            self.status_label.setText("正在初始化系统...")
        elif self.current_progress < 40:
            self.status_label.setText("正在加载检测模型...")
        elif self.current_progress < 60:
            self.status_label.setText("正在初始化摄像头...")
        elif self.current_progress < 80:
            self.status_label.setText("正在配置用户界面...")
        elif self.current_progress < 95:
            self.status_label.setText("正在准备启动...")
        else:
            self.status_label.setText("启动完成！")
        
        # 完成启动
        if self.current_progress >= 100:
            self.progress_timer.stop()
            QTimer.singleShot(500, self.finishSplash)  # 延迟500ms显示完成状态
            
    def finishSplash(self):
        """
        完成启动流程
        
        发出启动完成信号并关闭启动画面窗口。
        """
        self.splashFinished.emit()
        self.close()
        
    def paintEvent(self, event):
        """
        重写绘制事件
        
        参数：
            event: 绘制事件对象
            
        注意：
            背景图片已经在setPixmap中设置，这里直接调用父类方法即可。
        """
        super().paintEvent(event) 