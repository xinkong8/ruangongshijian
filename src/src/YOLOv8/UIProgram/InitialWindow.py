# -*- coding: utf-8 -*-
"""
室内多传感器寻敌系统欢迎界面
===============================

欢迎界面模块，启动画面后显示的系统介绍页面

功能特性：
- 显示系统欢迎信息和介绍
- 提供"进入系统"按钮
- 支持窗口拖拽移动
- 自定义关闭按钮
- 智能背景图片加载
- 响应式布局设计

窗口尺寸: 600x400
背景图片: BA.png (自动缩放适配)

交互功能：
- 点击"进入系统"按钮进入主程序
- 支持拖拽移动窗口
- 点击关闭按钮退出程序

作者: 郭晋鹏团队
版本: 2.0.0
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QFrame, QApplication
from PyQt5.QtCore import Qt, pyqtSignal, QPropertyAnimation, QEasingCurve, QTimer
from PyQt5.QtGui import QPixmap, QPainter, QLinearGradient, QBrush, QPen, QFont, QColor, QMouseEvent
import os

class InitialWindow(QWidget):
    """
    欢迎界面窗口类
    
    继承自QWidget，提供系统介绍和进入功能。
    支持拖拽移动和自定义样式。
    
    信号：
        enterSystem: 进入系统信号，用于通知启动主程序
        
    属性：
        drag_start_position: 拖拽起始位置
        is_dragging (bool): 是否正在拖拽窗口
    """
    
    # 进入系统信号
    enterSystem = pyqtSignal()
    
    def __init__(self, parent=None):
        """
        初始化欢迎界面
        
        参数：
            parent: 父窗口，默认为None
        """
        super().__init__(parent)
        self.drag_start_position = None  # 拖拽起始位置
        self.is_dragging = False  # 拖拽状态标志
        self.setupUI()  # 设置用户界面
        
    def setupUI(self):
        """设置用户界面"""
        # 设置窗口属性 - 缩小到一半
        self.setFixedSize(600, 400)
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
        
        # 主布局
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # 创建背景标签（全屏背景） - 调整为新尺寸
        self.background_label = QLabel(self)
        self.background_label.setGeometry(0, 0, 600, 400)
        
        # 加载BA.png作为背景并缩放填充
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
            
        bg_path = find_image_path("ui_imgs/icons/BA.png")
        if os.path.exists(bg_path):
            pixmap = QPixmap(bg_path)
            if not pixmap.isNull():
                scaled_pixmap = pixmap.scaled(600, 400, Qt.KeepAspectRatioByExpanding, Qt.SmoothTransformation)
                self.background_label.setPixmap(scaled_pixmap)
                self.background_label.setScaledContents(False)
                self.background_label.setAlignment(Qt.AlignCenter)
        
        # 确保背景在最底层
        self.background_label.lower()
        
        # 主框架（透明，用于布局）
        self.background_frame = QFrame()
        self.background_frame.setObjectName("backgroundFrame")
        self.background_frame.setStyleSheet("background: transparent;")
        main_layout.addWidget(self.background_frame)
        
        # 背景框架布局
        bg_layout = QVBoxLayout(self.background_frame)
        bg_layout.setContentsMargins(50, 50, 50, 50)
        bg_layout.setSpacing(30)
        
        # 创建自定义关闭按钮 - 调整位置
        self.close_btn = QPushButton("×", self)
        self.close_btn.setFixedSize(30, 30)
        self.close_btn.move(600 - 40, 10)
        self.close_btn.setObjectName("closeButton")
        self.close_btn.clicked.connect(self.close)
        
        # 移除Logo区域，只保留背景图片
        
        # 添加弹性空间
        bg_layout.addStretch()
        
        # 欢迎文字
        self.welcome_label = QLabel("欢迎指挥官")
        self.welcome_label.setAlignment(Qt.AlignCenter)
        self.welcome_label.setObjectName("welcomeLabel")
        bg_layout.addWidget(self.welcome_label)
        
        # 系统描述
        self.desc_label = QLabel("室内多传感器寻敌系统")
        self.desc_label.setAlignment(Qt.AlignCenter)
        self.desc_label.setObjectName("descLabel")
        bg_layout.addWidget(self.desc_label)
        
        # 版本信息
        self.version_label = QLabel("智能识别 • 精准定位 • 实时检测")
        self.version_label.setAlignment(Qt.AlignCenter)
        self.version_label.setObjectName("versionLabel")
        bg_layout.addWidget(self.version_label)
        
        # 按钮区域 - 按照UI文件的样式
        button_layout = QHBoxLayout()
        button_layout.addStretch()
        
        self.enter_btn = QPushButton("进入系统")
        self.enter_btn.setFixedSize(150, 40)  # 缩小到一半
        self.enter_btn.setObjectName("enterSystemButton")
        self.enter_btn.clicked.connect(self.onEnterSystem)
        button_layout.addWidget(self.enter_btn)
        
        button_layout.addStretch()
        bg_layout.addLayout(button_layout)
        
        # 底部空间
        bg_layout.addStretch()
        
        # 底部信息
        self.bottom_label = QLabel("© 2025 室内多传感器寻敌系统")
        self.bottom_label.setAlignment(Qt.AlignCenter)
        self.bottom_label.setObjectName("bottomLabel")
        bg_layout.addWidget(self.bottom_label)
        
        # 设置样式
        self.setStyles()
        
        # 居中显示
        self.centerWindow()
        
    def setStyles(self):
        """设置样式"""
        # 按照UI文件的样式设计
        self.setStyleSheet("""
            InitialWindow {
                background: transparent;
            }
            
            #welcomeLabel {
                color: #FFD700;
                font-size: 36px;
                font-weight: bold;
                font-family: 'Impact', 'Arial Black', sans-serif;

                background: rgba(0, 0, 0, 0.3);
                border-radius: 15px;
                padding: 30px;
                border: 2px solid rgba(255, 215, 0, 0.6);
            }
            
            QLabel#descLabel {
                color: rgba(255, 255, 255, 200);
                font-size: 24px;
                font-weight: normal;
                background: transparent;
                margin: 10px;
            }
            
            QLabel#versionLabel {
                color: rgba(76, 175, 80, 255);
                font-size: 16px;
                font-weight: normal;
                background: transparent;
                margin: 10px;
            }
            
            QLabel#bottomLabel {
                color: rgba(255, 255, 255, 120);
                font-size: 12px;
                background: transparent;
                margin: 10px;
            }
            
            #enterSystemButton {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, 
                    stop:0 rgba(255, 69, 0, 0.9), stop:1 rgba(220, 20, 60, 0.9));
                color: #FFFFFF;
                font-size: 12px;
                font-weight: bold;
                font-family: 'Arial Black', sans-serif;
                border: 3px solid rgba(255, 255, 255, 0.8);
                border-radius: 15px;
            }
            
            #enterSystemButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, 
                    stop:0 rgba(255, 99, 71, 1.0), stop:1 rgba(255, 69, 0, 1.0));
                border: 3px solid rgba(255, 215, 0, 1.0);
            }
            
            #enterSystemButton:pressed {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, 
                    stop:0 rgba(178, 34, 34, 0.9), stop:1 rgba(139, 0, 0, 0.9));
            }
            
            #closeButton {
                background: rgba(220, 20, 60, 0.8);
                color: white;
                font-size: 14px;
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
        """)
        
    def centerWindow(self):
        """窗口居中显示"""
        screen = QApplication.desktop().screenGeometry()
        x = (screen.width() - self.width()) // 2
        y = (screen.height() - self.height()) // 2
        self.move(x, y)
        
    def onEnterSystem(self):
        """进入系统按钮点击"""
        self.enterSystem.emit()
        self.close()
        
    def mousePressEvent(self, event: QMouseEvent):
        """鼠标按下事件"""
        if event.button() == Qt.LeftButton and self.isDraggableArea(event.pos()):
            self.is_dragging = True
            self.drag_start_position = event.globalPos() - self.pos()
            
    def mouseMoveEvent(self, event: QMouseEvent):
        """鼠标移动事件"""
        if self.is_dragging and event.buttons() == Qt.LeftButton:
            self.move(event.globalPos() - self.drag_start_position)
            
    def mouseReleaseEvent(self, event: QMouseEvent):
        """鼠标释放事件"""
        self.is_dragging = False
        
    def isDraggableArea(self, pos):
        """判断是否在可拖动区域"""
        # 除了按钮区域外都可以拖动
        return not (self.enter_btn.geometry().contains(pos) or 
                   self.close_btn.geometry().contains(pos)) 