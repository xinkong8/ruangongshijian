# -*- coding: utf-8 -*-
"""
UIProgram包
===========

YOLOv8人脸检测系统的用户界面模块

包含以下模块：
- SplashScreen: 启动画面，提供应用程序启动时的欢迎界面
- InitialWindow: 欢迎窗口，用户进入系统前的主界面  
- ModernMainWindow: 主程序窗口，包含完整的检测功能

模块结构：
- ui_imgs/: 界面图片资源文件夹
  - icons/: 图标和背景图片
    - loading.png: 启动画面背景
    - BA.png: 欢迎界面背景
    - R6.png: 主程序背景
    - face.png: 应用程序图标

功能特性：
- 现代化UI设计风格
- 半透明背景效果
- 支持图片、视频、摄像头、批量检测
- 批量检测支持图片浏览功能
- 全屏显示支持
- 智能路径查找，支持多种启动方式

版本: 1.0.0
作者: YOLOv8人脸检测系统开发团队
"""

__version__ = "1.0.0"
__author__ = "YOLOv8人脸检测系统开发团队"

# 导出主要类
from .SplashScreen import SplashScreen
from .InitialWindow import InitialWindow  
from .ModernMainWindow import ModernMainWindow

__all__ = [
    'SplashScreen',
    'InitialWindow', 
    'ModernMainWindow'
]
