# -*- coding: utf-8 -*-
"""
室内多传感器寻敌系统应用启动器
整合启动画面、欢迎界面和主程序
从根目录启动版本
"""

import sys
import os
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QIcon

# 添加UIProgram目录到系统路径
current_dir = os.path.dirname(os.path.abspath(__file__))
ui_dir = os.path.join(current_dir, 'UIProgram')
sys.path.append(ui_dir)
sys.path.append(current_dir)

# 导入自定义模块
from UIProgram.SplashScreen import SplashScreen
from UIProgram.InitialWindow import InitialWindow
from UIProgram.ModernMainWindow import ModernMainWindow

class AppLauncher:
    """应用程序启动器"""
    
    def __init__(self):
        self.app = None
        self.splash = None
        self.initial_window = None
        self.main_window = None
        
    def run(self):
        """运行应用程序"""
        # 设置高DPI支持（必须在创建QApplication之前）
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
        
        # 创建QApplication
        self.app = QApplication(sys.argv)
        self.app.setApplicationName("室内多传感器寻敌系统")
        self.app.setApplicationVersion("2.0.0")
        
        # 设置应用图标
        icon_path = "UIProgram/ui_imgs/icons/face.png"
        if os.path.exists(icon_path):
            self.app.setWindowIcon(QIcon(icon_path))
        
        # 显示启动画面
        self.showSplashScreen()
        
        # 运行应用
        sys.exit(self.app.exec_())
        
    def showSplashScreen(self):
        """显示启动画面"""
        self.splash = SplashScreen()
        self.splash.splashFinished.connect(self.showInitialWindow)
        self.splash.show()
        
    def showInitialWindow(self):
        """显示欢迎界面"""
        # 确保启动画面已关闭
        if self.splash:
            self.splash.close()
            self.splash = None
            
        # 创建并显示欢迎界面
        self.initial_window = InitialWindow()
        self.initial_window.enterSystem.connect(self.showMainWindow)
        self.initial_window.show()
        
    def showMainWindow(self):
        """显示主程序界面"""
        # 关闭欢迎界面
        if self.initial_window:
            self.initial_window.close()
            self.initial_window = None
            
        # 创建并显示主程序
        try:
            self.main_window = ModernMainWindow()
            self.main_window.show()
            
            # 确保主窗口在最前面
            self.main_window.raise_()
            self.main_window.activateWindow()
            
        except Exception as e:
            print(f"启动主程序时出现错误: {e}")
            # 如果主程序启动失败，提供备用方案
            from PyQt5.QtWidgets import QMessageBox
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setWindowTitle("启动错误")
            msg.setText(f"无法启动主程序：\n{str(e)}")
            msg.setInformativeText("请检查依赖库是否正确安装。")
            msg.exec_()
            
            # 重新显示欢迎界面
            self.showInitialWindow()

def main():
    """主函数"""
    launcher = AppLauncher()
    launcher.run()

if __name__ == "__main__":
    main() 