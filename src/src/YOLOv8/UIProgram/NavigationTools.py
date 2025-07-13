#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
室内多传感器寻敌系统 - 目标导航工具
===============================

提供目标导航功能，支持路径规划和导航动画

功能特性：
- 支持导入地图图片（黑白格式）
- 支持设置目标点位置
- 支持设置小车当前位置
- 自动规划避开障碍物的最短路径
- 提供小车沿路径行进的动画效果
- 支持缩放和平移地图

作者: 郭晋鹏团队
版本: 2.0.0
"""

from PyQt5.QtWidgets import (QWidget, QPushButton, QLabel, QHBoxLayout, QVBoxLayout, 
                            QFileDialog, QMessageBox, QButtonGroup, QFrame, QSizePolicy)
from PyQt5.QtCore import Qt, QPoint, QRect, QSize, QTimer, pyqtSignal
from PyQt5.QtGui import (QPainter, QPen, QColor, QPixmap, QImage, QBrush, QIcon, QFont,
                        QPainterPath, QTransform)
import cv2
import numpy as np
import sys
import os
import heapq
import math

# 确保可以从任何位置导入
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class NavigationArea(QWidget):
    """
    导航区域类
    
    提供一个可以进行导航操作的区域，支持路径规划和导航动画
    
    主要属性:
        map_image: 导入的地图图像
        target_point: 目标点位置
        car_position: 小车当前位置
        path: 规划的路径
        
    主要方法:
        loadMap: 加载地图图片
        setTargetPoint: 设置目标点
        setCarPosition: 设置小车位置
        planPath: 规划路径
        startNavigation: 开始导航动画
    """
    
    # 操作模式枚举
    MODE_VIEW = 0      # 查看模式
    MODE_TARGET = 1    # 设置目标点
    MODE_CAR = 2       # 设置小车位置
    
    # 符号尺寸
    TARGET_SIZE = 15
    CAR_SIZE = 20
    
    # 缩放设置
    MIN_SCALE = 0.1   # 最小缩放比例
    MAX_SCALE = 5.0   # 最大缩放比例
    ZOOM_FACTOR = 1.2 # 每次缩放的比例
    
    # 导航完成信号
    navigationFinished = pyqtSignal()
    
    def __init__(self, parent=None):
        """初始化导航区域"""
        super().__init__(parent)
        
        # 初始化属性
        self.map_image = None         # 原始地图图像(QPixmap)
        self.map_array = None         # 地图的numpy数组(用于路径规划)
        self.scaled_map = None        # 缩放后的地图
        self.target_point = None      # 目标点位置
        self.car_position = None      # 小车位置
        self.path = []                # 规划的路径
        self.current_mode = self.MODE_VIEW  # 当前操作模式
        self.has_map = False          # 是否已加载地图
        
        # 缩放和平移属性
        self.scale_factor = 1.0
        self.offset = QPoint(0, 0)
        self.panning = False
        self.pan_start_pos = QPoint()
        
        # 导航动画属性
        self.animation_timer = QTimer(self)
        self.animation_timer.timeout.connect(self.updateNavigation)
        self.animation_path_index = 0
        self.is_navigating = False
        
        # 设置鼠标跟踪
        self.setMouseTracking(True)
        
        # 设置焦点策略，使其能接收键盘事件
        self.setFocusPolicy(Qt.StrongFocus)
        
        # 创建空白图像
        self.createEmptyMap()
        
    def createEmptyMap(self):
        """创建空白地图"""
        self.map_image = QPixmap(800, 600)
        self.map_image.fill(Qt.white)
        self.scaled_map = self.map_image.copy()
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
        print("开始加载地图...")  # 调试输出
        
        if file_path is None:
            print("弹出文件选择对话框")  # 调试输出
            file_path, _ = QFileDialog.getOpenFileName(
                self, '导入地图', './', 
                "图像文件 (*.jpg *.jpeg *.png *.bmp)"
            )
            print(f"选择的文件路径: {file_path}")  # 调试输出
            
        if not file_path:
            print("未选择文件")  # 调试输出
            return False
            
        try:
            print(f"尝试加载图片: {file_path}")  # 调试输出
            # 加载原始图片
            self.map_image = QPixmap(file_path)
            if self.map_image.isNull():
                print("图片加载失败")  # 调试输出
                QMessageBox.warning(self, "错误", "无法加载地图文件")
                return False
                
            print(f"图片尺寸: {self.map_image.width()}x{self.map_image.height()}")  # 调试输出
            
            # 将QPixmap转换为OpenCV格式以便路径规划
            img = self.pixmapToArray(self.map_image)
            
            # 将图像转换为二值图像(黑白)
            if len(img.shape) > 2:
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            else:
                gray = img
                
            # 阈值处理，将图像转换为二值图像(0和255)
            _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
            
            # 保存为numpy数组，用于路径规划
            self.map_array = binary
            
            # 重置状态
            self.target_point = None
            self.car_position = None
            self.path = []
            self.scale_factor = 1.0
            self.offset = QPoint(0, 0)
            
            # 适配图像大小到窗口
            self.fitToView()
            
            self.has_map = True
            self.update()
            print("地图加载成功")  # 调试输出
            return True
            
        except Exception as e:
            print(f"加载地图错误: {e}")  # 调试输出
            import traceback
            traceback.print_exc()  # 打印详细错误信息
            QMessageBox.warning(self, "错误", f"加载地图失败: {str(e)}")
            return False
            
    def pixmapToArray(self, pixmap):
        """将QPixmap转换为numpy数组"""
        # 转换为QImage
        image = pixmap.toImage()
        
        # 获取图像尺寸
        width = image.width()
        height = image.height()
        
        # 转换为numpy数组
        ptr = image.constBits()
        ptr.setsize(image.byteCount())
        
        # 根据图像格式确定通道数
        if image.format() == QImage.Format_RGB32 or image.format() == QImage.Format_ARGB32:
            arr = np.array(ptr).reshape(height, width, 4)  # RGBA
            return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
        else:
            # 转换为标准格式
            converted = image.convertToFormat(QImage.Format_RGB32)
            ptr = converted.constBits()
            ptr.setsize(converted.byteCount())
            arr = np.array(ptr).reshape(height, width, 4)  # RGBA
            return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
            
    def fitToView(self):
        """适配图像大小到当前视图"""
        if self.map_image.isNull():
            return
            
        # 获取父窗口大小（如果有）
        parent_size = self.parentWidget().size() if self.parentWidget() else self.size()
        
        # 计算适合的缩放比例
        width_ratio = (parent_size.width() - 40) / self.map_image.width()
        height_ratio = (parent_size.height() - 40) / self.map_image.height()
        
        # 选择较小的比例，确保图像完全可见
        self.scale_factor = min(width_ratio, height_ratio, 1.0)
        
        # 缩放图像
        self.updateScaledMap()
        
    def updateScaledMap(self):
        """根据当前缩放因子更新缩放后的图像"""
        if self.map_image.isNull():
            return
            
        # 计算缩放后的尺寸
        new_width = int(self.map_image.width() * self.scale_factor)
        new_height = int(self.map_image.height() * self.scale_factor)
        
        # 缩放图像
        self.scaled_map = self.map_image.scaled(
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
            self.updateScaledMap()
            
    def zoomOut(self):
        """缩小地图"""
        if self.scale_factor > self.MIN_SCALE:
            self.scale_factor /= self.ZOOM_FACTOR
            self.updateScaledMap()
            
    def setMode(self, mode):
        """设置操作模式"""
        self.current_mode = mode
        if mode == self.MODE_TARGET:
            QMessageBox.information(self, "设置目标点", "请在地图上点击设置目标点位置")
        elif mode == self.MODE_CAR:
            QMessageBox.information(self, "设置小车位置", "请在地图上点击设置小车当前位置")
            
    def setTargetPoint(self, point=None):
        """
        设置目标点
        
        参数:
            point: 目标点位置，如果为None则进入设置目标点模式
        """
        if point is None:
            self.setMode(self.MODE_TARGET)
        else:
            self.target_point = point
            self.update()
            
    def setCarPosition(self, point=None):
        """
        设置小车位置
        
        参数:
            point: 小车位置，如果为None则进入设置小车位置模式
        """
        if point is None:
            self.setMode(self.MODE_CAR)
        else:
            self.car_position = point
            self.update()
            
    def planPath(self):
        """
        规划路径
        
        使用A*算法规划从小车位置到目标点的路径
        
        返回:
            bool: 是否成功规划路径
        """
        if not self.has_map or self.map_array is None:
            QMessageBox.warning(self, "错误", "请先导入地图")
            return False
            
        if self.car_position is None:
            QMessageBox.warning(self, "错误", "请先设置小车位置")
            return False
            
        if self.target_point is None:
            QMessageBox.warning(self, "错误", "请先设置目标点")
            return False
            
        # 转换为图像坐标
        start = self._mapToImageCoord(self.car_position)
        goal = self._mapToImageCoord(self.target_point)
        
        print(f"起点坐标: {start}, 终点坐标: {goal}")
        
        # 检查起点和终点是否在可行区域（白色区域，像素值 > 240）
        if start[1] < 0 or start[1] >= self.map_array.shape[0] or start[0] < 0 or start[0] >= self.map_array.shape[1]:
            QMessageBox.warning(self, "错误", "小车位置超出地图范围")
            return False
            
        if goal[1] < 0 or goal[1] >= self.map_array.shape[0] or goal[0] < 0 or goal[0] >= self.map_array.shape[1]:
            QMessageBox.warning(self, "错误", "目标点超出地图范围")
            return False
        
        # 打印起点和终点的像素值，用于调试
        start_pixel = self.map_array[start[1], start[0]]
        goal_pixel = self.map_array[goal[1], goal[0]]
        print(f"起点像素值: {start_pixel}, 终点像素值: {goal_pixel}")
        
        if start_pixel <= 240:  # 非白色区域
            QMessageBox.warning(self, "错误", f"小车位置不在可行区域 (像素值: {start_pixel})")
            return False
            
        if goal_pixel <= 240:  # 非白色区域
            QMessageBox.warning(self, "错误", f"目标点不在可行区域 (像素值: {goal_pixel})")
            return False
            
        # 使用A*算法规划路径
        print("开始规划路径...")
        path = self.astar(start, goal)
        
        if not path:
            QMessageBox.warning(self, "错误", "无法找到可行路径")
            return False
            
        # 平滑路径
        self.path = self.smooth_path(path)
        print(f"路径规划完成，共{len(self.path)}个点")
        
        # 更新界面
        self.update()
        return True

    def astar(self, start, goal):
        """
        A*算法实现路径规划
        
        参数:
            start: 起点坐标 (x, y)
            goal: 终点坐标 (x, y)
            
        返回:
            list: 路径点列表 [(x1, y1), (x2, y2), ...]
        """
        # 定义启发式函数 - 使用欧几里得距离
        def heuristic(a, b):
            return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
        
        # 定义邻居节点
        def get_neighbors(node):
            # 8个方向的邻居
            directions = [
                (0, 1), (1, 0), (0, -1), (-1, 0),  # 上右下左
                (1, 1), (1, -1), (-1, -1), (-1, 1)  # 对角线
            ]
            
            result = []
            for dx, dy in directions:
                x, y = node[0] + dx, node[1] + dy
                
                # 检查边界
                if 0 <= x < self.map_array.shape[1] and 0 <= y < self.map_array.shape[0]:
                    # 只在白色区域行驶 (像素值 > 240)
                    if self.map_array[y, x] > 240:
                        # 对角线移动时，需要检查两个相邻点是否也是白色区域
                        if dx != 0 and dy != 0:
                            # 检查水平和垂直相邻点
                            if (self.map_array[y, node[0]] > 240 and 
                                self.map_array[node[1], x] > 240):
                                result.append((x, y))
                        else:
                            result.append((x, y))
            
            return result
        
        # 初始化开放列表和关闭列表
        open_set = []
        closed_set = set()
        
        # 记录每个节点的父节点
        came_from = {}
        
        # g_score[n]表示从起点到节点n的实际代价
        g_score = {start: 0}
        
        # f_score[n]表示从起点经过节点n到终点的估计代价
        f_score = {start: heuristic(start, goal)}
        
        # 将起点加入开放列表
        heapq.heappush(open_set, (f_score[start], start))
        
        # 最大迭代次数，防止无限循环
        max_iterations = 100000
        iterations = 0
        
        while open_set and iterations < max_iterations:
            iterations += 1
            
            # 获取f值最小的节点
            _, current = heapq.heappop(open_set)
            
            # 如果到达终点，构建路径并返回
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                print(f"路径规划完成，迭代次数: {iterations}")
                return path
            
            # 将当前节点加入关闭列表
            closed_set.add(current)
            
            # 遍历邻居节点
            for neighbor in get_neighbors(current):
                # 如果邻居节点在关闭列表中，跳过
                if neighbor in closed_set:
                    continue
                
                # 计算从起点经过当前节点到邻居节点的代价
                # 对角线移动的代价为√2，直线移动的代价为1
                if abs(neighbor[0] - current[0]) == 1 and abs(neighbor[1] - current[1]) == 1:
                    # 对角线移动
                    move_cost = 1.414  # √2
                else:
                    # 直线移动
                    move_cost = 1.0
                
                tentative_g_score = g_score[current] + move_cost
                
                # 如果邻居节点不在开放列表中，或者找到了更好的路径
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    # 更新路径信息
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                    
                    # 将邻居节点加入开放列表
                    if neighbor not in [item[1] for item in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        print(f"无法找到路径，迭代次数: {iterations}")
        # 如果无法找到路径，返回空列表
        return []
        
    def is_safe_distance(self, x, y, safe_distance=3):
        """
        检查点(x,y)是否与障碍物(黑色或灰色区域)保持安全距离
        
        参数:
            x, y: 点坐标
            safe_distance: 安全距离阈值
            
        返回:
            bool: 是否安全
        """
        # 检查周围区域是否有障碍物
        min_x = max(0, x - safe_distance)
        max_x = min(self.map_array.shape[1] - 1, x + safe_distance)
        min_y = max(0, y - safe_distance)
        max_y = min(self.map_array.shape[0] - 1, y + safe_distance)
        
        # 检查区域内是否有黑色或灰色像素(像素值小于240)
        region = self.map_array[min_y:max_y+1, min_x:max_x+1]
        return np.all(region > 240)

    def smooth_path(self, path):
        """
        平滑路径，减少路径点数量，但确保不经过障碍物
        
        参数:
            path: 原始路径点列表
            
        返回:
            list: 平滑后的路径点列表
        """
        if len(path) <= 2:
            return path
            
        print(f"开始平滑路径，原始路径点数: {len(path)}")
            
        # 使用RDP算法简化路径
        def rdp(points, epsilon):
            """
            Ramer-Douglas-Peucker算法实现
            
            参数:
                points: 点列表
                epsilon: 简化阈值
                
            返回:
                list: 简化后的点列表
            """
            if len(points) <= 2:
                return points
                
            # 找到距离最远的点
            dmax = 0
            index = 0
            for i in range(1, len(points) - 1):
                d = self.point_line_distance(points[i], points[0], points[-1])
                if d > dmax:
                    dmax = d
                    index = i
            
            # 如果最大距离大于阈值，则递归处理
            if dmax > epsilon:
                # 递归处理前半部分和后半部分
                results1 = rdp(points[:index + 1], epsilon)
                results2 = rdp(points[index:], epsilon)
                
                # 合并结果，去掉重复的点
                return results1[:-1] + results2
            else:
                # 检查直线是否穿过障碍物
                if is_valid_path(points[0], points[-1]):
                    return [points[0], points[-1]]
                else:
                    # 如果穿过障碍物，保留原始路径
                    return points
        
        # 检查路径是否穿过障碍物
        def is_valid_path(p1, p2):
            # 检查从p1到p2的直线是否穿过障碍物
            # 使用Bresenham算法获取直线上的所有点
            points = self.bresenham(p1[0], p1[1], p2[0], p2[1])
            
            # 检查每个点是否在安全区域内
            for x, y in points:
                if not (0 <= x < self.map_array.shape[1] and 0 <= y < self.map_array.shape[0]):
                    return False
                    
                # 检查是否为白色区域 (像素值 > 240)
                if self.map_array[y, x] <= 240:
                    return False
            
            return True
        
        # 使用RDP算法简化路径，阈值设为3（降低阈值以保留更多细节）
        simplified_path = rdp(path, 3)
        print(f"RDP简化后路径点数: {len(simplified_path)}")
        
        # 确保路径点数量不会太少，至少保留原始点数的20%
        min_points = max(10, len(path) // 5)
        
        # 如果简化后的点数太少，增加中间点
        if len(simplified_path) < min_points:
            # 重新分配点，使其均匀分布
            result = [simplified_path[0]]  # 起点
            
            # 计算路径总长度
            total_distance = 0
            for i in range(len(simplified_path) - 1):
                p1 = simplified_path[i]
                p2 = simplified_path[i + 1]
                total_distance += math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
            
            # 计算点之间的间距
            spacing = total_distance / (min_points - 1)
            
            # 沿路径均匀分布点
            current_distance = 0
            next_point_distance = spacing
            
            for i in range(len(simplified_path) - 1):
                p1 = simplified_path[i]
                p2 = simplified_path[i + 1]
                
                # 计算当前段的长度
                segment_length = math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
                
                # 在当前段上添加点
                while current_distance + segment_length >= next_point_distance:
                    # 计算在当前段上的位置
                    t = (next_point_distance - current_distance) / segment_length
                    x = int(p1[0] * (1 - t) + p2[0] * t)
                    y = int(p1[1] * (1 - t) + p2[1] * t)
                    
                    # 确保点在白色区域
                    if 0 <= y < self.map_array.shape[0] and 0 <= x < self.map_array.shape[1]:
                        if self.map_array[y, x] > 240:
                            result.append((x, y))
                    
                    # 更新下一个点的距离
                    next_point_distance += spacing
                
                # 更新当前距离
                current_distance += segment_length
            
            # 确保终点被添加
            result.append(simplified_path[-1])
            print(f"重新分配后路径点数: {len(result)}")
            return result
        
        print(f"最终路径点数: {len(simplified_path)}")
        return simplified_path
        
    def bresenham(self, x1, y1, x2, y2):
        """
        Bresenham算法获取直线上的所有点
        
        参数:
            x1, y1: 起点坐标
            x2, y2: 终点坐标
            
        返回:
            list: 直线上的所有点
        """
        points = []
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy
        
        while True:
            points.append((x1, y1))
            
            if x1 == x2 and y1 == y2:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy
                
        return points
        
    def point_line_distance(self, point, line_start, line_end):
        """
        计算点到直线的距离
        
        参数:
            point: 点坐标(x, y)
            line_start: 直线起点(x, y)
            line_end: 直线终点(x, y)
            
        返回:
            float: 点到直线的距离
        """
        x0, y0 = point
        x1, y1 = line_start
        x2, y2 = line_end
        
        # 如果直线实际上是一个点
        if x1 == x2 and y1 == y2:
            return math.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2)
            
        # 计算点到直线的距离
        numerator = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
        denominator = math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
        
        return numerator / denominator
        
    def startNavigation(self):
        """
        开始导航动画
        
        返回:
            bool: 是否成功开始导航
        """
        if not self.path:
            QMessageBox.warning(self, "错误", "请先规划路径")
            return False
            
        # 重置动画状态
        self.animation_path_index = 0
        self.is_navigating = True
        
        # 启动定时器，控制动画速度
        # 减慢动画速度，从100ms改为300ms
        self.animation_timer.start(300)
        
        return True
        
    def stopNavigation(self):
        """停止导航动画"""
        self.animation_timer.stop()
        self.is_navigating = False
        self.update()
        
    def updateNavigation(self):
        """更新导航动画"""
        if not self.is_navigating or not self.path:
            return
            
        # 获取当前路径点和下一个路径点
        if self.animation_path_index < len(self.path) - 1:
            # 移动到下一个点
            self.animation_path_index += 1
            
            # 更新界面
            self.update()
        else:
            # 到达终点，停止动画
            self.animation_timer.stop()
            self.is_navigating = False
            
            # 发出导航完成信号
            self.navigationFinished.emit()
            
            # 显示提示
            QMessageBox.information(self, "导航完成", "小车已到达目标位置")
            
    def paintEvent(self, event):
        """绘制事件"""
        if not self.has_map or self.scaled_map.isNull():
            super().paintEvent(event)
            return
            
        # 创建QPainter
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setRenderHint(QPainter.SmoothPixmapTransform)
        
        # 计算图像在窗口中的位置(居中显示)
        x_offset = (self.width() - self.scaled_map.width()) // 2 + self.offset.x()
        y_offset = (self.height() - self.scaled_map.height()) // 2 + self.offset.y()
        
        # 绘制地图
        painter.drawPixmap(x_offset, y_offset, self.scaled_map)
        
        # 绘制目标点(如果有)
        if self.target_point is not None:
            # 转换为窗口坐标
            x = int(self.target_point.x() * self.scale_factor) + x_offset
            y = int(self.target_point.y() * self.scale_factor) + y_offset
            
            # 绘制目标点(红色圆形)
            painter.setPen(QPen(QColor(255, 0, 0), 2))
            painter.setBrush(QBrush(QColor(255, 0, 0, 150)))
            painter.drawEllipse(x - self.TARGET_SIZE // 2, y - self.TARGET_SIZE // 2, 
                               self.TARGET_SIZE, self.TARGET_SIZE)
            
            # 绘制标签
            painter.setPen(QPen(QColor(255, 0, 0), 1))
            painter.drawText(x + 10, y - 10, "目标点")
        
        # 绘制路径(如果有)
        if self.path:
            # 设置路径画笔(绿色实线)
            painter.setPen(QPen(QColor(0, 255, 0), 2, Qt.SolidLine))
            
            # 直接绘制路径线段
            for i in range(len(self.path) - 1):
                # 转换为窗口坐标
                x1 = int(self.path[i][0] * self.scale_factor) + x_offset
                y1 = int(self.path[i][1] * self.scale_factor) + y_offset
                x2 = int(self.path[i+1][0] * self.scale_factor) + x_offset
                y2 = int(self.path[i+1][1] * self.scale_factor) + y_offset
                
                # 绘制线段
                painter.drawLine(x1, y1, x2, y2)
        
        # 绘制小车位置(如果有)
        if self.car_position is not None:
            # 如果正在导航，使用路径点作为小车位置
            if self.is_navigating and self.path and self.animation_path_index < len(self.path):
                car_pos = self.path[self.animation_path_index]
                
                # 转换为窗口坐标
                x = int(car_pos[0] * self.scale_factor) + x_offset
                y = int(car_pos[1] * self.scale_factor) + y_offset
            else:
                # 使用设置的小车位置
                x = int(self.car_position.x() * self.scale_factor) + x_offset
                y = int(self.car_position.y() * self.scale_factor) + y_offset
            
            # 绘制小车(蓝色圆形)
            painter.setPen(QPen(QColor(0, 0, 255), 2))
            painter.setBrush(QBrush(QColor(0, 0, 255, 150)))
            painter.drawEllipse(x - self.CAR_SIZE // 2, y - self.CAR_SIZE // 2, 
                               self.CAR_SIZE, self.CAR_SIZE)
            
            # 绘制小车方向(如果在导航中)
            if self.is_navigating and self.animation_path_index < len(self.path) - 1:
                # 获取下一个点
                next_pos = self.path[self.animation_path_index + 1]
                
                # 转换为窗口坐标
                next_x = int(next_pos[0] * self.scale_factor) + x_offset
                next_y = int(next_pos[1] * self.scale_factor) + y_offset
                
                # 计算方向
                angle = math.atan2(next_y - y, next_x - x)
                
                # 绘制方向指示器
                painter.save()
                painter.translate(x, y)
                painter.rotate(angle * 180 / math.pi)
                
                # 绘制三角形指示方向
                path = QPainterPath()
                path.moveTo(self.CAR_SIZE // 2, 0)
                path.lineTo(self.CAR_SIZE // 4, -self.CAR_SIZE // 4)
                path.lineTo(self.CAR_SIZE // 4, self.CAR_SIZE // 4)
                path.closeSubpath()
                
                painter.fillPath(path, QBrush(QColor(255, 255, 0)))
                painter.restore()
            
            # 绘制标签
            painter.setPen(QPen(QColor(0, 0, 255), 1))
            painter.drawText(x + 10, y - 10, "小车")
        
        # 结束绘制
        painter.end()
        
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
        """鼠标按下事件"""
        if event.button() == Qt.LeftButton:
            # 获取鼠标位置
            pos = event.pos()
            
            # 将窗口坐标转换为图像坐标
            img_pos = self._mapToImagePos(pos)
            
            # 根据当前模式处理
            if self.current_mode == self.MODE_TARGET:
                # 设置目标点
                if self.has_map and self._isPointInImage(img_pos):
                    self.setTargetPoint(img_pos)
                    self.current_mode = self.MODE_VIEW  # 设置完后恢复查看模式
                    self.update()
            elif self.current_mode == self.MODE_CAR:
                # 设置小车位置
                if self.has_map and self._isPointInImage(img_pos):
                    self.setCarPosition(img_pos)
                    self.current_mode = self.MODE_VIEW  # 设置完后恢复查看模式
                    self.update()
            elif event.modifiers() & Qt.ControlModifier:
                # 按住Ctrl键拖动地图
                self.panning = True
                self.pan_start_pos = event.pos()
            
            event.accept()
            
    def mouseMoveEvent(self, event):
        """鼠标移动事件处理"""
        if self.panning:
            # 平移地图
            delta = event.pos() - self.pan_start_pos
            self.offset += delta
            self.pan_start_pos = event.pos()
            self.update()
            
    def mouseReleaseEvent(self, event):
        """鼠标释放事件处理"""
        if event.button() == Qt.LeftButton:
            if self.panning:
                self.panning = False
                
    def _mapToImagePos(self, pos):
        """将窗口坐标映射到图像坐标"""
        # 计算图像左上角在窗口中的位置
        x_offset = (self.width() - self.scaled_map.width()) // 2 + self.offset.x()
        y_offset = (self.height() - self.scaled_map.height()) // 2 + self.offset.y()
        
        # 计算相对于图像左上角的坐标
        x = (pos.x() - x_offset) / self.scale_factor
        y = (pos.y() - y_offset) / self.scale_factor
        
        return QPoint(int(x), int(y))
        
    def _isPointInImage(self, pos):
        """检查点是否在图像范围内"""
        return (0 <= pos.x() < self.map_image.width() and 
                0 <= pos.y() < self.map_image.height())
                
    def _mapToImageCoord(self, point):
        """将QPoint转换为图像坐标(x, y)元组"""
        return (int(point.x()), int(point.y()))
        
    def _imageToMapCoord(self, point):
        """将图像坐标(x, y)元组转换为QPoint"""
        return QPoint(point[0], point[1])


class NavigationToolbar(QWidget):
    """
    导航工具栏类
    
    提供导航操作按钮和控制功能
    
    主要属性:
        navigation_area: 关联的导航区域
        
    主要方法:
        setupUI: 设置用户界面
        connectSignals: 连接信号和槽
    """
    
    def __init__(self, navigation_area, parent=None):
        """
        初始化导航工具栏
        
        参数:
            navigation_area: 关联的导航区域
            parent: 父窗口
        """
        super().__init__(parent)
        self.navigation_area = navigation_area
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
        self.import_map_btn.setToolTip("导入地图文件")
        
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
        
        # 将按钮添加到地图操作布局
        map_layout.addWidget(self.import_map_btn)
        map_layout.addWidget(self.zoom_in_btn)
        map_layout.addWidget(self.zoom_out_btn)
        map_layout.addWidget(self.fit_view_btn)
        map_layout.addStretch()
        
        # 导航操作布局
        nav_layout = QHBoxLayout()
        
        # 设置目标点按钮
        self.set_target_btn = QPushButton("🎯 设置目标点")
        self.set_target_btn.setFixedSize(120, 30)
        self.set_target_btn.setToolTip("点击后在地图上设置目标位置")
        
        # 设置小车位置按钮
        self.set_car_btn = QPushButton("🚗 定位小车位置")
        self.set_car_btn.setFixedSize(120, 30)
        self.set_car_btn.setToolTip("点击后在地图上设置小车当前位置")
        
        # 规划路径按钮
        self.plan_path_btn = QPushButton("📝 绘制行进路线")
        self.plan_path_btn.setFixedSize(120, 30)
        self.plan_path_btn.setToolTip("规划从小车到目标点的路径")
        
        # 开始导航按钮
        self.start_nav_btn = QPushButton("▶️ 开始导航")
        self.start_nav_btn.setFixedSize(120, 30)
        self.start_nav_btn.setToolTip("开始导航动画")
        
        # 停止导航按钮
        self.stop_nav_btn = QPushButton("⏹️ 停止导航")
        self.stop_nav_btn.setFixedSize(120, 30)
        self.stop_nav_btn.setToolTip("停止导航动画")
        
        # 将按钮添加到导航操作布局
        nav_layout.addWidget(self.set_target_btn)
        nav_layout.addWidget(self.set_car_btn)
        nav_layout.addWidget(self.plan_path_btn)
        nav_layout.addWidget(self.start_nav_btn)
        nav_layout.addWidget(self.stop_nav_btn)
        nav_layout.addStretch()
        
        # 将所有布局添加到主布局
        main_layout.addLayout(map_layout)
        main_layout.addLayout(nav_layout)
        
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
            QPushButton:pressed {
                background: rgba(40, 40, 40, 1.0);
            }
            QLabel {
                color: white;
                font-size: 12px;
            }
        """)
        
    def connectSignals(self):
        """连接信号和槽"""
        # 地图操作按钮信号
        self.import_map_btn.clicked.connect(lambda: self.onImportMapClicked())
        self.zoom_in_btn.clicked.connect(self.navigation_area.zoomIn)
        self.zoom_out_btn.clicked.connect(self.navigation_area.zoomOut)
        self.fit_view_btn.clicked.connect(self.navigation_area.fitToView)
        
        # 导航操作按钮信号
        self.set_target_btn.clicked.connect(lambda: self.navigation_area.setMode(self.navigation_area.MODE_TARGET))
        self.set_car_btn.clicked.connect(lambda: self.navigation_area.setMode(self.navigation_area.MODE_CAR))
        self.plan_path_btn.clicked.connect(self.navigation_area.planPath)
        self.start_nav_btn.clicked.connect(self.navigation_area.startNavigation)
        self.stop_nav_btn.clicked.connect(self.navigation_area.stopNavigation)
        
    def onImportMapClicked(self):
        """导入地图按钮点击处理"""
        print("导入地图按钮被点击")  # 调试输出
        self.navigation_area.loadMap()


class NavigationWidget(QWidget):
    """
    导航组件类
    
    集成导航区域和工具栏，提供完整的导航功能
    
    主要属性:
        navigation_area: 导航区域
        toolbar: 工具栏
        
    主要方法:
        setupUI: 设置用户界面
        loadMap: 加载地图
    """
    
    # 导航完成信号
    navigationFinished = pyqtSignal()
    
    def __init__(self, parent=None):
        """
        初始化导航组件
        
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
        
        # 页面标题
        title_label = QLabel("目标导航")
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
        main_layout.addWidget(title_label)
        
        # 创建导航区域
        self.navigation_area = NavigationArea(self)
        
        # 创建工具栏
        self.toolbar = NavigationToolbar(self.navigation_area, self)
        
        # 将导航区域和工具栏添加到布局
        main_layout.addWidget(self.toolbar)
        main_layout.addWidget(self.navigation_area, 1)  # 1表示拉伸因子，使导航区域占据更多空间
        
        # 连接导航完成信号
        self.navigation_area.navigationFinished.connect(self._onNavigationFinished)
        
        # 设置初始状态
        self.setStyleSheet("""
            QWidget {
                background: transparent;
            }
        """)
        
    def _onNavigationFinished(self):
        """导航完成处理"""
        # 转发信号
        self.navigationFinished.emit()
        
    def loadMap(self, file_path=None):
        """加载地图"""
        print("NavigationWidget.loadMap() 被调用")  # 调试输出
        return self.navigation_area.loadMap(file_path)