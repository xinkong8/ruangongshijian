#coding:utf-8
from ultralytics import YOLO

# 加载模型
model = YOLO("yolov8n.pt")  # 加载预训练模型
# Use the model
if __name__ == '__main__':
    # Use the model
    results = model.train(data='datasets/faceData/data.yaml', epochs=300,device=0, batch=4,imgsz=640,cache=True,workers=12)  # 训练模型
    # 将模型转为onnx格式
    # success = model.export(format='onnx')




