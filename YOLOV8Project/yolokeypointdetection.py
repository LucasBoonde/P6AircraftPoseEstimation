#Bare kør det her og så træner den det automatisk og gemmer resultatet i runs

import multiprocessing
from ultralytics import YOLO

if __name__ == '__main__':
    multiprocessing.freeze_support()
    model = YOLO('yolov8n-pose.pt')

    results = model.train(data='C:/Users/kaspe/YOLOV8Project/roboflowv2/data.yaml', epochs=135, imgsz=640)
