from ultralytics import YOLO

CUSTOM_DATASET_PATH = '/home/pc/Robot_Class/week_3/Train_folder/data.yaml'

model = YOLO('yolo11s-seg.pt', task='segment')

results = model.train(data=CUSTOM_DATASET_PATH, epochs=100, 
                      imgsz=640,
                      batch=16,
                      lr0=0.001,
                      lrf=0.001)

model.save('yolo11n-hexagon_week3.pt')