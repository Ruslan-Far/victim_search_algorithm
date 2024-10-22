#!/usr/bin/env python3

import cv2
import numpy as np
from openvino.runtime import Core

# Настройка OpenVINO и загрузка модели
ie = Core()
model = ie.read_model(model="/home/ruslan/kpfu/magistracy/ml_models/yolov5s_openvino_mo/yolov5s.xml")  # Путь к IR модели YOLO
compiled_model = ie.compile_model(model=model, device_name="CPU")
input_layer = compiled_model.input(0)
output_layer = compiled_model.output(0)

# Порог уверенности для отображения объектов
CONF_THRESHOLD = 0.3

def draw_detections(frame, result, conf_threshold=CONF_THRESHOLD):
    """
    Функция для обработки результатов инференса и отображения bounding boxes на изображении.
    frame: исходное изображение (кадр)
    result: результаты инференса (массив numpy)
    conf_threshold: порог уверенности для отображения боксов
    """
    height, width = frame.shape[:2]
    # print("height:", height)
    # print("width:", width)
    
    # Результат YOLO содержит данные о предсказанных объектах в формате [batch, num_boxes, 85]
    for detection in result[0]:  # цикл по каждому предсказанному объекту
        confidence = detection[4]  # уверенность детекции
        if confidence >= conf_threshold:  # если уверенность выше порога
            # Получаем координаты бокса и масштабируем их обратно к размеру изображения
            x1, y1, x2, y2 = detection[:4]
            # x1 = int(x1 * width)  # Преобразуем в целые числа
            # y1 = int(y1 * height)
            # x2 = int(x2 * width)
            # y2 = int(y2 * height)
            x1 = int(x1)  # Преобразуем в целые числа
            y1 = int(y1)
            x2 = int(x2)
            y2 = int(y2)
            
            # Получаем индекс класса
            class_id = np.argmax(detection[5:])  # индекс класса с наибольшей вероятностью
            # print(x1)
            # print(y1)
            # print(x2)
            # print(y2)
            # print("++++++++++++++++++++")
            
            # Рисуем bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Выводим класс объекта и уверенность
            label = f"Class: {class_id}, Conf: {confidence:.2f}"
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    return frame

def process_image(frame):
    """
    Обработка изображения для выполнения инференса
    """
    # Предобработка изображения
    input_image = cv2.resize(frame, (640, 640))  # изменение размера под модель
    input_image = input_image.transpose((2, 0, 1))  # CHW формат
    input_image = np.expand_dims(input_image, axis=0)  # добавляем batch
    input_image = input_image.astype(np.float32) / 255.0  # нормализация

    # Выполнение инференса
    result = compiled_model([input_image])[output_layer]  # инференс через OpenVINO
    
    # Преобразуем результат инференса и рисуем bounding boxes
    frame_with_detections = draw_detections(frame, result)
    
    return frame_with_detections

def run_object_detection():
    """
    Основной цикл для выполнения инференса YOLO и отображения изображения
    """
    # Открытие видео или камеры
    cap = cv2.VideoCapture(0)  # Использование камеры (если нужно видео - укажи путь к файлу)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        
    # img_rgb = cv2.imread("/home/ruslan/kpfu/magistracy/test_images/face.jpg", cv2.IMREAD_COLOR)
    # img_rgb = cv2.resize(img_rgb, (640, 640))
    # Обработка изображения (инференс YOLO)
        frame = cv2.resize(frame, (640, 640))
        processed_frame = process_image(frame)
        # processed_frame = process_image(img_rgb)
        
        # Отображение обработанного кадра с детекцией
        cv2.imshow("YOLO Detection", processed_frame)
    # cv2.waitKey(0)
        
        # Нажми 'q', чтобы выйти
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Запуск детекции
run_object_detection()