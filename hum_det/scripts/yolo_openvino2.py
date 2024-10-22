#!/usr/bin/env python3

import cv2
import numpy as np
from openvino.runtime import Core
import openvino as ov
import openvino.properties as props
from notebook_utils import device_widget

NAME = "yolo_openvino"
IMG_PATH = "/home/ruslan/kpfu/magistracy/test_images/sdv1_45.JPG"
TRAIN_HEIGHT = 640
TRAIN_WIDTH = 640

CONFIDENCE_THRESHOLD = 0.5

classes = [
    "foot",
    "head",
    "hand",
    "person",
]

# Colors for the classes above (Rainbow Color Map).
colors = cv2.applyColorMap(
    src=np.arange(0, 255, 255 / len(classes), dtype=np.float32).astype(np.uint8),
    colormap=cv2.COLORMAP_RAINBOW,
).squeeze()


def process_results(frame, results, thresh=0.6):
    # The size of the original frame.
    h, w = frame.shape[:2]
    # The 'results' variable is a [1, 1, 100, 7] tensor.
    results = results.squeeze()
    boxes = []
    labels = []
    scores = []
    for _, label, score, xmin, ymin, xmax, ymax in results:
        # Create a box with pixels coordinates from the box with normalized coordinates [0,1].
        boxes.append(tuple(map(int, (xmin * w, ymin * h, (xmax - xmin) * w, (ymax - ymin) * h))))
        labels.append(int(label))
        scores.append(float(score))

    # Apply non-maximum suppression to get rid of many overlapping entities.
    # See https://paperswithcode.com/method/non-maximum-suppression
    # This algorithm returns indices of objects to keep.
    indices = cv2.dnn.NMSBoxes(bboxes=boxes, scores=scores, score_threshold=thresh, nms_threshold=0.6)

    # If there are no boxes.
    if len(indices) == 0:
        return []

    # Filter detected objects.
    return [(labels[idx], scores[idx], boxes[idx]) for idx in indices.flatten()]


def draw_boxes(frame, boxes):
    for label, score, box in boxes:
        # Choose color for the label.
        color = tuple(map(int, colors[label]))
        # Draw a box.
        x2 = box[0] + box[2]
        y2 = box[1] + box[3]
        cv2.rectangle(img=frame, pt1=box[:2], pt2=(x2, y2), color=color, thickness=3)

        # Draw a label name inside the box.
        cv2.putText(
            img=frame,
            text=f"{classes[label]} {score:.2f}",
            org=(box[0] + 10, box[1] + 30),
            fontFace=cv2.FONT_HERSHEY_COMPLEX,
            fontScale=frame.shape[1] / 1000,
            color=color,
            thickness=1,
            lineType=cv2.LINE_AA,
        )

    return frame


img_rgb = cv2.imread(IMG_PATH, cv2.IMREAD_COLOR)
img_rgb = cv2.resize(img_rgb, (TRAIN_WIDTH, TRAIN_HEIGHT))
proc_img_rgb = img_rgb.transpose((2, 0, 1))
proc_img_rgb = np.expand_dims(proc_img_rgb, axis=0)
proc_img_rgb = proc_img_rgb.astype(np.float32) / 255.0
# proc_img_rgb = proc_img_rgb.astype(np.float32)

ie = Core()
model = ie.read_model(model="/home/ruslan/kpfu/magistracy/ml_models/usar_engineer3_ep0-20_yolov8s/best_openvino_model/best.xml")
compiled_model = ie.compile_model(model=model, device_name="CPU")
input_layer = compiled_model.input(0)
output_layer = compiled_model.output(0)

result = compiled_model([proc_img_rgb])[output_layer]
# result = compiled_model([proc_img_rgb])
print(result.dtype)
print(result.shape)
# print(result)



# ----------------------------------------------------------------------------------------
# core = ov.Core()

# devices = core.available_devices

# for device in devices:
# 	device_name = core.get_property(device, props.device.full_name)
# 	print(f"{device}: {device_name}")
	
# model = core.read_model(model="/home/ruslan/kpfu/magistracy/ml_models/usar_engineer3_ep0-20_yolov8s/best_openvino_model/best.xml")
# compiled_model = core.compile_model(model=model, device_name="CPU")
# print(model.inputs)
# print(model.outputs)
# print(compiled_model.inputs)
# print(compiled_model.outputs)
# print(model.input(0))
# input_layer = model.input(0)
# print(input_layer.any_name)
# print(f"input precision: {input_layer.element_type}")
# print(f"input shape: {input_layer.shape}")
# output_layer = model.output(0)
# # print(output_layer.any_name)
# print(f"output precision: {output_layer.element_type}")
# print(f"output shape: {output_layer.shape}")
# ----------------------------------------------------------------------------------------

cv2.imshow(NAME, draw_detections(img_rgb, result))
cv2.waitKey(0)
cv2.destroyAllWindows()