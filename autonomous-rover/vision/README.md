# Vision — Autonomous Rover (YOLOv8) 🚀

> YOLOv8-based vision pipeline for the Autonomous Rover project (detects `box` and `crater`).
> This document describes dataset creation, Roboflow integration, training, evaluation, and inference — everything needed to reproduce the results.

---

## Table of contents

* [Project overview](#project-overview)
* [Dataset (collection & annotation)](#dataset-collection--annotation)
* [Roboflow — download & code snippet](#roboflow---download--code-snippet)
* [Training (hyperparameters & command)](#training-hyperparameters--command)
* [Results & outputs](#results--outputs)
* [Inference / usage](#inference--usage)
* [Notes & next steps](#notes--next-steps)

---

## Project overview

This module trains an object detector to recognise:

* `box` — obstacles/boxes on the track
* `crater` — ground depressions / hazards

The model is trained with YOLOv8 (Ultralytics). Dataset was collected using a DSLR and prepared with Roboflow.

---

## Dataset (collection & annotation)

**Data collection**

* Capture device: DSLR camera.
* Images captured: **~120** images (varied lighting, angles, and distances).
* Scenes: typical rover environment (boxes, holes/craters, background clutter).

**Annotation**

* Classes: `box`, `crater`.
* Tool: Roboflow Annotator (export in YOLO v8 / YOLO TXT format).
* Each image was annotated with bounding boxes for each object instance.

**Augmentation & preprocessing**
Performed augmentations to increase robustness:

* Resizing to YOLO input size (e.g., 640 × 640)
* Horizontal flips, vertical flips (where appropriate)
* Random rotations (angles -> +15 and -15 deg)

Final dataset version (export) used for training was uploaded to Roboflow and downloaded as a YOLOv8 dataset.

[📂 **Dataset**](./Dataset_Results/datasets/isro_model-1) 

---

## Roboflow — download & code snippet

Use this snippet in the notebook to download your dataset from Roboflow:

```python
!pip install roboflow

from roboflow import Roboflow
rf = Roboflow(api_key="xxxxxxxxxxxxxxxxxxx")  # replace xxxxxxxxxxxxxxxxx with your Roboflow API key
project = rf.workspace("xxxxxxx").project("xxxxxxxxxx")
version = project.version(1)
dataset = version.download("yolov8")
```

**How to get the exact snippet from Roboflow**

1. Go to your dataset page in Roboflow → Version → Download → choose **YOLOv8**.
2. Click **Download Code** — Roboflow will give you the code block with your exact `api_key`, `workspace`, `project`, and `version`.
3. Replace the placeholders above with those exact values (or paste Roboflow's downloaded snippet into your notebook cell).

> Tip: Keep your API key secret. Use environment variables in production (e.g., `os.environ["ROBOFLOW_API_KEY"]`) instead of hardcoding.

---


After downloading, the dataset folder will contain the standard YOLOv8 layout:

```
/dataset-name/
  images/
    train/
    valid/
    test/
  labels/
    train/
    valid/
    test/
  data.yaml   # (Roboflow produces a YAML pointing to train/valid/test and class names)
```

---

## Training — hyperparameters & command

```bash
!yolo task=detect mode=train model=yolov8s.pt data={dataset.location}/data.yaml epochs=50 imgsz=640 plots=True
```

### Hyperparameters

| Parameter     |                 Value                 | Notes                            |
| ------------- | ------------------------------------: | -------------------------------- |
| model         |                          `yolov8s.pt` | base Model used                  |
| epochs        |                                  `50` | number of training epochs        |
| imgsz         |                                 `640` | input resize (YOLO image size)   |
| batch         |                                  `16` | batch size used during training  |
| optimizer     |                               `AdamW` | optimizer from notebook          |
| lr            |                            `0.001667` | initial learning rate            |
| momentum      |                                 `0.9` |          -                       |
| weight_decay  |                              `0.0005` | regularization                   |
| augmentations | list (flip, rotate -> +15 and -15 deg)| Augmentation values used         |


---

## Results & outputs

**Training logs & outputs**

* YOLOv8 stores outputs in `runs/detect/train/<name>/` by default.

  * `runs/detect/train/weights/best.pt` — best weights
  * `runs/detect/train/results.png` — training curves and metrics

**Metrics summary**

* mAP@50: `0.969`
* mAP@50-95: `0.723`
* Precision: `0.91`
* Recall: `0.955`



## Original image

 ![IMG_9393_JPG rf 55e083c721c57aed0c4771baa2e4b48a](https://github.com/user-attachments/assets/9365cf05-f7f6-4491-918a-5d528f2945f6)


## Model prediction (from `yolo predict` or from `runs/detect/predict/`)

![IMG_9393_JPG rf 55e083c721c57aed0c4771baa2e4b48a](https://github.com/user-attachments/assets/4e12f035-aa7a-418f-ae9b-b03ff32153a2)

---

## Inference / usage

**Run detection on a folder of test images**

```bash
# using ultralytics CLI
yolo detect predict model=runs/detect/train/weights/best.pt source="test_images/" save=True
```

**In Python (script / notebook)**

```python
from ultralytics import YOLO

model = YOLO("runs/detect/train/weights/best.pt")
results = model.predict(source="test_images/image1.jpg", imgsz=640, conf=0.25)
# results[0].boxes, results[0].probs, etc.
```

**Output location**

* CLI predictions saved to `runs/detect/predict/` (default).
* You can also set `save=True` or specify the `project` and `name` in the CLI.

---

## Notes, tips & troubleshooting

* If you use Roboflow's hosted dataset, the `data.yaml` they produce is ready to use with YOLOv8; verify class order matches your labels.
* If training fails due to OOM (out-of-memory), reduce `batch` or `imgsz` — e.g., `batch=8` or `imgsz=512`.
* For faster iteration on CPU-only systems, use fewer epochs and smaller `imgsz`, but results will be lower. For production / real-time on the robot, export to ONNX / TensorRT and run on an edge device (Jetson/Coral).
* Keep the original dataset (or upload to Roboflow) so experiments are reproducible.

---
