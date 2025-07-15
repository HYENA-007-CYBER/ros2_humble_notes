# Detection Methods Comparison

## OpenCV (HSV Color Masking)

**Pros:**
- Fast and lightweight – Ideal for real-time applications on low-resource systems.
- Easy to implement – Requires minimal setup and no training phase.
- Simple integration with ROS 2 – Works directly with camera image topics.
- No internet or model dependencies – Fully offline method.

**Cons:**
- Highly sensitive to lighting – Varying illumination or shadows can affect detection accuracy.
- Only detects based on color – Cannot differentiate between objects of the same color.
- Not scalable – Needs separate tuning for each new object or color.
- Prone to false positives – May detect unrelated orange-colored items.

---

## YOLOv5 (You Only Look Once – Deep Learning)

**Pros:**
- High accuracy and robustness – Detects objects even in challenging lighting or partial occlusion.
- Supports multiple object classes – Not limited to color; recognizes object shapes.
- Custom training possible – Can be fine-tuned for detecting cones, boxes, etc.
- Less prone to false detections – Learns detailed visual features.

**Cons:**
- Heavier and slower on CPU – Real-time performance requires a GPU for smooth operation.
- Initial setup is complex – Needs model loading, dependencies, and possibly training.
- Requires labeled training data – For objects not already in YOLO’s pretrained classes.
- More dependencies – PyTorch, torchvision, and sometimes internet access for model download.

---

## Final Note

Both methods were tested for detecting an orange cone in a Gazebo environment using a camera plugin. While OpenCV gave fast results with minimal setup, YOLO provided more accurate and reliable detection, especially under varied lighting and background conditions.

