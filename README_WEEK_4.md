# Detection Methods Comparison

## OpenCV (HSV Color Masking)

**Pros:**
- Lightweight and fast – Ideal for real-time applications on systems without a GPU.
- Easy to implement – Simple color thresholding without the need for training.
- Straightforward ROS 2 integration – Can process raw image topics directly.
- Fully offline – No dependency on pre-trained models or internet access.

**Cons:**
- Highly sensitive to lighting – Detection accuracy drops under different illumination.
- Limited to color – Cannot differentiate between similarly colored objects.
- Poor scalability – Requires manual tuning for every new object or environment.
- Prone to false positives – May detect background elements or textures with similar colors.

---

## YOLOv5 (You Only Look Once – Deep Learning)

**Pros:**
- High accuracy – Performs well under varied lighting and partial occlusion.
- Learns object shape – Not dependent on object color alone.
- Supports multiple objects – Can detect cones, boxes, and more simultaneously.
- Custom training support – Can be fine-tuned on new classes and datasets.

**Cons:**
- Slower on CPU – Inference speed can be low without a GPU.
- More complex setup – Requires PyTorch, model downloads, and proper dependency management.
- Requires labeled data – For detecting custom or untrained objects.
- Heavier dependencies – Needs libraries like seaborn, gitpython, and more.

---

## Conclusion

OpenCV is fast and simple but less reliable in complex environments. YOLOv5 offers superior robustness and flexibility at the cost of setup complexity and compute requirements. Use OpenCV for quick prototyping and YOLOv5 for accuracy-critical applications.
