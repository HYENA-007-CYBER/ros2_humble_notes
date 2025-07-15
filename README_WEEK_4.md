# Detection Methods Comparison – OpenCV vs YOLOv5

During the Week 4 task focused on detecting an orange cone in a Gazebo simulation, two approaches were implemented and evaluated: one based on OpenCV with HSV (Hue, Saturation, Value) color masking, and the other using the YOLOv5 object detection model. Both methods had their benefits and limitations depending on the scenario and hardware used.

## OpenCV (HSV Color Masking)

This method involved filtering the image by isolating orange hues using HSV (Hue, Saturation, Value) thresholding. A specific lower and upper HSV range was defined:

- Lower bound: `[5, 100, 100]`
- Upper bound: `[25, 255, 255]`

These values were manually tuned to match the cone color in the Gazebo environment.

### Advantages

- Worked without requiring any pre-trained models, internet connection, or GPU.
- Lightweight and efficient, allowing smooth real-time detection even on CPU-only systems.
- ROS 2 integration was straightforward — simply subscribed to the camera topic, processed the frames, and applied masking.
- Lighting conditions in the Gazebo environment remained stable, so no significant lighting-related detection issues were observed.

### Limitations

- Detection was purely based on color, which led to false positives when other orange-colored elements appeared in the environment.
- Although lighting didn’t cause problems in simulation, the method remains sensitive to lighting variations in general use cases.
- Any change in cone shade required manual adjustment of HSV values, which reduced flexibility.
- Object shape or partial visibility had no influence on detection — as long as the color matched, it was considered a detection, even if inaccurate.

Overall, this method provided quick results and was easy to set up, but lacked reliability for more complex environments.

---

## YOLOv5 (You Only Look Once – Deep Learning)

YOLOv5 was used by loading the small model variant (`yolov5s`) from the Ultralytics repository via PyTorch. The model came pretrained on the COCO dataset.

**Note**: The pretrained YOLOv5 model supports only the 80 object classes included in the COCO dataset (such as person, car, bottle, chair, etc.). Since the orange cone was not one of those classes, the model did not detect it by default. This made custom training necessary for cone detection.

### Advantages

- Detection relied on learned object features instead of color, making it more robust in terms of shape recognition and background separation.
- Performed better when the cone was partially visible or placed against different backgrounds.
- Avoided false positives from similarly colored but unrelated objects.
- Supported training for custom objects like the orange cone, allowing for future extensibility.

### Limitations

- Ran entirely on CPU, which slowed down inference and made real-time detection challenging.
- Initial setup involved resolving missing dependencies (`seaborn`, `gitpython`, `pillow`, `setuptools`, etc.), downloading weights, and handling model loading.
- Since the default model did not include the cone, custom training was necessary for production-level detection. This required dataset preparation, annotation, and additional compute resources.
- Integration required more effort compared to OpenCV, including ensuring the camera data was correctly formatted (RGB) and resizing frames to match the model’s input expectations.

---

## Practical Comparison Based on Use Cases

| Scenario                              | OpenCV                        | YOLOv5                           |
|---------------------------------------|-------------------------------|----------------------------------|
| System Resource Requirement           | Very low (CPU-only is fine)   | High (GPU recommended)           |
| Ease of Setup                         | Simple                        | Requires deep learning stack     |
| Internet Requirement                  | No                            | Yes (for model download)         |
| Speed on CPU                          | Real-time                     | Slower                           |
| Lighting Robustness                   | Low                           | High                             |
| Object Differentiation                | Only by color                 | By learned features (shape, etc) |
| Multi-object Support                  | No                            | Yes                              |
| Training Needed                       | No                            | Yes (for custom objects)         |
| Scalability                           | Poor                          | Excellent                        |

---

## Final Thoughts

Both methods served different purposes depending on priorities. OpenCV offered simplicity and speed, making it suitable for quick testing or when only color matters. On the other hand, YOLOv5 offered depth and flexibility, especially for applications requiring accuracy and robustness under varying conditions.

For real-time applications on limited hardware or single-object color detection, OpenCV was the easier and faster path. But for scalable object detection tasks with more reliability, especially in real-world scenarios, YOLOv5 held the advantage.

## Conclusion

The selection of method depends on project scope:
- Use **OpenCV** for quick, color-based prototyping in controlled environments.
- Use **YOLOv5** when robustness, multiple object detection, and real-world deployment are priorities.
