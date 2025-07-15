# Detection Methods Comparison – OpenCV vs YOLOv5

During the fourth week of the project, I tried out two different methods to detect an orange cone in the Gazebo simulation environment. The first was a basic color-based approach using OpenCV and HSV (Hue, Saturation, Value) masking, and the second was a more advanced deep learning method using YOLOv5. Both had their own strengths and challenges, especially when it came to setup, performance, and accuracy on my system.
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

The pretrained YOLOv5 model supports only the 80 object classes included in the COCO dataset (such as person, car, bottle, chair, etc.). Since the orange cone was not one of those classes, the model did not detect it by default. This made custom training necessary for cone detection.

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

## Conclusion

The selection of method depends on project scope:
- Use **OpenCV** for quick, color-based prototyping in controlled environments.
- Use **YOLOv5** when robustness, multiple object detection, and real-world deployment are priorities.
