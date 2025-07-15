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

---
# Difficulties Faced

I came across many errors during the Week 4 task — detecting an orange cone in Gazebo using OpenCV and YOLOv5, integrated with ROS 2 Humble.

---

## Environment Setup & Package Issues

### 1. `colcon build` Failures  
Initial builds failed with errors like `option --editable not recognized` and `option --uninstall not recognized`.  
Cause traced to outdated or unsupported fields in `setup.py`. Removing deprecated fields such as `tests_require` and cleaning up the structure resolved the issue.

### 2. Numpy Version Conflict  
A version mismatch occurred between OpenCV and YOLOv5.  
YOLOv5 required `numpy` versions `<2.0`, while OpenCV 4.12+ required `>=2.0`.  
Downgrading to `numpy==1.24.4` served as a functional middle ground and allowed both systems to operate.

### 3. No Disk Space  
Builds and package installations triggered "No space left on device" errors.  
Resolved by deleting unnecessary pip caches, removing unused packages, and cleaning ROS build folders (`build`, `install`, `log`).

---

## OpenCV-Related Challenges

### 4. HSV Tuning  
HSV (Hue, Saturation, Value) thresholds were tuned manually to the cone’s color:
- Lower: `[5, 100, 100]`
- Upper: `[25, 255, 255]`  
Although lighting inside Gazebo was consistent, this method would be sensitive in real-world conditions where cone color or lighting varies.

---

## YOLOv5-Specific Problems

### 5. Pretrained Class Limitation  
YOLOv5's pretrained model (`yolov5s`) only supports 80 COCO classes, which do not include traffic cones.  
As a result, detection of the orange cone was not possible without custom training.

### 6. Missing Dependencies  
Several runtime errors occurred due to missing modules such as `seaborn`, `gitpython`, `pillow`, and `requests`.  
These had to be installed manually to ensure smooth execution of the YOLO node.

### 7. CPU-Only Performance  
System lacked a GPU, confirmed with `torch.cuda.is_available() == False`.  
YOLOv5 inference on CPU was slow and unsuitable for real-time processing, especially at full image resolution.

---
## Model Unavailability Problems

### 8. Object Used Was a Cylinder, Not a Cone  
A simple orange **cylinder** was used in the Gazebo world instead of an actual cone due to **unavailability of a suitable cone model**.  
This substitution slightly altered the original task objective but allowed the detection methods to be tested effectively. Both OpenCV and YOLO were applied on this object with adjusted parameters.
