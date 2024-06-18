import cv2
import numpy as np
import matplotlib.pyplot as plt
from ultralytics import YOLO

# Paths
model_path = '/home/kodenfly123/ros2_ws/src/vision_model_package/ZipTilROS/TrainedYOLOv8PoseModel/weights/best.pt'
img_path = '/home/kodenfly123/ros2_ws/src/vision_model_package/ZipTilROS/TestsSortedToSingleFrames/Test1_250cm/Luma_Rectified_Left/tiff/1111.tiff'

# Load the model
model = YOLO(model_path)

# Run inference
results = model(img_path)  # returns a list of Results objects

# Load the image
image = cv2.imread(img_path)

# Process results
for result in results:
    # Draw bounding boxes
    for bbox in result.boxes.xyxy:  # Assuming bounding boxes are in the 'boxes.xyxy' attribute
        x1, y1, x2, y2 = bbox.cpu().numpy().astype(int)
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Draw bounding box in green

    # Draw keypoints
    keypoints_tensor = result.keypoints.xy  # Assuming keypoints are in the 'keypoints.xy' attribute
    keypoints_array = keypoints_tensor.cpu().numpy()

    # If there's an extra list wrapping the keypoints, access the first item
    if len(keypoints_array) == 1:
        keypoints_array = keypoints_array[0]

    # Draw keypoints and IDs
    for idx, keypoint in enumerate(keypoints_array):
        x, y = keypoint[:2].astype(int)  # Assuming keypoints are in (x, y) format
        cv2.circle(image, (x, y), 5, (0, 0, 255), -1)  # Draw keypoints in red
        cv2.putText(image, str(idx + 1), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)  # Draw keypoint ID in white, offset by 1

# Convert BGR image to RGB
image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

# Display the image using Matplotlib
plt.imshow(image_rgb)
plt.axis('off')
plt.show()
