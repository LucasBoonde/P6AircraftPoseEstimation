from ultralytics import YOLO


def YoloPredict():
    model_path = '/home/kodenfly123/Desktop/ZipTilROS/TrainedYOLOv8PoseModel/weights/best.pt'

    img_path = '/home/kodenfly123/Desktop/ZipTilROS/TestDataResultAndGroundtruths/LumaLeftData/ImagesAndPredictionResults/Test1.jpg'

    # Load a model
    model = YOLO(model_path)  # pretrained YOLOv8n model

    # Run batched inference on a list of images
    results = model(img_path)  # return a list of Results objects

    # Process results
    for result in results:
        # Assuming each result has a 'keypoints' attribute with a '.xy' tensor
        keypoints_tensor = result.keypoints.xy  # This is the tensor

        # Move the tensor to CPU and convert to a NumPy array
        keypoints_array = keypoints_tensor.cpu().numpy()

        # If there's an extra list wrapping the keypoints, and you're sure it always contains exactly one item
        if len(keypoints_array) == 1:
            keypoints_array = keypoints_array[0]  # Access the first item

        # Convert the NumPy array to a Python list
        keypoints_list = keypoints_array.tolist()

        return(keypoints_list)


def main():
    print("Hej: ")
    print(YoloPredict())

if __name__ == "__main__":
    main()
