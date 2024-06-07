import argparse
import numpy as np
import cv2
import yaml
from ultralytics import YOLO


def opencv_matrix_constructor(loader, node):
    mapping = loader.construct_mapping(node, deep=True)
    return {
        'rows': mapping['rows'],
        'cols': mapping['cols'],
        'dt': mapping['dt'],
        'data': mapping['data']
    }


def preprocess_yaml_file(filepath):
    with open(filepath, 'r') as file:
        lines = file.readlines()

    # Remove the first line if it contains a YAML directive
    if lines[0].startswith('%YAML'):
        lines = lines[1:]

    return ''.join(lines)


def load_yaml_with_opencv_matrix(filepath):
    yaml.SafeLoader.add_constructor("tag:yaml.org,2002:opencv-matrix", opencv_matrix_constructor)

    yaml_content = preprocess_yaml_file(filepath)
    return yaml.safe_load(yaml_content)


def backproject_disparity(calibration_left, calibration_right, disparity_image):
    if disparity_image is None:
        print("No disparity image is loaded")
        return
    points = []
    scale = 2
    fx = calibration_left[0] / scale #Focal lengths påvirkning på pixel i x retningen for de to "ens" kameraer i stereo
    fy = calibration_left[5] / scale #Focal lengths påvirkning på pixel i y retningen for de to "ens" kameraer i stereo
    cx = calibration_left[2] / scale #The principal point's x offset
    cy = calibration_left[6] / scale #The principal point's y offset

    tx = calibration_right[3] / calibration_right[0] #Translation along x between the 2 cameras


    # Construct the 4x4 projection matrix K_proj
    K_proj = np.array([
        [fy * tx, 0, 0, -fy * cx * tx],
        [0, fx * tx, 0, -fx * cy * tx],
        [0, 0, 0, fx * fy * tx],
        [0, 0, -fy, fy * (cx - cx)] #Fejl her cx - cx??
    ])

    # Initialize an empty array to hold the world coordinates
    # The shape is based on the disparity_array, and we'll have 3 values for each point (X, Y, Z)
    world_coordinates = np.zeros((disparity_image.shape[0], disparity_image.shape[1], 3))

    
    # Create a grid of x, y coordinates
    x, y = np.meshgrid(np.arange(disparity_image.shape[1]), np.arange(disparity_image.shape[0]))

    # Filter out the points where disparity is <= 8 or > 252
    mask = (disparity_image > 10) & (disparity_image <= 255)

    # Only keep the points that pass the filter
    x, y, d = x[mask], y[mask], disparity_image[mask]

    # Calculate beta for all points at once
    beta = fx / d

    # Create 4D homogeneous coordinates for all points
    uv = np.column_stack((x, y, d, np.ones_like(x)))

    # Calculate world coordinates for all points
    sensor_coord = np.dot(K_proj, uv.T) / beta

    # Normalize by the last element
    sensor_coord_scaled = sensor_coord / sensor_coord[-1, :] * scale

    world_coordinates[y, x, :] = sensor_coord_scaled[:3, :].T



    return world_coordinates

def YoloPredict():
    model_path = '/home/kodenfly123/Desktop/ZipTilROS/TrainedYOLOv8PoseModel/weights/best.pt'

    img_path = '/home/kodenfly123/Desktop/ZipTilROS/TestsSortedToSingleFrames/Test1_250cm/Luma_Rectified_Left/tiff/1111.tiff'
    
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
    parser = argparse.ArgumentParser(description='Load a YAML file with custom opencv-matrix tags into a Python dict.')
    # parser.add_argument('intrinsics', type=str, help='Path to the YAML file')
    parser.add_argument('extrinsics', type=str, help='Path to the YAML file')
    parser.add_argument('disparity', type=str, help='Path to the YAML file')
    #parser.add_argument('lumaleft', type=str, help='Path to the YAML file')

    args = parser.parse_args()
    
    """ Først initieres en parser som kan oversætte yaml filerne med opencv matricer til python sprog. Herefter tilføjes 3 argumenter som skal gives når scriptet køres. 
        Og disse argumenter bliver parset og herefter overført til args variablen.
    """

    # intrinsics = load_yaml_with_opencv_matrix(args.intrinsics)
    #lumaleft = cv2.imread(args.lumaleft, cv2.IMREAD_UNCHANGED)
    extrinsics = load_yaml_with_opencv_matrix(args.extrinsics)
    disparity = cv2.imread(args.disparity, cv2.IMREAD_UNCHANGED) / 16 
    """ #The flag cv2.IMREAD_UNCHANGED is used to load the image in its original format, which is typically a 16-bit image where the pixel values 
    represent disparity in sub-pixel units. Dividing by 16 scales the disparity values from sub-pixel units (often expressed in 1/16th of a pixel) to actual pixel units, 
    which are easier to work wit
    """

    lumaleft_world_coordinates = backproject_disparity(extrinsics["P1"]["data"], extrinsics["P2"]["data"], disparity)

    keypoints_list = YoloPredict()
    xyz_coordinates_list = []

    for keypoint in keypoints_list:
        # Each 'keypoint' is an [x, y] list
        x, y = keypoint
        
        #Obs: The neural network can estimae keypoints to sub pixel precision, and therefore returns coordinates in floats
        #which does not fit into the pixel grid as it is specified by whole integers.
        x = int(round(x))
        y = int(round(y))
        
        
        if y < lumaleft_world_coordinates.shape[0] and x < lumaleft_world_coordinates.shape[1]:
            xyz_coordinate = lumaleft_world_coordinates[y, x, :]
            xyz_coordinates_list.append(xyz_coordinate)
        else:
            print(f"Warning: Key point ({x}, {y}) is out of the image bounds.")

    print("Keypoint list:")
    print(keypoints_list)
    print("\n")
    print("Corresponding world coordinate list (X,Y,Z):")
    print(xyz_coordinates_list)

    
    
    
    


if __name__ == "__main__":
    main()
