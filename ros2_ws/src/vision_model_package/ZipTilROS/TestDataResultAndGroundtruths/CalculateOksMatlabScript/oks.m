%Input groundtruth and prediction file and write "oks" in the command
%window to run the calculations.

function main()
    % Hardcoded file paths
    groundTruthFile = "C:\Users\kaspe\multisense-rgbd\TestDataResultAndGroundtruths\LumaLeftData\GroundTruthLabels\Test3Position2_jpg.rf.7c88925f3428437a3967c0ca67098e6a.txt"                                ;
    predictionFile = "C:\Users\kaspe\multisense-rgbd\TestDataResultAndGroundtruths\LumaLeftData\ImagesAndPredictionResults\Test3Position2_details.txt"                                                  ;

    % Read keypoints and bounding boxes
    [gt_keypoints, gt_bbox] = readData(groundTruthFile);
    [pred_keypoints, pred_bbox] = readData(predictionFile);

   % Calculate scale from the square root of ground truth bounding box area
    scale = sqrt((gt_bbox(3) - gt_bbox(1)) * (gt_bbox(4) - gt_bbox(2)));
    fprintf('Scale: %f\n', scale);

    % Ki values - define based on keypoints' importance
    ki = ones(22, 1) * 0.1;  
    important_keypoints = [2, 8, 6, 10, 13, 14, 18, 19, 20, 21, 22, 17];
    ki(important_keypoints) = 0.05; 

    % Calculate OKS
    oks_value = calculateOKS(gt_keypoints, pred_keypoints, scale, ki);
    fprintf('The OKS is: %.4f\n', oks_value);
end

function oks = calculateOKS(gt, pred, scale, ki)
    num_keypoints = size(gt, 1);
    oks = 0;
    total_distance = 0; % Initialize the total distance

    for i = 1:num_keypoints
        d_i = norm(gt(i,:) - pred(i,:)); % Euclidean distance can be found using the norm function
        total_distance = total_distance + d_i; % Accumulate total Euclidean distance
        exp_term = exp(- (d_i^2) / (2 * scale^2 * ki(i)^2));
        fprintf('Key point %d: Distance = %f, Exponential term = %f\n', i, d_i, exp_term);
        oks = oks + exp_term;
    end

    oks = oks / num_keypoints;
    average_distance = total_distance / num_keypoints; % Calculate average distance
    fprintf('Average Euclidean Distance: %f\n', average_distance); % Print average distance
end


function [keypoints, bbox] = readData(filename)
    % Open the file
    fid = fopen(filename, 'r');
    % Read and discard the header line
    fgetl(fid);  % Assumes the first line is "Keypoints:"
    keypoints = zeros(22, 2);
    index = 1;

    % Read keypoints
    tline = fgetl(fid);
    while ischar(tline)
        if contains(tline, 'Bounding Boxes')
            break;
        elseif isempty(strtrim(tline))  % Skip empty lines
            tline = fgetl(fid);
            continue;
        end
        % Parse keypoint data
        coords = sscanf(tline, '%f, %f');
        keypoints(index, :) = coords';
        index = index + 1;
        tline = fgetl(fid);
    end

    % Read bounding box
    % Here we continue from the 'Bounding Boxes' line
    bbox_line = fgetl(fid);
    while isempty(strtrim(bbox_line))  % Skip any potential empty lines after 'Bounding Boxes'
        bbox_line = fgetl(fid);
    end
    bbox = sscanf(bbox_line, '%f, %f, %f, %f');
    
    fclose(fid);
end
