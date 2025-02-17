import cv2
import numpy as np
from scipy.spatial import KDTree
import math
import glob
import os
import re

#TODO check all these guesses and thresholds
RESIZE_FACTOR = 1
BRIGHTNESS_THRESHOLD = 20
SIZE_THRESHOLD = 10 * RESIZE_FACTOR      # Has to match with RESIZE_FACTOR
NEIGHBOR_RADIUS = 800 * RESIZE_FACTOR   # Radius for finding neighbors  #TODO do circle stuff
PERCENTAGE_OF_DESCRIPTORS = 1  # Percentage of descriptors to extract
MAX_ROTATION = 15  # Maximum rotation between images in degrees

fs = cv2.FileStorage("calibration.json", cv2.FILE_STORAGE_READ)
camera_matrix = fs.getNode("camera_matrix").mat()
distortion_coefficients = fs.getNode("distortion_coefficients").mat()
new_camera_matrix = fs.getNode("new_camera_matrix").mat()
fs.release()

    
# Create mapping arrays for x and y
map_x = None
map_y = None

try:
    map_x = np.load("map_x.npy")
    map_y = np.load("map_y.npy")
except:
    print("WARNING: Mapping arrays not found")

if new_camera_matrix is None:
    print("ERROR: Camera calibration not found")

def get_image_paths(folder_path):
    image_paths = glob.glob(os.path.join(folder_path, "image_*.jpg"))
    # Sort files numerically by extracting numeric parts of filenames
    def extract_number(path):
        match = re.search(r"(\d+)", os.path.basename(path))
        return int(match.group(1)) if match else float('inf')    
    image_paths = sorted(image_paths, key=extract_number)
    return image_paths

def load_images(image_paths, undistort=True):
    images = [cv2.imread(img_path) for img_path in image_paths]
    # TODO do undistortion here
    images = [cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) for img in images]
    if undistort:
        images = [cv2.undistort(img, camera_matrix, distortion_coefficients, None, new_camera_matrix) for img in images]
        # TODO remove this if camera is on rotation axis
        if map_x is not None and map_y is not None:
            print("INFO: Using remapping")
            images = [cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR) for img in images]
    images = [cv2.resize(img, (0,0), fx=RESIZE_FACTOR, fy=RESIZE_FACTOR) for img in images]
    return images


def extract_stars(image, visualize=False):
    # Threshold the image to isolate white dots
    _, thresholded = cv2.threshold(image, BRIGHTNESS_THRESHOLD, 255, cv2.THRESH_BINARY)

    # Find contours (dots)
    contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Create a color version of the image for visualization
    visualization = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    # Extract dot information and annotate the image
    dot_data = []
    for contour in contours:
        # Get bounding circle
        (x, y), radius = cv2.minEnclosingCircle(contour)
        size = int(np.pi * radius**2)  # Approximate size as area of the circle
        if size > SIZE_THRESHOLD:  # Skip small dots
            # Get brightness (average intensity inside contour)
            mask = np.zeros_like(image)
            cv2.drawContours(mask, [contour], -1, 255, -1)
            brightness = cv2.mean(image, mask=mask)[0]
            # Append to list
            dot_data.append((x, y, size, brightness))
            
            if visualize:
                # Draw the contour
                cv2.circle(visualization, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                
                # Add labels for size and brightness
                label = f"S:{size}, B:{brightness:.1f}"
                cv2.putText(visualization, label, (int(x) - 40, int(y) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1, cv2.LINE_AA)
    
    # Display the result
    if visualize:
        # cv2.imshow("Contours with Labels", visualization)
        cv2.imwrite("detectedStars.jpg", visualization)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return dot_data

def find_neighbours(position, dot_data, image=None):
    kdtree = KDTree([dot[:2] for dot in dot_data])   #TODO this should only happen once and not in every function call
    neighbors_idx = kdtree.query_ball_point(position, NEIGHBOR_RADIUS)
    neighbors = [dot_data[i] for i in neighbors_idx]

    if image is not None:
        annotated_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        # Annotate the image with neighbor labels
        for i, idx in enumerate(neighbors_idx):
            x, y, s, b = dot_data[idx]
            cv2.circle(annotated_image, (int(x), int(y)), 5, (0, 255, 0), -1)  # Mark neighbors
            cv2.putText(
                annotated_image, f"S:{s}, B:{b:.1f}", (int(x) + 5, int(y) - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1
            )
        # Highlight the query position
        cv2.circle(annotated_image, (int(position[0]), int(position[1])), 7, (255, 0, 0), -1)
        # cv2.imshow("Contours with Labels", annotated_image)
        cv2.imwrite("detectedNeighbours.jpg", annotated_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return neighbors



def get_formation(position, neighbours, image=None):
    # Calculate distances and angles
    distances_and_angles = []
    for neighbor in neighbours:
        neighbor_position = neighbor[:2]
        dx = neighbor_position[0] - position[0]
        dy = neighbor_position[1] - position[1]
        
        # Calculate distance
        distance = math.sqrt(dx**2 + dy**2)
        
        # Calculate angle (in degrees)
        angle = math.degrees(math.atan2(dy, dx))

        brightness = neighbor[3]
        size = neighbor[2]
        
        # Append results
        distances_and_angles.append((neighbor_position, distance, angle, size, brightness))

    # Sort by distance
    distances_and_angles.sort(key=lambda x: x[1], reverse=True)
    # Remove the query position from the list
    # distances_and_angles = distances_and_angles[:-1]   #TODO check if this is correct

    if image is not None:
        # Annotate the image with lines, distances, and angles
        annotated_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        for i, (pos, dist, angle, _, _) in enumerate(distances_and_angles):
            x, y = pos
            # Draw a line between the query position and the neighbor
            cv2.line(annotated_image, (int(position[0]), int(position[1])), (int(x), int(y)), (0, 255, 0), 1)
            
            # Annotate the line with distance and angle
            mid_x, mid_y = (position[0] + x) / 2, (position[1] + y) / 2
            label = f"{i}: D={dist:.1f}, A={angle:.1f}"
            cv2.putText(
                annotated_image, label, (int(mid_x) + 5, int(mid_y) - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1
            )
        # Highlight the query position
        cv2.circle(annotated_image, (int(position[0]), int(position[1])), 7, (255, 0, 0), -1)
        # cv2.imshow("detectedFormations", annotated_image)
        cv2.imwrite("detectedFormation.jpg", annotated_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return distances_and_angles


def get_descriptors(image, visualize=False):
    visualization_image = image if visualize else None

    dot_data = extract_stars(image, visualize)
    # Sort the dot_data list by brightness (dot[3]) in descending order
    sorted_dots = sorted(dot_data, key=lambda dot: dot[3], reverse=True)
    brightest_dots = [dot[:2] for dot in sorted_dots[:int(len(dot_data) * PERCENTAGE_OF_DESCRIPTORS)]]

    descriptors = []
    
    for dot in brightest_dots:
        neighbours = find_neighbours(dot, dot_data, visualization_image)
        descriptors.append(get_formation(dot, neighbours, visualization_image))


    if visualize:
        annotated_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        for indx, (dot, descriptor) in enumerate(zip(brightest_dots, descriptors)):
            for i, (pos, dist, angle,_,_) in enumerate(descriptor):
                x, y = pos
                # Draw a line between the query position and the neighbor
                cv2.line(annotated_image, (int(dot[0]), int(dot[1])), (int(x), int(y)), (0, 255, 0), 1)
                
                # Annotate the line with distance and angle
                mid_x, mid_y = (dot[0]*1.3 + x*0.7) / 2, (dot[1]*1.3 + y*0.7) / 2
                label = f"{i}: D={dist:.1f}, A={angle:.1f}"
                cv2.putText(
                    annotated_image, label, (int(mid_x) + 5, int(mid_y) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1
                )
            # Highlight the query position
            cv2.circle(annotated_image, (int(dot[0]), int(dot[1])), 7, (255, 0, 0), -1)
            cv2.putText(
                    annotated_image, f"{indx}", (int(dot[0]), int(dot[1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 4
                )
        # cv2.imshow("detectedFormations", annotated_image)
        cv2.imwrite("detectedFormations.jpg", annotated_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    total_size = 0
    for descriptor in descriptors:
        total_size += descriptor[0][3]
    if len(descriptors) < 5 or total_size/len(descriptors) > 300:
        return [], []

    return brightest_dots, descriptors


def get_percantage_dif(value1, value2):
    if value1 == 0 and value2 == 0:
        return 0
    return abs(value1 - value2) / ((value1 + value2) / 2)

def calc_error(feature, feature2, rot_guess):
    # TODO The distance error calculation does not work like this 
    # because the images would need to be distorted first 
    # AND distances get larger when they are closer to the edge of the frame

    rot_error = abs(feature[2] - feature2[2] - rot_guess)   # include distance scaling  # TODO account for wrap around at 360. To avoid this, use x,y distance instead of angle
    d_error = get_percantage_dif(feature[1], feature2[1]) 
    s_error = get_percantage_dif(feature[3], feature2[3])
    b_error = abs(feature[4] - feature2[4]) 
    return d_error, rot_error, s_error, b_error

def acceptable_error(d_error, rot_error, s_error, b_error):
    # TODO replace this magic numbers with something that makes sense
    if d_error < 0.1:  # TODO do image distortion first
        if rot_error < 5: 
            return True
            if s_error < 1.0:  #dont do percentage error for size
                # return True
                if b_error < 40:   # TODO this sucks
                    return True
    return False

def compare_match(descriptor, descriptor2):  # TODO make return value a likelihood of a match
    features = min(len(descriptor), len(descriptor2))

    possible_matches = []

    if features > 2:
        # TODO try to use less rotation guesses for better performance
        # (is the list sorted by distance? Then taking the two points with largest distance makes sense)

        possible_rotations = [descriptor[0][2] - descriptor2[i][2] for i in range(len(descriptor2))]
        possible_rotations = [x for x in possible_rotations if -MAX_ROTATION < x < MAX_ROTATION]  # TODO maybe negate?
        # possible_rotations = [
        #     x + offset
        #     for x in (descriptor[0][2] - descriptor2[i][2] for i in range(len(descriptor2)))
        #     for offset in np.arange(-2,2,0.5)    # TODO maybe reduce step size or environment size for perfoemance
        #     if -MAX_ROTATION < x + offset < MAX_ROTATION
        # ]
        # possible_rotations = [4.5]
        # possible_rotations = [x*0.5  for x in range(-MAX_ROTATION*2,MAX_ROTATION*2)]
        for rot_guess in possible_rotations:
            self_error = calc_error(descriptor[-1], descriptor2[-1], rot_guess)
            rot_error = 0
            d_error =  0
            s_error = self_error[2]
            b_error = self_error[3]

            matched_features = 0
            for feature_1 in descriptor[:-1]: # TODO sort by angle and dont compare every point with every other point
                best_error = (math.inf, math.inf, math.inf, math.inf)
                for feature_2 in descriptor2[:-1]:
                    error = calc_error(feature_1, feature_2, rot_guess)
                    if  (error[0] < best_error[0] and
                        error[1] < best_error[1] ): 
                        # error[2] < best_error[2] and
                        # error[3] < best_error[3]):
                        best_error = error

                if acceptable_error(*best_error):
                    rot_error += best_error[1] 
                    d_error += best_error[0] 
                    s_error += best_error[2]
                    b_error += best_error[3]
                    matched_features += 1

            if matched_features > features*0.5 and matched_features > 2 and acceptable_error(d_error/matched_features, rot_error/matched_features, s_error/(matched_features+1), b_error/(matched_features+1)):
                # print(f"compared {features} features with matches: {matched_features} and rotation guess: {rot_guess}")
                # print(rot_error/matched_features)
                # print(d_error/matched_features)
                # print(s_error/(matched_features+1))
                # print(b_error/(matched_features+1))
                possible_matches.append((rot_guess, rot_error/matched_features, d_error/matched_features, s_error/(matched_features+1), b_error/(matched_features+1), matched_features))
    return possible_matches

def match_descriptors(descriptors1, descriptors2, image1=None, image2=None):
    # assert len(points1) is len(descriptors1)
    # assert len(points2) is len(descriptors2)

    matched_points1 = []
    matched_points2 = []
    meta = []

    for indx in range(len(descriptors1)):
        best_match = (0, 0, 0, 0, 0, 0)
        best_match_indx = 0

        #find best match
        for indx2 in range(len(descriptors2)):
            descriptor = descriptors1[indx]
            descriptor2 = descriptors2[indx2]
            compare_results = compare_match(descriptor, descriptor2)
            if len(compare_results) > 0:
                match_meta = sorted(compare_results, key=lambda x: x[5], reverse=True)[0]
                if match_meta[5] > best_match[5]:
                    best_match = match_meta
                    best_match_indx = indx2

        # add best match to list
        if best_match[5] > 0:
            point2 = descriptors2[best_match_indx][-1][0]
            if point2 in matched_points2:
                # print("WARNING: point already matched")
                existing_index = matched_points2.index(point2)
                if best_match[5] > meta[existing_index][2][5]:
                    matched_points1[existing_index] = descriptors1[indx][-1][0]
                    matched_points2[existing_index] = point2
                    meta[existing_index] = (indx, best_match_indx, best_match)

            else:
                matched_points1.append(descriptors1[indx][-1][0])
                matched_points2.append(point2)
                meta.append((indx, best_match_indx, best_match))
        else:
            # print("WARNING: No match found")
            pass

    if image1 is not None and image2 is not None:
        # Concatenate images side by side
        height = max(image1.shape[0], image2.shape[0])
        width = image1.shape[1] + image2.shape[1]
        visualization = np.zeros((height, width, 3), dtype=np.uint8)

        # Place images in the visualization
        visualization[:image1.shape[0], :image1.shape[1]] = cv2.cvtColor(image1, cv2.COLOR_GRAY2BGR)
        visualization[:image2.shape[0], image1.shape[1]:] = cv2.cvtColor(image2, cv2.COLOR_GRAY2BGR)
        # add yellow line between images
        cv2.line(visualization, (image1.shape[1], 0), (image1.shape[1], height), (0, 255, 255), 2)

        # Draw lines between matched points
        for p1, p2, meta in zip(matched_points1, matched_points2, meta):
            point1 = (int(p1[0]), int(p1[1]))
            point2 = (int(p2[0] + image1.shape[1]), int(p2[1]))  # Offset by image1 width
            cv2.line(visualization, point1, point2, (0, 255, 0), 2)
            indx2, indx, match_meta = meta
            label = f"{indx}:{indx2}({match_meta[0]:.2f}){match_meta[1]:.2f}/{match_meta[2]:.2f}/{match_meta[3]:.2f}/{match_meta[4]:.2f}~{match_meta[5]}"
            cv2.putText(visualization, label, ((point1[0]+point2[0])//2, (point1[1]+point2[1]-10)//2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

        # Show the visualization
        cv2.imwrite("detectedMatches.jpg", visualization)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    print(f"INFO: Found {len(matched_points1)} matches")
    return matched_points1, matched_points2


def descriptor_transformation(descriptor, H):
    warped_descriptor = []
    for neighbor in descriptor:
        x = neighbor[0][0]
        y = neighbor[0][1]
        warped_point = np.array([x, y, 1.0]).dot(H.T)
        warped_point = warped_point[:2] / warped_point[2]
        warped_descriptor.append((warped_point, neighbor[1], neighbor[2], neighbor[3], neighbor[4]))
    return warped_descriptor

def get_transform_matrix(points1, points2):
    # H, _ = cv2.findHomography(np.array(points1), np.array(points2), cv2.RANSAC, 50.0 * ss.RESIZE_FACTOR)
    # H = cv2.getPerspectiveTransform(np.array(points1[:4], dtype=np.float32), np.array(points2[:4], dtype=np.float32))
   
   #TODO check which threshold makes sense
    # matrix = cv2.estimateAffine2D( np.array(points2, dtype=np.float32), np.array(points1, dtype=np.float32), cv2.RANSAC, ransacReprojThreshold=300.0)[0]
    matrix = cv2.estimateAffinePartial2D( np.array(points2, dtype=np.float32), np.array(points1, dtype=np.float32), cv2.RANSAC, ransacReprojThreshold=300.0)[0]
    
    rotation_scaling = matrix[:2, :2]
    # Compute scaling factors
    scale_x = np.linalg.norm(rotation_scaling[0])  # Norm of the first row
    scale_y = np.linalg.norm(rotation_scaling[1])  # Norm of the second row
    # Normalize the rotation-scaling matrix
    normalized_rotation = rotation_scaling / np.array([scale_x, scale_y]).reshape(2, 1)
    # Replace the rotation-scaling part with the normalized rotation matrix
    normalized_matrix = matrix.copy()
    normalized_matrix[:2, :2] = normalized_rotation
    matrix = normalized_matrix

    H = np.eye(3)
    H[:2, :] = matrix

    # scale_rotation_matrix = matrix[:2, :2]
    
    # # Compute scaling factors as the norms of the rows
    # scale_x = np.linalg.norm(scale_rotation_matrix[0])
    # scale_y = np.linalg.norm(scale_rotation_matrix[1])
    
    # print("INFO: Scale factors:", scale_x, scale_y)

    return H