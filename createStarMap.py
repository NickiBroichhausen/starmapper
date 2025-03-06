import cv2
import numpy as np
import os
import StarSift as ss


def decompose_rotation(H):
    """Extract rotation matrix from homography."""
    R = H[:3, :3]
    return R

def compute_rotation_angle(R):
    """Compute the rotation angle (in radians) from a rotation matrix."""
    angle = np.arccos((np.trace(R) - 1) / 2)  # Trace formula
    return angle if not np.isnan(angle) else 0

def rotation_matrix(theta, center=None):
    """Create a 3x3 rotation matrix for a given angle theta (in radians)."""
    if center:
        cx, cy = center
        cos_a = np.cos(theta)
        sin_a = np.sin(theta)
        
        # Translate to origin, rotate, translate back
        return np.array([
            [cos_a, -sin_a, cx - cx * cos_a + cy * sin_a],
            [sin_a,  cos_a, cy - cx * sin_a - cy * cos_a],
            [0,      0,     1]
        ])
    else:
        return np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ])

def get_loop_closing_delta_R(H_loop, H_list):
    n = len(H_list)
    
    # Decompose loop closure homography to find rotation
    R_loop = decompose_rotation(H_loop)
    print(R_loop)
    theta_loop = - compute_rotation_angle(R_loop)
    
    # Compute the angular adjustment per step
    print(f"INFO: rotation in loop: {np.degrees(theta_loop)}")
    delta_theta = theta_loop / n
    print(f"INFO: loop closing correction: {np.degrees(delta_theta)}")
    #TODO sometimes this is negative, why?
    delta_R = rotation_matrix(-delta_theta)
    return delta_R

def createStarMap(folder, visualisation=False):
    paths = ss.get_image_paths(folder)
    images = ss.load_images(paths)  

    start_image = 0 # TODO 10,11 not working
    last_image = len(images) - 1

    descriptors1 = np.load(os.path.join(folder, f"{os.path.basename(paths[start_image])}-descriptors.npy"), allow_pickle=True)
    descriptors_loop_start = descriptors1.copy()

    combined_descriptors = list(descriptors1)
    combined_H = np.eye(3)

    combined_matrix = np.eye(3)

    prev_x_min = images[start_image].shape[1]
    prev_x_max = 0
    prev_y_min = images[start_image].shape[0]
    prev_y_max = 0

    px_distances = [0]
    px_distances_x = [0]
    image_angles = [int(paths[start_image].split("_")[-1].split(".")[0])]

    H_list = []

    filtered_images = []
    filtered_images.append(images[start_image])
    filtered_paths = []
    filtered_paths.append(paths[start_image])

    for i in range(start_image+1,  len(images)):
        print(f"processing image {paths[i]}")

        descriptors2 = np.load(os.path.join(folder, f"{os.path.basename(paths[i])}-descriptors.npy"), allow_pickle=True)

        print(f"matching descriptor with len {len(descriptors2)}")
        if visualisation:
            points1, points2 = ss.match_descriptors(descriptors1, descriptors2, images[i-1], images[i])
        else:
            points1, points2 = ss.match_descriptors(descriptors1, descriptors2)

        if len(points1) < 3:
            print(f"WARNING: not enough points to calculate homography")
            continue # skip this picture
        else:
            filtered_images.append(images[i])
            filtered_paths.append(paths[i])

        H = ss.get_transform_matrix(points1, points2)

        H_list.append(H)
        descriptors1 = descriptors2


    images = filtered_images
    paths = filtered_paths

    loop_closing_correction = get_loop_closing_delta_R(combined_H, H_list)
    combined_H = np.eye(3)
    for i, H in enumerate(H_list):
        i = i + 1
        H = loop_closing_correction @ H        
        combined_H = combined_H.dot(H)

    #transform dot in center of image by combined_H to get the angle to the center of the first image
    anchor_point = np.array([images[0].shape[1] // 2, images[0].shape[0] // 2, 1])  # [x, y, 1]
    transformed_point = combined_H @ anchor_point
    transformed_point /= transformed_point[2]  # Normalize to get [x, y]
    t_x, t_y = int(transformed_point[0]), int(transformed_point[1])
    angle_of_panorama = np.arctan2(t_y - images[0].shape[0]//2, t_x - images[0].shape[1]//2)
    print(f"INFO: angle of panorama: {np.degrees(angle_of_panorama)}")
    combined_H = np.eye(3)
    combined_H = combined_H.dot(rotation_matrix(-angle_of_panorama, center=(images[0].shape[1]//2, images[0].shape[0]//2)))

    combined_image_total = cv2.cvtColor(images[start_image], cv2.COLOR_GRAY2BGR)
    combined_image_total = cv2.warpAffine(combined_image_total, combined_H[:2, :], (combined_image_total.shape[1], combined_image_total.shape[0]))
    for i, H in enumerate(H_list):
        i = i + 1
        H = loop_closing_correction @ H
        name = os.path.join(folder, f"{os.path.basename(paths[i-1])}-{os.path.basename(paths[i])}.npy")
        np.save(name, H)

        combined_H = combined_H.dot(H)

        warped_descriptors = []
        x_min, x_max, y_min, y_max = images[i].shape[1], 0, images[i].shape[0], 0
        descriptors2 = np.load(os.path.join(folder, f"{os.path.basename(paths[i])}-descriptors.npy"), allow_pickle=True)  # TODO remove +1

        for descriptor in descriptors2:
            warped_descriptor = ss.descriptor_transformation(descriptor, combined_H)
            transformed_point = warped_descriptor[-1][0]
            if (transformed_point[0] - 10 < prev_x_min or transformed_point[0] + 10 > prev_x_max or 
                transformed_point[1] - 10 < prev_y_min or transformed_point[1] + 10 > prev_y_max):
                warped_descriptors.append(warped_descriptor)
                if transformed_point[0] < x_min:
                    x_min = transformed_point[0]
                if transformed_point[0] > x_max:
                    x_max = transformed_point[0]
                if transformed_point[1] < y_min:
                    y_min = transformed_point[1]
                if transformed_point[1] > y_max:
                    y_max = transformed_point[1]
        print(f"warped descriptors {len(warped_descriptors)}")
        combined_descriptors = combined_descriptors + warped_descriptors
        prev_x_max, prev_x_min, prev_y_max, prev_y_min = x_max, x_min, y_max, y_min
        descriptors1 = descriptors2

        # Calculate the distance and angle between the two images
        image_angles.append(int(paths[i].split("_")[-1].split(".")[0]))
        
        anchor_point = np.array([images[start_image].shape[1] // 2, images[start_image].shape[0] // 2, 1])  # [x, y, 1]
        # Transform the anchor point using the homography matrix
        transformed_point = combined_H @ anchor_point
        transformed_point /= transformed_point[2]  # Normalize to get [x, y]
        # Extract transformed (x, y) coordinates
        t_x, t_y = int(transformed_point[0])-images[start_image].shape[1] // 2, int(transformed_point[1]) - images[start_image].shape[0] // 2
        # px_distances.append(t_x) #**2 + t_y**2)
        px_distances.append(t_x**2 + t_y**2)
        px_distances_x.append(t_x)

        if visualisation:
            height1, width1 = max(combined_image_total.shape[0], int(y_max)), max(combined_image_total.shape[1], int(x_max))
            combined_image_total_resized = np.zeros((height1, width1, 3), dtype=np.uint8)
            combined_image_total_resized[:combined_image_total.shape[0], :combined_image_total.shape[1]] = combined_image_total
            # warped_image2 = cv2.warpPerspective(images[i-1], combined_H, (combined_image_total.shape[1], combined_image_total.shape[0]))
            warped_image2 = cv2.cvtColor(images[i], cv2.COLOR_GRAY2BGR)
            warped_image2 = cv2.rectangle(warped_image2, (0, 0), 
              (warped_image2.shape[1] - 1, warped_image2.shape[0] - 1), 
              (0,0,255), 10)
            warped_image2 = cv2.warpAffine(warped_image2, combined_H[:2, :], (width1, height1))
            combined_image_total = cv2.addWeighted(combined_image_total_resized,1, warped_image2, 0.5, 0.5)
            cv2.imwrite("aligendImageTotal.jpg", warped_image2)

    print(f"saving combined descriptors {len(combined_descriptors)}")
    np.save(os.path.join(folder, f"combined-descriptors.npy"),  np.array(combined_descriptors, dtype=object), allow_pickle=True)

    descriptorsFirst = np.load(os.path.join(folder, f"{os.path.basename(paths[0])}-descriptors.npy"), allow_pickle=True)
    descriptorsLast = np.load(os.path.join(folder, f"{os.path.basename(paths[-1])}-descriptors.npy"), allow_pickle=True)
    #get offset:
    points1, points2 = ss.match_descriptors(descriptorsFirst, descriptorsLast, images[0], images[-1])
    if len(points1) < 2:
        loop_closing_x = 0
        loop_closing_y = 0
        print(f"ERROR: 0 deg doesnt match 360 deg")
    else:
        H = ss.get_transform_matrix(points1, points2)
        loop_closing_x = H[0, 2]  # Translation in x
        loop_closing_y = H[1, 2]  # Translation in y

    anchor_point = np.array([images[start_image].shape[1] // 2, images[start_image].shape[0] // 2, 1])  # [x, y, 1]
    # Transform the anchor point using the homography matrix
    transformed_point = combined_H @ anchor_point
    transformed_point /= transformed_point[2]  # Normalize to get [x, y]
    # Extract transformed (x, y) coordinates
    t_x, t_y = int(transformed_point[0])-images[start_image].shape[1] // 2, int(transformed_point[1]) - images[start_image].shape[0] // 2
    t_x_closed = int(t_x + np.sqrt(loop_closing_x**2 + loop_closing_y**2))
    print(f"loop closing correction: {np.sqrt(loop_closing_x**2 + loop_closing_y**2)}")
    
    np.save(os.path.join(folder, f"star_map_x.npy"), t_x_closed)
    print(f"star_map_x: {t_x_closed}")

    print(f"px_distance_current: {px_distances}")
    print(f"px_distance_current_x: {px_distances_x}")
    print(f"image_angles: {image_angles}")
    coefficients = np.polyfit(px_distances_x, image_angles, 3)  # Fit a cubic curve
    np.save(os.path.join(folder, "panorama_curvature_params.npy"), coefficients)

    if visualisation:
        cv2.line(combined_image_total, tuple([anchor_point[0], anchor_point[1]]), tuple([anchor_point[0] + t_x_closed , int(anchor_point[1])]), (0, 0, 255), 20)

        # Draw horizontal scale (0 to t_x)
        for i in range(0, t_x_closed + 1, 1000):  # Step of 50 pixels
            x_pos = anchor_point[0] + i
            cv2.line(combined_image_total, (x_pos, anchor_point[1] - 50), (x_pos, anchor_point[1] + 50), (0, 0, 255), 5)
            cv2.putText(combined_image_total, str(i)+"px", (x_pos - 100, anchor_point[1] - 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 5)
            cv2.putText(combined_image_total, str(int((i/t_x_closed)*360))+"deg", (x_pos - 100, anchor_point[1] + 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 5)

        cv2.line(combined_image_total, (anchor_point[0] + t_x, anchor_point[1] - 50), (anchor_point[0] + t_x, anchor_point[1] + 50), (0, 0, 255), 5)
        cv2.putText(combined_image_total, str(t_x)+"px", (anchor_point[0] + t_x - 100, anchor_point[1] - 200), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 5)
        cv2.putText(combined_image_total, str(int((t_x/t_x_closed)*360))+"deg", (anchor_point[0] + t_x - 100, anchor_point[1] + 200), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 5)

        cv2.imwrite(os.path.join(folder,"StarMap.jpg"), combined_image_total)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

