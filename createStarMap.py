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
    # points1_loop, points2_loop = ss.match_descriptors(descriptors1, descriptors_loop_closing, start_image, last_image)
    # H_loop = ss.get_transform_matrix(points1_loop, points2_loop)
    # H_loop = H[-1]

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
    # folder = "run-all"
    paths = ss.get_image_paths(folder)
    # paths = paths[5:7]
    # paths = list(reversed(paths))   # TODO remove this line if the camera is not upside down 
    images = ss.load_images(paths)  # TODO make optional as only used for visualisation

    start_image = 0 # TODO 10,11 not working
    last_image = len(images) - 1

    # _, descriptors1 = ss.get_descriptors(images[start_image], True)
    # np.save(os.path.join(folder, f"{os.path.basename(paths[start_image])}-descriptors.npy"), descriptors1)
    descriptors1 = np.load(os.path.join(folder, f"{os.path.basename(paths[start_image])}-descriptors.npy"), allow_pickle=True)
    descriptors_loop_start = descriptors1.copy()

    combined_descriptors = list(descriptors1)
    combined_H = np.eye(3)

    combined_matrix = np.eye(3)

    # print(images[start_image].shape[1], images[start_image].shape[0])
    # combined_image_total = np.zeros((images[start_image].shape[0], images[start_image].shape[1]), dtype=np.uint8)

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
        # _, descriptors2 = ss.get_descriptors(images[i], True)

        # np.save(os.path.join(folder, f"{os.path.basename(paths[i])}-descriptors.npy"), descriptors1)
        descriptors2 = np.load(os.path.join(folder, f"{os.path.basename(paths[i])}-descriptors.npy"), allow_pickle=True)  # TODO remove +1


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

        # name = os.path.join(folder, f"{os.path.basename(paths[i-1])}-{os.path.basename(paths[i])}.npy")
        # np.save(name, H)

        combined_H = combined_H.dot(H)
        # warped_descriptors = []
        # x_min, x_max, y_min, y_max = images[i].shape[1], 0, images[i].shape[0], 0
        # for descriptor in descriptors2:
        #     warped_descriptor = ss.descriptor_transformation(descriptor, combined_H)
        #     transformed_point = warped_descriptor[-1][0]
        #     # print(transformed_point)
        #     # TODO formations at edges might be lost
        #     if (transformed_point[0] - 10 < prev_x_min or transformed_point[0] + 10 > prev_x_max or 
        #         transformed_point[1] - 10 < prev_y_min or transformed_point[1] + 10 > prev_y_max):
        #         warped_descriptors.append(warped_descriptor)
        #         if transformed_point[0] < x_min:
        #             x_min = transformed_point[0]
        #         if transformed_point[0] > x_max:
        #             x_max = transformed_point[0]
        #         if transformed_point[1] < y_min:
        #             y_min = transformed_point[1]
        #         if transformed_point[1] > y_max:
        #             y_max = transformed_point[1]
        # print(f"warped descriptors {len(warped_descriptors)}")
        # combined_descriptors = combined_descriptors + warped_descriptors
        # prev_x_max, prev_x_min, prev_y_max, prev_y_min = x_max, x_min, y_max, y_min
        # # print(x_min, x_max, y_min, y_max)
        descriptors1 = descriptors2

        # # Calculate the distance and angle between the two images
        # image_angles.append(int(paths[i].split("_")[-1].split(".")[0]))
        
        # anchor_point = np.array([images[start_image].shape[1] // 2, images[start_image].shape[0] // 2, 1])  # [x, y, 1]
        # # Transform the anchor point using the homography matrix
        # transformed_point = combined_H @ anchor_point
        # transformed_point /= transformed_point[2]  # Normalize to get [x, y]
        # # Extract transformed (x, y) coordinates
        # t_x, t_y = int(transformed_point[0])-images[start_image].shape[1] // 2, int(transformed_point[1]) - images[start_image].shape[0] // 2
        # # px_distances.append(t_x) #**2 + t_y**2)
        # px_distances.append(t_x**2 + t_y**2)


        # # ## display single result
        # # height1, width1 = images[i-1].shape[:2]
        # # warped_image2 = cv2.warpAffine(images[i], H[:2,:], (width1, height1))
        # # # warped_image2 = cv2.warpPerspective(images[i-1], H, (width1, height1))
        # # combined_image = cv2.addWeighted(images[i-1], 0.5, warped_image2, 0.5, 0)
        # # cv2.imwrite("aligendImage.jpg", warped_image2)
        # # cv2.imwrite("Combined.jpg", combined_image)
        # # cv2.waitKey(0)
        # # cv2.destroyAllWindows()

        # if visualisation:
        #     height1, width1 = max(combined_image_total.shape[0], int(y_max)), max(combined_image_total.shape[1], int(x_max))
        #     # print(height1, width1)
        #     combined_image_total_resized = np.zeros((height1, width1, 3), dtype=np.uint8)
        #     combined_image_total_resized[:combined_image_total.shape[0], :combined_image_total.shape[1]] = combined_image_total
        #     # warped_image2 = cv2.warpPerspective(images[i-1], combined_H, (combined_image_total.shape[1], combined_image_total.shape[0]))
        #     warped_image2 = cv2.cvtColor(images[i], cv2.COLOR_GRAY2BGR)
        #     warped_image2 = cv2.rectangle(warped_image2, (0, 0), 
        #       (warped_image2.shape[1] - 1, warped_image2.shape[0] - 1), 
        #       (0,0,255), 10)
        #     warped_image2 = cv2.warpAffine(warped_image2, combined_H[:2, :], (width1, height1))
        #     combined_image_total = cv2.addWeighted(combined_image_total_resized,1, warped_image2, 0.5, 0.5)
        #     cv2.imwrite("aligendImageTotal.jpg", warped_image2)
        #     cv2.imwrite(os.path.join(folder,"StarMap.jpg"), combined_image_total)
        #     cv2.waitKey(0)
        #     cv2.destroyAllWindows()

    images = filtered_images
    paths = filtered_paths

    # TODO this has to come form H combined in the end
    # for that keep array of H's and apply correction in the end
    loop_closing_correction = get_loop_closing_delta_R(combined_H, H_list)


    combined_H = np.eye(3)
    for i, H in enumerate(H_list):
        i = i + 1
        H = loop_closing_correction @ H
        
        # name = os.path.join(folder, f"{os.path.basename(paths[i-1])}-{os.path.basename(paths[i])}.npy")
        # np.save(name, H)

        
        combined_H = combined_H.dot(H)

    #transform dot in center of image by combined_H to get the angle to the center of the first image
    anchor_point = np.array([images[0].shape[1] // 2, images[0].shape[0] // 2, 1])  # [x, y, 1]
    # print(f"anchor_point: {anchor_point}")
    transformed_point = combined_H @ anchor_point
    transformed_point /= transformed_point[2]  # Normalize to get [x, y]
    t_x, t_y = int(transformed_point[0]), int(transformed_point[1])
    # print(f"t_x: {t_x}, t_y: {t_y}")
    angle_of_panorama = np.arctan2(t_y - images[0].shape[0]//2, t_x - images[0].shape[1]//2)
    # print(f"correction: {t_y - images[0].shape[0]//2}, {t_x - images[0].shape[1]//2}")
    print(f"INFO: angle of panorama: {np.degrees(angle_of_panorama)}")
    combined_H = np.eye(3)
    #rotate combined_H by that angle to correct the panorama
    # TODO this doesnt rotate around the center of the image
    combined_H = combined_H.dot(rotation_matrix(-angle_of_panorama, center=(images[0].shape[1]//2, images[0].shape[0]//2)))
    # print(f"combined_H: {combined_H}")

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
            # print(transformed_point)
            # TODO formations at edges might be lost
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
        # print(x_min, x_max, y_min, y_max)
        descriptors1 = descriptors2

        # Calculate the distance and angle between the two images
        image_angles.append(int(paths[i].split("_")[-1].split(".")[0]))
        
        anchor_point = np.array([images[start_image].shape[1] // 2, images[start_image].shape[0] // 2, 1])  # [x, y, 1]
        # Transform the anchor point using the homography matrix
        transformed_point = combined_H @ anchor_point
        transformed_point /= transformed_point[2]  # Normalize to get [x, y]
        # Extract transformed (x, y) coordinates
        # TODO scale this with loop closing, if image_360 is not really at 360. FOr that use H between last and first image
        t_x, t_y = int(transformed_point[0])-images[start_image].shape[1] // 2, int(transformed_point[1]) - images[start_image].shape[0] // 2
        # px_distances.append(t_x) #**2 + t_y**2)
        px_distances.append(t_x**2 + t_y**2)
        px_distances_x.append(t_x)


        # ## display single result
        # height1, width1 = images[i-1].shape[:2]
        # warped_image2 = cv2.warpAffine(images[i], H[:2,:], (width1, height1))
        # # warped_image2 = cv2.warpPerspective(images[i-1], H, (width1, height1))
        # combined_image = cv2.addWeighted(images[i-1], 0.5, warped_image2, 0.5, 0)
        # cv2.imwrite("aligendImage.jpg", warped_image2)
        # cv2.imwrite("Combined.jpg", combined_image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        if visualisation:
            height1, width1 = max(combined_image_total.shape[0], int(y_max)), max(combined_image_total.shape[1], int(x_max))
            # print(height1, width1)
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
            # cv2.imwrite(os.path.join(folder,"StarMap.jpg"), combined_image_total)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()



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

    

    # px_distance_current = (t_x-image.shape[1] // 2)**2 + (t_y-image.shape[0] // 2)**2   # TODO maybe use top left corner instead of center? Maybe change this for the total distance?

    anchor_point = np.array([images[start_image].shape[1] // 2, images[start_image].shape[0] // 2, 1])  # [x, y, 1]
    # Transform the anchor point using the homography matrix
    transformed_point = combined_H @ anchor_point
    transformed_point /= transformed_point[2]  # Normalize to get [x, y]
    # Extract transformed (x, y) coordinates
    t_x, t_y = int(transformed_point[0])-images[start_image].shape[1] // 2, int(transformed_point[1]) - images[start_image].shape[0] // 2
    t_x_closed = int(t_x + np.sqrt(loop_closing_x**2 + loop_closing_y**2))
    print(f"loop closing correction: {np.sqrt(loop_closing_x**2 + loop_closing_y**2)}")
    

    # TODO first picture is not always zero. use picture name to add offset
    # np.save(os.path.join(folder, f"star_map_size_squared.npy"), t_x**2 + t_y**2)
    np.save(os.path.join(folder, f"star_map_x.npy"), t_x_closed)
    # print(f"star_map_size_squared: {t_x**2 + t_y**2}")
    print(f"star_map_x: {t_x_closed}")

    # TODO check if this improves with real data
    print(f"px_distance_current: {px_distances}")
    print(f"px_distance_current_x: {px_distances_x}")
    print(f"image_angles: {image_angles}")
    coefficients = np.polyfit(px_distances_x, image_angles, 3)  # Fit a cubic curve (degree 3)
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


        # # Draw circular scale (0 to 360 degrees)
        # for angle in range(0, 361, 30):  # Mark every 30°
        #     rad = np.deg2rad(angle)
        #     radius = 50  # Scale size
        #     end_x = int(anchor_point[0] + radius * np.cos(rad))
        #     end_y = int(anchor_point[1] + radius * np.sin(rad))
        #     cv2.line(combined_image_total, anchor_point, (end_x, end_y), (0, 255, 0), 1)
        #     cv2.putText(combined_image_total, str(angle), (end_x, end_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        cv2.imwrite(os.path.join(folder,"StarMap.jpg"), combined_image_total)
        cv2.waitKey(0)
        cv2.destroyAllWindows()



# TODO match last and first image to calculate the rotation in pixels


# warped_images = [images[0]]  # Start with the first image
# base_image = images[0]

# # Loop over all pairs of images for alignment
# for i in range(len(images)-20):
#     # Extract descriptors
#     dots1, descriptors1 = get_descriptors(images[i], True)
#     dots2, descriptors2 = get_descriptors(images[i+1], True)
    
#     points1, points2 = match_descriptors(dots1, descriptors1, dots2, descriptors2, images[i], images[i+1])

#     H, _ = cv2.findHomography(np.array(points1), np.array(points2))


#     # Get the size of the base image
#     height1, width1 = base_image.shape[:2]

#     # Warp the current image to the base image's coordinate system
#     # Calculate the bounding box of the transformed image
#     corners = np.array([[0, 0], [images[i].shape[1]-1, 0], [images[i].shape[1]-1, images[i].shape[0]-1], [0, images[i].shape[0]-1]], dtype=np.float32)
#     transformed_corners = cv2.perspectiveTransform(corners[None, :, :], H)[0]

#     # Find the bounding box of the transformed corners
#     min_x = min(transformed_corners[:, 0])
#     max_x = max(transformed_corners[:, 0])
#     min_y = min(transformed_corners[:, 1])
#     max_y = max(transformed_corners[:, 1])

#     print(min_x, max_x, min_y, max_y)
#     # Calculate the new width and height of the warped image
#     new_width = int(max_x - min_x)
#     new_height = int(max_y - min_y)
#     print(new_width, new_height)
#     # Warp the current image (apply homography) to the new canvas size
#     warp_offset_x = int(min_x)
#     warp_offset_y = int(-min_y)
#     warped_image = cv2.warpPerspective(images[i], H, (new_width, new_height))
#     print(warp_offset_x, warp_offset_y)
#     # Create a blank canvas large enough to hold the base image and the warped image
#     final_width = max(width1, new_width + warp_offset_x)
#     final_height = max(height1, new_height + warp_offset_y)

#     # Create a final image canvas
#     combined_image = np.zeros((final_height, final_width), dtype=np.uint8)

#     # Place the base image on the final canvas (without changing its position)
#     combined_image[warp_offset_y:warp_offset_y+height1, warp_offset_x:warp_offset_x+width1] = base_image

#     # Place the warped image on the canvas at the calculated position
#     combined_image[warp_offset_y:warp_offset_y+new_height, warp_offset_x:warp_offset_x+new_width] = warped_image

#     # Update the base image to be the new combined image
#     base_image = combined_image

# # Save the final combined image
# cv2.imwrite("Combined_all_images.jpg", base_image)
# cv2.imshow("Aligned Images", base_image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


# # Now combine all warped images into one large image
# # Get the size of the final image based on the aligned images
# height = max(image.shape[0] for image in warped_images)
# width = sum(image.shape[1] for image in warped_images)

# # Create a blank canvas for the combined image
# combined_image = np.zeros((height, width, 3), dtype=np.uint8)

# # Place each warped image onto the combined image
# current_x = 0
# for warped_image in warped_images:
#     combined_image[:, current_x:current_x+warped_image.shape[1]] = warped_image
#     current_x += warped_image.shape[1]

# # Save the final combined image
# cv2.imwrite("Combined_all_images.jpg", combined_image)
# cv2.imshow("Aligned Images", combined_image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

