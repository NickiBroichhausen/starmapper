import cv2
import numpy as np
import os
import StarSift as ss


def createStarMap(folder, visualisation=False):
    # folder = "run-all"
    paths = ss.get_image_paths(folder)
    paths = list(reversed(paths))   # TODO remove this line if the camera is not upside down 
    images = ss.load_images(paths)  # TODO make optional as only used for visualisation

    start_image = 0 # TODO 10,11 not working

    # _, descriptors1 = ss.get_descriptors(images[start_image], True)
    # np.save(os.path.join(folder, f"{os.path.basename(paths[start_image])}-descriptors.npy"), descriptors1)
    descriptors1 = np.load(os.path.join(folder, f"{os.path.basename(paths[start_image])}-descriptors.npy"), allow_pickle=True)

    combined_descriptors = descriptors1
    combined_H = np.eye(3)

    combined_matrix = np.eye(3)

    # print(images[start_image].shape[1], images[start_image].shape[0])
    # combined_image_total = np.zeros((images[start_image].shape[0], images[start_image].shape[1]), dtype=np.uint8)
    combined_image_total = cv2.cvtColor(images[start_image], cv2.COLOR_GRAY2BGR)

    prev_x_min = images[start_image].shape[1]
    prev_x_max = 0
    prev_y_min = images[start_image].shape[0]
    prev_y_max = 0

    px_distances = [0]
    image_angles = [int(paths[start_image].split("_")[-1].split(".")[0])]

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
        H = ss.get_transform_matrix(points1, points2)

        name = os.path.join(folder, f"{os.path.basename(paths[i-1])}-{os.path.basename(paths[i])}.npy")
        np.save(name, H)

        combined_H = combined_H.dot(H)
        warped_descriptors = []
        x_min, x_max, y_min, y_max = images[i].shape[1], 0, images[i].shape[0], 0
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
        combined_descriptors = np.concatenate((combined_descriptors, warped_descriptors))
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
        t_x, t_y = int(transformed_point[0])-images[start_image].shape[1] // 2, int(transformed_point[1]) - images[start_image].shape[0] // 2
        # px_distances.append(t_x) #**2 + t_y**2)
        px_distances.append(t_x**2 + t_y**2)


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
            cv2.imwrite(os.path.join(folder,"StarMap.jpg"), combined_image_total)
            cv2.waitKey(0)
            cv2.destroyAllWindows()


    print(f"saving combined descriptors {len(combined_descriptors)}")
    np.save(os.path.join(folder, f"combined-descriptors.npy"),  np.array(combined_descriptors, dtype=object), allow_pickle=True)

    # px_distance_current = (t_x-image.shape[1] // 2)**2 + (t_y-image.shape[0] // 2)**2   # TODO maybe use top left corner instead of center? Maybe change this for the total distance?

    # anchor_point = np.array([images[start_image].shape[1] // 2, images[start_image].shape[0] // 2, 1])  # [x, y, 1]
    # # Transform the anchor point using the homography matrix
    # transformed_point = combined_H @ anchor_point
    # transformed_point /= transformed_point[2]  # Normalize to get [x, y]
    # # Extract transformed (x, y) coordinates
    # t_x, t_y = int(transformed_point[0])-images[start_image].shape[1] // 2, int(transformed_point[1]) - images[start_image].shape[0] // 2
    np.save(os.path.join(folder, f"star_map_size_squared.npy"), t_x**2 + t_y**2)

    # TODO check if this improves with real data
    print(f"px_distance_current: {px_distances}")
    print(f"image_angles: {image_angles}")
    coefficients = np.polyfit(px_distances, image_angles, 3)  # Fit a cubic curve (degree 3)
    np.save(os.path.join(folder, "panorama_curvature_params.npy"), coefficients)



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

