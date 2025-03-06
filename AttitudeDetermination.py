

import StarSift as ss
import numpy as np
import cv2
import os


def find_attitude(folder, image_path, visualisation=False):   # TODO allow star map to be None, as it is only for visualization
    print("loading star map descriptors")
    image = ss.load_images([image_path])[0]
    star_map = None
    if visualisation:
        star_map = ss.load_images([os.path.join(folder, "StarMap.jpg")], False)[0]  
        
    descriptors1 = np.load(os.path.join(folder, "combined-descriptors.npy"), allow_pickle=True)
    print(f"descriptors1: {descriptors1.shape}")

    print("getting new descriptors")
    _, descriptors2 = ss.get_descriptors(image, True)

    if len(descriptors2) == 0:
        print(f"INFO: Picture {image_path} not useful. Removed")
        # os.rename(image_path, os.path.join(folder, f"xx_{os.path.basename(image_path)}"))
        return 0
    
    print("matching descriptors")
    points1, points2 = ss.match_descriptors(descriptors1, descriptors2, image1=star_map, image2=image)
                                            
    H = ss.get_transform_matrix(points1, points2)
    print(H)

    anchor_point = np.array([image.shape[1] // 2, image.shape[0] // 2, 1])  # [x, y, 1]

    # Transform the anchor point using the homography matrix
    transformed_point = H @ anchor_point
    transformed_point /= transformed_point[2]  # Normalize to get [x, y]

    # Extract transformed (x, y) coordinates
    t_x, t_y = int(transformed_point[0]), int(transformed_point[1])

    print(f"Estimated position: ({t_x}, {t_y})")

    if visualisation:
        star_map = cv2.cvtColor(star_map, cv2.COLOR_GRAY2BGR)
        marked_image = cv2.drawMarker(star_map, (int(t_x), int(t_y)), (255,0,0), markerType=cv2.MARKER_TRIANGLE_DOWN, markerSize=200, thickness=20)
        cv2.imwrite("estimatedPositon.jpg", marked_image)


    total_size_squared = np.load(os.path.join(folder, "star_map_size_squared.npy"))

    px_distance_current = (t_x-image.shape[1] // 2)**2 + (t_y-image.shape[0] // 2)**2 

    size_x = np.load(os.path.join(folder, "star_map_x.npy"))
    print(f"x distance angel: {((t_x-image.shape[1] // 2)/size_x)*360}")
    angle = 360 * np.sqrt(px_distance_current / total_size_squared)

    panorama_curvature_params = np.load(os.path.join(folder, "panorama_curvature_params.npy"))
    cubic_poly = np.poly1d(panorama_curvature_params)            # Create a polynomial function
    y_fit = cubic_poly(t_x-image.shape[1] // 2)
    # y_fit = cubic_poly(px_distance_current)
    print(f"corrected angleL: {y_fit}")
    return angle%360
