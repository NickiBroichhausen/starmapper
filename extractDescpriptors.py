
import StarSift as ss
import numpy as np
import os

def analyse(image_path, target_folder, visualisation=False):
    image = ss.load_images([image_path])[0]
    _, descriptors = ss.get_descriptors(image, visualisation)
    np.save(os.path.join(target_folder, f"{os.path.basename(image_path)}-descriptors.npy"), descriptors, allow_pickle=True)
    print(f"descriptors: {len(descriptors)}")
    # return descriptors
