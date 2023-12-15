import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def estimate_transformation_matrix(object_points, image_points, intrinsics, distortion):

    _, rotation_vector, translation_vector = cv2.solvePnP(
        objectPoints=object_points,
        imagePoints=image_points,
        cameraMatrix=intrinsics,
        distCoeffs=distortion,
        flags=cv2.SOLVEPNP_P3P,
    )

    transformation_matrix = np.eye(4)
    transformation_matrix[0:3, 3] = np.squeeze(translation_vector)
    transformation_matrix[0:3, 0:3] = Rotation.from_rotvec(
        np.squeeze(rotation_vector)
    ).as_matrix()

    return np.asmatrix(transformation_matrix)


transformation_matrix = estimate_transformation_matrix(wp, ip, intrinsics, distortion)
