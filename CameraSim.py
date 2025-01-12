from scipy.spatial.transform import Rotation as R
import pybullet as p
import numpy as np


class CameraSim(object):
    """
    Simulate a camera in pybullet.
    We assume that the default camera view direction is at z direction of world frame,
    and the default up vector of the camera is aligned with inverse y direction of world frame.
    """

    def __init__(self, width, height, fov, fov_error,
                 eye_position, target_position, up_vector,
                 near_val, far_val):
        self.width = width
        self.height = height
        self.aspect = width / height
        self.standard_fov = fov
        self.curr_fov = self.standard_fov
        self.fov_error = fov_error
        self.intrinsic = self.get_intrinsic(fov, width, height)

        self.view_matrix = p.computeViewMatrix(cameraEyePosition=eye_position,
                                               cameraTargetPosition=target_position,
                                               cameraUpVector=up_vector)
        # camera w.r.t. world
        self.pose = self.compute_pose(np.asarray(eye_position),
                                      np.asarray(target_position),
                                      np.asarray(up_vector))

        self.near_val = near_val
        self.far_val = far_val
        self.projection_matrix = p.computeProjectionMatrixFOV(fov=fov,
                                                              aspect=self.aspect,
                                                              nearVal=near_val,
                                                              farVal=far_val)

    def get_camera_image(self, shadow=1, get_pcl=False, add_noise=False, renderer=p.ER_TINY_RENDERER):
        width, height, rgb, depth, mask = p.getCameraImage(width=self.width,
                                                           height=self.height,
                                                           viewMatrix=self.view_matrix,
                                                           projectionMatrix=self.projection_matrix,
                                                           shadow=shadow,
                                                           renderer=renderer)
        depth = np.reshape(depth, [self.width, self.height])
        depth = self.recover_depth_from_z_buffer(depth)
        
        rgb = np.reshape(rgb, (self.height, self.width, 4)).astype(np.uint8)
        mask = np.reshape(mask, [self.width, self.height]).astype(np.uint16)
        if add_noise:
            depth = self.add_noise(depth)
        if get_pcl:
            point_cloud = self.compute_point_cloud(depth, self.intrinsic)
            return rgb, depth, mask, point_cloud
        else:
            return rgb, depth, mask

    def recover_depth_from_z_buffer(self, depth):
        ret = self.far_val * self.near_val / (self.far_val - (self.far_val - self.near_val) * depth)
        return ret

    def add_noise(self,
                  depth,
                  lateral=True,
                  axial=True,
                  missing_value=True,
                  default_angle=85.0):
        """
        Add noise according to kinect noise model.
        Please refer to the paper "Modeling Kinect Sensor Noise for Improved 3D Reconstruction and Tracking".
        """
        h, w = depth.shape
        point_cloud = self.compute_point_cloud(depth, self.intrinsic)
        surface_normal = self.compute_surface_normal_central_difference(point_cloud)
        # surface_normal = self.compute_surface_normal_least_square(point_cloud)
        cos = np.squeeze(np.dot(surface_normal, np.array([[0.0, 0.0, 1.0]], dtype=surface_normal.dtype).T))
        angles = np.arccos(cos)
        # adjust angles that don't satisfy the domain of noise model ([0, pi/2) for kinect noise model).
        cos[angles >= np.pi / 2] = np.cos(np.deg2rad(default_angle))
        angles[angles >= np.pi / 2] = np.deg2rad(default_angle)
        # add lateral noise
        if lateral:
            sigma_lateral = 0.8 + 0.035 * angles / (np.pi / 2 - angles)
            x, y = np.meshgrid(np.arange(w), np.arange(h))
            # add noise offset to x axis
            new_x = x + np.round(np.random.normal(scale=sigma_lateral)).astype(np.int)
            # remove points that are out of range
            invalid_ids = np.logical_or(new_x < 0, new_x >= w)
            new_x[invalid_ids] = x[invalid_ids]
            # add noise offset to y axis
            new_y = y + np.round(np.random.normal(scale=sigma_lateral)).astype(np.int)
            # remove points that are out of range
            invalid_ids = np.logical_or(new_y < 0, new_y >= h)
            new_y[invalid_ids] = y[invalid_ids]
            depth = depth[new_y, new_x]
        # add axial noise
        if axial:
            # axial noise
            sigma_axial = 0.0012 + 0.0019 * (depth - 0.4) ** 2
            depth = np.random.normal(depth, sigma_axial)
        # remove some value according to the angle
        # the larger the angle, the higher probability the depth value is set to zero
        if missing_value:
            missing_mask = np.random.uniform(size=cos.shape) > cos
            depth[missing_mask] = 0.0
        return depth

    def set_pose(self, eye_position, target_position, up_vector):
        eye_position = np.asarray(eye_position)
        target_position = np.asarray(target_position)
        up_vector = np.asarray(up_vector)
        self.pose = self.compute_pose(eye_position, target_position, up_vector)
        self.view_matrix = p.computeViewMatrix(cameraEyePosition=eye_position,
                                               cameraTargetPosition=target_position,
                                               cameraUpVector=up_vector)

    def recover_intrinsic(self):
        self.intrinsic = self.get_intrinsic(self.standard_fov, self.width, self.height)

    @staticmethod
    def compute_point_cloud(depth, intrinsic):
        """
        Compute point cloud by depth image and camera intrinsic matrix.
        :param depth: A float numpy array representing the depth image.
        :param intrinsic: A 3x3 numpy array representing the camera intrinsic matrix
        :return: Point cloud in camera space.
        """
        h, w = depth.shape
        w_map, h_map = np.meshgrid(np.arange(w), np.arange(h))
        image_coordinates = np.stack([w_map, h_map, np.ones_like(h_map, dtype=np.float32)], axis=2).astype(np.float32)
        inv_intrinsic = np.linalg.inv(intrinsic)
        camera_coordinates = np.expand_dims(depth, axis=2) * np.dot(image_coordinates, inv_intrinsic.T)
        return camera_coordinates

    @staticmethod
    def compute_surface_normal_central_difference(point_cloud):
        """
        Compute surface normal from point cloud.
        Notice: it only applies to point cloud map represented in camera space.
        The x axis directs in width direction, and y axis is in height direction.
        :param point_cloud: An HxWx3-d numpy array representing the point cloud map.The point cloud map
                            is restricted to the map in camera space without any other transformations.
        :return: An HxWx3-d numpy array representing the corresponding normal map.
        """
        h, w, _ = point_cloud.shape
        gradient_y, gradient_x, _ = np.gradient(point_cloud)
        normal = np.cross(gradient_x, gradient_y, axis=2)
        normal[normal == np.nan] = 0
        norm = np.linalg.norm(normal, axis=2, keepdims=True)
        flag = norm[..., 0] != 0
        normal[flag] = normal[flag] / norm[flag]
        return normal

    @staticmethod
    def compute_surface_normal_least_square(point_cloud, kernel=7):
        """
        Compute surface normal from point cloud by least square fitting.
        The x axis directs in width direction, and y axis is in height direction.
        :param point_cloud: An HxWx3-d numpy array representing the point cloud map.The point cloud map
                            is restricted to the map in camera space without any other transformations.
        :param kernel: Kernel size.
        :return: An HxWx3-d numpy array representing the corresponding normal map.
        """
        y, x = np.meshgrid(np.arange(kernel), np.arange(kernel))
        y, x = y - y[kernel // 2, kernel // 2], x - x[kernel // 2, kernel // 2]
        y, x = y.flatten(), x.flatten()
        height, width, _ = point_cloud.shape
        shifted_pcl = np.zeros((height, width, 3, kernel ** 2), dtype=point_cloud.dtype)
        # todo: large time and space cost for allocating such a large memory block
        for i, (k, l) in enumerate(zip(x, y)):
            shifted_pcl[max(0, -k):min(height, height-k), max(0, -l):min(width, width-l), :, i] = \
                point_cloud[max(0, k):min(height, height+k), max(0, l):min(width, width+l), :]
        centered_pcl = shifted_pcl - np.mean(shifted_pcl, axis=-1, keepdims=True)
        m00 = np.sum(centered_pcl[:, :, 0, :] ** 2, axis=-1)  # H * W
        m01 = m10 = np.sum(centered_pcl[:, :, 0, :] * centered_pcl[:, :, 1, :], axis=-1)
        m11 = np.sum(centered_pcl[:, :, 1, :] ** 2, axis=-1)
        v0 = np.sum(centered_pcl[:, :, 0, :] * centered_pcl[:, :, 2, :], axis=-1)
        v1 = np.sum(centered_pcl[:, :, 1, :] * centered_pcl[:, :, 2, :], axis=-1)
        inv_det = 1 / (m00 * m11 - m01 * m10)
        inv_det[np.logical_or(inv_det == np.inf, inv_det == -np.inf)] = 0
        a = inv_det * (m11 * v0 - m01 * v1)
        b = inv_det * (-m10 * v0 + m00 * v1)
        n = np.stack([-a, -b, np.ones_like(a)], axis=2)
        n = n / np.linalg.norm(n, axis=2, keepdims=True)
        return n

    @staticmethod
    def bilateral_filter(depth, kernel=7, sigma_d=7/3, sigma_r=0.01/3):
        """
        Bilateral filtering for depth image
        :param depth: Original depth image with unit represented as meter.
        :param kernel: Kernel size.
        :param sigma_d: Spatial parameter.
        :param sigma_r: Range parameter.
        :return: The processed depth image.
        """
        y, x = np.meshgrid(np.arange(kernel), np.arange(kernel))
        y, x = y - y[kernel//2, kernel//2], x - x[kernel//2, kernel//2]
        w_d = np.exp(-(x ** 2 + y ** 2) / (2 * sigma_d ** 2)).reshape(1, 1, -1)  # 1 * 1 * kernel ** 2
        y, x = y.flatten(), x.flatten()
        height, width = depth.shape
        shifted_depths = np.zeros((height, width, kernel**2), dtype=depth.dtype)
        for i, (k, l) in enumerate(zip(x, y)):
            shifted_depths[max(0, -k):min(height, height-k), max(0, -l):min(width, width-l), i] = \
                depth[max(0, k):min(height, height+k), max(0, l):min(width, width+l)]
        w_r = np.exp(-(shifted_depths - shifted_depths[..., kernel**2//2:kernel**2//2+1]) ** 2 / (2 * sigma_r ** 2))
        w = w_d * w_r
        processed_depth = np.sum(w * shifted_depths, axis=2) / np.sum(w, axis=2)
        return processed_depth

    @staticmethod
    def get_intrinsic(fov, width, height):
        fov = fov / 180 * np.pi
        focal_length = (height / 2) / np.tan(fov / 2)
        intrinsic = np.array([[focal_length, 0.0, width / 2],
                              [0.0, focal_length, height / 2],
                              [0.0, 0.0, 1.0]], dtype=np.float32)
        return intrinsic

    @staticmethod
    def compute_pose(eye_position, target_position, up_vector):
        eye_position = np.asarray(eye_position)
        target_position = np.asarray(target_position)
        up_vector = np.asarray(up_vector)
        y = -up_vector / np.linalg.norm(up_vector)
        z = target_position - eye_position
        z = z / np.linalg.norm(z)
        x = np.cross(y, z)
        rotation_matrix = np.array([x, y, z]).T
        # camera to world
        pose = np.eye(4, dtype=np.float32)
        pose[0:3, 0:3] = rotation_matrix
        pose[0:3, 3] = eye_position
        return pose

    @staticmethod
    def encode_depth(depth):
        """
        Encode depth map to compress the stored memory.
        :param depth: An HxW-d numpy array
        :return: An HxWx4-d numpy array representing the encoded depth map.
        """
        precision = 1e9  # reserve 9 decimal places
        depth = np.round(depth * precision).astype(np.int)  # convert to integer
        # 2**32-1 = 4294967295
        depth[depth >= 2 ** 32] = 0  # the maximum depth value should be less than or equal to 4.294967295m
        r = depth % (2 ** 8)  # 0 to 2**8-1
        g = (depth - r) // (2 ** 8) % (2 ** 8)  # 2**8 to 2**16-1
        b = (depth - r - g * (2 ** 8)) // (2 ** 16) % (2 ** 8)  # 2**16 to 2**24-1
        a = (depth - r - g * (2 ** 8) - b * (2 ** 16)) // (2 ** 24) % (2 ** 8)  # 2**24 to 2**31-1
        encoded_depth = np.stack([r, g, b, a], axis=2).astype(np.uint8)
        return encoded_depth

    @staticmethod
    def decode_depth(encoded_depth):
        precision = 1e9
        encoded_depth = encoded_depth.astype(np.float32)
        depth = encoded_depth[..., 0] + \
                encoded_depth[..., 1] * 2 ** 8 + \
                encoded_depth[..., 2] * 2 ** 16 + \
                encoded_depth[..., 3] * 2 ** 24
        depth = depth / precision
        return depth