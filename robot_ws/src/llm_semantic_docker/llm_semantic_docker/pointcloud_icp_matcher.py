#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Point Cloud ICP Matcher Engine for Precision Docking.
Performs ROI cropping, voxel downsampling, statistical outlier removal,
normal estimation, and Point-to-Plane (or SVD fallback) ICP registration.
Calculates Fitness, Inlier RMSE, and precise relative offsets (dx, dy, dyaw).
"""

import numpy as np
from typing import Dict, Any, Tuple, Optional
from scipy.spatial import cKDTree

# Try importing Open3D, with fallback to pure scipy/numpy SVD ICP
try:
    HAS_OPEN3D = True
    import open3d as o3d
except ImportError:
    HAS_OPEN3D = False


class PointCloudICPMatcher:
    """High-performance Point Cloud Preprocessing and ICP Registration Engine."""

    def __init__(self, voxel_size: float = 0.01):
        self.voxel_size = voxel_size

    def preprocess_cloud(self,
                         points: np.ndarray,
                         roi_bounds: Optional[Dict[str, Tuple[float, float]]] = None,
                         nb_neighbors: int = 20,
                         std_ratio: float = 1.5) -> Tuple[np.ndarray, np.ndarray]:
        """
        Filters point cloud:
        1. ROI Bounding Box Crop
        2. Voxel Grid Downsampling
        3. Statistical Outlier Removal
        4. Surface Normal Estimation
        Returns: (filtered_points, normals)
        """
        if points is None or points.shape[0] == 0:
            return np.empty((0, 3)), np.empty((0, 3))

        # 1. ROI Crop
        pts = points[:, :3]
        if roi_bounds:
            mask = np.ones(pts.shape[0], dtype=bool)
            if 'x' in roi_bounds:
                mask &= (pts[:, 0] >= roi_bounds['x'][0]) & (pts[:, 0] <= roi_bounds['x'][1])
            if 'y' in roi_bounds:
                mask &= (pts[:, 1] >= roi_bounds['y'][0]) & (pts[:, 1] <= roi_bounds['y'][1])
            if 'z' in roi_bounds:
                mask &= (pts[:, 2] >= roi_bounds['z'][0]) & (pts[:, 2] <= roi_bounds['z'][1])
            pts = pts[mask]

        if pts.shape[0] < 30:
            return pts, np.empty((0, 3))

        # 2. Voxel Downsample
        if HAS_OPEN3D:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts)
            pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)

            # 3. Statistical Outlier Removal
            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)

            # 4. Normal Estimation
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.03, max_nn=30)
            )
            pcd.orient_normals_towards_camera_location(camera_location=np.array([0., 0., 0.]))
            filtered_pts = np.asarray(pcd.points)
            normals = np.asarray(pcd.normals)
            return filtered_pts, normals
        else:
            # Fallback numpy/scipy voxelization & PCA normal estimation
            filtered_pts = self._voxel_grid_downsample_np(pts, self.voxel_size)
            if filtered_pts.shape[0] < 10:
                return filtered_pts, np.empty((0, 3))
            normals = self._estimate_normals_pca(filtered_pts, k=15)
            return filtered_pts, normals

    def match_icp(self,
                  source_points: np.ndarray,
                  target_points: np.ndarray,
                  source_normals: Optional[np.ndarray] = None,
                  target_normals: Optional[np.ndarray] = None,
                  init_transform: Optional[np.ndarray] = None,
                  max_correspondence_dist: float = 0.05,
                  max_iterations: int = 50) -> Dict[str, Any]:
        """
        Executes ICP Registration aligning source_points (live camera) to target_points (template).
        Returns:
            dict containing:
                'fitness': float (0.0 ~ 1.0)
                'inlier_rmse': float (meters)
                'transformation': 4x4 matrix
                'delta_x': float (meters)
                'delta_y': float (meters)
                'delta_z': float (meters)
                'delta_yaw': float (radians)
                'converged': bool
        """
        if init_transform is None:
            init_transform = np.eye(4, dtype=np.float64)

        if source_points.shape[0] < 20 or target_points.shape[0] < 20:
            return {
                'fitness': 0.0,
                'inlier_rmse': 999.0,
                'transformation': init_transform,
                'delta_x': 0.0,
                'delta_y': 0.0,
                'delta_z': 0.0,
                'delta_yaw': 0.0,
                'converged': False
            }

        if HAS_OPEN3D and source_normals is not None and target_normals is not None and len(target_normals) > 0:
            src_pcd = o3d.geometry.PointCloud()
            src_pcd.points = o3d.utility.Vector3dVector(source_points[:, :3])
            src_pcd.normals = o3d.utility.Vector3dVector(source_normals[:, :3])

            tgt_pcd = o3d.geometry.PointCloud()
            tgt_pcd.points = o3d.utility.Vector3dVector(target_points[:, :3])
            tgt_pcd.normals = o3d.utility.Vector3dVector(target_normals[:, :3])

            reg_result = o3d.pipelines.registration.registration_icp(
                src_pcd, tgt_pcd, max_correspondence_dist, init_transform,
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iterations)
            )
            T = np.asarray(reg_result.transformation)
            fitness = float(reg_result.fitness)
            rmse = float(reg_result.inlier_rmse)
        else:
            # SVD Point-to-Point ICP Fallback
            T, fitness, rmse = self._icp_svd(
                source_points[:, :3], target_points[:, :3],
                init_transform=init_transform,
                max_dist=max_correspondence_dist,
                max_iter=max_iterations
            )

        # Extract delta translation and delta yaw
        dx = float(T[0, 3])
        dy = float(T[1, 3])
        dz = float(T[2, 3])
        dyaw = float(np.arctan2(T[1, 0], T[0, 0]))

        return {
            'fitness': fitness,
            'inlier_rmse': rmse,
            'transformation': T,
            'delta_x': dx,
            'delta_y': dy,
            'delta_z': dz,
            'delta_yaw': dyaw,
            'converged': bool(fitness > 0.3)
        }

    def verify_match(self, icp_result: Dict[str, Any], thresholds: Dict[str, Any]) -> Tuple[bool, str]:
        """
        Multi-criteria verification gate checking:
        1. Fitness >= min_fitness
        2. RMSE <= max_rmse
        3. |dx| <= max_allowable_dx
        4. |dy| <= max_allowable_dy
        5. |dyaw| <= max_allowable_dyaw
        """
        min_fit = thresholds.get('min_fitness', 0.65)
        max_rmse = thresholds.get('max_rmse', 0.025)
        max_dx = thresholds.get('max_allowable_dx', 0.15)
        max_dy = thresholds.get('max_allowable_dy', 0.10)
        max_dyaw_rad = np.radians(thresholds.get('max_allowable_dyaw_deg', 15.0))

        if icp_result['fitness'] < min_fit:
            return False, f"Fitness too low: {icp_result['fitness']:.3f} < {min_fit}"

        if icp_result['inlier_rmse'] > max_rmse:
            return False, f"Inlier RMSE too high: {icp_result['inlier_rmse']:.3f}m > {max_rmse}m"

        if abs(icp_result['delta_x']) > max_dx:
            return False, f"Delta X too large: {abs(icp_result['delta_x']):.3f}m > {max_dx}m"

        if abs(icp_result['delta_y']) > max_dy:
            return False, f"Delta Y too large: {abs(icp_result['delta_y']):.3f}m > {max_dy}m"

        if abs(icp_result['delta_yaw']) > max_dyaw_rad:
            return False, f"Delta Yaw too large: {np.degrees(abs(icp_result['delta_yaw'])):.1f}deg > {np.degrees(max_dyaw_rad):.1f}deg"

        return True, "Verified (Fitness, RMSE, and DeltaPose within tolerances)"

    @staticmethod
    def _voxel_grid_downsample_np(points: np.ndarray, voxel_size: float) -> np.ndarray:
        """Pure numpy voxel grid downsampling."""
        if points.shape[0] == 0:
            return points
        voxel_indices = np.floor(points / voxel_size).astype(np.int64)
        _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
        return points[unique_indices]

    @staticmethod
    def _estimate_normals_pca(points: np.ndarray, k: int = 15) -> np.ndarray:
        """PCA surface normal estimation using cKDTree."""
        tree = cKDTree(points)
        normals = np.zeros_like(points)
        _, indices = tree.query(points, k=min(k, points.shape[0]))
        for i, idxs in enumerate(indices):
            neighbors = points[idxs]
            centered = neighbors - np.mean(neighbors, axis=0)
            cov = np.dot(centered.T, centered)
            eig_vals, eig_vecs = np.linalg.eigh(cov)
            normal = eig_vecs[:, 0] # Minor axis is normal
            if normal[2] < 0: # orient towards +Z (camera)
                normal = -normal
            normals[i] = normal
        return normals

    @staticmethod
    def _icp_svd(src: np.ndarray, tgt: np.ndarray,
                 init_transform: np.ndarray,
                 max_dist: float = 0.05,
                 max_iter: int = 40) -> Tuple[np.ndarray, float, float]:
        """SVD-based Rigid Registration ICP fallback."""
        tree = cKDTree(tgt)
        T = np.copy(init_transform)
        src_homo = np.hstack([src, np.ones((src.shape[0], 1))])

        fitness = 0.0
        rmse = 999.0

        for _ in range(max_iter):
            transformed_src = (T @ src_homo.T).T[:, :3]
            dists, indices = tree.query(transformed_src, k=1)
            valid_mask = dists < max_dist
            n_inliers = np.sum(valid_mask)
            if n_inliers < 10:
                break

            fitness = n_inliers / float(src.shape[0])
            rmse = float(np.sqrt(np.mean(dists[valid_mask] ** 2)))

            p_src = transformed_src[valid_mask]
            p_tgt = tgt[indices[valid_mask]]

            # SVD alignment
            centroid_src = np.mean(p_src, axis=0)
            centroid_tgt = np.mean(p_tgt, axis=0)
            H = (p_src - centroid_src).T @ (p_tgt - centroid_tgt)
            U, _, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            if np.linalg.det(R) < 0:
                Vt[2, :] *= -1
                R = Vt.T @ U.T
            t = centroid_tgt - R @ centroid_src

            delta_T = np.eye(4)
            delta_T[:3, :3] = R
            delta_T[:3, 3] = t
            T = delta_T @ T

        return T, float(fitness), float(rmse)
