import numpy as np
from datetime import datetime
from scipy.spatial import cKDTree
from scipy.spatial.distance import cdist
import scipy.signal
import open3d as o3d
from joblib import Parallel, delayed
import matplotlib.path as mplPath
from scipy.spatial import ConvexHull
from sklearn.cluster import DBSCAN

# converts 2d polar (in degrees) to 2d cartesian
def pol2cart(radius, angle):
    x = radius * np.cos(np.radians(angle))
    y = radius * np.sin(np.radians(angle))
    return(x, y)

def cart2pol(x, y):
    r = np.sqrt(np.square(x) + np.square(y))
    theta = np.degrees(np.arctan2(y, x))
    return r, theta

def cylindrical2cart(r, theta, z):
    x, y = pol2cart(r, theta)
    return (x, y, z)

def from_file(path):
    pc = np.loadtxt(path)
    pc = pc.reshape(-1, 3)
    return np.loadtxt(path)

def to_file(pc, *, folder=None, filename=None):
    if filename == None:
        filename = datetime.now().strftime("%Y%m%d_%H%M%S") + ".xyz"

    if folder:
        np.savetxt(f"{folder}/{filename}", pc, fmt="%f")
    else:
        np.savetxt(f"{filename}", pc, fmt="%f")

def rotation_matrix(yaw, pitch, roll):
    # Convert angles from degrees to radians
    yaw = np.radians(yaw)
    pitch = np.radians(pitch)
    roll = np.radians(roll)

    # Yaw rotation matrix around the z-axis
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # Pitch rotation matrix around the y-axis
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    # Roll rotation matrix around the x-axis
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    # Combined rotation matrix
    R = np.dot(Rz, np.dot(Ry, Rx))
    return R

def rotate_points(yaw, pitch, roll, points):
    R = rotation_matrix(yaw, pitch, roll)
    # Apply rotation to each point
    transformed_points = np.dot(points, R.T)
    return transformed_points


def create_translation_matrix(shift):
    """
    Create a translation matrix for 3D space.

    :param shift: A tuple or list of three values representing the shift in x, y, and z directions.
    :return: 4x4 numpy array representing the translation matrix.
    """
    tx, ty, tz = shift
    return np.array([
        [1, 0, 0, tx],
        [0, 1, 0, ty],
        [0, 0, 1, tz],
        [0, 0, 0,  1]
    ])

def create_rotation_matrix(angles):
    """
    Create a combined rotation matrix for rotations around x, y, and z axes.

    :param angles: A tuple or list of three values representing the rotation angles (in degrees) around x, y, and z axes.
    :return: 4x4 numpy array representing the combined rotation matrix.
    """
    rx, ry, rz = np.radians(angles)  # Convert angles to radians
    cx, cy, cz = np.cos([rx, ry, rz])
    sx, sy, sz = np.sin([rx, ry, rz])

    # Rotation matrices for each axis
    Rx = np.array([
        [1, 0,  0, 0],
        [0, cx, -sx, 0],
        [0, sx, cx, 0],
        [0, 0,  0, 1]
    ])

    Ry = np.array([
        [cy, 0, sy, 0],
        [0,  1, 0,  0],
        [-sy, 0, cy, 0],
        [0,  0, 0,  1]
    ])

    Rz = np.array([
        [cz, -sz, 0, 0],
        [sz, cz,  0, 0],
        [0,  0,   1, 0],
        [0,  0,   0, 1]
    ])

    # Combine rotation matrices
    R = np.dot(Rz, np.dot(Ry, Rx))
    return R

def remove_statistical_outliers(points, nb_neighbors=20, std_ratio=2.0):
    """
    Remove statistical outliers from a point cloud using open3d.geometry.statistical_outlier_removal.

    Parameters:
    - points: numpy array of shape (N, 3) representing N 3D points
    - nb_neighbors: number of neighbors to use for radius search
    - std_ratio: standard deviation ratio

    Returns:
    - inliers: numpy array of shape (M, 3) containing the filtered point cloud 
      and a numpy array of shape (N,) containing the indices of the outliers
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    _, ind = pcd.remove_statistical_outlier(nb_neighbors, std_ratio)
    return np.asarray(pcd.select_by_index(ind).points), ind

def remove_radius_outliers(points, nb_points=24, radius=0.05):
    """
    Remove radius outliers from a point cloud using open3d.geometry.radius_outlier_removal.

    Parameters:
    - points: numpy array of shape (N, 3) representing N 3D points
    - nb_points: minimum number of points within the radius
    - radius: radius of the sphere

    Returns:
    - inliers: numpy array of shape (M, 3) containing the filtered point cloud 
        and a numpy array of shape (N,) containing the indices of the outliers
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    _, ind = pcd.remove_radius_outlier(nb_points, radius)
    return np.asarray(pcd.select_by_index(ind).points), ind

def apply_transformation(array, points):
    """
    Apply the transformation defined by the array to the given points, optimized using NumPy.

    :param array: A list or tuple of six values, where the first three are shifts and the last three are rotation angles.
    :param points: A list of points (each a tuple or list of three values) to transform.
    :return: Transformed points.
    """
    shift = array[:3]
    angles = array[3:]
    T = create_translation_matrix(shift)
    R = create_rotation_matrix(angles)
    transformation_matrix = np.dot(T, R)

    # Convert points to a NumPy array and add a row of 1s for homogeneous coordinates
    points_array = np.array(points)
    points_homogeneous = np.hstack((points_array, np.ones((points_array.shape[0], 1))))

    # Apply transformation to all points at once
    transformed_points = np.dot(points_homogeneous, transformation_matrix.T)

    # Drop the homogeneous coordinate and return
    return transformed_points[:, :3]

def subtract_point_clouds(cloud1, cloud2, threshold):
    """
    Returns a mask of points in cloud1 that are farther from a certain distance from any point in cloud2.
    
    Parameters:
    cloud1 (numpy.ndarray): The point cloud from which points are to be removed.
    cloud2 (numpy.ndarray): The point cloud used to check distances.
    threshold (float): The distance threshold for removing points.
    
    Returns:
    numpy.ndarray: The mask of resulting point cloud after subtraction.
    """

    tree = cKDTree(cloud2)
    distances, indices = tree.query(cloud1, k=1)  # Find the nearest point in cloud2 for each point in cloud1
    mask = distances > threshold  # Create a mask of points that are farther than the threshold
    return mask

def knn_filter(cloud1, k, threshold):
    """
    Returns a mask of points in cloud1 that are within a certain distance from their kth nearest point.
    
    Parameters:
    cloud1 (numpy.ndarray): The point cloud from which points are to be removed.
    cloud2 (numpy.ndarray): The point cloud used to check distances.
    threshold (float): The distance threshold for removing points.
    
    Returns:
    numpy.ndarray: The mask of resulting point cloud after subtraction.
    """
    tree = cKDTree(cloud1)
    distances, indices = tree.query(cloud1, k=[k])  # Find the nearest point in cloud2 for each point in cloud1
    mask = distances < threshold  # Create a mask of points that are closer then the threshold
    return mask

def gaussian_filter_knn(points, k, sigma):
    """
    Apply a Gaussian filter to a 3D point cloud based on k nearest neighbors.

    Parameters:
    - points: numpy array of shape (N, 3) representing N 3D points
    - k: number of nearest neighbors to consider
    - sigma: the standard deviation of the Gaussian function

    Returns:
    - filtered_points: numpy array of shape (N, 3) containing the filtered point cloud
    """

    def gaussian_weight(distance, sigma):
        return np.exp(-(distance**2) / (2 * sigma**2))


    tree = cKDTree(points)
    filtered_points = np.empty_like(points)

    for i, point in enumerate(points):
        # Query k nearest neighbors
        distances, indices = tree.query(point, k=k)
        neighbors = points[indices]

        # Compute weights for each neighbor
        weights = gaussian_weight(distances, sigma)
        weights /= np.sum(weights)  # Normalize weights

        # Calculate the weighted average
        weighted_avg = np.sum(neighbors * weights[:, np.newaxis], axis=0)
        filtered_points[i] = weighted_avg

    return filtered_points

def estimate_normals(points, radius):
    """
    Estimate normals for each point in the point cloud using PCA on neighbors within a specified radius.
    This version uses a radius-based approach instead of k-nearest neighbors for normal estimation.
    Handles cases where there are not enough neighbors to compute a covariance matrix.
    """
    tree = cKDTree(points)
    normals = np.zeros_like(points)

    for i, point in enumerate(points):
        idx = tree.query_ball_point(point, radius)
        neighbors = points[idx]

        # Check if there are enough neighbors to compute the covariance matrix
        if len(neighbors) < 3:
            # Not enough neighbors, assign a default normal or skip
            normals[i] = np.array([0, 0, 1])  # Example default normal
        else:
            cov_matrix = np.cov(neighbors - point, rowvar=False)
            eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
            normal = eigenvectors[:, np.argmin(eigenvalues)]
            normals[i] = normal if normal[2] > 0 else -normal

    return normals

def bilateral_filter_point_cloud(points, normals, radius, sigma_s, sigma_n):
    tree = cKDTree(points)
    n_points = len(points)

    def process_point(i):
        point = points[i]
        idx = tree.query_ball_point(point, radius)
        neighbors = points[idx]
        neighbor_normals = normals[idx]

        spatial_weights = np.exp(-cdist([point], neighbors, 'sqeuclidean')[0] / (2 * sigma_s**2))
        
        normal_differences = 1 - np.abs(np.einsum('ij,j->i', neighbor_normals, normals[i]))
        normal_weights = np.exp(-(normal_differences**2) / (2 * sigma_n**2))
        
        weights = spatial_weights * normal_weights
        weights /= weights.sum()

        return np.dot(weights, neighbors)

    filtered_points = Parallel(n_jobs=-1)(delayed(process_point)(i) for i in range(n_points))
    return np.array(filtered_points)

def gaussian_filter_radius(points, radius, sigma):
    """
    Apply a Gaussian filter to a point cloud.

    Parameters:
    - points (np.array): Nx3 array of the point cloud data.
    - sigma (float): Standard deviation for Gaussian kernel.
    - radius (float): Radius to consider for the neighborhood points.

    Returns:
    - np.array: Filtered point cloud of the same shape as input.
    """
    # Create a KDTree for fast neighborhood lookup
    kdtree = cKDTree(points)

    # Find points within the specified radius
    indices = kdtree.query_ball_tree(kdtree, r=radius)

    # Apply Gaussian filter
    filtered_points = np.zeros_like(points)
    for i, point_indices in enumerate(indices):
        if not point_indices:
            continue
        neighbors = points[point_indices]
        distances = np.linalg.norm(neighbors - points[i], axis=1)
        weights = np.exp(-(distances**2) / (2 * sigma**2))
        weights /= weights.sum()
        filtered_points[i] = np.sum(neighbors * weights[:, np.newaxis], axis=0)

    return filtered_points

def add_bottom_surface(pc, z_percentile=1, percent_height=1, grid_spacing=0.1, eps=100, min_samples=10, crop=False):
    """
    Add points to a point cloud to create a flat bottom surface at a specified z_value
    for multiple, non-intersecting polygons, including points within a certain distance above it.

    Parameters:
    pc (np.ndarray): A NumPy array of shape (N, 3) representing the point cloud.
    z_value (float): The Z percentile at which to add the flat bottom.
    percent_height (float): The percent of total height above z_value to include points in the boundary calulcations.
    grid_spacing (float): Spacing between grid points in the flat bottom.
    eps (float): The maximum distance between two samples for them to be considered as in the same neighborhood (DBSCAN parameter).
    min_samples (int): The number of samples in a neighborhood for a point to be considered as a core point (DBSCAN parameter).
    crop (bool) : whether to remove all points below the z_percentile

    Returns:
    np.ndarray: The modified point cloud with a flat bottom added.
    """

    z_value = np.percentile(pc[:, 2], z_percentile)
    dist = (pc[:, 2].max() - pc[:, 2].min()) * percent_height/100
    # Filter points that are within dist above the z_value
    bottom_points = pc[(pc[:, 2] >= z_value) & (pc[:, 2] <= z_value + dist)]

    if len(bottom_points) == 0:
        return pc  # No points to form a bottom surface

    # Cluster the bottom points into separate polygons
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(bottom_points[:, :2])
    labels = clustering.labels_

    # Create a grid of points at the z_value level
    x_min, x_max = np.min(pc[:, 0]), np.max(pc[:, 0])
    y_min, y_max = np.min(pc[:, 1]), np.max(pc[:, 1])
    x_grid, y_grid = np.meshgrid(np.arange(x_min, x_max, grid_spacing), np.arange(y_min, y_max, grid_spacing))
    grid_points = np.column_stack([x_grid.ravel(), y_grid.ravel()])

    # Initialize an array to hold all new bottom surface points
    all_bottom_surface_points = np.empty((0, 3))

    # Process each polygon separately
    for label in set(labels):
        if label == -1:  # Skip noise points
            continue

        # Points in the current polygon
        polygon_points = bottom_points[labels == label]

        # Create a polygon path
        if len(polygon_points) < 3:  # Not enough points to form a polygon
            continue

        try:
            hull = ConvexHull(polygon_points[:, :2])
            polygon_path = mplPath.Path(polygon_points[hull.vertices, :2])

            # Filter grid points that fall inside the polygon
            inside = polygon_path.contains_points(grid_points)
            bottom_surface_points = grid_points[inside]
            bottom_surface_points = np.hstack([bottom_surface_points, np.full((len(bottom_surface_points), 1), z_value)])

            # Add these points to the all_bottom_surface_points array
            all_bottom_surface_points = np.vstack((all_bottom_surface_points, bottom_surface_points))
        except scipy.spatial.qhull.QhullError:
            # Handle error if ConvexHull cannot be formed
            continue

    # Add the new bottom points to the original point cloud

    if crop:
        pc = pc[pc[:, 2] >= z_value + grid_spacing]

    new_pc = np.vstack((pc, all_bottom_surface_points))

    return new_pc

def chamfer_distance(point_cloud_a, point_cloud_b):
    """
    Calculate the Chamfer Distance between two point clouds.

    Parameters:
    point_cloud_a (numpy.ndarray): First point cloud as an Nx3 numpy array.
    point_cloud_b (numpy.ndarray): Second point cloud as an Nx3 numpy array.

    Returns:
    float: The Chamfer Distance between the two point clouds.
    """
    # Build a KDTree for point_cloud_b
    tree = cKDTree(point_cloud_b)

    # Find the nearest neighbor in point_cloud_b for each point in point_cloud_a
    distances_a_to_b, _ = tree.query(point_cloud_a)

    # Build a KDTree for point_cloud_a
    tree = cKDTree(point_cloud_a)

    # Find the nearest neighbor in point_cloud_a for each point in point_cloud_b
    distances_b_to_a, _ = tree.query(point_cloud_b)

    # Combine the distances from a to b and b to a
    chamfer_distance = np.sqrt(np.mean(distances_a_to_b**2)) + np.sqrt(np.mean(distances_b_to_a**2)) + np.std(distances_a_to_b) + np.std(distances_b_to_a)
    return chamfer_distance

def get_surface_points(input_cloud, voxel_size=0.1):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(input_cloud)

    # Estimate normals
    pcd.estimate_normals()

    # Optionally, you might want to orient the normals (if necessary)
    pcd.orient_normals_consistent_tangent_plane(k=100)

    # Downsample the point cloud
    downpcd = pcd.voxel_down_sample(voxel_size=voxel_size)

    # Surface reconstruction using Poisson reconstruction
    mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(downpcd, depth=9)

    # Extract surface points from mesh
    surface_points = o3d.geometry.PointCloud()
    surface_points.points = mesh.vertices

    # Convert the processed point cloud back to a NumPy array
    return np.asarray(surface_points.points)


def to_mesh(points_3d, voxel_size=0.1):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_3d)    

    pcd.normals = o3d.utility.Vector3dVector(np.zeros(
    (1, 3)))  # invalidate existing normals
    
    # Downsample the point cloud
    downpcd = pcd.voxel_down_sample(voxel_size=voxel_size)

    downpcd.estimate_normals()
    downpcd.orient_normals_consistent_tangent_plane(20)

    with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            downpcd, depth=9)

    mesh.compute_vertex_normals()
    mesh.paint_uniform_color([1, 0.706, 0])
    return mesh

def mesh_to_stl(mesh, *, folder=None, filename=None):
    if filename == None:
        filename = datetime.now().strftime("%Y%m%d_%H%M%S") + ".stl"
    o3d.io.write_triangle_mesh(f"{folder}/{filename}", mesh)

def stl_to_mesh(filename):
    mesh = o3d.io.read_triangle_mesh(filename)
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color([1, 0.706, 0])
    return mesh

def mesh_to_pc(mesh, num_samples=100000):
    pc = mesh.sample_points_uniformly(number_of_points=num_samples)
    pc = np.asarray(pc.points)
    return pc
