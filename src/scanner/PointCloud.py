import numpy as np
from datetime import datetime
from scipy.spatial import cKDTree, KDTree
from scipy.spatial.distance import cdist
import scipy.signal
import open3d as o3d
from sklearn.linear_model import RANSACRegressor
from sklearn.neighbors import NearestNeighbors

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

def apply_transformation(array, points):
    """
    Apply the transformation defined by the array to the given points.

    :param array: A list or tuple of six values, where the first three are shifts and the last three are rotation angles.
    :param points: A list of points (each a tuple or list of three values) to transform.
    :return: Transformed points.
    """
    shift = array[:3]
    angles = array[3:]
    T = create_translation_matrix(shift)
    R = create_rotation_matrix(angles)
    transformation_matrix = np.dot(T, R)

    transformed_points = []
    for point in points:
        # Convert point to homogeneous coordinates
        point_homogeneous = np.array([point[0], point[1], point[2], 1])
        # Apply transformation
        transformed_point = np.dot(transformation_matrix, point_homogeneous)
        # Convert back to 3D coordinates and add to the list
        transformed_points.append(transformed_point[:3])

    return transformed_points

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

def median_filter(points, k):
    """
    Apply a median filter to a 3D point cloud based on k nearest neighbors.

    Parameters:
    - points: numpy array of shape (N, 3) representing N 3D points
    - k: number of nearest neighbors to consider

    Returns:
    - filtered_points: numpy array of shape (N, 3) containing the filtered point cloud
    """

    tree = cKDTree(points)
    filtered_points = np.empty_like(points)

    for i, point in enumerate(points):
        # Query k nearest neighbors
        distances, indices = tree.query(point, k=k)
        neighbors = points[indices]

        # Compute median of neighbors for each dimension
        median = np.median(neighbors, axis=0)
        filtered_points[i] = median

    return filtered_points

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
    """
    Apply a bilateral filter to a point cloud using a k-d tree for spatial searching.
    Only points within a specified radius are considered for computing the filter response.
    
    points: A numpy array of shape (N, 3) representing the point cloud.
    normals: A numpy array of shape (N, 3) representing the estimated normals for the point cloud.
    sigma_s: The spatial standard deviation of the Gaussian.
    sigma_n: The standard deviation for the normals in the Gaussian range component.
    radius: The radius within which to evaluate nearest neighbors for filtering.
    
    Returns a filtered point cloud of the same shape as points.
    """
    tree = cKDTree(points)
    filtered_points = np.zeros_like(points)

    for i, point in enumerate(points):
        # Find points within the specified radius for spatial component
        idx = tree.query_ball_point(point, radius)
        neighbors = points[idx]
        neighbor_normals = normals[idx]
        
        # Compute spatial weights
        spatial_weights = np.exp(-cdist([point], neighbors, 'sqeuclidean')[0] / (2 * sigma_s**2))
        
        # Compute normal weights
        normal_differences = 1 - np.abs(np.einsum('ij,j->i', neighbor_normals, normals[i]))
        # normal_differences = -(np.log(1-normal_differences))
        normal_weights = np.exp(-(normal_differences**2) / (2 * sigma_n**2))
        
        # Combine weights
        weights = spatial_weights * normal_weights
        
        # Normalize weights
        weights /= weights.sum()
        
        # Calculate the filtered point
        filtered_points[i] = np.dot(weights, neighbors)
    
    return filtered_points

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

# adds points to make the bottom/top of the point cloud flat
def fillFace(pc, density=10, num_points_from_edge=10, dist_from_edge=0.4, bottom=False, top=False):

    # get cylindrical coordinates
    x = pc[:,0]
    y = pc[:,1]
    z = pc[:,2]

    r, theta = cart2pol(x, y)
    unique_theta = np.unique(theta)

    # get min z 
    min_idx = np.argpartition(z, num_points_from_edge)
    min_z = np.median(z[min_idx[:num_points_from_edge]])
    pc = pc[pc[:, 2] > min_z]

    # get max z
    max_idx = np.argpartition(z, -num_points_from_edge)
    max_z = np.median(z[max_idx[-num_points_from_edge]])
    pc = pc[pc[:, 2] < max_z]

    # fill in points on plane of min_z
    all_new_r = np.empty(0)
    all_new_theta = np.empty(0)
    all_new_z = np.empty(0)
   
    if bottom:
        # bottom face
        for t in unique_theta:
            indices = np.where(theta == t)
            curr_min_z = np.max(z[indices])
            if np.abs(curr_min_z - min_z) > dist_from_edge:
                continue
            min_idx = np.argmin(z[indices])
            target_r = r[indices][min_idx]

            new_r = np.random.ranf((density)) * target_r
            new_z = np.full(new_r.shape, min_z)
            new_theta = np.full(new_r.shape, t)

            all_new_r = np.concatenate((all_new_r, new_r))
            all_new_z = np.concatenate((all_new_z, new_z))
            all_new_theta = np.concatenate((all_new_theta, new_theta))
    if top:
        # top face
        max_z = max_z
        for t in unique_theta:

            indices = np.where(theta == t)
            curr_max_z = np.max(z[indices])
            if np.abs(curr_max_z - max_z) > dist_from_edge:
                continue

            max_idx = np.argmax(z[indices])
            target_r = r[indices][max_idx]

            new_r = (np.random.ranf(density)) * target_r
            new_z = np.full(new_r.shape, max_z)
            new_theta = np.full(new_r.shape, t)

            all_new_r = np.concatenate((all_new_r, new_r))
            all_new_z = np.concatenate((all_new_z, new_z))
            all_new_theta = np.concatenate((all_new_theta, new_theta))

    # convert new points back to cartesian
    x, y = pol2cart(all_new_r, all_new_theta)
    new_points = np.column_stack((x, y, all_new_z))
    pc = np.vstack((pc, new_points))
    return pc

def icp(source, target):
    source_pcd = o3d.geometry.PointCloud()
    target_pcd = o3d.geometry.PointCloud()
    source_pcd.points = o3d.utility.Vector3dVector(source)
    target_pcd.points = o3d.utility.Vector3dVector(target)

    icp_result = o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd, 1.0, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50))

    # Transform the source point cloud using the found transformation matrix
    source_pcd_transformed = source_pcd.transform(icp_result.transformation)

    return np.asarray(source_pcd_transformed.points)

def feature_based_registration(source, target):
    # Convert input point clouds to Open3D PointCloud objects
    source_pcd = o3d.geometry.PointCloud()
    target_pcd = o3d.geometry.PointCloud()
    source_pcd.points = o3d.utility.Vector3dVector(source)
    target_pcd.points = o3d.utility.Vector3dVector(target)

    # Compute normals for source and target point clouds
    source_pcd.estimate_normals()
    target_pcd.estimate_normals()

    # Fast Global Registration (ICP)
    icp_coarse = o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd, 0.1, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50))

    icp_fine = o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd, 0.05, icp_coarse.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50))

    # Transform the source point cloud using the found transformation matrix
    source_pcd_transformed = source_pcd.transform(icp_fine.transformation)

    return np.asarray(source_pcd_transformed.points)

from scipy.stats import wasserstein_distance

def compute_normals_with_open3d(point_cloud):
    """
    Compute normals for a point cloud using Open3D.

    Parameters:
    point_cloud (o3d.geometry.PointCloud): Open3D point cloud object.

    Returns:
    o3d.geometry.PointCloud: Point cloud with normals.
    """
    # Estimate normals
    point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    return point_cloud

def cosine_similarity(normals1, normals2):
    """
    Calculate cosine similarity between two sets of normals.

    Parameters:
    normals1, normals2 (numpy.ndarray): Arrays of normals.

    Returns:
    float: Average cosine similarity.
    """
    # Normalize the normals
    normals1_normalized = normals1 / np.linalg.norm(normals1, axis=1, keepdims=True)
    normals2_normalized = normals2 / np.linalg.norm(normals2, axis=1, keepdims=True)

    # Calculate cosine similarity
    similarity = np.sum(normals1_normalized * normals2_normalized, axis=1)
    return np.mean(similarity)

def feature_based_similarity(cloud1, cloud2):
    """
    Calculate feature-based similarity between two point clouds based on normals.

    Parameters:
    cloud1, cloud2 (numpy.ndarray): Nx3 numpy arrays representing point clouds.

    Returns:
    float: Cosine similarity of normals between the two point clouds.
    """
    # Convert numpy arrays to Open3D PointCloud
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(cloud1)
    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(cloud2)

    # Compute normals
    pcd1 = compute_normals_with_open3d(pcd1)
    pcd2 = compute_normals_with_open3d(pcd2)

    # Extract normals
    normals1 = np.asarray(pcd1.normals)
    normals2 = np.asarray(pcd2.normals)

    # Calculate similarity
    similarity = cosine_similarity(normals1, normals2)
    return similarity

def calculate_overlap_distance(subset, larger_set, threshold):
    """
    Optimized calculation of the average distance between overlapping points in a subset and a larger point cloud set.

    Parameters:
    subset (numpy.ndarray): Subset point cloud as an Nx3 numpy array.
    larger_set (numpy.ndarray): Larger point cloud set as an Mx3 numpy array.
    threshold (float): Distance threshold for considering points as overlapping.

    Returns:
    float: Average distance between overlapping points in the subset and the larger set.
    """
    # Create a k-d tree for the larger set
    tree = cKDTree(larger_set)

    # Find the nearest neighbor in the larger set for each point in the subset
    distances, _ = tree.query(subset, distance_upper_bound=threshold)

    # Filter out distances that are infinity (which means no neighbors within the threshold)
    valid_distances = distances[distances != np.inf]

    # Compute average distance if there are overlapping points
    average_distance = np.mean(valid_distances) if len(valid_distances) > 0 else 0
    return average_distance
def hausdorff_distance(cloud1, cloud2):
    """
    Calculate the Hausdorff Distance between two point clouds.

    Parameters:
    cloud1, cloud2 (numpy.ndarray): Nx3 numpy arrays representing point clouds.

    Returns:
    float: The Hausdorff Distance between the two point clouds.
    """
    def one_sided_hausdorff(cloudA, cloudB):
        max_distance = 0
        for point in cloudA:
            # Compute distances from this point to all points in cloudB
            distances = np.sqrt(np.sum((cloudB - point) ** 2, axis=1))
            # Find the minimum distance to cloudB for this point
            min_distance = np.min(distances)
            # Update max_distance if this point has a larger distance
            max_distance = max(max_distance, min_distance)
        return max_distance

    # Calculate one-sided Hausdorff distances and return the maximum
    hausdorff_A_to_B = one_sided_hausdorff(cloud1, cloud2)
    hausdorff_B_to_A = one_sided_hausdorff(cloud2, cloud1)
    return hausdorff_A_to_B

def earth_movers_distance(cloud1, cloud2):
    """
    Calculate the Earth Mover's Distance (EMD) between two point clouds.

    Parameters:
    cloud1 (numpy.ndarray): First point cloud as an Nx3 numpy array.
    cloud2 (numpy.ndarray): Second point cloud as an Nx3 numpy array.

    Returns:
    float: The Earth Mover's Distance between the two point clouds.
    """
    # Compute EMD for each dimension (x, y, z)
    emd_x = wasserstein_distance(cloud1[:, 0], cloud2[:, 0])
    emd_y = wasserstein_distance(cloud1[:, 1], cloud2[:, 1])
    emd_z = wasserstein_distance(cloud1[:, 2], cloud2[:, 2])

    # Combine the EMD from each dimension
    emd_total = (emd_x + emd_y + emd_z) / 3
    return emd_total


def chamfer_distance(point_cloud_a, point_cloud_b, only_a_to_b=False):
    # Create KD-Trees
    tree_a = cKDTree(point_cloud_a)
    tree_b = cKDTree(point_cloud_b)

    # Find the closest points and distances from A to B
    distances_a_to_b, _ = tree_a.query(point_cloud_b)
    distances_b_to_a, _ = tree_b.query(point_cloud_a)


    # Compute the Chamfer Distance
    chamfer_a_to_b = np.mean((distances_a_to_b)) ** 2
    chamfer_b_to_a = np.mean((distances_b_to_a)) ** 2

    if only_a_to_b:
        return chamfer_a_to_b
    else:
        return chamfer_a_to_b + chamfer_b_to_a


def to_mesh(points_3d):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_3d)    

    pcd.normals = o3d.utility.Vector3dVector(np.zeros(
    (1, 3)))  # invalidate existing normals

    pcd.estimate_normals()
    pcd.orient_normals_consistent_tangent_plane(100)


    with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=9)
    
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color([1, 0.706, 0])
    return mesh

def mesh_to_stl(mesh, *, folder=None, filename=None):
    if filename == None:
        filename = datetime.now().strftime("%Y%m%d_%H%M%S") + ".stl"
    o3d.io.write_triangle_mesh(f"{folder}/{filename}", mesh)

def stl_to_mesh(filename):
    mesh = o3d.io.read_triangle_mesh(filename)
    return mesh

def mesh_to_pc(mesh, num_samples=100000):
    pc = mesh.sample_points_uniformly(number_of_points=num_samples)
    pc = np.asarray(pc.points)
    return pc
