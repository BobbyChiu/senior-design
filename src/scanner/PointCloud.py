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

def to_file(pc, folder, filename=None):
    if filename == None:
        filename = datetime.now().strftime("%Y%m%d_%H%M%S") + ".xyz"
    np.savetxt(f"{folder}/{filename}", pc, fmt="%f")

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
    distances, indices = tree.query(cloud1, k=k)  # Find the nearest point in cloud2 for each point in cloud1
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

    # get max z
    max_idx = np.argpartition(z, -num_points_from_edge)
    max_z = np.median(z[max_idx[-num_points_from_edge]])

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
        for t in unique_theta:
            indices = np.where(theta == t)
            curr_max_z = np.max(z[indices])
            if np.abs(curr_max_z - max_z) > dist_from_edge:
                continue

            max_idx = np.argmax(z[indices])
            target_r = r[indices][max_idx]

            new_r = np.random.ranf((density)) * target_r
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


def best_fit_transform(A, B):
    '''
    Calculates the least-squares best-fit transform between corresponding 3D points A->B
    Input:
      A: Nx3 numpy array of corresponding 3D points
      B: Nx3 numpy array of corresponding 3D points
    Returns:
      T: 4x4 homogeneous transformation matrix
      R: 3x3 rotation matrix
      t: 3x1 translation vector
    '''
    
    assert A.shape == B.shape

    # Find the centroid of each set
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    
    # Centre the points
    AA = A - centroid_A
    BB = B - centroid_B

    # Dot is matrix multiplication for array
    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)

    # Calculate rotation matrix
    R = np.dot(Vt.T, U.T)

    # Special reflection case
    if np.linalg.det(R) < 0:
       Vt[2,:] *= -1
       R = np.dot(Vt.T, U.T)

    # Calculate translation vector
    t = centroid_B.T - np.dot(R, centroid_A.T)

    # Construct the transformation matrix
    T = np.identity(4)
    T[:3, :3] = R
    T[:3, 3] = t

    return T, R, t

def nearest_neighbor(src, dst):
    '''
    Find the nearest (Euclidean) neighbor in dst for each point in src
    Input:
        src: Nx3 array of points
        dst: Nx3 array of points
    Output:
        distances: Euclidean distances of the nearest neighbor
        indices: dst indices of the nearest neighbor
    '''

    # Create a KD-Tree
    kdtree = KDTree(dst)

    # Query the nearest neighbour
    distances, indices = kdtree.query(src, k=1)

    return distances, indices

def icp(A, B, init_pose=None, max_iterations=20, tolerance=0.001):
    '''
    The Iterative Closest Point method: finds best-fit transform that maps points A on to points B
    Input:
        A: Nx3 numpy array of source 3D points
        B: Nx3 numpy array of destination 3D point
        init_pose: 4x4 homogeneous transformation matrix
        max_iterations: exit algorithm after max_iterations
        tolerance: convergence criteria
    Output:
        T: final homogeneous transformation matrix that maps A on to B
        distances: Euclidean distances (errors) of the nearest neighbor
        i: number of iterations to converge
    '''

    # Make points homogeneous, copy them to maintain the originals
    src = np.ones((4,A.shape[0]))
    dst = np.ones((4,B.shape[0]))
    src[:3,:] = np.copy(A.T)
    dst[:3,:] = np.copy(B.T)

    # Apply the initial pose estimation
    if init_pose is not None:
        src = np.dot(init_pose, src)

    prev_error = 0

    for i in range(max_iterations):
        # Find the nearest neighbors between the current source and destination points
        distances, indices = nearest_neighbor(src[:3,:].T, dst[:3,:].T)

        # Compute the transformation between the current source and nearest destination points
        T,_,_ = best_fit_transform(src[:3,:].T, dst[:3,indices].T)

        # Update the current source
        src = np.dot(T, src)

        # Check error
        mean_error = np.mean(distances)
        if np.abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error
    # Calculate final transformation
    T,_,_ = best_fit_transform(A, src[:3,:].T)

    return T, distances, i+1

def apply_icp_transform(A, T):
     # Transform source points
    homogenous_coordinates = np.hstack((A, np.ones((A.shape[0], 1))))
    transformed_A = (T @ homogenous_coordinates.T).T[:, :3]
    return transformed_A

def combine_point_clouds(A, B):
    """
    Combine two point clouds using ICP.
    
    Parameters:
    A -- numpy array of shape (N, 3), the source point cloud.
    B -- numpy array of shape (M, 3), the target point cloud.
    T -- numpy array of shape (4, 4), the transformation matrix to apply to A.

    Returns:
    combined -- numpy array of shape (N+M, 3), the combined point cloud.
    """

    T = icp(A, B)[0]
    transformed_A = apply_icp_transform(A, T)

    # Concatenate with target points
    combined = np.vstack((transformed_A, B))
    return combined

def chamfer_distance(point_cloud_a, point_cloud_b):
    # Create KD-Trees
    tree_a = cKDTree(point_cloud_a)
    tree_b = cKDTree(point_cloud_b)

    # Find the closest points and distances from A to B
    distances_a_to_b, _ = tree_a.query(point_cloud_b)
    distances_b_to_a, _ = tree_b.query(point_cloud_a)

    # Compute the Chamfer Distance
    chamfer_a_to_b = np.mean(np.square(distances_a_to_b))
    chamfer_b_to_a = np.mean(np.square(distances_b_to_a))

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

def mesh_to_stl(mesh, folder, filename=None):
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