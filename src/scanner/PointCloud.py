import numpy as np
from datetime import datetime
from scipy.spatial import cKDTree, KDTree
import open3d as o3d
from sklearn.linear_model import RANSACRegressor

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

def gaussian_filter(points, k, sigma):
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

# adds points to make the bottom/top of the point cloud flat
def fillFace(pc, bottom=False, top=False):

    # get cylindrical coordinates
    x = pc[:,0]
    y = pc[:,1]
    z = pc[:,2]

    r, theta = cart2pol(x, y)
    unique_theta = np.unique(theta)

    # get min z 
    min_idx = np.argpartition(z, 10)
    min_z = np.median(z[min_idx[:10]])

    # get max z
    max_idx = np.argpartition(z, -10)
    max_z = np.median(z[max_idx[-10]])

    # fill in points on plane of min_z
    all_new_r = np.empty(0)
    all_new_theta = np.empty(0)
    all_new_z = np.empty(0)
   
    if bottom:
        # bottom face
        for t in unique_theta:
            indices = np.where(theta == t)
            curr_min_z = np.max(z[indices])
            if np.abs(curr_min_z - min_z) > 0.4:
                continue
            min_idx = np.argmin(z[indices])
            target_r = r[indices][min_idx]

            new_r = np.random.ranf((10)) * target_r
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
            if np.abs(curr_max_z - max_z) > 0.4:
                continue

            max_idx = np.argmax(z[indices])
            target_r = r[indices][max_idx]

            new_r = np.random.ranf((10)) * target_r
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

    # Transform source points
    homogenous_coordinates = np.hstack((A, np.ones((A.shape[0], 1))))
    transformed_A = (T @ homogenous_coordinates.T).T[:, :3]

    # Concatenate with target points
    combined = np.vstack((transformed_A, B))

    return combined


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
