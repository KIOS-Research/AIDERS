import os
import threading

import database.queries
import numpy as np
import open3d
import utils


def processPointCloud(_sessionId):
    print('Processing', flush=True)
    if utils.threadStarted("processPointCloudDataSession" + _sessionId):
        return {"message": "Lidar Session id " + _sessionId + " is already processing."}
    thread = threading.Thread(target=processPointsByOpen3d, args=(_sessionId,))
    thread.name = "processPointCloudDataSession" + _sessionId
    thread.start()
    return {"message": "Lidar Session id " + _sessionId + " is starting processing."}


def processPointsByOpen3d(_sessionId):
    # Process Points
    lidarListOfPoints = database.queries.getAllPointsByLidarSessionId(_sessionId)

    pointsValues = [[d[1], d[0], -d[2]] for d in lidarListOfPoints]
    pointsColors = [[d[3] / 255, d[4] / 255, d[5] / 255] for d in lidarListOfPoints]

    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(pointsValues)
    pcd.colors = open3d.utility.Vector3dVector(pointsColors)

    pcd.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = 3 * avg_dist
    mesh = open3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd, open3d.utility.DoubleVector([radius, radius * 2])
    )
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    if not mesh.has_triangle_normals():
        mesh.compute_triangle_normals()
    outputDirectory = f"/media/lidarPointCloudMesh/"  # output directory
    if not os.path.exists(outputDirectory):
        os.mkdir(outputDirectory)
    fullPath = outputDirectory + f"lidarSession_{str(_sessionId)}.glb"
    if os.path.exists(fullPath):
        os.remove(fullPath)
    open3d.io.write_triangle_mesh(fullPath, mesh)
    savedPath = f"lidarPointCloudMesh/lidarSession_{str(_sessionId)}.glb"
    database.queries.updateProcessedSessionBySessionIdAndPath(_sessionId, savedPath)
