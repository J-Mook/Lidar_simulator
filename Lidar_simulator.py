from numpy.core.fromnumeric import shape
from pycaster.pycaster import rayCaster
import os
import open3d as o3d
import numpy as np
from tqdm import tqdm
import math, random
import time
PI = math.pi

###########################CPU accelation##################################
import multiprocessing
from functools import partial
from contextlib import contextmanager

@contextmanager
def poolcontext(*args, **kwargs):
    pool = multiprocessing.Pool(*args, **kwargs)
    yield pool
    pool.terminate()
###########################CPU accelation##################################


##initialize
Source_point = [500, 500, 400] #lidar location (width ,depth ,height)
Source_target = [0, 0, 100]
camera_moving_mount = 1
Angular_Resolution = [math.radians(0.2), math.radians(0.2)] #[ vertical , horizontal ]
FOV_mode = False
scanner_model_specifications = 'l'
noise_mode = False
ply_save = False

activate_CPU = 8

target_dir = np.array(Source_point) - np.array(Source_target)


models_data = {'xs' : [[106, 118, 133],[161,181,205],[70,78,88],[0.035]], 
                's' : [[343, 360, 382],[384,442,520],[237,272,319],[0.050]], 
                'm' : [[317, 590, 826],[458, 650, 1118],[292,404,686],[0.100]], 
                'l' : [[600, 1082, 1644],[870, 1239, 2150],[557, 772, 1326],[0.200]]} #[[width],[lenth],[height],[noise]]


##search stl file data
input_data_folder_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data")
save_ply_folder_path = os.path.join(input_data_folder_path, "ply")
stl_file_name = "TestSpecimenAssy.stl"
# stl_file_name = "MeshCAA.stl"
# stl_file_name = "airplane_0627.stl"

stl_file = os.path.join(input_data_folder_path, stl_file_name)
caster = rayCaster.fromSTL(stl_file, scale = 1.0)

def find_intersection_point(pTarget, Source_loc, Source_target_loc):
    pointsIntersection = caster.castRay(Source_loc, pTarget)
    if pointsIntersection != []:
        if check_fov(Source_loc, Source_target_loc, pointsIntersection[0]):
            # return pointsIntersection[0]
            if noise_mode:
                noise_x = pointsIntersection[0][0] + random.gauss(0, models_data[scanner_model_specifications][3][0]/3)
                noise_y = pointsIntersection[0][1] + random.gauss(0, models_data[scanner_model_specifications][3][0]/3)
                noise_z = pointsIntersection[0][2] + random.gauss(0, models_data[scanner_model_specifications][3][0]/3)
                
                return tuple((noise_x, noise_y, noise_z))
            else:                    
                return pointsIntersection[0]
    
def make_pcd_spr2pnt(pSource, tSource, angle_sqr, sphere_redius):
    # point to surface
    
    pcd = []
    target_list = []
    result_list = []
    min_angle_xy, max_anlge_xy, min_angle_z, max_anlge_z = setting_ROI_angle(pSource, tSource)
    seperate_xy = int((max_anlge_xy - min_angle_xy) / angle_sqr[1])
    seperate_z = int((max_anlge_z - min_angle_z) / angle_sqr[0])

    for i in range(0, seperate_z):
        for j in range(0, seperate_xy):
            pTarget = creat_sphere(sphere_redius, (i * angle_sqr[0]) + min_angle_z, (j * angle_sqr[1]) + min_angle_xy, pSource)
            if check_fov(pSource, tSource, pTarget):
                target_list.append(pTarget)

    if activate_CPU == 0:
        for target_point in tqdm(target_list, leave = False, position = 2):
            result = find_intersection_point(target_point, pSource, tSource)
            result_list.append(result)
    else:
        #########################################CPU accelation##################################
        with poolcontext(processes=activate_CPU) as pool:
            # result_list = pool.map(partial(find_intersection_point, Source_loc = pSource, Source_target_loc = tSource), target_list)
            result_list = list(tqdm(pool.imap_unordered(partial(find_intersection_point, Source_loc = pSource, Source_target_loc = tSource), target_list), total=len(target_list)))
        #########################################CPU accelation##################################
    
    pcd = none_check(result_list)
    return pcd, len(target_list)

def none_check(input_list):
    output_list = list(filter(None.__ne__, input_list))
    return output_list

def setting_ROI_angle(start_point, end_point):
    point_dir = np.array(end_point) - np.array(start_point)
    source_angle_xy = math.atan2(point_dir[1], point_dir[0])
    source_angle_z = cal_angle([point_dir[0],point_dir[1], 0], [point_dir[0], point_dir[1], point_dir[2]])
    
    if FOV_mode:
        xy_angle = math.atan(abs(((models_data[scanner_model_specifications][0][2] - models_data[scanner_model_specifications][0][0]) / 2 ) / (models_data[scanner_model_specifications][1][2] - models_data[scanner_model_specifications][1][0])))
        xy_min = source_angle_xy - xy_angle
        xy_max = source_angle_xy + xy_angle
        z_anlge = math.atan(abs(((models_data[scanner_model_specifications][2][2] - models_data[scanner_model_specifications][2][0]) / 2 ) / (models_data[scanner_model_specifications][1][2] - models_data[scanner_model_specifications][1][0])))
        z_max = source_angle_z + z_anlge + PI/2
        z_min = source_angle_z - z_anlge + PI/2
    else:
        xy_min = source_angle_xy - PI/2
        xy_max = source_angle_xy + PI/2
        z_min = source_angle_z - PI/4 + PI/2
        z_max = source_angle_z + PI/4 + PI/2

    return xy_min, xy_max, z_min, z_max

def creat_sphere(r, phi, theta, source_point):
    x = r * math.sin(phi) * math.cos(theta) + source_point[0]
    y = r * math.sin(phi) * math.sin(theta) + source_point[1]
    z = r * math.cos(phi) + source_point[2]
    return [x, y, z]

def cal_angle(v, w):
    return np.arccos(np.dot(v,w)/(np.linalg.norm(v)*np.linalg.norm(w)))

def find_sphere_redius(file, point):
    mesh = o3d.io.read_triangle_mesh(file)
    mesh = mesh.sample_points_poisson_disk(1000)
    # o3d.visualization.draw_geometries([mesh])
    mesh = np.array(mesh.points)
    longest_distance = 0
    for k in mesh:
        pnt2src_dist = math.dist(k,point)
        longest_distance = max(pnt2src_dist,longest_distance)

    return longest_distance

def camera_move(start_point, moving_mount):
    move_redius = math.dist(start_point,[0,0,0])
    fist_angle = math.atan2(start_point[1], start_point[0])
    move_angle = 2 * PI * moving_mount / camera_moving_mount + fist_angle
    x = move_redius * math.cos(move_angle)
    y = move_redius * math.sin(move_angle)
    z = start_point[2]

    return [x, y, z]

def check_fov(source_point, target_point, check_point):
    if FOV_mode:
        if math.dist(source_point, check_point) < models_data[scanner_model_specifications][1][0] * 0.9:
            return False
        else:
            return True
    else:
        return True
    
def calc_projection(a, b):
    return abs(np.linalg.norm((np.dot(a, b) / np.dot(b, b)) * b))


##################################(main)####################################
if __name__ == '__main__':
    if ply_save == True:
        print("\n ply_save = True")
        save_ply_folder_path = os.path.join(input_data_folder_path, "results")
        print("\n result folder: ",save_ply_folder_path)
        if not os.path.exists(save_ply_folder_path):
            print("Making results folder: ", save_ply_folder_path)
            os.makedirs(save_ply_folder_path)

    start = time.time()
    ##scanning multiple location
    for move_num in tqdm(range(camera_moving_mount),leave = False, position = 0):

        pcd_list = []
        pcd = o3d.geometry.PointCloud()

        if camera_moving_mount != 1:
            now_point = camera_move(Source_point, move_num)
        else:
            now_point = Source_point
        if FOV_mode:
            redius = models_data[scanner_model_specifications][1][2]
        else:
            redius = find_sphere_redius(stl_file, now_point)        
        pcd_list, pnt_amount = make_pcd_spr2pnt(now_point, Source_target, Angular_Resolution, redius)
        ## scanning data convert to pointcloud data
        pcd_array = np.asarray(pcd_list, dtype=np.float32)
        pcd.points = o3d.utility.Vector3dVector(pcd_array)

        # pcd = pcd.voxel_down_sample(voxel_size=0.001)

        num_picked_points = len(np.array(pcd.points))
        #scanning data visulaization
        # o3d.visualization.draw_geometries([pcd])
        if len(pcd_list) != 0 and ply_save == True:
            print("\n!!! points are detected !!!")
            print("Simulation time: ", time.time() - start, " (sec)")
            # # o3d.visualization.draw_geometries([pcd])
            ply_name = stl_file_name.split(".")[0]+ "_" + "Lidar_position_" + format(move_num + 1, '03') + "_num_of_points_"+ format(num_picked_points, '05') + "_noise_" + str(gaussian_mode) +"_AngularResolution_" + str([math.degrees(Angular_Resolution[0]), math.degrees(Angular_Resolution[1])]) + "_model_" + str(scanner_model_specifications)+ ".ply"
            save_path = os.path.join(save_ply_folder_path, ply_name)
            ret = o3d.io.write_point_cloud(save_path, pcd, write_ascii = True) 
            if ret:
                print("\n!!! points are saved !!!")
            else:
                print("\n!!! points are not saved properly. Error in pointcloud writing. !!!")
            #o3d.visualization.draw_geometries([pcd])
            #pass
        else:
            print("\n!!! points are not picked or not detected !!!")
            print("Simulation time: ", time.time() - start, " (sec)")

        print(pnt_amount, ", ", num_picked_points, ", ", time.time() - start)
        # print(pnt_amount)
    o3d.visualization.draw_geometries([pcd])
