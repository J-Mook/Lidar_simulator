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
FOV_mode = True
scanner_model_specifications = 'l'
Crop_radius = 0 #(0 : not crop)
Sampling_type = "gaussian" # select gaussian or uniform (if "" is no sampling)
gaussian_density = 4
ply_save = False

activate_CPU = 4
    
models_data = {'xs' : [[106, 118, 133],[161,181,205],[70,78,88],[0.035]], 
                's' : [[343, 360, 382],[384,442,520],[237,272,319],[0.050]], 
                'm' : [[317, 590, 826],[458, 650, 1118],[292,404,686],[0.100]], 
                'l' : [[600, 1082, 1644],[870, 1239, 2150],[557, 772, 1326],[0.200]]} #[[width],[lenth],[height],[noise]]


time1 = time.time()
##search stl file data
input_data_folder_path = str(os.path.join(os.path.dirname(os.path.realpath(__file__)), "data"))
save_ply_folder_path = os.path.join(input_data_folder_path, "ply")
stl_file_name = "TestSpecimenAssy.stl"
# stl_file_name = "MeshCAA.stl"
# stl_file_name = "airplane_0627.stl"

stl_file = os.path.join(input_data_folder_path, stl_file_name)
caster = rayCaster.fromSTL(stl_file, scale = 1.0)

ply_file_name = "test_specimen_assy_poisson_sampling_66593pts.ply"
# ply_file_name = "airplane_0627_poisson_sampling_37797pts.ply"
# ply_file_name = "test_specimen_assy_montecarlo_sampling_50000pts.ply"
input_ply_file = os.path.join(input_data_folder_path, ply_file_name)

# from numba import jit
# @jit # numba mode
def find_intersection_point(pTarget, Source_loc, Source_target_loc, error_correction):
    # print("process_di :",os.getpid())
    pointsIntersection = caster.castRay(Source_loc, (pTarget + error_correction)) # allow error
    if len(pointsIntersection) <= 0:
        if Sampling_type != "": 
            proj_dist = np.array(math.tan(cal_angle(np.array(Source_target_loc) - np.array(Source_loc), np.array(pTarget) - np.array(Source_loc))) * calc_projection(np.array(Source_target) - np.array(Source_loc), np.array(pTarget) - np.array(Source_loc)))
            return np.append(pTarget, proj_dist)
        else:
            # pcd.append(tuple(pTarget))
            return tuple(pTarget)
    
def make_pcd_ply2pnt(pSource, tSource, ply_file_path):
    # point to surface
    pcd = []
    result_list=[]
    pcd_temp = []
    ply_filtered = []
    ply = o3d.io.read_point_cloud(ply_file_path)
    ply_arr = np.array(ply.points)
    target_dir = np.array(pSource) - np.array(tSource)
    mesh_error_correction = target_dir / np.linalg.norm(target_dir)
    
    for suface_pcd in ply_arr:
        if check_fov(pSource, tSource, suface_pcd):
            ply_filtered.append(suface_pcd)

    if activate_CPU == 0:
        for suface_pcd in tqdm(ply_filtered, leave = False, position = 2):
            result = find_intersection_point(suface_pcd, pSource, tSource, mesh_error_correction)
            result_list.append(result)
    else:
        ########################################CPU accelation##################################
        with poolcontext(processes = activate_CPU) as pool:
            # result_list = pool.map(partial(find_intersection_point, Source_loc = pSource, Source_target_loc = tSource, error_correction = mesh_error_correction), ply_filtered)
            result_list = list(tqdm(pool.imap_unordered(partial(find_intersection_point, Source_loc = pSource, Source_target_loc = tSource, error_correction = mesh_error_correction), ply_filtered), total=len(ply_filtered)))
        ########################################CPU accelation##################################
    
    pcd_temp = none_check(result_list)
    
    if (Crop_radius > 0):
        pcd_temp = list(filter(lambda x: x[:][3] <= Crop_radius, pcd_temp))

    if Sampling_type != "":
        pcd_temp = sorted(pcd_temp, key = lambda x : x[:][3])
        pcd_temp_len = int(len(pcd_temp)/3)
        for t in  range(pcd_temp_len):
            if Sampling_type == "gaussian":
                temp = pcd_temp.pop(int(abs(random.gauss(0, pcd_temp_len/gaussian_density))))
            elif Sampling_type == "uniform":
                temp = pcd_temp.pop(int(abs(random.randrange(0, pcd_temp_len))))
            pcd.append(tuple((temp[0],temp[1],temp[2])))
    else:
        pcd = pcd_temp

    return pcd, len(ply_arr)
    
def none_check(input_list):
    output_list = list(filter(None.__ne__, input_list))
    return output_list

def setting_ROI_angle(start_point, end_point):
    point_dir = np.array(end_point) - np.array(start_point)
    source_angle_xy = math.atan2(point_dir[1], point_dir[0])
    source_angle_z = cal_angle([point_dir[0],point_dir[1], 0], [point_dir[0], point_dir[1], point_dir[2]])
    
    if FOV_mode:
        xy_angle = abs(math.atan2(((models_data[scanner_model_specifications][0][2] - models_data[scanner_model_specifications][0][0]) / 2 ) , (models_data[scanner_model_specifications][1][2] - models_data[scanner_model_specifications][1][0])))
        xy_min = source_angle_xy - xy_angle
        xy_max = source_angle_xy + xy_angle
        z_anlge = abs(math.atan2(((models_data[scanner_model_specifications][2][2] - models_data[scanner_model_specifications][2][0]) / 2 ) , (models_data[scanner_model_specifications][1][2] - models_data[scanner_model_specifications][1][0])))
        z_min = source_angle_z - z_anlge
        z_max = source_angle_z + z_anlge
    else:
        xy_min = source_angle_xy - PI/2
        xy_max = source_angle_xy + PI/2
        z_min = source_angle_z - PI/4
        z_max = source_angle_z + PI/4

    return xy_min, xy_max, z_min, z_max

def creat_sphere(r, phi, theta, source_point):
    x = r * math.sin(phi) * math.cos(theta) + source_point[0]
    y = r * math.sin(phi) * math.sin(theta) + source_point[1]
    z = r * math.cos(phi) + source_point[2]
    return [x, y, z]

def rotation_matrix(axis, theta):
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(math.degrees(theta))
    b, c, d = -axis * math.sin(math.degrees(theta))
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

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
    
    min_angle_xy, max_anlge_xy, min_angle_z, max_angle_z = setting_ROI_angle(source_point, target_point)
    check_xy_angle = math.atan2(check_point[1] - source_point[1], check_point[0] - source_point[0])
    check_z_angle = cal_angle([source_point[0],source_point[1], 0], [source_point[0] - check_point[0],
                                source_point[1] - check_point[1], source_point[2] - check_point[2]])

    if  (check_xy_angle > max_anlge_xy and check_xy_angle < min_angle_xy + 2*PI) or (check_xy_angle < min_angle_xy and check_xy_angle > max_anlge_xy - 2*PI):
        return False
    elif check_z_angle > max_angle_z or check_z_angle < min_angle_z:
        return False
    elif FOV_mode and math.dist(source_point, check_point) > models_data[scanner_model_specifications][1][2]:
        return False
    elif FOV_mode and math.dist(source_point, check_point) < models_data[scanner_model_specifications][1][0] * 0.9:
        return False
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
        pcd_list, pnt_amount = make_pcd_ply2pnt(now_point, Source_target, input_ply_file)
        ## scanning data convert to pointcloud data
        pcd_array = np.asarray(pcd_list, dtype=np.float32)
        pcd.points = o3d.utility.Vector3dVector(pcd_array)
        
        num_picked_points = len(np.array(pcd.points))
        #scanning data visulaization
        
        if len(pcd_list) != 0 and ply_save == True:
            print("\n!!! points are detected !!!")
            print("Simulation time: ", time.time() - start, " (sec)")
            # # o3d.visualization.draw_geometries([pcd])
            ply_name = stl_file_name.split(".")[0]+ "_" + "picker_position_" + format(move_num + 1, '03') + "_num_of_points_"+ format(num_picked_points, '05') + "_gazing_" + str(Sampling_type != "") +"_density_" + str(gaussian_density) + "_crop_" + str(Crop_radius)+ ".ply"
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

    # o3d.visualization.draw_geometries([pcd])
