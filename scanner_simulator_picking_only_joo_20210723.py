from pycaster.pycaster import rayCaster
import os
import open3d as o3d
import numpy as np
from tqdm import tqdm
import math, random

import time

##initialize
scanner_position = [[500, 500, 400], [-500, -400, 300]] #scanner position 
num_scanner_position = len(scanner_position)

scan_target_position = [[0, 0, 100]]

#Gazing with Gaussian, there is some bugs in the gazing mode.. working on it
gaussian_mode = True #
gaussian_crop = 100
gaussian_density = 3


#Phoxi scanner model data (scan area)
#To check FOV(field of view), not properly implented yet
'''
models_data = {'xs' : [[106, 118, 133],[161,181,205],[70,78,88],[0.035]], 
                's' : [[343, 360, 382],[384,442,520],[237,272,319],[0.050]], 
                'm' : [[317, 590, 826],[458, 650, 1118],[292,404,686],[0.100]], 
                'l' : [[600, 1082, 1644],[870, 1239, 2150],[557, 772, 1326],[0.200]]} #[[width],[lenth],[height],[noise]]
'''
   
def make_pcd_ply2pnt(pSource, ply_file_path):
    # point to surface
    pcd = []
    gaussian_temp = []
    ply = o3d.io.read_point_cloud(ply_file_path)
    ply_arr = np.array(ply.points)
    mesh_error_correction = pSource / np.linalg.norm(pSource)

    for suface_pcd in tqdm(ply_arr, leave = False, position = 2):
        try:
            pointsIntersection = caster.castRay(pSource, (suface_pcd + mesh_error_correction)) # allow error
            # if check_fov(pSource, pointsIntersection[0]): not properly implemented yet
            if len(pointsIntersection) <= 0:
                if gaussian_mode: 
                    gaussian_temp.append(np.append(suface_pcd, np.array(math.tan(cal_angle(np.array(scan_target_position) - np.array(pSource), np.array(suface_pcd) - np.array(pSource))) * calc_projection(np.array(scan_target_position) - np.array(pSource), np.array(suface_pcd) - np.array(pSource)))))
                else:
                    pcd.append(tuple(suface_pcd))
        except:
            pass
    if gaussian_mode:
        if (gaussian_crop > 0):
            gaussian_temp = list(filter(lambda x: x[:][3] <= gaussian_crop, gaussian_temp))
        gaussian_temp = sorted(gaussian_temp, key = lambda x : x[:][3])
        gaussian_temp_len = int(len(gaussian_temp)/3)
        for t in  range(gaussian_temp_len):
            try:
                temp = gaussian_temp.pop(int(abs(random.gauss(0, gaussian_temp_len/gaussian_density))))
                pcd.append(tuple((temp[0],temp[1],temp[2])))
            except:
                pass

    return pcd, len(ply_arr)
    

def cal_angle(v, w):
    return np.arccos(np.dot(v,w)/(np.linalg.norm(v)*np.linalg.norm(w)))

'''
def check_fov(source_point, check_point):
    if math.dist(source_point, check_point) < models_data[model_select][1][0] * 0.9:
        return False
    else:
        return True
    # return True
'''
def calc_projection(a, b):
    # P = np.outer(a, a) / a.dot(a)
    # return P.dot(b.T)
    return abs(np.linalg.norm((np.dot(a, b) / np.dot(b, b)) * b))

def make_x_axis_points_pcd(center_point,axis_size,num_points):
    x_range = (-axis_size, axis_size)
    #x_range = (-0.0, axis_size)    
    y_range = (-5.0, 5.0)
    z_range = (-5.0, 5.0)

    pcd = []
    for i in range(num_points):
        pcd.append((center_point[0]+random.uniform(*x_range), center_point[1]+random.uniform(*y_range), center_point[2]+random.uniform(*z_range))) 
    ply = o3d.geometry.PointCloud()
    ply.points = o3d.utility.Vector3dVector(np.asarray(pcd, dtype=np.float32))
    return ply

def make_y_axis_points_pcd(center_point,axis_size,num_points):
    x_range = (-5.0, 5.0)
    #y_range = (-0.0, axis_size)
    y_range = (-axis_size, axis_size)    
    z_range = (-5.0, 5.0)

    pcd = []
    for i in range(num_points):
        pcd.append((center_point[0]+random.uniform(*x_range), center_point[1]+random.uniform(*y_range), center_point[2]+random.uniform(*z_range)))
    ply = o3d.geometry.PointCloud()
    ply.points = o3d.utility.Vector3dVector(np.asarray(pcd, dtype=np.float32))
    return ply

def make_z_axis_points_pcd(center_point,axis_size,num_points):
    x_range = (-5.0, 5.0)
    y_range = (-5.0, 5.0)
    z_range = (-axis_size, axis_size)    
    #z_range = (-0.0, axis_size)

    pcd = []
    for i in range(num_points):
        pcd.append((center_point[0]+random.uniform(*x_range), center_point[1]+random.uniform(*y_range), center_point[2]+random.uniform(*z_range)))
    ply = o3d.geometry.PointCloud()
    ply.points = o3d.utility.Vector3dVector(np.asarray(pcd, dtype=np.float32))
    return ply    



##input data folder: ./data
input_data_folder_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data")

##output ply (picked points from source ply file) save folder
#if you want to save the result ply file, set ply_save == True
ply_save = True

if ply_save == True:
    print("\n ply_save = True")
    save_ply_folder_path = os.path.join(input_data_folder_path, "results")
    print("\n result folder: ",save_ply_folder_path)
    if not os.path.exists(save_ply_folder_path):
        print("Making results folder: ", save_ply_folder_path)
        os.makedirs(save_ply_folder_path)

#model mesh data
stl_file_name = "airplane_0627.stl"
stl_file = os.path.join(input_data_folder_path, stl_file_name)

#source ply file
ply_file_name = "airplane_0627_poisson_sampling_37797pts.ply"
input_ply_file = os.path.join(input_data_folder_path, ply_file_name)

#ray caster 
caster = rayCaster.fromSTL(stl_file, scale = 1)


## if move_num > 1, scanning from location

for move_num in tqdm(range(num_scanner_position),leave = False, position = 0):
    start = time.time()
    pcd_list = []

    pcd_list, pnt_mount = make_pcd_ply2pnt(scanner_position[move_num], input_ply_file)
    num_picked_points = len(pcd_list)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd_list, dtype=np.float32))

    if len(pcd_list) != 0 and ply_save == True:
        print("\n!!! points are picked !!!")
        print("Simulation time: ", time.time() - start, " (sec)")
        # # o3d.visualization.draw_geometries([pcd])
        ply_name = stl_file_name.split(".")[0]+ "_" + "scanner_position_" + format(move_num + 1, '03') + "_num_of_points_"+ format(num_picked_points, '05') + "_gazing_" + str(gaussian_mode) +"_density_" + str(gaussian_density) + "_crop_" + str(gaussian_crop)+ ".ply"
        save_path = os.path.join(save_ply_folder_path, ply_name)
        ret = o3d.io.write_point_cloud(save_path, pcd, write_ascii = True) 
        if ret:
            print("\n!!! points are saved !!!")
        else:
            print("\n!!! points are not saved properly. Error in pointcloud writing. !!!")
        #o3d.visualization.draw_geometries([pcd])
        #pass
    else:
        print("\n!!! points are not picked !!!")
        print("Simulation time: ", time.time() - start, " (sec)")
