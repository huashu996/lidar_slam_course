# 实现voxel滤波，并加载数据集中的文件进行验证
import sys
import open3d as o3d 
import os
import numpy as np
import pandas as pd
from pyntcloud import PyntCloud
 
# 功能：对点云进行voxel滤波
# 输入：
#     point_cloud：输入点云
#     leaf_size: voxel尺寸
def voxel_filter(point_cloud, leaf_size):
    filtered_points = []
    #三个维度最小/大值
    x_min, y_min, z_min = np.amin(point_cloud, axis=0)
    x_max, y_max, z_max = np.amax(point_cloud, axis=0)
 
    #确定每一个维度的格子数量
    Dx = (x_max - x_min)//leaf_size + 1    #保证0-leaf_size 在第一个格子内
    Dy = (y_max - y_min)//leaf_size + 1
    Dz = (z_max - z_min)//leaf_size + 1
    print("Dx x Dy x Dz is {} x {} x {}".format(Dx, Dy, Dz))
 
    dict = { }  #建立一个空的字典 放h对应点的数据
    index_ = { } #放h对应点的数量
    for i in range(len(point_cloud)):
        hx = (point_cloud[i, 0] - x_min)//leaf_size + 1
        hy = (point_cloud[i, 1] - y_min)//leaf_size + 1
        hz = (point_cloud[i, 2] - z_min)//leaf_size + 1
        h = hx + hy*Dx + hz*Dx*Dy
        # 如果h不相同把点放入，相同则计算平均
        if (h not in dict):
            dict[h] = point_cloud[i]
            index_[h] = 1
        else:
            val = dict.get(h, 0)  #先把字典中的数据取出来
            num = index_.get(h, 0)
            dict[h] = (val * num + point_cloud[i])/(num + 1) #来一次点就需要求相同h的所有点的平均
            index_[h] = num + 1
 
        for key,value in dict.items():#当两个参数时
            filtered_points.append(value)
    # 把点云格式改成array，并对外返回
    filtered_points = np.array(filtered_points, dtype=np.float64)
    return filtered_points
 
def main():
    # # 从ModelNet数据集文件夹中自动索引路径，加载点云
    # cat_index = 10 # 物体编号，范围是0-39，即对应数据集中40个物体
    # root_dir = '/Users/renqian/cloud_lesson/ModelNet40/ply_data_points' # 数据集路径
    # cat = os.listdir(root_dir)
    # filename = os.path.join(root_dir, cat[cat_index],'train', cat[cat_index]+'_0001.ply') # 默认使用第一个点云
    # point_cloud_pynt = PyntCloud.from_file(file_name)
 
    # 加载自己的点云文件
    #读取点云txt文件
    points = np.genfromtxt("./car_0001.txt", delimiter=",")
    points = pd.DataFrame(points[:, 0:3])
    points.columns = ['x', 'y', 'z']
    point_cloud_pynt = PyntCloud(points)
 
 
    # 转成open3d能识别的格式
    point_cloud_o3d = point_cloud_pynt.to_instance("open3d", mesh=False)
    # o3d.visualization.draw_geometries([point_cloud_o3d]) # 显示原始点云
 
    # 调用voxel滤波函数，实现滤波
    points = np.asarray(point_cloud_o3d.points)#转为矩阵格式
    filtered_cloud = voxel_filter(points, 0.1)
    point_cloud_o3d.points = o3d.utility.Vector3dVector(filtered_cloud)#将点云转换成open3d中的数据形式并用point_cloud_o3d来保存
    # 显示滤波后的点云
    o3d.visualization.draw_geometries([point_cloud_o3d])
 
if __name__ == '__main__':
    main()
