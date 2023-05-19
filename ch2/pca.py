# 实现PCA分析和法向量计算，并加载数据集中的文件进行验证
 
import open3d as o3d 
import os
import numpy as np
import pandas as pd
from pyntcloud import PyntCloud
 
# 功能：计算PCA的函数
# 输入：
#     data：点云，NX3的矩阵
#     correlation：区分np的cov和corrcoef，不输入时默认为False
#     sort: 特征值排序，排序是为了其他功能方便使用，不输入时默认为True
# 输出：
#     eigenvalues：特征值
#     eigenvectors：特征向量
def PCA(data, correlation=False, sort=True):
 
    X = np.asarray(data).T #np.array与np.asarray功能是一样的，都是将输入转为矩阵格式,再转置
    X_mean = np.mean(X,axis=1).reshape(3,1) #axis=1是每一行的平均值，等于0是没一列的平均值
    X_head = X - X_mean
    H = X_head.dot(X_head.T) #点乘转置
    eigenvalues,eigenvectors = np.linalg.eig(H) #求矩阵的特征值和特征向量
    #排序
    if sort:
        sort = eigenvalues.argsort()[::-1] #.argsort返回数组从小到大排序后对应的索引值
        eigenvalues = eigenvalues[sort]
        eigenvectors = eigenvectors[:, sort]
    return eigenvalues, eigenvectors
#```handlebars
def main():
 
    # 加载txt格式原始点云
    points = pd.read_csv("/home/cxl/三维点云课程资源/1点云介绍及降采样/modelnet40_normal_resampled/modelnet40_normal_resampled/car/car_0001.txt") #将读入的数据按照numpyarray的方式存储
    points = points.iloc[:,0:3] #iloc遍历的数数字，0:3对应0,1,和2
    points.columns = ["x","y","z"] #给选取到的数据 附上标题 
    point_cloud_pynt = PyntCloud(points) #PyntCloud python的点云处理库
    point_cloud_o3d = point_cloud_pynt.to_instance("open3d", mesh=False) #实例化
    o3d.visualization.draw_geometries([point_cloud_o3d]) # 显示原始点云
 
    # 从点云中获取点，只对点进行处理
    points = point_cloud_pynt.points
    print('total points number is:', points.shape[0])
 
    # 用PCA分析点云主方向
    w, v = PCA(points)
    point_cloud_vector = v[:, 0] #点云主方向对应的向量
    print('the main orientation of this pointcloud is: ', point_cloud_vector)
    # 绘制三个主方向
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector([np.mean(points, axis=0), np.mean(points, axis=0) + v[:, 2], np.mean(points, axis=0) + v[:, 1], np.mean(points, axis=0) + v[:, 0]])
    line_set.lines = o3d.utility.Vector2iVector([[0, 1], [0, 2], [0, 3]])
    line_set.colors = o3d.utility.Vector3dVector([[0, 0, 1], [0, 1, 0], [1, 0, 0]])
    o3d.visualization.draw_geometries([point_cloud_o3d, line_set])
    
    # 循环计算每个点的法向量
    pcd_tree = o3d.geometry.KDTreeFlann(point_cloud_o3d) #建立KD树
    normals = []
    for point in point_cloud_o3d.points:
        [k, idx, _] = pcd_tree.search_knn_vector_3d(point, knn=50)#返回锚点的50个最近邻居的索引列表
        w,v = PCA(points.iloc[idx,:])#前面的冒号就是取行数，后面的冒号是取列数
        normals.append(v[:,2])
 
    normals = np.array(normals, dtype=np.float64)
    # TODO: 此处把法向量存放在了normals中
    point_cloud_o3d.normals = o3d.utility.Vector3dVector(normals)
    o3d.visualization.draw_geometries([point_cloud_o3d], point_show_normal=True)
    # 利用法向量滚球法重建表面
    radii = [0.005, 0.01, 0.02, 0.04]
    rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
                   point_cloud_o3d, o3d.utility.DoubleVector(radii))
    o3d.visualization.draw_geometries([point_cloud_o3d, rec_mesh])
 
 
 
if __name__ == '__main__':
    main()
