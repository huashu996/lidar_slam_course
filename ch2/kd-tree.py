# kdtree的具体实现，包括构建和查找
 
import random
import math
import numpy as np
 
from result_set import KNNResultSet, RadiusNNResultSet
 
# Node类，Node是tree的基本组成元素
class Node:
    def __init__(self, axis, value, left, right, point_indices):
        self.axis = axis                        #以哪个维度去切
        self.value = value                      #维度的值
        self.left = left                        #左节点
        self.right = right#右节点
        self.point_indices = point_indices#这一片区域存储的值（切之前）
 
#判断是否是叶子
    def is_leaf(self):
        if self.value is None:
            return True
        else:
            return False
 
    def __str__(self):
        output = ''
        output += 'axis %d, ' % self.axis
        if self.value is None:
            output += 'split value: leaf, '
        else:
            output += 'split value: %.2f, ' % self.value
        output += 'point_indices: '
        output += str(self.point_indices.tolist())
        return output
 
# 功能：构建树之前需要对value进行排序，同时对一个的key的顺序也要跟着改变
# 输入：
#     key：键
#     value:值
# 输出：
#     key_sorted：排序后的索引
#     value_sorted：排序后的值
def sort_key_by_vale(key, value):
    assert key.shape == value.shape
    assert len(key.shape) == 1
    sorted_idx = np.argsort(value)
    key_sorted = key[sorted_idx]
    value_sorted = value[sorted_idx]
    return key_sorted, value_sorted
 
 
def axis_round_robin(axis, dim):
    if axis == dim-1:
        return 0
    else:
        return axis + 1
 
# 功能：通过递归的方式构建树
# 输入：
#     root: 树的根节点
#     db: 点云数据
#     point_indices：排序后的键
#     axis: scalar  维度
#     leaf_size: scalar A leaf node can contain more than 1 point
# 输出：
#     root: 即构建完成的树
def kdtree_recursive_build(root, db, point_indices, axis, leaf_size):
    #处理根节点 初始化根节点
    if root is None:
        root = Node(axis, None, None, None, point_indices)
 
    # determine whether to split into left and right
    if len(point_indices) > leaf_size: #如果区域的点大于leaf_size
        #point_indices  db某一个维度从小到大排列的下标
        point_indices_sorted, _ = sort_key_by_vale(point_indices, db[point_indices, axis])  # M
 
        # 作业1
        # 屏蔽开始
        #ceil是向上取整  找到左边的点
        middle_left_idx = math.ceil(point_indices_sorted.shape[0] / 2) - 1 #某一维度中间的索引
        #print(middle_left_idx)
        middle_left_point_idx = point_indices_sorted[middle_left_idx]   # 某一维度中间的值
        #print(middle_left_point_idx)
        middle_left_point_value = db[middle_left_point_idx,axis]   #中间节点
        #print("middle_left_point_value",middle_left_point_value)
        #右边
        middle_right_idx = middle_left_idx + 1
        middle_right_point_idx = point_indices_sorted[middle_right_idx]
        middle_right_point_value = db[middle_right_point_idx,axis]
 
        root.value = (middle_left_point_value + middle_right_point_value) * 0.5
        # 如果分割后的左边或右边区域点数量大于leaf_size，再次分割
        root.left = kdtree_recursive_build(root.left,db,point_indices_sorted[0:middle_right_idx],
                                           axis_round_robin(axis,dim=db.shape[1]),leaf_size)
 
        root.right = kdtree_recursive_build(root.right,db,point_indices_sorted[middle_right_idx:],
                                           axis_round_robin(axis,dim=db.shape[1]),leaf_size)
        # 屏蔽结束
    return root
 
 
# 功能：翻转一个kd树
# 输入：
#     root：kd树
#     depth: 当前深度
#     max_depth：最大深度
def traverse_kdtree(root: Node, depth, max_depth):
    depth[0] += 1
    if max_depth[0] < depth[0]:
        max_depth[0] = depth[0]
 
    if root.is_leaf():
        print(root)
    else:
        traverse_kdtree(root.left, depth, max_depth)
        traverse_kdtree(root.right, depth, max_depth)
 
    depth[0] -= 1
 
# 功能：1、构建kd树（利用kdtree_recursive_build功能函数实现的对外接口）
# 输入：
#     db_np：原始数据
#     leaf_size：scale
# 输出：
#     root：构建完成的kd树
def kdtree_construction(db_np, leaf_size):
    N, dim = db_np.shape[0], db_np.shape[1] #个数与维度
 
    # build kd_tree recursively
    root = None
    root = kdtree_recursive_build(root,
                                  db_np,
                                  np.arange(N),
                                  axis=0,
                                  leaf_size=leaf_size)
    return root
 
 
# 功能：通过kd树实现knn搜索，即找出最近的k个近邻
# 输入：
#     root: kd树
#     db: 原始数据
#     result_set：搜索结果
#     query：索引信息
# 输出：
#     搜索失败则返回False
def kdtree_knn_search(root: Node, db: np.ndarray, result_set: KNNResultSet, query: np.ndarray):
    if root is None:
        return False
 
    # # leaf_points.shape: (124668, 3)
    # leaf_points = db[root.point_indices, :]
    # print("leaf_points.shape:",leaf_points.shape)
    # # query是一维数组 query.shape: (3,)
    # print("query.shape:", query.shape)
    # # 添加一个维度，转成：query.shape: (1, 3)
    # print("query.shape:",np.expand_dims(query, 0).shape)
 
    #比较子节点中的每一个点，
    if root.is_leaf():
        # compare the contents of a leaf
        leaf_points = db[root.point_indices, :]
        # print("query:",query)
        # print("leaf_points:", leaf_points)
        #axis=1表示按行向量处理，求多个行向量的范数，默认是二范数
        diff = np.linalg.norm(np.expand_dims(query, 0) - leaf_points, axis=1)
        for i in range(diff.shape[0]):
            result_set.add_point(diff[i], root.point_indices[i])
        return False
 
    # 作业2
    # 提示：仍通过递归的方式实现搜索
    # 屏蔽开始
    #在左边则左边是必须要递归搜索的，如果切分线的距离大于最差距离，则右边不需要搜索了
    if query[root.axis] <= root.value:
        kdtree_knn_search(root.left,db,result_set,query)
        if math.fabs(query[root.axis] - root.value) < result_set.worstDist():
            kdtree_knn_search(root.right,db,result_set,query)
    else:
        kdtree_knn_search(root.right,db,result_set,query)
        if math.fabs(query[root.axis] - root.value) < result_set.worstDist():
            kdtree_knn_search(root.right,db,result_set,query)
    # 屏蔽结束
 
    return False
 
# 功能：通过kd树实现radius搜索，即找出距离radius以内的近邻
# 输入：
#     root: kd树
#     db: 原始数据
#     result_set:搜索结果
#     query：索引信息
# 输出：
#     搜索失败则返回False
def kdtree_radius_search(root: Node, db: np.ndarray, result_set: RadiusNNResultSet, query: np.ndarray):
    if root is None:
        return False
 
    if root.is_leaf():
        # compare the contents of a leaf
        leaf_points = db[root.point_indices, :]
        diff = np.linalg.norm(np.expand_dims(query, 0) - leaf_points, axis=1)
        for i in range(diff.shape[0]):
            result_set.add_point(diff[i], root.point_indices[i])
        return False
    
    # 作业3
    # 提示：通过递归的方式实现搜索
    # 屏蔽开始
    if query[root.axis] <= root.value:
        kdtree_radius_search(root.left,db,result_set,query)
        if math.fabs(query[root.axis] - root.value) < result_set.worstDist():
            kdtree_radius_search(root.right,db,result_set,query)
    else:
        kdtree_radius_search(root.right,db,result_set,query)
        if math.fabs(query[root.axis] - root.value) < result_set.worstDist():
            kdtree_radius_search(root.left,db,result_set,query)
 
    # 屏蔽结束
 
    return False
 
 
 
def main():
    # configuration
    db_size = 64
    dim = 3
    leaf_size = 4 #小于4个点不再分割
    k = 1
 
    db_np = np.random.rand(db_size, dim) #随机产生64个三维的点
    #print(db_np)
    # 开始建立KD树
    root = kdtree_construction(db_np, leaf_size=leaf_size)
 
    depth = [0]
    max_depth = [0]
    traverse_kdtree(root, depth, max_depth)
    print("tree max depth: %d" % max_depth[0])
 
if __name__ == '__main__':
    main()
