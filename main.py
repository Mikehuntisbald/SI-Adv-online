# -*- coding: utf-8 -*-
from pc_array.msg import PointCloud2Array

import os
import argparse
import time
import random
import numpy as np
from tqdm import tqdm
from pathlib import Path
from collections import OrderedDict

import torch
import torch.nn as nn
import torch.nn.functional as F

from data_utils.ModelNetDataLoader import ModelNetDataLoader
from data_utils.ShapeNetDataLoader import PartNormalDataset
from torch.utils.data import DataLoader, TensorDataset


from utils.logging import Logging_str
from utils.utils import set_seed

from attacks import PointCloudAttack
from utils.set_distance import ChamferDistance, HausdorffDistance

import open3d as o3d
from std_msgs.msg import Header

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from multiprocessing import Pool
import torch.multiprocessing as mp

def process_point_cloud(point_cloud, attack):
    pc_np = np.array(list(pc2.read_points(point_cloud, skip_nans=True)))
    if pc_np.shape[0] <= 30:
        return None
    pc_tensor = torch.from_numpy(pc_np).float().cuda()
    target = torch.tensor([3], dtype=torch.int32).cuda()
    pc_tensor = pc_tensor.unsqueeze(0)
    zeros = torch.zeros(pc_tensor.size(0), pc_tensor.size(1), 3).cuda()
    pc_tensor = torch.cat((pc_tensor, zeros), dim=-1)
    t0 = time.time()
    adv_points, adv_target, query_costs = attack.run(pc_tensor, target)
    t1 = time.time()
    print(f"Elapsed time: {t1 - t0:.2f} seconds")
    return adv_points.squeeze(0).cpu().numpy(), adv_target, query_costs

def load_data(args):
    """Load the dataset from the given path.
    """
    print('Start Loading Dataset...')
    if args.dataset == 'ModelNet40':
        TEST_DATASET = ModelNetDataLoader(
            root=args.data_path,
            npoint=args.input_point_nums,
            split='test',
            normal_channel=True
        )
    elif args.dataset == 'ShapeNetPart':
        TEST_DATASET = PartNormalDataset(
            root=args.data_path,
            npoints=args.input_point_nums,
            split='test',
            normal_channel=True
        )
    else:
        raise NotImplementedError

    testDataLoader = torch.utils.data.DataLoader(
        TEST_DATASET,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers
    )
    print('Finish Loading Dataset...')
    return testDataLoader



def data_preprocess(data):
    """Preprocess the given data and label.
    """
    points, target, *_ = data

    points = points # [B, N, C]
    target = target[:, 0] # [B]
    print("data_preprocess ", target)
    points = points.cuda()
    target = target.cuda()
    # print('pre-processed')
    return points, target


def save_tensor_as_txt(points, filename):
    """Save the torch tensor into a txt file.
    """
    points = points.squeeze(0).detach().cpu().numpy()
    with open(filename, "a") as file_object:
        for i in range(points.shape[0]):
            # msg = str(points[i][0]) + ' ' + str(points[i][1]) + ' ' + str(points[i][2])
            msg = str(points[i][0]) + ' ' + str(points[i][1]) + ' ' + str(points[i][2]) + \
                ' ' + str(points[i][3].item()) +' ' + str(points[i][3].item()) + ' '+ str(1-points[i][3].item())
            file_object.write(msg+'\n')
        file_object.close()
    print('Have saved the tensor into {}'.format(filename))

class PointCloudAttackNode(Node):
    def __init__(self, args):
        super().__init__('point_cloud_attack_node')
        # --dataset ModelNet40 --data_path /home/SI-Adv/data/modelnet40_normal_resampled/ --query_attack_method ours --surrogate_model dgcnn --target_model paconv --step_size 0.32
        # start attack
        atk_success = 0
        avg_query_costs = 0.
        avg_mse_dist = 0.
        avg_chamfer_dist = 0.
        avg_hausdorff_dist = 0.
        avg_time_cost = 0.
        batch_id = 0
        chamfer_loss = ChamferDistance()
        hausdorff_loss = HausdorffDistance()
        self.args = args
        args.num_class = 40
        args.dataset = 'ModelNet40'
        args.query_attack_method = None
        args.transfer_attack_method = 'ifgm_ours'
        args.surrogate_model = 'centerpoint'
        args.target_model = 'dgcnn'
        args.step_size = 0.01
        args.max_steps = 500
        args.eps = 0.1
        self.attack = PointCloudAttack(args)
        self.publisher_ = self.create_publisher(PointCloud2, '/attacked_cloud', 1)
        self.subscription = self.create_subscription(PointCloud2Array, '/raw_cloud', self.listener_callback, 100)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        print("Received")
        t = msg.header.stamp 
        header = Header()
        header.stamp = t  # 使用当前时间
        header.frame_id = msg.header.frame_id
        all_points_list = []

        for point_cloud in msg.clouds:
            
            # Convert ROS PointCloud2 to numpy array
            pc_np = np.array(list(pc2.read_points(point_cloud, skip_nans=True)))
            if(pc_np.shape[0] <= 20):
                continue
            # Convert numpy array to torch tensor
            pc_tensor = torch.from_numpy(pc_np).float().cuda()
            target = torch.tensor([3], dtype=torch.int32).cuda()
            # Ensure the tensor is in the right shape: [batch_size, n_points, n_features]
            pc_tensor = pc_tensor.unsqueeze(0)  # assuming you get one point cloud per message

            zeros = torch.zeros(pc_tensor.size(0), pc_tensor.size(1), 1).cuda()

            # 在最后一个维度上连接原始张量和全0张量
            pc_tensor = torch.cat((pc_tensor, zeros), dim=-1)
            

            # This is where you call the attack code...
            t0 = time.time()
            adv_points, adv_target, query_costs = self.attack.run(pc_tensor, target)
            print("adv_target", adv_target.cpu().numpy())
            print("target", target.cpu().numpy())
            all_points_list.append(adv_points.squeeze(0).cpu().numpy())
            t1 = time.time()
            # avg_time_cost += t1 - t0
            print(f"Elapsed time: {t1 - t0:.2f} seconds")
            # if adv_target != target:
            #     print("adv_target", adv_target.cpu().numpy())
            # adv_points.squeeze(0)

        # with mp.Pool(processes=mp.cpu_count()) as pool:
        #     results = [pool.apply_async(process_point_cloud, (point_cloud, self.attack)) for point_cloud in msg.clouds]
            
        #     for result in results:
        #         output = result.get()
        #         if output is not None:
        #             adv_points, adv_target, query_costs = output
        #             all_points_list.append(adv_points.squeeze(0).cpu().numpy())
                

        pc_np = np.array(list(pc2.read_points(msg.cloud, skip_nans=True)))
        all_points_list.append(pc_np)

        # if not args.query_attack_method is None:
        #     print('>>>>>>>>>>>>>>>>>>>>>>>')
        #     print('Query cost: ', query_costs)
        #     print('>>>>>>>>>>>>>>>>>>>>>>>')
        #     avg_query_costs += query_costs
        # atk_success += 1 if adv_target != target else 0

            
        # # modified point num count
        # points = points[:,:,:3].data # P, [1, N, 3]
        # pert_pos = torch.where(abs(adv_points-points).sum(2))
        # count_map = torch.zeros_like(points.sum(2))
        # count_map[pert_pos] = 1.
        # # print('Perturbed point num:', torch.sum(count_map).item())

        # avg_mse_dist += np.sqrt(F.mse_loss(adv_points, points).detach().cpu().numpy() * 3072)
        # avg_chamfer_dist += chamfer_loss(adv_points, points)
        # avg_hausdorff_dist += hausdorff_loss(adv_points, points)
        
        # Convert numpy data to PointCloud2 format.
        # pc2_data = pc2.create_cloud_xyz32(header, adv_points.squeeze(0).cpu().numpy())
        # pc2_data.header.frame_id = msg.header.frame_id
        if all_points_list == []:
            return
        all_points_concatenated = np.vstack(all_points_list)
        pc2_data = pc2.create_cloud_xyz32(header, all_points_concatenated)
        # pc2_data.header.frame_id = msg.header.frame_id
        
        # Publish the PointCloud2 data.
        self.publisher_.publish(pc2_data)
        print("published")

def main():
    # 必要时初始化PyTorch多进程环境
    mp.set_start_method('spawn', force=True)
    rclpy.init()
    node = PointCloudAttackNode(args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    # # load data
    # test_loader = load_data(args)

    # num_class = 0
    # if args.dataset == 'ModelNet40':
    #     num_class = 40
    # elif args.dataset == 'ShapeNetPart':
    #     num_class = 16
    # assert num_class != 0
    # args.num_class = num_class

    # # load model
    # attack = PointCloudAttack(args)

    # # start attack
    # atk_success = 0
    # avg_query_costs = 0.
    # avg_mse_dist = 0.
    # avg_chamfer_dist = 0.
    # avg_hausdorff_dist = 0.
    # avg_time_cost = 0.
    # batch_id = 0
    # chamfer_loss = ChamferDistance()
    # hausdorff_loss = HausdorffDistance()
    # for batch_id, data in tqdm(enumerate(test_loader), total=len(test_loader)):
    #     # prepare data for testing
    #     points, target = data_preprocess(data)
    #     target = target.long()

    #     # start attack
    #     t0 = time.time()
    #     adv_points, adv_target, query_costs = attack.run(points, target)
    #     t1 = time.time()
    #     avg_time_cost += t1 - t0
    #     print(f"Elapsed time: {t1 - t0:.2f} seconds")
    #     print("adv_target", adv_target.cpu().numpy())
    #     # # 创建一个点云对象
    #     # pcd = o3d.geometry.PointCloud()
    #     # # 去除大小为1的维度并转换为numpy数组
    #     # result_points = adv_points.squeeze(0).cpu().numpy().astype(np.float64)

    #     # pcd.points = o3d.utility.Vector3dVector(result_points)
    #     # adv_string = "output"+str(batch_id)+"_adv.pcd"
    #     # string = "output"+str(batch_id)+"_ori.pcd"

    #     # o3d.io.write_point_cloud(adv_string, pcd)
    #     # o3d.io.write_point_cloud(string, pcd)

    #     if not args.query_attack_method is None:
    #         print('>>>>>>>>>>>>>>>>>>>>>>>')
    #         print('Query cost: ', query_costs)
    #         print('>>>>>>>>>>>>>>>>>>>>>>>')
    #         avg_query_costs += query_costs
    #     atk_success += 1 if adv_target != target else 0

    #     # modified point num count
    #     points = points[:,:,:3].data # P, [1, N, 3]
    #     pert_pos = torch.where(abs(adv_points-points).sum(2))
    #     count_map = torch.zeros_like(points.sum(2))
    #     count_map[pert_pos] = 1.
    #     # print('Perturbed point num:', torch.sum(count_map).item())

    #     avg_mse_dist += np.sqrt(F.mse_loss(adv_points, points).detach().cpu().numpy() * 3072)
    #     avg_chamfer_dist += chamfer_loss(adv_points, points)
    #     avg_hausdorff_dist += hausdorff_loss(adv_points, points)

    # atk_success /= batch_id + 1
    # print('Attack success rate: ', atk_success)
    # avg_time_cost /= batch_id + 1
    # print('Average time cost: ', avg_time_cost)
    # if not args.query_attack_method is None:
    #     avg_query_costs /= batch_id + 1
    #     print('Average query cost: ', avg_query_costs)
    # avg_mse_dist /= batch_id + 1
    # print('Average MSE Dist:', avg_mse_dist)
    # avg_chamfer_dist /= batch_id + 1
    # print('Average Chamfer Dist:', avg_chamfer_dist.item())
    # avg_hausdorff_dist /= batch_id + 1
    # print('Average Hausdorff Dist:', avg_hausdorff_dist.item())





if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Shape-invariant 3D Adversarial Point Clouds')
    parser.add_argument('--batch-size', type=int, default=1, metavar='N', 
                        help='input batch size for training (default: 1)')
    parser.add_argument('--input_point_nums', type=int, default=1024,
                        help='Point nums of each point cloud')
    parser.add_argument('--seed', type=int, default=2022, metavar='S',
                        help='random seed (default: 2022)')
    parser.add_argument('--dataset', type=str, default='ModelNet40',
                        choices=['ModelNet40', 'ShapeNetPart'])
    parser.add_argument('--data_path', type=str, 
                        default='./data/modelnet40_normal_resampled/')
    parser.add_argument('--normal', action='store_true', default=False,
                        help='Whether to use normal information [default: False]')
    parser.add_argument('--num_workers', type=int, default=4,
                        help='Worker nums of data loading.')

    parser.add_argument('--transfer_attack_method', type=str, default=None,
                        choices=['ifgm_ours'])
    parser.add_argument('--query_attack_method', type=str, default=None,
                        choices=['simbapp', 'simba', 'ours'])
    parser.add_argument('--surrogate_model', type=str, default='pointnet_cls',
                        choices=['pointnet_cls', 'pointnet2_cls_msg', 'dgcnn', 'pointconv', 'pointcnn', 'paconv', 'pct', 'curvenet', 'simple_view'])
    parser.add_argument('--target_model', type=str, default='pointnet_cls',
                        choices=['pointnet_cls', 'pointnet2_cls_msg', 'dgcnn', 'pointconv', 'pointcnn', 'paconv', 'pct', 'curvenet', 'simple_view'])
    parser.add_argument('--defense_method', type=str, default=None,
                        choices=['sor', 'srs', 'dupnet'])
    parser.add_argument('--top5_attack', action='store_true', default=False,
                        help='Whether to attack the top-5 prediction [default: False]')

    parser.add_argument('--max_steps', default=50, type=int,
                        help='max iterations for black-box attack')
    parser.add_argument('--eps', default=0.16, type=float,
                        help='epsilon of perturbation')
    parser.add_argument('--step_size', default=0.07, type=float,
                        help='step-size of perturbation')
    args = parser.parse_args()

    # basic configuration
    set_seed(args.seed)
    args.device = torch.device("cuda")

    # main loop
    main()
