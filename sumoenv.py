import os
import sys
import traci
import numpy as np


class SumoEnv:
    # place_len = 7.5
    # place_offset = 8.50
    # lane_len = 10
    # lane_ids = ['-gneE0_0', '-gneE0_1', '-gneE1_0', '-gneE1_1', '-gneE2_0', '-gneE2_1', '-gneE3_0', '-gneE3_1']


    def __init__(self, label='default', gui_f=False):
        self.label = label
        self.wait_time_next = 0.
        self.ncars = 0

        if 'SUMO_HOME' in os.environ:
            tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
            sys.path.append(tools)
        else:
            sys.exit("please declare environment variable 'SUMO_HOME'")

        exe = 'sumo-gui.exe' if gui_f else 'sumo.exe'
        sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin', exe)
        self.sumoCmd = [sumoBinary, '-c', 'intersection.sumocfg','--start', '--quit-on-end']

        return

    def __carbox(self):
        LANE = ["-x2_0", "-x2_1", "-x2_2", "-x2_3",
                "x1_0", "x1_1", "x1_2", "x1_3",
                "Y1_0", "Y1_1", "Y1_2", "Y1_3", "Y1_4", "Y1_5",
                "-Y2_0", "-Y2_1", "-Y2_2", "-Y2_3", "-Y2_4", "-Y2_5"]
        y = 30  # 矩阵的列，检测长度150m，元胞5m
        x = 20  # 矩阵的行，20个进口道
        length = 5  # 网格长度
        v_max = 20  # 最大速度
        carbox = np.array(np.zeros((1, x, y, 2)), dtype="float32")  # 创造全0数组,2层(位置和速度
        for carID in traci.vehicle.getIDList():
            v_lane = traci.vehicle.getLaneID(carID)  # 获取车辆所在车道ID
            if v_lane in LANE:
                index_ = LANE.index(v_lane)  # 获取车道下标，得到矩阵的行所在位置
                v_distance = traci.vehicle.getDistance(carID)
                delt_distance = traci.lane.getLength(v_lane) - v_distance  # 获取矩阵的列所在位置
                if delt_distance <= y * length:  # 在检测范围内
                    v_x = index_
                    v_y = math.floor(delt_distance / length)
                    carbox[0][v_x][v_y][0] = 1  # 位置矩阵
                    carbox[0][v_x][v_y][1] = traci.vehicle.getSpeed(carID) / v_max  # 速度归一化值
        return carbox

    def reward_1(decide, action_decide, wait_time):
        k1, k2 = -0.25, -1
        wait_time_next, r_phase, num = 0, 0, 0
        wait_time_next = 0

        LANE = ["-x2_0", "-x2_1", "-x2_2", "-x2_3",
                "x1_0", "x1_1", "x1_2", "x1_3",
                "Y1_0", "Y1_1", "Y1_2", "Y1_3", "Y1_4", "Y1_5",
                "-Y2_0", "-Y2_1", "-Y2_2", "-Y2_3", "-Y2_4", "-Y2_5"]
        for i in range(len(LANE)):
            # 累计等待时间
            wait_time_next += traci.lane.getWaitingTime(LANE[i])
        r_wait_time = wait_time_next - wait_time
            #刹车数量之和
        for car in traci.vehicle.getIDList():
            if traci.vehicle.getLaneID(car) in LANE and traci.vehicle.getAcceleration(car) < 0:
                num += 1
        # 切换相位
        if decide != action_decide:
            r_phase = 1
        reward = k1 * r_wait_time + k2 * num
        return reward, wait_time_next



    def reset(self):
        self.wait_time_next = 0.
        self.ncars = 0
        traci.start(self.sumoCmd, label=self.label)
        traci.trafficlight.setProgram('J6', '0')
        traci.simulationStep()
        return self.__carbox()

    def close(self):
        traci.close()
