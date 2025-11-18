#!/usr/bin/env python3

import rospy
import json
import os
from datetime import datetime, timedelta
from collections import defaultdict

from std_msgs.msg import String

import dao.db_interface as db


LOG_DIR = '/home/kist/catkin_ws/src/system_manager/src/log'
FILE_NAME = ''

def task_msg_callback(msg):
    # 수신 시간 (UTC 기준)
    receive_time = rospy.get_rostime()
    
    # UTC → 한국시간 변환
    dt = datetime.utcfromtimestamp(receive_time.to_sec()) + timedelta(hours=9)
    time_str = dt.strftime('%H:%M:%S')

    # rospy.loginfo("수신1 시각 (KST): %s", dt.strftime('%Y-%m-%d %H:%M:%S'))

    try:
        # JSON --> ROS
        ros_msg = json.loads(msg.data)
        
        log_file = open(FILE_NAME, 'a')
        
        if isinstance(ros_msg, str):
            if ros_msg.lower() == 'quit':
                log_file.write(f"[{time_str}] Quit System Manager\n")
                log_file.flush()
                
                rospy.signal_shutdown("Quit command received")

        task = {}
        robot_list = []
        finish = 0
        for k, v in ros_msg.items():
            try:
                task[int(k)] = v
                
                if v['task'] == None:
                    finish = 1
                    robot_list = db.selectIdRobotByState('work')
                    situation = 'Finish ' + next(iter(task.values()))['situation']
                else:
                    finish = 0
                    robot_list = db.selectIdRobotByState('idle') + db.selectIdRobotBySituationNState('Finish', 'move')
                    situation = next(iter(task.values()))['situation']
                    
            except ValueError:
                rospy.logerr(f"[System Manager] Wrong Robot Id Input")
                return
            
        for key in task:
            if key not in robot_list or any(k not in task[key] for k in ['ip', 'situation', 'task', 'position', 'orientation']):
                return
        
        ids = defaultdict(set)
        for key, val in task.items():
            ids[val['task']].add(key)
            
        log_file.write(f"[{time_str}] {situation}\n")
        for k, i in ids.items():
            robot_ids = sorted(i)
            if len(robot_ids) == 1:
                robot_str = f"Robot {robot_ids[0]}"
            else:
                robot_str = f"Robots {', '.join(map(str, robot_ids))}"
            
            if finish:
                log_file.write(f"[{time_str}] {robot_str} returned to initial positions\n")
            else:
                log_file.write(f"[{time_str}] {robot_str} assigned to '{k}' task\n")
        log_file.flush()
    
    except Exception as e:
        rospy.logerr(f"[System Manager] An unexpected error occurred: {str(e)}")


if __name__ == '__main__':    
    rospy.init_node('save_data')
    
    # 한국 시간 기준 오늘 날짜
    kst_now = datetime.utcnow() + timedelta(hours=9)
    today_str = kst_now.strftime('%Y-%m-%d')

    # 오늘 날짜로 파일 생성
    os.makedirs(LOG_DIR, exist_ok=True)    
    FILE_NAME = os.path.join(LOG_DIR, f'{today_str}.log')
    
    rospy.Subscriber("/system_manager/task_msg", String, task_msg_callback)
    
    rospy.spin()