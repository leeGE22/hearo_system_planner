import re
import random

import dao.db_interface as db


def Task_Allocation(context):    
    # context = {
    #     'big table1': {'idx': 1, 'situation': 'Study alone', 'position': {'x': 1.0, 'y': 2.0, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.7071, 'w': 0.7071}},
    #     'chair1': {'idx': 1, 'situation': 'Study alone', 'position': {'x': 2.0, 'y': 2.0, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.7071, 'w': 0.7071}}
    # }
    
    robot_ids = db.selectIdRobotByState('idle')
    if not all(isinstance(rid, (int)) for rid in robot_ids):
        return ''
    
    situation = context[next(iter(context))]['situation']
    
    tasks = []
    for key, value in context.items():
        _key = re.sub(r'\d+', '', key)
        tasks.append((_key, value.get('position'), value.get('orientation')))
    task_names = [task[0] for task in tasks]
    print(' tasks : ', task_names)
    
         
    # task_allocation
    alloc_result = [(rid, 0) for rid in robot_ids] # (robot_id, task_idx[1~m])
    positions = random.sample(range(len(robot_ids)), len(tasks))
    for num, pos in zip(list(range(1, len(tasks) + 1)), positions):
        alloc_result[pos] = (robot_ids[pos], num)
    print('result : ', alloc_result)
    
    result = {}
    for (r, t) in alloc_result:
        if t != 0:
            robot_ip = db.selectIpByIdRobot(r)
            result[r] = {'ip': robot_ip, 'situation': situation, 'task': tasks[t-1][0], 'position': tasks[t-1][1], 'orientation': tasks[t-1][2]}
        
    return result