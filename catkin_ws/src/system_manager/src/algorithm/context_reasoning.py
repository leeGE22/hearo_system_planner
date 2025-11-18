import dao.db_interface as db


def Context_Reasoning(command):
    # command = Study alone
    command_lower = command.lower()
    
    task_list = db.selectTaskName()

    context = {}
    if 'study' in command_lower and 'alone' in command_lower:
        need_task = ['big table', 'chair']
        need_task = list(dict.fromkeys(need_task)) # 중복 제거 + 순서 유지
        
        if not set(need_task).issubset(set(task_list)):
            context = ''
        else:
            context[need_task[0]+'1'] = {'idx': 1, 'situation': command, 'position': {'x': 1.0, 'y': 2.0, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.7071, 'w': 0.7071}}
            context[need_task[1]+'1'] = {'idx': 1, 'situation': command, 'position': {'x': 2.0, 'y': 2.0, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.7071, 'w': 0.7071}}
    elif 'study' in command_lower and 'friend' in command_lower:
        need_task = ['small table', 'chair']
        need_task = list(dict.fromkeys(need_task)) # 중복 제거 + 순서 유지
        
        if not set(need_task).issubset(set(task_list)):
            context = ''
        else:
            context[need_task[0]+'1'] = {'idx': 1, 'situation': command, 'position': {'x': 5.0, 'y': 2.0, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}
            context[need_task[0]+'2'] = {'idx': 2, 'situation': command, 'position': {'x': 5.0, 'y': 4.0, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}
            context[need_task[1]+'1'] = {'idx': 1, 'situation': command, 'position': {'x': 7.0, 'y': 2.0, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}
            context[need_task[1]+'2'] = {'idx': 2, 'situation': command, 'position': {'x': 7.0, 'y': 4.0, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}
    elif 'talk' in command_lower and 'friend' in command_lower:
        need_task = ['big table', 'chair']
        need_task = list(dict.fromkeys(need_task)) # 중복 제거 + 순서 유지
        
        if not set(need_task).issubset(set(task_list)):
            context = ''
        else:
            context[need_task[0]+'1'] = {'idx': 1, 'situation': command, 'position': {'x': 1.0, 'y': 2.0, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}
            context[need_task[1]+'1'] = {'idx': 2, 'situation': command, 'position': {'x': 1.0, 'y': 2.0, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}
            context[need_task[1]+'2'] = {'idx': 2, 'situation': command, 'position': {'x': 1.0, 'y': 2.0, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}
        
    return context