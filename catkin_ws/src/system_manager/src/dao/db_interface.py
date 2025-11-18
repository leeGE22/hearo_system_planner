from dao.db.connection import connect_to_mysql
from dao.db.config import mysql_config


def select(table_name, field_name, conditions=None):
    try:
        cnx = connect_to_mysql(mysql_config, attempts=3)

        with cnx.cursor(buffered=True) as cur:
            if conditions:
                query = f"SELECT {field_name} FROM {table_name} WHERE {conditions}"
            else:
                query = f"SELECT {field_name} FROM {table_name}"
            cur.execute(query)

            result = cur.fetchall()

            return result

    except mysql.connector.Error as err:
        print(f"Error: {err}")
        return []

    finally:
        if cnx.is_connected():
            cnx.close()
            
def update(table_name, update_data, conditions):
    try:
        cnx = connect_to_mysql(mysql_config, attempts=3)

        with cnx.cursor(buffered=True) as cur:
            query = f"UPDATE {table_name} SET {update_data} WHERE {conditions}"
            cur.execute(query)

            cnx.commit()
            return True

    except mysql.connector.Error as err:
        print(f"Error: {err}")
        return False

    finally:
        if cnx.is_connected():
            cnx.close()

""" ------------------------------------------------------------------------------------------------------ """

"""
GET ip by idRobot
"""
def selectIpByIdRobot(id):
    result = select(table_name='tb_robot', field_name='ip', conditions=f'idRobot = "{id}"')
    if result:
        return result[0][0]
    else:
        return result

"""
GET task by idRobot
"""
def selectTaskByIdRobot(id):
    result = select(table_name='tb_robot', field_name='task', conditions=f'idRobot = "{id}"')
    if result:
        return result[0][0]
    else:
        return result

"""
GET posInit by idRobot
"""
def selectPosInitByIdRobot(id):
    result = select(table_name='tb_robot', field_name='posInit', conditions=f'idRobot = "{id}"')
    if result:
        return result[0][0]
    else:
        return result

"""
GET idRobots by situation
"""
def selectIdRobotBySituation(situation):
    result = select(table_name='tb_robot', field_name='idRobot', conditions=f'situation = "{situation}"')
    if result:
        return [r[0] for r in result]
    else:
        return result

"""
GET idRobots by state
"""
def selectIdRobotByState(state):
    result = select(table_name='tb_robot', field_name='idRobot', conditions=f'state = "{state}"')
    if result:
        return [r[0] for r in result]
    else:
        return result

"""
GET idRobots by situation and state
"""
def selectIdRobotBySituationNState(situation, state):
    result = select(table_name='tb_robot', field_name='idRobot', conditions=f'situation = "{situation}" AND state = "{state}"')
    if result:
        return [r[0] for r in result]
    else:
        return result

"""
GET all idRobots
"""
def selectIdRobot():
    result = select(table_name='tb_robot', field_name='idRobot')
    if result:
        return [r[0] for r in result]
    else:
        return result

"""
GET all taskNames
"""
def selectTaskName():
    result = select(table_name='tb_task', field_name='taskName')
    if result:
        return [r[0] for r in result]
    else:
        return result

"""
UPDATE situation by idRobot
"""
def updateSituationByIdRobot(id, situation):
    result = update(table_name='tb_robot', update_data=f'situation = "{situation}"', conditions=f'idRobot = "{id}"')
    return result

"""
UPDATE task by idRobot
"""
def updateTaskByIdRobot(id, task):
    if task == None:
        result = update(table_name='tb_robot', update_data=f'task = NULL', conditions=f'idRobot = "{id}"')
    else:
        result = update(table_name='tb_robot', update_data=f'task = "{task}"', conditions=f'idRobot = "{id}"')
    return result

"""
UPDATE state by idRobot
"""
def updateStateByIdRobot(id, state):
    result = update(table_name='tb_robot', update_data=f'state = "{state}"', conditions=f'idRobot = "{id}"')
    return result
