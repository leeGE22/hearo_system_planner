import os
from dotenv import load_dotenv


load_dotenv("/home/kist/ros2_ws/src/robot_bridge/robot_bridge/.env")

# MySQL 접속 정보를 환경 변수로부터 읽어옴
mysql_config = {
    "host": os.getenv('DB_HOST'),
    "user": os.getenv('DB_USER'),
    "password": os.getenv('DB_PASSWORD'),
    "database": os.getenv('DATABASE')
}

# print("Loaded DB_HOST:", os.getenv("DB_HOST"))
# print("Loaded DB_USER:", os.getenv("DB_USER"))

# bitlog_config = {
#     "host": mysql_config["host"],
#     "user": mysql_config["user"],
#     "passwd": mysql_config["password"],
#     "port": os.getenv('DB_PORT')
# }