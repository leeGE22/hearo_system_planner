import os
from dotenv import load_dotenv

# .env 파일에서 환경 변수 로드
load_dotenv()

# MySQL 접속 정보를 환경 변수로부터 읽어옴
mysql_config = {
    "host": os.getenv('DB_HOST'),
    "user": os.getenv('DB_USER'),
    "password": os.getenv('DB_PASSWORD'),
    "database": os.getenv('DATABASE')
}

# bitlog_config = {
#     "host": mysql_config["host"],
#     "user": mysql_config["user"],
#     "passwd": mysql_config["password"],
#     "port": os.getenv('DB_PORT')
# }