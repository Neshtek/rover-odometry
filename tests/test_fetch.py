import pymongo
import os
from os.path import join, dirname
from dotenv import load_dotenv

dotenv_path = join(dirname(__file__), '../','.env')
load_dotenv(dotenv_path)

MONGODB_ATLAS_URL = os.getenv('MONGODB_ATLAS_URL')
DATABASE = os.getenv('DATABASE')
DRONE_COLLECTION = os.getenv('DRONE_COLLECTION')
ROVER_COLLECTION = os.getenv('ROVER_COLLECTION')

url = MONGODB_ATLAS_URL
db = DATABASE
drone_col = DRONE_COLLECTION
rover_col = ROVER_COLLECTION
serial = "ERROR000000000"

mc = pymongo.MongoClient(url)
mydb = mc[db]
drone_collection = mydb[drone_col]

data = drone_collection.find_one({'serial': serial})
for key, val in data.items():
    if 'location' in key:
        print(val)
        print(val['lat'])