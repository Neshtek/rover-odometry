import argparse
from src.Mongo import mongo_connect
import src.settings as settings
import src.start as start

def main():
    url = settings.MONGODB_ATLAS_URL
    db = settings.DATABASE
    drone_col = settings.DRONE_COLLECTION
    rover_col = settings.ROVER_COLLECTION

    drone_collection = mongo_connect(mongo_url=url, database=db, collection=drone_col)
    rover_collection = mongo_connect(mongo_url=url, database=db, collection=rover_col)

    serial = settings.get_serial()
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default='127.0.0.1:14550')
    args = parser.parse_args()

    print ('Connecting to vehicle on: %s' % args.connect)
    start.main_start(serial=serial, connection=args.connect, drone_collection=drone_collection, rover_collection=rover_collection)

if __name__ == '__main__':
    main()