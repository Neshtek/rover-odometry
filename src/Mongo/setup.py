import pymongo
from ..Rover import Rover

# Connect Mongo
def mongo_connect(mongo_url, database, collection):
    mc = pymongo.MongoClient(mongo_url)
    mydb = mc[database]
    data_collection = mydb[collection]
    return data_collection

def connect_rover_by_serial(rover:Rover, rover_collection):
    rover_data = rover_collection.find_one({'serial': rover.serial})
    if rover_data:
        rover.drone_serial = rover_data['droneSerial']
        update_rover_by_serial(rover=rover, rover_collection=rover_collection)
    else:
        insert_rover(rover=rover, rover_collection=rover_collection)
    
def update_rover_by_serial(rover:Rover, rover_collection):
    rover.update_rover()
    rover_collection.update_one({'serial': rover.serial}, {'$set': {'battery': rover.battery, 'location': {'lat': rover.lat, 'lon': rover.lon}}})
    print('DRONE UPDATED')

def insert_rover(rover:Rover, rover_collection):
    rover.update_rover()
    rover_collection.insert_one({'serial': rover.serial, 'battery': rover.battery, 'location': {'lat': rover.lat, 'lon': rover.lon}, 'takeOffStatus': False, 'userId': None, 'roverStatus':"Free"})
    print('DRONE ADDED')

def update_rover_status(rover:Rover, rover_collection, status):
    rover_collection.update_one({'serial': rover.serial}, {'$set': {'roverStatus': status}})
    rover.rover_status=status
    print('DRONE STATUS UPDATED')

def update_rover_status(rover:Rover, rover_collection, status):
    rover_collection.update_one({'serial': rover.rover_serial}, {'$set': {'roverStatus': status}})
    rover.rover_status=status
    print('ROVER STATUS UPDATED')    

# def get_rover_status(rover:Rover, rover_collection):
#     rover_document = rover_collection.find_one({'serial': rover.rover_serial})
#     rover_status = rover_document['roverStatus']
#     rover.rover_status = rover_status
#     print('ROVER STATUS UPDATED')

# def update_rover_takeoff_status(rover:Rover, rover_collection, takeoff_status):
#     rover_collection.update_one({'serial': rover.serial}, {'$set': {'takeoffStatus': takeoff_status}})
#     rover.takeoff_status = takeoff_status
#     print('Takeoff Status UPDATED')

def find_position(rover:Rover, drone_collection):
    data = drone_collection.find_one({'serial': rover.drone_serial})
    for key, val in data.items():
        if 'location' in key:
            lat = val['lat']
            lon = val['lon']
        if 'yaw' in key:
            angle = val
    return lat, lon, angle

if __name__ == '__main__':
    pass