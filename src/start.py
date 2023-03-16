from src.reposition import reposition
from .Rover import Rover
from .Mongo import connect_rover_by_serial

def init_rover_on_mongo(rover:Rover, data_collection):
    connect_rover_by_serial(rover=rover, data_collection=data_collection)

def main_start(serial=None, connection=None, drone_collection=None, rover_collection=None):
    if serial != None:
        print(serial)
        rover = Rover(rover_serial=serial, connection=connection)
        print(rover)
        
        init_rover_on_mongo(rover=rover, data_collection=rover_collection)
        
        reposition(rover=rover, drone_collection=drone_collection)
                   
if __name__ == '__main__':
    pass
else:
    main_start()