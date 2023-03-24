from .Rover import Rover
from .Mongo import find_position
import math
from time import sleep
from shapely.geometry import LineString, Point

# Reposition rover after it completes cleaning cycle
def reposition(rover:Rover, drone_collection):
    rover.setup_arm()
    rover.change_vehicle_mode('GUIDED')
    print('Turn Right')
    rover.change_yaw(angle=math.radians(90), speed=0.1)
    sleep(1)
    
    print('Move Forward')
    while rover.ul_front_edge.check_drive_ok() == True:
        print('moving forward')
        rover.move_forward(speed=1)
        sleep(2)

    print('Turn Right')
    rover.change_yaw(angle=math.radians(90), speed=0.1)
    sleep(1)

    print('finding drone position')
    lat, lon, angle = find_position(rover=rover, drone_collection=drone_collection)
    slope = math.tan(math.radians(angle))
    line = create_line(x=lon, y=lat, slope=slope)
    print('line created')

    while True:
        rover.update_rover()
        point = Point(rover.lon, rover.lat)
        if line.distance(point) < 0.000001:
            rover.change_yaw(angle=math.radians(90), speed=0.1)
            sleep(1)
            break

        if rover.ul_front_edge.check_drive_ok() == True:
            rover.move_forward(speed=0.1)
            sleep(2)
        else:
            print('Drone not found')
            break

def create_line(x, y, slope):
    line_x = [x - 10, x + 10]
    line_y = [y + (slope * (possible_x - x)) for possible_x in line_x]
    line = LineString([(line_x[0], line_y[0]), (line_x[1], line_y[1])])
    
    return line

if __name__ == '__main__':
    pass