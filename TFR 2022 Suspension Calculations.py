#TFR 2022 Suspension Calculations
#Author: Scott Blender (scott.blender@temple.edu)
#variables:
#Spring Rate: force per unit displacement for a suspension spring alone
#Wheel Center Rate: vertical force per unit vertical displacement at the location along the spinde corresponding to the wheel centerline, measured relative to the chassis
#Tire Rate: vertical force per unit vertical displacement of the tire at its operating load
#Ride Rate: vertical force per unit vertical displacement of the tire ground contact reference point relative to the chassis
#Roll Rate: moment(torque) resisting body roll per degree of body roll
#Assumptions for above defintions include zero degrees of camber and no lateral tire distortion.

#Car Specs //Make sure all values are converted to feet
weight_front = float(input("Enter front weight: "))
weight_rear = float(input("Enter rear weight: "))
weight_total = weight_front + weight_rear
track_front = float(input("Enter front track: "))
track_rear = float(input("Enter rear track: "))
wheelbase = float(input("Enter wheelbase: "))
cg_height = float(input("Enter cg height: "))
cg_to_roll_axis_front = float(input("Enter cg to roll axis front: "))
cg_to_roll_axis_rear = float(input("Enter cg to roll axis rear: "))
roadway_bank_angle = float(input("Enter roadway bank angle: ")) #degree of roll 
radius_of_vehicle_path = float(input("Enter radius of vehicle path: "))
speed = float(input("Enter speed: "))
roll_axis_height_front = float(input("Enter front roll axis height: "))
roll_axis_height_rear = float(input("Enter rear roll axis height: "))
drive_torque = float(input("Enter drive torque: ")) #torque
longitudinal_acceleration = float(input("Enter longitudinal acceleration: "))
g = 32.2
ride_travel = 1.75
roll_rate_front = 29564.6
roll_rate_rear = 29564.6
roll_rate_total = roll_rate_front + roll_rate_rear

#Functions to calculate variables
import math
def calc_roll_rate(torque, degrees_of_roll):
    roll_rate = torque/degrees_of_roll
    return roll_rate
def calc_horizontal_lateral_acceleration(speed, radius, gravity):
    horizontal_lateral_acceleration = (math.pow(speed, 2))/(radius*gravity)
    return horizontal_lateral_acceleration
def calc_lateral_acceleration(max_horizontal_acceleration, roadway_bank_angle):
    lateral_acceleration = (max_horizontal_acceleration*(math.cos(math.radians(roadway_bank_angle)))) - math.sin(math.radians(roadway_bank_angle))
    return lateral_acceleration
def calc_cg_position(weight_axle, weight_total, wheelbase):
    cg_position = (weight_axle * wheelbase)/weight_total
    return cg_position
def calc_effective_weight(weight_total, horizontal_lateral_acceleration, roadway_bank_angle):
    effective_weight = weight_total*((horizontal_lateral_acceleration*math.sin(math.radians(roadway_bank_angle))+math.cos(math.radians(roadway_bank_angle))))
    return effective_weight
def calc_effective_axle_weights(effective_weight, cg_position, wheelbase):
    effective_axle_weight = (effective_weight*cg_position)/wheelbase
    return effective_axle_weight
def calc_roll_gradient(weight_total, cg_to_roll_axis, roll_rate_front, roll_rate_rear):
    roll_gradient = (-1*weight_total*cg_to_roll_axis)/(roll_rate_front + roll_rate_rear)
    return roll_gradient
def calc_weight_transfer(lateral_acceleration, track, weight, distance_from_cg_to_roll_axis, k_phi_axle, k_phi_front_plus_rear, distance_from_wheels_to_cg, wheelbase, roll_axis_height):
    weight_transfer = lateral_acceleration*(weight/track)*((((distance_from_cg_to_roll_axis*k_phi_axle)/(k_phi_front_plus_rear))+(distance_from_wheels_to_cg/wheelbase)*roll_axis_height))
    return weight_transfer
def calc_tire_load_outside(weight_axle, load_transfer):
    tire_load = (weight_axle/2) - load_transfer
    return tire_load
def calc_tire_load_inside(weight_axle, load_transfer):
    tire_load = (weight_axle/2) + load_transfer
    return tire_load
def calc_static_load_change(tire_load, weight_axle):
    static_load_change = tire_load - weight_axle/2 
    return static_load_change
def calc_ride_rate(load_change_of_wheel_in_cornering, ride_travel):
    ride_rate = load_change_of_wheel_in_cornering/ride_travel
    return ride_rate
def calc_ride_frequency(ride_rate, weight_2):
    ride_frequency = (1/(2*math.pi))*math.sqrt((ride_rate * 12 * g)/weight_2)
    return ride_frequency
def calc_new_ride_rate(ride_frequency_new, ride_frequency_old, ride_rate):
    new_ride_rate = math.pow((ride_frequency_new/ride_frequency_old),2)*ride_rate
    return new_ride_rate
def calc_new_roll_rate(new_ride_rate, track):
    new_roll_rate = (12*new_ride_rate*math.pow(track, 2))/2
    return new_roll_rate
def calc_wheel_center_rate(new_ride_rate, vertical_tire_rate):
    wheel_center_rate = (new_ride_rate*vertical_tire_rate)/(vertical_tire_rate-new_ride_rate)
    return wheel_center_rate
def calc_additional_roll_rate_needed(desired_total_roll_rate, wheel_center_rate, track, tire_rate):
    additional_roll_rate_needed = ((desired_total_roll_rate*12*tire_rate*(math.pow(track,2)/2)/(12*tire_rate*(math.pow(track,2)/2))-desired_total_roll_rate))-(12*wheel_center_rate*(math.pow(track,2)/2))
    return additional_roll_rate_needed
#Calculations
horizontal_lateral_acceleration = calc_horizontal_lateral_acceleration(speed, radius_of_vehicle_path, g)
lateral_acceleration_in_car_axis_system = calc_lateral_acceleration(horizontal_lateral_acceleration, roadway_bank_angle)
print("Lateral Accleration is: " + str(lateral_acceleration_in_car_axis_system))
roll_gradient = calc_roll_gradient(weight_total, ((cg_to_roll_axis_front + cg_to_roll_axis_rear)/2), roll_rate_front, roll_rate_rear)
print("Roll Gradient is: "+ str(roll_gradient))
cg_position_front = calc_cg_position(weight_front, weight_total, wheelbase)
cg_position_rear =  calc_cg_position(weight_rear, weight_total, wheelbase)
effective_weight = calc_effective_weight(weight_total, horizontal_lateral_acceleration, roadway_bank_angle)
effective_weight_front = calc_effective_axle_weights(effective_weight, cg_position_front, wheelbase)
effective_weight_rear = calc_effective_axle_weights(effective_weight, cg_position_rear, wheelbase)
lateral_load_transfer_front = calc_weight_transfer(lateral_acceleration_in_car_axis_system, track_front, weight_total, cg_to_roll_axis_front, roll_rate_front, roll_rate_total, cg_position_front, wheelbase, roll_axis_height_front)
lateral_load_transfer_rear = calc_weight_transfer(lateral_acceleration_in_car_axis_system, track_rear, weight_total, cg_to_roll_axis_rear, roll_rate_rear, roll_rate_total, cg_position_rear, wheelbase, roll_axis_height_rear)
print("Front Lateral Load Transfer is: " + str(lateral_load_transfer_front))
print("Rear Lateral Load Transfer is: " + str(lateral_load_transfer_rear))
tire_load_front_oustide = calc_tire_load_outside(effective_weight_front, lateral_load_transfer_front)
tire_load_front_inside = calc_tire_load_inside(effective_weight_front, lateral_load_transfer_front)
tire_load_rear_outside = calc_tire_load_outside(effective_weight_rear, lateral_load_transfer_rear)
tire_load_rear_inside = calc_tire_load_inside(effective_weight_rear, lateral_load_transfer_rear)
change_in_static_load_front_outside = calc_static_load_change(tire_load_front_oustide, weight_front)
change_in_static_load_front_inside = calc_static_load_change(tire_load_front_inside, weight_front)
change_in_static_load_rear_outside = calc_static_load_change(tire_load_rear_outside, weight_rear)
change_in_static_load_rear_inside =  calc_static_load_change(tire_load_rear_inside, weight_rear)
print("Front Outside Change in Static Load is: " + str(change_in_static_load_front_outside))
print("Front Inside Change in Static Load is: " + str(change_in_static_load_front_inside))
print("Rear Outside Change in Static Load is: " + str(change_in_static_load_rear_outside))
print("Rear Inside Change in Static Load is: " + str(change_in_static_load_rear_inside))
ride_rate_front = calc_ride_rate(change_in_static_load_front_outside, ride_travel)
ride_rate_rear = calc_ride_rate(change_in_static_load_rear_outside, ride_travel)
ride_frequency_front = calc_ride_frequency(ride_rate_front, weight_front/2)*60
ride_frequency_rear = calc_ride_frequency(ride_rate_rear, weight_rear/2)*60
print("Ride Rate Front is: " + str(ride_rate_front))
print("Ride Rate Rear is: " + str(ride_rate_rear))
print("Ride Frequency Front is: " + str(ride_frequency_front))
print("Ride Frequency Rear is: " + str(ride_frequency_rear))
ride_frequency_front_new = 114
ride_frequency_rear_new = 126
ride_rate_front_new = calc_new_ride_rate(ride_frequency_front_new, ride_frequency_front, ride_rate_front)
ride_rate_rear_new = calc_new_ride_rate(ride_frequency_rear_new, ride_frequency_rear, ride_rate_rear)
roll_rate_front_new = calc_new_roll_rate(ride_rate_front_new, track_front)
roll_rate_rear_new = calc_new_roll_rate(ride_rate_rear_new, track_rear)
roll_rate_total_new = roll_rate_front_new + roll_rate_rear_new
roll_gradient_new = calc_roll_gradient(weight_total, ((cg_to_roll_axis_front + cg_to_roll_axis_rear)/2), roll_rate_front_new, roll_rate_rear_new)
print("Roll Rate Front is: " + str(roll_rate_front_new))
print("Roll Rate Rear is: " + str(roll_rate_rear_new))
print("Roll Gradient is:" + str(roll_gradient_new))
lateral_load_transfer_front_new = calc_weight_transfer(lateral_acceleration_in_car_axis_system, track_front, weight_total, cg_to_roll_axis_front, roll_rate_front_new, roll_rate_total_new, cg_position_front, wheelbase, roll_axis_height_front)
lateral_load_transfer_rear_new = calc_weight_transfer(lateral_acceleration_in_car_axis_system, track_rear, weight_total, cg_to_roll_axis_rear, roll_rate_rear_new, roll_rate_total_new, cg_position_rear, wheelbase, roll_axis_height_rear)
print("Front Lateral Load Transfer is: " + str(lateral_load_transfer_front_new))
print("Rear Lateral Load Transfer is: " + str(lateral_load_transfer_rear_new))
tire_load_front_oustide = calc_tire_load_outside(effective_weight_front, lateral_load_transfer_front_new)
tire_load_front_inside = calc_tire_load_inside(effective_weight_front, lateral_load_transfer_front_new)
tire_load_rear_outside = calc_tire_load_outside(effective_weight_rear, lateral_load_transfer_rear_new)
tire_load_rear_inside = calc_tire_load_inside(effective_weight_rear, lateral_load_transfer_rear_new)
change_in_static_load_front_outside = calc_static_load_change(tire_load_front_oustide, weight_front)
change_in_static_load_front_inside = calc_static_load_change(tire_load_front_inside, weight_front)
change_in_static_load_rear_outside = calc_static_load_change(tire_load_rear_outside, weight_rear)
change_in_static_load_rear_inside =  calc_static_load_change(tire_load_rear_inside, weight_rear)
print("Front Outside Change in Static Load is: " + str(change_in_static_load_front_outside))
print("Front Inside Change in Static Load is: " + str(change_in_static_load_front_inside))
print("Rear Outside Change in Static Load is: " + str(change_in_static_load_rear_outside))
print("Rear Inside Change in Static Load is: " + str(change_in_static_load_rear_inside))