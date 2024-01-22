import math
import time

class DroneMission:
    def __init__(self, initial_position, initial_altitude, target_coordinates, speed, move_frequency):
        # self.initial_position = initial_position
        # self.initial_altitude = initial_altitude
        self.target_coordinates = target_coordinates
        self.speed = speed
        self.vertical_speed = 1
        self.move_frequency = move_frequency
        self.current_time = 0
        self.current_target_index = 0
        self.current_position = initial_position
        self.current_altitude = initial_altitude
        self.heading = 0
        self.is_paused = False
        self.is_over = False
    

    def update(self):
        self.check_if_over()

        if self.is_over or self.is_paused:
            return self.current_position[0], self.current_position[1], self.current_altitude, self.heading
            
        current_target_pos = (self.target_coordinates[self.current_target_index][0], self.target_coordinates[self.current_target_index][1])
        current_target_altitude = self.target_coordinates[self.current_target_index][2]
        next_target_index = self.current_target_index + 1

        # Calculate the heading before updating the position
        dx = current_target_pos[0] - self.current_position[0]
        dy = current_target_pos[1] - self.current_position[1]
        self.heading = math.degrees(math.atan2(dy, dx))

        lat_b, lon_b = current_target_pos
        distance = self.speed  # Distance to cover in this iteration
        
        self.move_towards(self.current_position, current_target_pos, distance, current_target_altitude)
        
        
        distance_to_current_target = math.sqrt((self.current_position[0] - lat_b)**2 + (self.current_position[1] - lon_b)**2) * 111000
        if distance_to_current_target < 6:
            # print(f"reached target {next_target_index}!")
            self.current_target_index = next_target_index
               
        self.current_time += distance / self.speed

        return self.current_position[0], self.current_position[1], self.current_altitude, self.heading


    def move_towards(self, point_a, point_b, distance, target_altitude):
        lat_a, lon_a = point_a
        lat_b, lon_b = point_b
        
        total_distance = math.sqrt((lat_b - lat_a)**2 + (lon_b - lon_a)**2) * 111000
        if total_distance == 0:
            return point_b
                              
        interpolation_factor = distance / total_distance
        new_lat = lat_a + interpolation_factor * (lat_b - lat_a)
        new_lon = lon_a + interpolation_factor * (lon_b - lon_a)
        self.current_position = (new_lat, new_lon)
        
        if abs(self.current_altitude - target_altitude) > 0.5:
            if self.current_altitude < target_altitude:
                self.current_altitude += self.vertical_speed / self.move_frequency # go up
            else:
                self.current_altitude -= self.vertical_speed / self.move_frequency # go down
        else:
            self.current_altitude = target_altitude
    

    def check_if_over(self):
        self.is_over = self.current_target_index >= len(self.target_coordinates)
    

    def cancel(self):
        self.current_target_index = len(self.target_coordinates)

    
    def pause(self):
        self.is_paused = True


    def resume(self):
        self.is_paused = False
