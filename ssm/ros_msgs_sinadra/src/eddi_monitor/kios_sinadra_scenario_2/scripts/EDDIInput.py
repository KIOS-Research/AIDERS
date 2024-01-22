

class EDDIInput:
    def __init__(self):
        self.dense_building_area = True
        self.highest_temperature_in_area = 60.0
        self.risk_of_fire_and_explosions_in_area = True
        self.area_distance_to_catastrophy_epicenter = 10.0

    def process_dense_building_area (self, out_0):
        self.dense_building_area = out_0.data

    def process_highest_temperature_in_area (self, out_1):
        self.highest_temperature_in_area = out_1.data

    def process_risk_of_fire_and_explosions_in_area (self, out_2):
        self.risk_of_fire_and_explosions_in_area = out_2.data

    def process_area_distance_to_catastrophy_epicenter (self, out_3):
        self.area_distance_to_catastrophy_epicenter = out_3.data


    def processing (self, simulator_outputs):
        if 'out_0' in simulator_outputs:
            self.process_dense_building_area(simulator_outputs['out_0'])
        else:
            self.dense_building_area = True

        if 'out_1' in simulator_outputs:
            self.process_highest_temperature_in_area(simulator_outputs['out_1'])
        else:
            self.highest_temperature_in_area = 60.0

        if 'out_2' in simulator_outputs:
            self.process_risk_of_fire_and_explosions_in_area(simulator_outputs['out_2'])
        else:
            self.risk_of_fire_and_explosions_in_area = True

        if 'out_3' in simulator_outputs:
            self.process_area_distance_to_catastrophy_epicenter(simulator_outputs['out_3'])
        else:
            self.area_distance_to_catastrophy_epicenter = 10.0

