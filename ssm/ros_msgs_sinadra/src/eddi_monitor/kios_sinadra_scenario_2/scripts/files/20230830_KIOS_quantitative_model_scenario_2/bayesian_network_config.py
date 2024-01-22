class BayesianNetworkConfig:
    """Bayesian network config to change the node's outcomes dependent
    on the received DRA data.
    """

    # ===== Initialization methods =====================================

    def __init__(self) -> None:
        """Initialize the (simulator's) DRA data."""
        self.dra_data = None

    # ===== Update methods =============================================

    def update_dra_data(self, dra_data: "SINADRARiskSensorData") -> None:
        """Updates the saved DRA input feature data class instance.

        Parameters
        ----------
        dra_data : SINADRARiskSensorData
            Latest DRA input feature data class instance for the Bayesian network.
        """
        self.dra_data = dra_data.data

    # ===== Bayesian network's node methods ============================

    def node_Density_of_buildings_that_could_collide_due_to_the_disaster(self):
        """Method for the CPT node 'Density_of_buildings_that_could_collide_due_to_the_disaster' that allows setting the node as output node, 
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys 
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_Dense = 0.0
        outcome_Broad = 0.0
        '''----------Manual edit begin----------'''
        if self.dra_data.dense_building_area:
            outcome_Dense = 1.0
        else:
            outcome_Broad = 1.0
        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'Dense': outcome_Dense,
            'Broad': outcome_Broad}

    def node_Distance_to_the_disaster_epicenter(self):
        """Method for the CPT node 'Distance_to_the_disaster_epicenter' that allows setting the node as output node, 
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys 
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_Close = 0.0
        outcome_Far = 0.0
        '''----------Manual edit begin----------'''
        if self.dra_data.area_distance_to_catastrophy_epicenter <= 500:
            outcome_Close = 1.0
        else:
            outcome_Far = 1.0
        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'Close': outcome_Close,
            'Far': outcome_Far}

    def node_Extreme_temperatures(self):
        """Method for the CPT node 'Extreme_temperatures' that allows setting the node as output node, 
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys 
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_Hot = 0.0
        outcome_Normal = 0.0
        '''----------Manual edit begin----------'''
        if self.dra_data.highest_temperature_in_area >= 50.0:
            outcome_Hot = 1.0
        else:
            outcome_Normal = 1.0
        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'Hot': outcome_Hot,
            'Normal': outcome_Normal}

    def node_Increased_risk_of_fire_or_explosions(self):
        """Method for the CPT node 'Increased_risk_of_fire_or_explosions' that allows setting the node as output node, 
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys 
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_Yes = 0.0
        outcome_No = 0.0
        '''----------Manual edit begin----------'''
        if self.dra_data.risk_of_fire_and_explosions_in_area:
            outcome_Yes = 1.0
        else:
            outcome_No = 1.0
        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'Yes': outcome_Yes,
            'No': outcome_No}

    def node_Expected_human_injury_criticality_in_area(self):
        """Method for the CPT node 'Expected_human_injury_criticality_in_area' that allows setting the node as output node, 
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys 
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_High = 0.0
        outcome_Low = 0.0
        '''----------Manual edit begin----------'''
        is_bn_output = True
        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'High': outcome_High,
            'Low': outcome_Low}
