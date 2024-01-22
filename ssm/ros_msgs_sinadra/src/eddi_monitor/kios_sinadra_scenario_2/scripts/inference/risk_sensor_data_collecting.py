from typing import List, Dict, TYPE_CHECKING
import importlib

if TYPE_CHECKING:
    from inference.interfaces import BayesianNetworkData, BayesianNetworkInputFeatureData


class RiskSensorDataBuilder:
    """Provides usability to build the Bayesian network input feature interfaces for several vehicles based on input
    from the CARLA simulator.
    """

    def __init__(self) -> None:
        """Initializes the object and sets the package location of the SINADRA risk sensor data interface classes."""
        self._risk_data_module = importlib.import_module(".sinadra_risk_sensor_data",
                                                         package="inference")

    def collect_data_and_build_risk_sensor_data(self, bayesian_network_data: List["BayesianNetworkData"],
                                                input_feature_data: Dict[str, "BayesianNetworkInputFeatureData"],
                                                ) -> List["BayesianNetworkData"]:
        """Builds the correct SINADRA risk sensor data input feature data interface for each Bayesian network for each 
        vehicle. Uses the CARLA input feature data for the feature extraction and adds the built and updated SINADRA 
        risk sensor data input feature instances to the Bayesian network data instance of the corresponding vehicle.
        
        Parameters
        ----------
        bayesian_network_data : List[BayesianNetworkData]
            List of the Bayesian network data for all vehicles. Used for the mapping from vehicle states to Bayesian 
            networks each.
        input_feature_data : List[BayesianNetworkInputFeatureData]
            CARLA input data for each vehicle which is used for extracting the situation-specific Bayesian network input 
            features.

        Returns
        -------
        List[BayesianNetworkData]
            List of the updated Bayesian network data instances for each vehicle. In case of a recognized Bayesian 
            network for a given vehicle state (in a situation class) the SINADRA risk sensor input feature data 
            instance is added. 
        """

        for bn_data in bayesian_network_data:
            bayesian_network_id = bn_data.bayesian_network_id
            if bayesian_network_id:
                actor_id = bn_data.actor_id
                starts_with_number = str.isnumeric(bayesian_network_id[0])
                corrected_bn_id = f"Data{bayesian_network_id}" if starts_with_number else bayesian_network_id
                corrected_bn_id = corrected_bn_id.replace(" ", "")
                risk_data_class = getattr(self._risk_data_module, corrected_bn_id)
                risk_data_instance = risk_data_class()
                risk_data_instance.collect_data(input_feature_data[actor_id])
                bn_data.sinadra_risk_sensor_data = risk_data_instance

        return bayesian_network_data

