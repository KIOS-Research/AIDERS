#####
# For each Bayesian network there must be a subclass of
# SINADRARiskSensorData that includes all required data for this
# specific BN.
# The specific BN data class will be the data interface for the
# corresponding BN config class (allowing setting dynamic evidences
# based on this data).
#
# Naming Convention:
# The data class shall be named the same as the Bayesian network.
# (= name of the folder with the Bayesian network's files)
# In case of leading numbers (not allowed in Python), the prefix "Data"
# is added.
# Also, remove any spaces for the Python data class name.
#
# In addition, for each data class there is a specific collect data
# method which needs to be implemented using CARLA data.
# Other collect data methods for other simulators can be added as well.
#####
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from inference.interfaces import BayesianNetworkInputFeatureData


class SINADRARiskSensorData:
    """Input feature data class for Bayesian networks. Each Bayesian network shall have its own custom implementation
    with its custom input features for situation-specific runtime updates.
    Equals the concrete input into the Bayesian network.
    """

    def collect_data(self, pedestrian_id: int, input_feature_data: "BayesianNetworkInputFeatureData") -> None:
        """Method that extracts the required input feature from the CARLA simulator.

        Parameters
        ----------
        input_feature_data : BayesianNetworkInputFeatureData
            CARLA specific input comprising the CARLA world instance and the CARLA vehicle instance for the given
            vehicle.

        Raises
        -------
        NotImplementedError
            Interface base class, thus, this method must be implemented by the child classes.
        """
        raise NotImplementedError

class Data20230830_KIOS_quantitative_model_scenario_2(SINADRARiskSensorData):
    def __init__(self):
        self.data = None

    def collect_data(self, input_feature_data: 'BayesianNetworkInputFeatureData') -> None:
        self.data = input_feature_data.data
