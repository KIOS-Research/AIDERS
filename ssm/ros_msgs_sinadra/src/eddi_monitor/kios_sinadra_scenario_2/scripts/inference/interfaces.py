from typing import List, Optional, ClassVar, TYPE_CHECKING, Tuple, Any
from dataclasses import dataclass
import math

if TYPE_CHECKING:
    from inference.sinadra_risk_sensor_data import SINADRARiskSensorData
    from model_generation.config_template import BayesianNetworkConfig
    from bayesian_network.bayesian_networks import BayesianNetwork
    from multiprocessing import Event
    from actor_situation_class_detection.actor_role_classification.bayesian_network_id import BayesianNetId
    from pgmpy.models.BayesianModel import BayesianModel
    from data_model.positions import Location
    from data_model.vehicle import Vehicle
    from data_model.map import Map
    from data_model.environment import Environment


@dataclass
class Outcome:
    """Data class that describes a specific state with its value of a node in a Bayesian network.
    (Python dataclasses.dataclass object.)

    Attributes
    ----------
    name : str
        Name of this specific state/outcome.
    value : float
        Value/probability for this specific state/outcome.
    """

    name: str
    value: float


@dataclass
class Node:
    """Data class that describes a Bayesian network node.
    (Python dataclasses.dataclass object.)

    Attributes
    ----------
    title : str
        The title of the node.
    outcomes : List[Outcome]
        List of the outcomes of the nodes with their state names and values/probabilities.
    """
    title: str
    outcomes: List[Outcome]


@dataclass()
class BayesianNetworkOutput:
    """This data class describes the output after a Bayesian network inference for a specific vehicle. For given output
    nodes the infered values and their states are collected in here.
    (Python dataclasses.dataclass object.)

    Attributes
    ----------
    actor_id : int
        Identifier of the respective entity for which the inference runs.
    bayesian_network_id : Optional[str]
        Identifier for the Bayesian network that was used for the inference.
    output_nodes : List[Node]
        List of the defined output nodes for this Bayesian network. The infered node values and states are saved in
        here.
    """

    actor_id: int
    bayesian_network_id: str
    output_nodes: List[Node]


@dataclass
class BayesianNetworkData:
    """Data class that is typically used as input for the methods that are part of the Bayesian network inference
    module. Each vehicle got its own data instance. If there is no Bayesian network ID set there will be no inference
    for the respective vehicle.
    (Python dataclasses.dataclass object.)

    Attributes
    ----------
    actor_id : int
        Identifier of the respective entity for which the inference runs.
    bayesian_network_id : str
        Identifier for the Bayesian network that shall be used for the inference. Will only be set if the inference
        will be performed for this specific vehicle. (The default value is None.)
    sinadra_risk_sensor_data : Optional[SINADRARiskSensorData]
        Input data class comprising the situation input features that will be fed to the Bayesian network configuration
        for situation-specific runtime updates of the evidences. Will only be set if the inference will be performed
        for this specific vehicle. (The default value is None.)
    bayesian_network_config : Optional[BayesianNetworkConfig]
        Configuration object of this specific Bayesian network that allows situation-specific runtime updates of the
        evidences of the Bayesian network. Will only be set if the inference will be performed for this specific
        vehicle. (The default value is None.)
    bayesian_network_model : Optional[BayesianModel]
        Bayesian network for this specific vehicle as pgmpy Bayesian model object. Will only be set if the inference
        will be performed for this specific vehicle. (The default value is None.)
    """

    actor_id: int
    bayesian_network_id: str
    sinadra_risk_sensor_data: Optional["SINADRARiskSensorData"] = None
    bayesian_network_config: Optional["BayesianNetworkConfig"] = None
    bayesian_network_model: Optional["BayesianModel"] = None


@dataclass()
class BayesianNetworkInputFeatureData:
    """Data class that is used as input for the creation of the Bayesian network's specific input feature data classes
    that are used in the Bayesian network configurations.
    (Python dataclasses.dataclass object.)
    Equals the output from the simulation.

    Attributes
    ----------

    """

    data: Any
