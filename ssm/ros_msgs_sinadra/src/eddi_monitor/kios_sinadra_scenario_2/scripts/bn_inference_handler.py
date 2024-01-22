from inference.file_extraction import BayesianNetworkFileExtractor
from inference.interfaces import BayesianNetworkData, BayesianNetworkInputFeatureData
from inference.risk_sensor_data_collecting import RiskSensorDataBuilder
from inference.inference import BayesianNetworkInference
import multiprocessing
import signal


class ActorBNMapping:
    def __init__(self, actor_id, bn_id):
        self.actor_id = actor_id
        self.bn_id = bn_id


class BNInferenceHandler:
    def __init__(self, actor_bn_mapping):
        self.actor_bn_mapping = actor_bn_mapping

        # Initial Setup
        self.bayesian_network_data, self.bn_inference_multiprocessing_pool = self._initial_setup()

    def update_data_for_actors(self, inputs):
        # Interface Builder
        self._build_data_interface_for_bn_inference(inputs)

    def infer(self):
        # BN Inference
        bayesian_network_output = self._infere_bns(self.bayesian_network_data, self.bn_inference_multiprocessing_pool)

        return bayesian_network_output

    @staticmethod
    def _worker_init():
        # This method is needed for parallel processing based on pools
        # SIGINT (=CTRL+C) will be handled by SIG_IGN (=Handler that ignores the signal)
        signal.signal(signal.SIGINT, signal.SIG_IGN)

    def _infere_bns(self, bayesian_network_data, bn_inference_multiprocessing_pool):
        # Do the inference
        bayesian_network_inference = BayesianNetworkInference()
        bayesian_network_output = (
            bayesian_network_inference.bn_inferences(bayesian_network_data,
                                                     bn_inference_multiprocessing_pool)
        )

        return bayesian_network_output

    def _build_data_interface_for_bn_inference(self, inputs):
        # Collect the data for the BN inference
        risk_sensor_data_builder = RiskSensorDataBuilder()
        self.bayesian_network_data = risk_sensor_data_builder.collect_data_and_build_risk_sensor_data(
            self.bayesian_network_data, inputs
        )

    def _initial_setup(self):
        # Setup multiprocessing Pool
        bn_inference_multiprocessing_pool = multiprocessing.Pool(processes=2, initializer=self._worker_init)

        # Setup for the BNs to infer
        bayesian_network_data = [
            BayesianNetworkData(actor_id=x.actor_id, bayesian_network_id=x.bn_id) for x in self.actor_bn_mapping
        ]

        # Load the BNs from the files folder
        bn_file_extractor = BayesianNetworkFileExtractor()
        bayesian_network_data = bn_file_extractor.get_all_network_instances_for_situation(bayesian_network_data)

        return bayesian_network_data, bn_inference_multiprocessing_pool