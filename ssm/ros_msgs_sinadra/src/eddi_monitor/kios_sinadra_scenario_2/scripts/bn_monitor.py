from bn_inference_handler import ActorBNMapping, BNInferenceHandler
from bayesian_networks import bayesian_network_identifiers
from inference.interfaces import BayesianNetworkInputFeatureData


# TODO: Should be outsourced to be inherited by the other monitors as well
class Monitor:
    def execute_step(self, eddi_input):
        raise NotImplementedError


class BayesianNetworkMonitor(Monitor):
    def __init__(self):
        # Required because the BN inference component can be used for
        # multiple actor inferences at once in theory but for this EDDI
        # monitor this is limited to only one actor / inference at once
        # because the forwarding of the data input interface leads to
        # serialization issues with Python's multiprocessing
        # -> parallelization is deactivated in that monitor
        # -> Create a default actor with ID 0
        actor_bn_mapping = [ActorBNMapping(0, bayesian_network_identifiers[0])]
        self.bn_inference_handler = BNInferenceHandler(actor_bn_mapping)

    def execute_step(self, eddi_input):
        # We only use one actor with ID 0
        actor_id = 0

        actor_input_data = BayesianNetworkInputFeatureData(data=eddi_input)
        inputs = {actor_id: actor_input_data}

        self.bn_inference_handler.update_data_for_actors(inputs)
        bn_outputs = self.bn_inference_handler.infer()

        return bn_outputs


if __name__ == "__main__":
    bn_monitor = BayesianNetworkMonitor()

    # Placeholder for the data input from the ROS Wrapper component
    class DummyInput:
        def __init__(self, speed, distance):
            self.vehicleSpeed = speed
            self.pedestrianDistance = distance

    # Testing the BN inference two times to compare with the original BN package
    input0 = DummyInput(4 / 3.6, 0.4)
    output0 = bn_monitor.execute_step(input0)
    print(output0)

    input1 = DummyInput(4 / 3.6, 5.3)
    output1 = bn_monitor.execute_step(input1)
    print(output1)
