from eddi_messages.msg import BayesianNetworkOutput, NodeOutput, OutcomeOutput


def convert_output(eddi_output):
    eddi_msg = BayesianNetworkOutput()
    eddi_msg.output_nodes = []
    eddi_output = eddi_output[0]
    
    for node in eddi_output.output_nodes:
        node_msg = NodeOutput()
        node_msg.title = node.title
        node_msg.outcomes = []
        
        for outcome in node.outcomes:
            outcome_msg = OutcomeOutput()
            outcome_msg.outcome_name = outcome.name
            outcome_msg.outcome_value = outcome.value
            node_msg.outcomes.append(outcome_msg)
            
        eddi_msg.output_nodes.append(node_msg)

    return eddi_msg


def calculate_time_diff(*args):
    raise NotImplementedError


def calculate_location_diff(*args):
    raise NotImplementedError


def check_for_persons_in_vicinity(*args):
    raise NotImplementedError


def extract_distance_to_closest_obstacle(*args):
    raise NotImplementedError


def compute_velocity_delta_with_obstacles(*args):
    raise NotImplementedError
