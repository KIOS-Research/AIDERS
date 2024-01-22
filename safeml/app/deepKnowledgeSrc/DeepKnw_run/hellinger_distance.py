########################################################################################################################
"""
Description :  Calculates the hellinger distance for a particular neuron between two dictionaries with keys being their
               features and value being their max-activations along with the total number of features activated in that
               neuron.
Python version : 3.7.3
author Vageesh Saxena
"""
########################################################################################################################


################################################ Importing libraries ###################################################
import pickle

import numpy as np

########################################################################################################################

_SQRT2 = np.sqrt(2)  # sqrt(2) with default precision np.float64


def hellinger_distance(p, q):
    """
    Calculates the hellinger distance for two different probability distribution.
    :param p,q: a dictionary with its keys being unique features and value being their max-activations(dtype:dict)
    :return: average hellinger distance and total number of features activated for a particular neuron
    """
    all_unique_features = set(p.keys()) | set(q.keys())

    # Adding features from all_unique_features to the dictionaries p and q
    for input in all_unique_features:
        if input not in p:
            p[input] = 0
        if input not in q:
            q[input] = 0

    # preparing the sorted dictionaries
    a, b = ({} for i in range(2))
    sorted_keys = sorted(p.keys())
    for input in sorted_keys:
        a[input] = p[input]
        b[input] = q[input]

    hellinger_distance = np.sqrt(np.sum((np.sqrt(list(a.values())) - np.sqrt(list(b.values()))) ** 2)) / _SQRT2
    return hellinger_distance / len(a), len(a)


def find_shared_neurons(listA, listB):
    """
    :param listA: list of unique neurons in A(dtype:list of int)
    :param listA: list of unique neurons in B(dtype:list of int)
    :return: shared neurons between list A and B(dtype:list of int)
    """
    shared_neurons = set.intersection(set(listA), set(listB))
    return list(shared_neurons)


# def calculate_hellinger_distance(model1, model2, filename):
#     """
#     :param model1:data from trained model 1
#     :param model2:data from trained model 2
#     :param filename: pickled file name and directory to store the results
#     """
#     hellinger_dict = {}
#
#     # Finding shared neurons between the two models
#     model1_neurons = list(model1['max_activation_index'].unique())
#     model2_neurons = list(model2['max_activation_index'].unique())
#     shared_neurons = find_shared_neurons(model1_neurons, model2_neurons)
#
#     for neuron in shared_neurons:
#         # Loading the data for both the models
#         model1_data = model1[model1['max_activation_index'] == neuron]
#         model1_data["normalized_max_activations"] = model1_data["max_activations"].apply(
#             lambda x: x / model1_data["max_activations"].sum())
#         model2_data = model2[model2['max_activation_index'] == neuron]
#         model2_data["normalized_max_activations"] = model2_data["max_activations"].apply(
#             lambda x: x / model2_data["max_activations"].sum())
#
#         # Getting all the unique features from both the models so the average can be taken.
#         model1_dict, model2_dict = ({} for i in range(2))
#         unique_features_model1 = model1_data['inputs'].unique()
#         unique_features_model2 = model2_data['inputs'].unique()
#
#         for feature in unique_features_model1:
#             temp = model1_data[model1_data['inputs'] == feature]
#             model1_dict[feature] = temp['normalized_max_activations'].mean()
#
#         for feature in unique_features_model2:
#             temp = model2_data[model2_data['inputs'] == feature]
#             model2_dict[feature] = temp['normalized_max_activations'].mean()
#
#         distance, num_features = hellinger_distance(model1_dict, model2_dict)
#         # Hellinger Dictionary contains the distance between two model for a given activation and total number of
#         # features compared between these two models.
#         hellinger_dict[neuron] = (distance, num_features)
#
#         # Dumping the dictionary
#         with open(filename, 'wb') as handle:
#             pickle.dump(hellinger_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)

