from .utils import *
from .Dataprocessing import get_yolo_dataset
import os, sys
from .OOD_data import*
from ultralytics.data import build_dataloader
from ultralytics import YOLO
from ultralytics.data import YOLODataset
from .utils import get_yolo_outs
from sklearn.model_selection import train_test_split
from .Dataprocessing import load_leaves_V
from .Dataprocessing import load_GTSRB
import statistics
import operator
from .yolov8_mod.validator import BaseValidator
import pandas as pd
pd.options.mode.chained_assignment = None
from collections import defaultdict
from .idc_knw import *
import joblib
from .Dataprocessing import load_MNISTVAL, load_CIFARVAL, load_driving_data, load_SVHN_, load_EMNIST, load_MNIST, \
    load_CIFAR, load_MNIST_32, split_data, load_Imagnet, load_cifar_vgg, load_coco
from .hellinger_distance import *


def scaleHD(points, val, rmax=1, rmin=0):
    maxval = points[max(points, key=points.get)]

    minval = points[min(points, key=points.get)]

    X_std = (float(val) - minval) / (maxval - minval)
    return X_std


def get_adv(attack, dataset, X_test, Y_test):
    adv_image_all = []

    adv_image_all.append(np.load('dataset/adv_image/{}_{}_image.npy'.format(attack, dataset)))

    adv_image_all = np.concatenate(adv_image_all, axis=0)

    test = np.concatenate([X_test, adv_image_all], axis=0)

    return test

def save_KnwTrNeurons(neurons, filename):

    filename = filename + '_knw.sav'
    joblib.dump(neurons, filename)

    return


def load_KnwTrNeurons(filename):
    neurons =joblib.load(filename, mmap_mode='r')
    return neurons


def filter_neurons_by_layer(TopN, skip_layers):

    relevant = {}
    layers = []

    for x in TopN:
        if x[0] in relevant:
            relevant[x[0]].append(x[1])
        else:
            relevant[x[0]] = [x[1]]

    return relevant


class KnowledgeCoverage:

    def __init__(self, model, data_set, model_name, subject_layer, trainable_layers, dense_layers, method, percent,thershold, attack, skip_layers, nbr_Trknw, config,classes,scaler=default_scale,selected_class=1):

        self.activation_table = defaultdict(bool)
        self.dense_layers = dense_layers
        self.model = model
        self.attack = attack
        self.scaler = scaler
        self.data_set = data_set
        self.skip_layers = skip_layers = ([] if skip_layers is None else skip_layers)
        self.model_name = model_name
        self.subject_layer = subject_layer
        self.trainable_layers = trainable_layers
        self.selected_class = selected_class
        self.method = method
        self.percentage = percent
        self.distance_thershold = thershold
        self.user_input = nbr_Trknw
        self.config_module = config
        self.DATASET_DIR = self.config_module.get("DATASET_DIR")
        self.Experiment_DIR = self.config_module.get("Experiment_DIR")
        self.DATA_DIRe = self.config_module.get("DATA_DIR")
        self.experiment_folder = self.Experiment_DIR
        self.samp = self.config_module.get("sample_size")
        self.model_path = self.config_module.get("trained_weights")
        self.BATCH = self.config_module.get("Batch_size")
        self.testpath= self.config_module.get("test_path")
        self.classes=classes


    def KnwTransferYolo(self, test_inputs, statep):
        """
               :param test_inputs: Inputs

               """

        saved_dir = os.path.join(self.DATA_DIRe, statep + '_' + self.data_set + '_layers.sav')
        fields_to_generate = ['inputs', 'activations', 'max_activations', 'max_activation_index', 'neuron_index']
        data = []
        T = next(iter(test_inputs))
        outs, pred= get_yolo_outs(self.model, T)
        layer_count = len(outs)
        neuron_dictionary = {}

        layer_index = 1

        for input_index in (range(self.BATCH)):

                    T = next(iter(test_inputs))
                    x = T['img']
                    neuron_layer = {}
                    outs, pred = get_yolo_outs(self.model, T)
                    inside_layer = {}
                    out_for_input = outs

                    filter_outs = np.zeros((out_for_input.shape[-1],))
                    filter_count_by_layer = len(filter_outs)
                    max_act = np.max(out_for_input)  # maximum activation given by an input for one layer
                    # print("max_act",max_act)
                    neuron_id = np.argmax(out_for_input, axis=None)  # ==> this gives over 1k of neurons
                    # print("neuron_id",neuron_id)
                    index_neuron = np.unravel_index(np.argmax(out_for_input), out_for_input.shape)
                    # print("index_neuron",index_neuron)
                    for filter_index in range(out_for_input.shape[-1]):
                        maxact = np.max(out_for_input[..., filter_index])
                        neuronTuple = {(layer_index, filter_index): maxact}
                        inside_layer.update(neuronTuple)

                    filter_key = max(inside_layer, key=inside_layer.get)

                    val_max = inside_layer.get(filter_key)

                    neuron_dictionary.update({layer_index: filter_key})
                    global_neuron_index = (layer_index, (neuron_id, index_neuron))


                    # print("['inputs', 'activations', 'max_activations', 'max_activation_index', 'neuron_index']")
                    data.append([input_index, layer_index, max_act, global_neuron_index, neuron_id])
                    # print("data",[input_index, layer_index, max_act, global_neuron_index, neuron_id] )
                    # input_index=+1
        df = pd.DataFrame(data,
                          columns=fields_to_generate)

        with open(saved_dir, 'wb') as out_file:
            pickle.dump(df, out_file)

        return data

    def coverage_score(self, topN, testdata, train_inputs, test_inputs,y_test, subsetTop):

        covered_neurons = []
        testmodel = testdata
        method = self.method

        modeltest_neurons = list(testmodel['max_activation_index'].unique())
        Trknw_shared_neurons = find_shared_neurons(modeltest_neurons, topN)
        number_Trknw_shared_neurons = len(Trknw_shared_neurons)
        test_features = {}
        for neuron in Trknw_shared_neurons:
            model_data = testmodel[testmodel['max_activation_index'] == neuron]
            model_data["normalized_max_activations"] = model_data["max_activations"].apply(
                lambda x: x / model_data["max_activations"].sum())

            # Getting all the unique features.
            model_dict = {}
            unique_features = model_data['inputs'].unique()
            test_features[neuron] = len(list(unique_features))


        if method == 'idc':

            topN = topN[: int(len(topN) * self.percentage)]

            coverage, covered_combinations, max_comb= self.idc_pipline(self.model_name, train_inputs,
                                                                        self.subject_layer, test_inputs,
                                                                        self.selected_class, topN, subsetTop,y_test)
            covered_neurons = topN

        return coverage, len(covered_neurons), covered_combinations, max_comb

    def coverage_neurons(self, model1, model2):

        # Finding shared neurons between the two out and in domains
        model1_neurons = list(model1['max_activation_index'].unique())
        # print("model1_neurons",model1_neurons)
        model2_neurons = list(model2['max_activation_index'].unique())
        # print("model2_neurons", model2_neurons)

        initial_shared_neurons = find_shared_neurons(model1_neurons, model2_neurons)
        number_shared_neurons = len(initial_shared_neurons)

        neurons_inputs = {}
        scaledHD = {}
        hellinger_dict = {}
        scaled_hellinger = {}
        model1_pos_dict, model2_pos_dict = ({} for i in range(2))
        model1_features_list, model2_features_list = ([] for i in range(2))

        for neuron in initial_shared_neurons:
            # Loading the data for both models
            model1_data = model1[model1['max_activation_index'] == neuron]
            model1_data["normalized_max_activations"] = model1_data["max_activations"].apply(
                lambda x: x / model1_data["max_activations"].sum())
            model2_data = model2[model2['max_activation_index'] == neuron]
            model2_data["normalized_max_activations"] = model2_data["max_activations"].apply(
                lambda x: x / model2_data["max_activations"].sum())

            # Getting all the unique features from both the models so the average can be taken.
            model1_dict, model2_dict, model3_dict = ({} for i in range(3))
            unique_features_model1 = model1_data['inputs'].unique()
            unique_features_model2 = model2_data['inputs'].unique()
            model1_pos_dict[neuron] = model1_data['inputs'].nunique()
            model2_pos_dict[neuron] = model2_data['inputs'].nunique()

            for feature in unique_features_model1:
                temp = model1_data[model1_data['inputs'] == feature]
                model1_dict[feature] = temp['normalized_max_activations'].mean()


            for feature in unique_features_model2:

                temp = model2_data[model2_data['inputs'] == feature]
                model2_dict[feature] = temp['normalized_max_activations'].mean()
                model2_features_list.append(model2_pos_dict[neuron])

            distance, num_features = hellinger_distance(model1_dict, model2_dict)
            hellinger_dict[neuron] = (distance, num_features)
            scaled_hellinger[neuron] = distance
            neurons_inputs[neuron] = num_features

        scaled_hellingerall = {key: scaled_hellinger[key] for key in initial_shared_neurons}


        save_KnwTrNeurons(scaled_hellinger, '%s/%s_'
                          % (self.experiment_folder, self.data_set) + 'HDALL')
        avoided, gained, preferred = ([] for i in range(3))

        for neuron in initial_shared_neurons:
             if (self.distance_thershold-0.05)<=scaled_hellinger[neuron]<=self.distance_thershold:
                if model1_pos_dict[neuron] > model2_pos_dict[neuron]:
                    avoided.append(neuron)
                elif model1_pos_dict[neuron] == model2_pos_dict[neuron]:
                    preferred.append(neuron)
                else:
                    gained.append(neuron)
        #

        """features length l: num_features describes the number of(unique) maximally activated features 
        using a feature preference distribution """

        """To change the type of neurons just change this 
        # neurons_subset """

        # neurons_subset = [k for k in preferred if neurons_inputs[k] > 10]<==preferred
        # For Yolo ==> changed number of features for 1 instead of 10
        neurons_subset = [k for k in preferred if neurons_inputs[k] > 1]

        preferred_subset = {key: hellinger_dict[key] for key in neurons_subset}
        # print("preferred_subset",len(preferred_subset))
        preferred_neurons = {key: scaled_hellinger[key] for key in preferred}
        gained_neurons = {key: hellinger_dict[key] for key in gained}
        avoided_neurons = {key: hellinger_dict[key] for key in avoided}

        save_KnwTrNeurons(avoided_neurons, '%s/%s_'
                          % (self.experiment_folder, self.data_set) + 'Avoided_HD')

        # print("gained_neurons",len(gained_neurons))
        save_KnwTrNeurons(preferred_neurons, '%s/%s_'
                          % (self.experiment_folder, self.data_set) + 'Preffered_HD')

        save_KnwTrNeurons(gained_neurons, '%s/%s_'
                          % (self.experiment_folder, self.data_set) + 'Gained_HD')
        k_value = min(self.user_input, len(preferred_subset))

        top_10_neurons = heapq.nlargest(k_value, preferred_subset, key=preferred_subset.get)


        sub_set_N = []
        least_10_neurons = heapq.nsmallest(k_value, preferred_subset, key=preferred_subset.get)



        save_KnwTrNeurons(top_10_neurons, '%s/%s_'
                          % (self.experiment_folder, self.data_set) + 'top20')

        save_KnwTrNeurons(least_10_neurons, '%s/%s_'
                          % (self.experiment_folder, self.data_set) + 'least20')




        return len(initial_shared_neurons), len(gained_neurons),len(preferred_neurons)


    def idc_pipline(self, model_name, train_inputs, subject_layer, test_inputs, selected_class, Tr_knw_neuron,
                    subsetTop, y_test):


        covered_combinations = ()



        model_ul = YOLO(self.model_path)
        coverage, covered_combinations, max_comb, df_details = measure_idcYolo(self.model,model_ul, model_name,
                                                               test_inputs, train_inputs,
                                                               Tr_knw_neuron, subsetTop,self.data_set,self.BATCH, self.samp,self.DATA_DIRe,self.classes,
                                                               covered_combinations)
        print(df_details)
        Class_cov, TCC = get_ClassCov(df_details, max_comb)

        return coverage, covered_combinations, max_comb

    def load_data(self, statep):
        if self.data_set=='coco':
            X_train, y_train, X_test, y_test, X_val, y_val,X_val_z, y_val_z=load_coco()
            img_rows, img_cols = 32, 32

        elif self.data_set == 'leaves':
            X_train, y_train, X_test, y_test, X_val, y_val, X_val_z, y_val_z  = load_leaves_V("original")
            img_rows, img_cols = 256, 256


        if self.data_set == 'mnist':
            if statep == 'zero_shot':
                # X_train, y_train, X_test, y_test, X_val, y_val = load_MNISTVAL("fashion", channel_first=False) #==> initial study
                # X_train, y_train, X_test, y_test, X_val, y_val
                X_train, y_train, X_test, y_test, X_val, y_val = load_MNISTVAL("emnist", channel_first=False)
            else:
                X_train, y_train, X_test, y_test, X_val, y_val = load_MNISTVAL("ini", channel_first=False)
            img_rows, img_cols = 28, 28

        elif self.data_set == 'imagnet':
            if statep == 'zero_shot':
                X_train, y_train, X_test, y_test, X_val, y_val = load_Imagnet("zero", channel_first=False)

            else:
                X_train, y_train, X_test, y_test, X_val, y_val = load_Imagnet("ini", channel_first=False)
            img_rows, img_cols = 224, 224

        elif self.data_set == 'cifar':
            if statep == 'zero_shot':
                X_train, y_train, X_test, y_test, X_val, y_val = load_CIFARVAL("zeroshot")

            else:
                X_train, y_train, X_test, y_test, X_val, y_val = load_CIFARVAL("ini")
            img_rows, img_cols = 32, 32

        elif self.data_set == 'svhn':
            if statep == 'zero_shot':
                X_train1, y_train1, X_test, y_test, X_val1, y_val1 = load_MNIST_32()#==>initial emnist
                X_train, X_val, y_train, y_val= load_GTSRB()
            else:
                X_train, y_train, X_test, y_test, X_val, y_val = load_SVHN_("ini")

            img_rows, img_cols = 32, 32
        elif self.data_set == 'GTSRB':
            if statep == 'zero_shot':
                X_train, y_train, X_test, y_test, X_val, y_val = load_SVHN_("ini")

            else:
                X_train, X_val, y_train, y_val = load_GTSRB()
                X_train, X_test, y_train, y_test = train_test_split(X_train, y_train, test_size=0.1, random_state=1)
            img_rows, img_cols = 32, 32

        return X_train, y_train, X_test, y_test, X_val, y_val, img_rows, img_cols

    def runYolo(self, split, typTrknw, data_path, val_path, OOD_data, OOD_yaml_path, test_loader):

        ####################################
        # 1.Load Data#
        ####################################

        print("loading data, ....")
        trained_weights = self.model_path
        n_workers = 16
        yolo_valset = get_yolo_dataset(trained_weights, data_path, val_path, self.BATCH, mode='val')
        val_loader = build_dataloader(yolo_valset, batch=self.samp, shuffle=True, workers=n_workers,
                                             rank=-1)
        # yolo_testset = get_yolo_dataset(trained_weights, data_path, self.testpath, self.BATCH, mode='val')
        # test_loader = build_dataloader(yolo_testset, batch=self.samp, shuffle=True, workers=n_workers,
        #                               rank=-1)



        OOD_set = get_yolo_dataset(trained_weights, OOD_yaml_path, OOD_data, self.BATCH, mode='val')
        # OOD_loader = build_dataloader(OOD_set, batch=self.samp, shuffle=True, workers=n_workers,
        #                               rank=-1)
        #
        # corrupt_loader \
        OOD_loader= FinalDataLoaderCorrupted(yolo_valset, val_path, batch=self.samp, corr_type=['frost', 'fog'])
        # n_batches = len(corrupt_loader)

        print("knowlege transfer/change estimation, ...")
        print("validation set size:", len(yolo_valset))
        # print(val_loader.dataset.labels)
        print("zero shot learning set size:", len(OOD_set))
        # print(os.path.join(self.experiment_folder, "{}_gained_HD_knw.sav".format(self.data_set)))
        try:
            print("Loading preferred neurons")
            top_10_neurons = load_KnwTrNeurons(
                os.path.join(self.experiment_folder, "{}_top20_knw.sav".format(self.data_set)))

            least_10_neurons = load_KnwTrNeurons(
                os.path.join(self.experiment_folder, "{}_least20_knw.sav".format(self.data_set)))
            preferred_neurons = load_KnwTrNeurons(
                os.path.join(self.experiment_folder, "{}_Preffered_HD_knw.sav".format(self.data_set)))
            gained_neurons = load_KnwTrNeurons(
                os.path.join(self.experiment_folder, "{}_Gained_HD_knw.sav".format(self.data_set)))


        except Exception as e:
            print("Preferred neurons must be calculated. Doing it now!")

            ############################################################################################################
                # 2.Knowledge Analysis: Quantize Neuron Outputs during validation and zeroshot stage#
            ############################################################################################################

            nc_val = self.KnwTransferYolo(val_loader, "validation")
            # nc_zero = self.KnwTransferYolo(corrupt_loader, "zero_shot")
            nc_zero = self.KnwTransferYolo(OOD_loader, "zero_shot")
            print("zeroshot done,...")
            zero_shot_data = pd.read_pickle(os.path.join(self.DATA_DIRe,
                    "zero_shot_{}_layers.sav".format(self.data_set)) ) # , sep='\t',error_bad_lines=False)
            valid_data = pd.read_pickle(os.path.join(self.DATA_DIRe,
                        "validation_{}_layers.sav".format(self.data_set)) ) # , sep='\t',error_bad_lines=False)
            print("transfer knowledge neurons extraction, ...")
                ############################################################################################################
                # 3.Knowledge Shift Analysis and 4. TRansfer Knowlege Neurons Selection#
                ############################################################################################################

            shared_neurons, gained, preferred_neurons = self.coverage_neurons(valid_data, zero_shot_data)




        top_10_neurons = load_KnwTrNeurons(
            os.path.join(self.experiment_folder, "{}_top20_knw.sav".format(self.data_set)))

        least_10_neurons = load_KnwTrNeurons(
            os.path.join(self.experiment_folder, "{}_least20_knw.sav".format(self.data_set)))
        preferred_neurons = load_KnwTrNeurons(
            os.path.join(self.experiment_folder, "{}_Preffered_HD_knw.sav".format(self.data_set)))
        gained_neurons = load_KnwTrNeurons(
            os.path.join(self.experiment_folder, "{}_Gained_HD_knw.sav".format(self.data_set)))
        print("neurons are loaded ")
        print(len(gained_neurons))


        nc_test = self.KnwTransferYolo(test_loader,  "testing")

        test_data = pd.read_pickle(os.path.join(self.DATA_DIRe,
            "testing_{}_layers.sav".format(self.data_set)))  # , sep='\t',error_bad_lines=False

        if typTrknw == 'top':
            TopN = top_10_neurons
        elif typTrknw == 'least':
            TopN = least_10_neurons
        elif typTrknw == 'gained':
            TopN = gained_neurons

        elif typTrknw == 'preferred':
            TopN = preferred_neurons

        TopN = dict(sorted(TopN.items(), key=operator.itemgetter(1), reverse=False))
        TopN = list(TopN.keys())


        subsetTop = []

        ############################################################################################################
        # 5.Knowledge Coverage Estimation#
        ############################################################################################################
        print("test set coverage estimation, ...")

        Knw_coverage, number_covered, covered_combinations, max_comb = self.coverage_score(
            TopN, test_data, val_loader, val_loader,val_loader.dataset.labels, subsetTop)






        return Knw_coverage, number_covered, covered_combinations, max_comb, len(val_loader), len(val_loader), TopN