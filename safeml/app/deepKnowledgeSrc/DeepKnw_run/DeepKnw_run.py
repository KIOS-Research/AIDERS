from .Dataprocessing import *
from tensorflow.keras.models import model_from_json, load_model, save_model
#from Coverages import TrKnw_run
from .TrKnw_Yolo import *
from tensorflow.keras import applications
from tensorflow.python.client import device_lib
import tensorflow as tf
import os
import torch
import cv2
import pandas as pd
import random
import shutil
from distutils.dir_util import copy_tree
from .OOD_data import*
from ultralytics.data import build_dataloader
from ultralytics import YOLO
from ultralytics.data.loaders import LoadImages
from .yolov8_mod.model import create_yolov8_model
# import importlib
os.environ['TF_GPU_ALLOCATOR']="cuda_malloc_async"
os.environ["TF_CPP_VMODULE"]="gpu_process_state=10,gpu_cudamallocasync_allocator=10"
import cv2
import torchvision.transforms as transforms

class DeepKnw:
    def __init__(self, config):
        self.covered_combinations = ()


        self.config_module = {}
        exec(open(config).read(), self.config_module)
        self.DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.dataset = self.config_module.get("dataset_format")
        self.DATASET_DIR = self.config_module.get("DATASET_DIR")
        self.Experiment_DIR = self.config_module.get("Experiment_DIR")
        self.DATA_DIR = self.config_module.get("DATA_DIR")
        self.yaml_path = self.config_module.get("ID_yaml")
        self.OOD_yaml_path = self.config_module.get("OOD_yaml")
        self.val_path = self.config_module.get("val_path")
        self.test_path = self.config_module.get("test_path")
        self.OOD_data=self.config_module.get("OOD_data_path")
        self.samp=self.config_module.get("sample_size")
        self.Batch=self.config_module.get("Batch_size")
        self.model_path = self.config_module.get("trained_weights")
        self.classes= {0: 'human'}
        self.transform = transforms.ToTensor()

        self.coverage=0
        self.dataAnalyser={}

    def getdata(self):
        X_train, X_test, y_train, y_test=load_GTSRB(self.DATASET_DIR)

        return X_test, X_train

    def getSVHN(self):
        X_train, y_train, X_test, y_test,X_val, y_val = load_SVHN_("ini",self.DATASET_DIR)

        return X_test, X_train

    def getTestloader(self,testpath,batch):
        yolo_testset = get_yolo_dataset(self.model_path, self.yaml_path, testpath, batch, mode='val')
        test_loader = build_dataloader(yolo_testset, batch=self.samp, shuffle=True, workers=16,
                                      rank=-1)
        return test_loader
    def estimate_coverage(self, test_loader):
        startTime = time.time()

        model_name = 'yolo'
        model_p=os.path.splitext(self.model_path)[0]

        approach = 'knw'
        dataset=self.dataset

        use_adv =  False
        nbr_Trknw = 40
        percent =  0.4
        threshold =  0.05
        TypeTrknw =  'gained'
        split =  0
        attack =  'pgd'
        selected_class = -1
        logfile_name = 'resultknw.log'
        logfile = open(logfile_name, 'a')

        if model_name == 'DKox':
            if tf.executing_eagerly():
                tf.compat.v1.disable_eager_execution()
            model = tf.keras.models.load_model("leaf_disease_coloured.h5")

            print("Model for grape leaves disease detection is loaded")
        elif model_name =='yolo':
            trained_weights =self.model_path
            # print(self.model_path)
                # "./Networks/KIOS/best.pt"
            names = {0: 'human'}
            num_classes = len(names)
            model = YOLO(trained_weights)
            model.info(False)  # True for detailed info

            model_mod = create_yolov8_model(trained_weights, nc=num_classes, class_names=names)

            model = model_mod

            samp = self.samp


            print("YOLO8 is loaded")
        else:

            try:
                json_file = open(model_p + '.json', 'r')
                file_content = json_file.read()
                json_file.close()
                model = model_from_json(file_content)
                model.load_weights(model_p + '.h5')
                model.compile(loss='categorical_crossentropy',
                              optimizer='adam',
                              metrics=['accuracy'])
            except:
                print("exeception")
                model = load_model(model_p + '.h5')

            # trainable_layers = get_trainable_layers(model)
        trainable_layers = []
        dense_layers = []
        experiment_folder = 'experiments'
        isExist = os.path.exists(experiment_folder)
        if not isExist:
                os.makedirs(experiment_folder)
        dataset_folder = 'dataset'
        isExist = os.path.exists(dataset_folder)
        if not isExist:
                os.makedirs(dataset_folder)

        subject_layer =  -1

        skip_layers = []

        ####################
        
        ####################

        if approach == 'knw':
            method = 'idc'

            knw = KnowledgeCoverage(model, dataset, model_name, subject_layer, trainable_layers, dense_layers, method,
                                    percent, threshold, attack, skip_layers, nbr_Trknw, self.config_module,self.classes,selected_class=1)

            Knw_coverage, covered_TrKnw, combinations, max_comb, testsize, zero_size, Trkneurons = \
                knw.runYolo(split, TypeTrknw, self.yaml_path, self.val_path, self.OOD_data,self.OOD_yaml_path , test_loader)



            print("The test set coverage: %.2f%% for dataset  %s and the model %s " % (Knw_coverage, dataset, model_name))

            self.coverage=Knw_coverage
            logfile.close()

        endTime = time.time()
        elapsedTime = endTime - startTime
        print("Elapsed Time = %s" % elapsedTime)
        return self.coverage

    def DesignDataAnalyzer(self):
        nbr_Trknw = 40
        percent = 0.4
        threshold = 0.05
        TypeTrknw = 'gained'
        split = 0
        BATCH = self.Batch
        model_name = 'yolo'
        try:
            print("Loading preferred neurons....")

            gained_neurons = load_KnwTrNeurons(
                os.path.join(self.Experiment_DIR, "{}_Gained_HD_knw.sav".format(self.dataset)))
        except Exception as e:
            print("Run design time activity first!")

        TopN = gained_neurons
        model_name = 'yolo'
        trained_weights = self.model_path

        names = {0: 'human'}
        num_classes = len(names)
        model_mod = create_yolov8_model(trained_weights, nc=num_classes, class_names=names)
        print(model_mod.info(False))
        TopN = dict(sorted(TopN.items(), key=operator.itemgetter(1), reverse=False))
        TopN = list(TopN.keys())
        topN = TopN[: int(len(TopN) * percent)]
        n_workers = 16

        relevant_neurons = get_relevantneurons(topN)
        startTime = time.time()

        Testset_TCC, Train_qtizer, train_outs= get_designTime(self.dataset, self.DATA_DIR, model_name)

        endTime = time.time()
        elapsedTime = endTime - startTime

        new_data = {'train_outs': train_outs, 'relevant_neurons':relevant_neurons,'Testset_TCC':Testset_TCC,'Train_qtizer':Train_qtizer}
        self.dataAnalyser.update(new_data)
        return model_mod,train_outs
    def Runtime_Estimate(self, img_frame,model):

        # img = next(iter(T))

        startTime = time.time()
        im= cv2.imread(img_frame)

        cvImg = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
        resized_image = cv2.resize(cvImg, (640, 640))
        resized_tensor = self.transform(resized_image).unsqueeze(0).to(self.DEVICE)


        model_name = 'yolo'
        covered_combinations = ()

        KNW_runtime=measure_Runtime(model, model_name, resized_tensor, self.dataAnalyser['train_outs'], self.dataAnalyser['relevant_neurons'], self.dataAnalyser['Testset_TCC'], self.dataAnalyser['Train_qtizer'], self.dataset, self.DATA_DIR,
                        covered_combinations)



        endTime = time.time()
        elapsedTime = endTime - startTime
        print("Final Elapsed Time = %s" % elapsedTime)



        return KNW_runtime



