# Sfeml monitor
# Last modified: 06-10-2023
# version: 0.0.2
# Author: akram

import numpy as np
import torch

from sklearn.decomposition import PCA
from .yolov8_mod.model import create_yolov8_model
import matplotlib.pyplot as plt

import cv2
import torchvision.transforms as transforms

class SafeMLMonitor():

    def __init__(self, weights, trained_feats, distances, dist_perf, pca_train, n_components=100, selected_ponts_num=8, names={0: 'human'}, device = None, batch_count=50):
        self.names = names
        self.weights = weights
        self.trained_feature_inst = np.load(trained_feats)
        self.distances = np.load(distances)
        self.performance = np.load(dist_perf)
        self.pca_train = np.load(pca_train)
        self.n_components = n_components
        self.selected_components_num = selected_ponts_num 
        if device is None:
            self.DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        else:
            self.DEVICE = device
        self.model_mod = self.get_mod_model(weights, self.names)
        self.model_mod = self.model_mod.float().to(self.DEVICE)
        self.model_mod.eval()
        self.visualise_graph = False
        self.calculator = None
        self.half = False
        self.count_inputs = False
        self.stream = False
        self.batch_count = batch_count
        self.transform = transforms.ToTensor()
        self.stream_batch = None

        self.pca = PCA(n_components=self.n_components)
        train_components = self.pca.fit_transform(self.pca_train)
        
        from .safeml_kios_2 import WassersteinDistance
        self.libs = [WassersteinDistance()]
        
    def get_mod_model(self, weights, names):
        num_classes = len(names)
        return(create_yolov8_model(weights, nc=num_classes, class_names=names))
    
    def get_simple_estimate(self, dist, degree=2):
        x = self.distances
        y = 1 - self.performance
        z = np.polyfit(x, y, degree)

        fit = np.poly1d(z)

        a = fit.coefficients[0]
        b = fit.coefficients[1]
        c = fit.coefficients[2]
        inacc = a * (dist)**2 + b * (dist) + c
        return(inacc)
    
    def get_1D_estimate(self, dist, degree=1):
        x = self.distances
        y = 1 - self.performance
        z = np.polyfit(x, y, degree)

        fit = np.poly1d(z)

        a = fit.coefficients[0]
        b = fit.coefficients[1]

        inacc = a * (dist) + b
        return(inacc)
    
    def get_calculator(self, calc_str):
        for a in self.libs:
            if calc_str in str(type(a)).split(".")[-1][:-2].lower():
                return(a)
        # If the string is not in modeule list
        raise ModuleNotFoundError
    
    def preprocess(self, batch):
        """Preprocesses batch of images for YOLO training."""
        batch = batch.to(self.DEVICE, non_blocking=True)
        #batch = (batch['img'].half() if self.args.half else batch['img'].float()) / 255
        batch = (batch.half() if self.half else batch.float()) / 255
        #nb = len(batch)
        return batch
    
    def get_inst_preds(self, batch):
        #self.batch = self.preprocess(batch)
        _, detection_output = self.model_mod.custom_forward(batch)
        _, yolov8_features = detection_output[0], detection_output[1]
        if len(yolov8_features[2].shape) > 3 and  yolov8_features[2].shape[0] != 1:
            a =  yolov8_features[2]
        else:
            a = yolov8_features[2][0][None, :, :, :]
        return(a)
    
    def get_inst_feats(self, batch):
        #self.batch = self.preprocess(batch)
        feats, _ = self.model_mod.get_feats_forward(batch)
        # _, yolov8_features = detection_output[0], detection_output[1]
        # if len(yolov8_features[2].shape) > 3 and  yolov8_features[2].shape[0] != 1:
        #     a =  yolov8_features[2]
        # else:
        #     a = yolov8_features[2][0][None, :, :, :]
        return(feats)
    
    def get_inst_features(self, batch):
        #a = self.get_inst_preds(batch)
        a = self.get_inst_feats(batch)
        # B = a.reshape((a.shape[0], a.shape[1]*a.shape[2]*a.shape[3]))
        B = a.reshape((a.shape[0], a.shape[1]*a.shape[2]))
        # feats = self.pca.transform(B.cpu().detach().numpy())[:, :self.selected_components_num]
        feats = self.pca.transform(B)[:, :self.selected_components_num]
        return(feats)
    
    def update(self, im, calc = 'wasserstein'):
        '''
        Instantiation for KIOS: input cv2 format numpy array.
        Input assumed to be in BGR format, not normalized. 
        '''
        cvImg = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
        resized_image = cv2.resize(cvImg, (640, 640))
        resized_tensor = self.transform(resized_image).unsqueeze(0).to(self.DEVICE)
        if self.count_inputs < self.batch_count:
            self.count_inputs += 1
            inst = self.get_inst_features(resized_tensor)
            if self.stream_batch is None:
                self.stream_batch = inst
            else:
                self.stream_batch = np.concatenate((self.stream_batch, inst), axis=0)
            return(None)
        else:
            inst = self.get_inst_features(resized_tensor)
            self.stream_batch = np.concatenate((self.stream_batch, inst), axis=0)
            self.stream_batch = self.stream_batch[1:, :]
            if self.calculator is None:
                self.calculator = self.get_calculator(calc)
            _, Dist = self.calculator.measure_metric_p_value_gpu(self.trained_feature_inst, self.stream_batch, verbose = True)
            #scue = max(min(self.get_simple_estimate(Dist.cpu().item()), 1), 0)
            scue = max(min(self.get_1D_estimate(Dist.cpu().item()), 1), 0)
            return(scue)


    def get_scue(self, batch, calc = 'wasserstein'):
        if self.calculator is None:
            self.calculator = self.get_calculator(calc)
        if batch.shape[0]>30:
            done = False
            inst = None
            i, j = 0, 30
            while not(done):
                if inst is None:
                    inst = self.get_inst_features(batch[i:j, :, :, :])
                else:
                    inst = np.concatenate((inst, self.get_inst_features(batch[i:i+30, :, :, :])), axis=0)
                i += 30
                if j + 30 < batch.shape[0]:
                    j += 30
                else:
                    j = batch.shape[0]
                if i > batch.shape[0]:
                    done = True
            _, Dist = self.calculator.measure_metric_p_value_gpu(self.trained_feature_inst, inst, verbose = True)
            #scue = max(min(self.get_simple_estimate(Dist.cpu().item()), 1), 0)
            scue = max(min(self.get_1D_estimate(Dist.cpu().item()), 1), 0)
            return(scue)
        else:
            inst = self.get_inst_features(batch)
            _, Dist = self.calculator.measure_metric_p_value_gpu(self.trained_feature_inst, inst, verbose = True)
            #scue = max(min(self.get_simple_estimate(Dist.cpu().item()), 1), 0)
            scue = max(min(self.get_1D_estimate(Dist.cpu().item()), 1), 0)
            return(scue)