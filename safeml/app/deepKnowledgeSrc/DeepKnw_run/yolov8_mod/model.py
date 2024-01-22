# Modified code to extract the features
# from: https://github.com/PabloMessina/MedVQA/tree/708801c8966395c890a873678c01cc8d05ac9899
# Ultralytics version: 8.0.149
# Modified: Akram @ IESE

from ultralytics.nn.tasks import DetectionModel
import torch
from sklearn.decomposition import PCA
import numpy as np

class YOLOv8DetectionAndFeatureExtractorModel(DetectionModel):
    def __init__(self, cfg='yolov8n.yaml', ch=3, nc=None, verbose=True):  # model, input channels, number of classes
        super().__init__(cfg, ch, nc, verbose)
        if torch.cuda.is_available():
            self.device = 'cuda'
        else:
            self.device = 'cpu'
        self.half = False
        self.save_hybrid = False
    
    def feats_compression(self, feats, comp=5):
        """
        This function compresses the features using PCA analysis. 
        This helps in reducing the amount of features, and saves disk space
        """
        pca = PCA(n_components=comp)
        #print("ZZZZZZZZZZZZZ", feats.shape)
        b, c, d = feats.shape
        components = pca.fit_transform(feats.reshape((b, c*d)).detach().cpu().numpy())
        #explained = pca.explained_variance_ratio_
        #explained = [sum(explained[:x]) for x in range(1,len(explained))]
        #print(explained), components.shape
        return(components)
            
    def get_feats_forward(self, x):
        """
        This is a modified version of the original _forward_once() method in BaseModel,
        found in ultralytics/nn/tasks.py.
        The original method returns only the detection output, while this method returns
        both the detection output and the features extracted by the last convolutional layer.
        """
        y = []
        features = None
        if torch.is_tensor(x):
           
            if torch.max(x[0]).item() > 1:
                x =  self.preprocess(x)
            if x.shape[0] == 1:
                for m in self.model:
                    if m.f != -1:  # if not from previous layer
                        x = y[m.f] if isinstance(m.f, int) else [x if j == -1 else y[j] for j in m.f]  # from earlier layers
                    if torch.is_tensor(x):
                        features = x # keep the last tensor as features
                    x = m(x)  # run
                    if torch.is_tensor(x):
                        features = x # keep the last tensor as features
                    y.append(x if m.i in self.save else None)  # save output
                if torch.is_tensor(x):
                    features = x # keep the last tensor as features
                f = []
                for item in features:
                    f.append(self.feats_compression(item))
                features = None
                feats = np.array(f)
                dets_out = x
            else:
                divider = 10
                sub_batch_num, rem = int(x.shape[0]/divider), x.shape[0] % divider
                anch = 0
                x_list = []
                for i in range(sub_batch_num):
                    # print("ZZZZZZZZZZZZZZZ", i, anch, anch+divider)
                    x_list.append(x[anch:anch+divider])
                    anch = (i+1)*divider
                if anch < x.shape[0] or x.shape[0] > 30:
                    print("Maximum batch supported right now is 30.")
                    print("Known bug: cant use batch size larger than 10, that isnt divisible by 10.")
                    raise NotImplementedError
                    #x_list.append(x[anch:anch+rem])
                x = None
                feats = None
                dets_out = None
                for T in x_list:
                    #x =  self.preprocess(x)
                    for m in self.model:
                        if m.f != -1:  # if not from previous layer
                            T = y[m.f] if isinstance(m.f, int) else [T if j == -1 else y[j] for j in m.f]  # from earlier layers
                        # if torch.is_tensor(x):
                        #     features = x # keep the last tensor as features
                        T = m(T)  # run
                        if torch.is_tensor(T):
                            features = T # keep the last tensor as features
                        y.append(T if m.i in self.save else None)  # save output
                    if torch.is_tensor(T):
                        features = T # keep the last tensor as features
                    f = []
                    for item in features:
                        f.append(self.feats_compression(item))
                    features = None
                    if feats is None:
                        feats = np.array(f)
                        dets_out = T
                    else:
                        feats = np.concatenate((feats, np.array(f)), axis=0)
                        dets_out = (torch.cat((dets_out[0], T[0]), axis = 0), [torch.cat((dets_out[1][0], T[1][0]), axis=0), torch.cat((dets_out[1][1], T[1][1]), axis=0), torch.cat((dets_out[1][2], T[1][2]), axis=0)])
                    T = None
                    f = None
                    torch.cuda.empty_cache()
            return feats, dets_out # return features and detection output
        else:
            x = self.preprocess(x)['img']
            if x.shape[0] == 1:
                for m in self.model:
                    if m.f != -1:  # if not from previous layer
                        x = y[m.f] if isinstance(m.f, int) else [x if j == -1 else y[j] for j in m.f]  # from earlier layers
                    if torch.is_tensor(x):
                        features = x # keep the last tensor as features
                    x = m(x)  # run
                    if torch.is_tensor(x):
                        features = x # keep the last tensor as features
                    y.append(x if m.i in self.save else None)  # save output
                if torch.is_tensor(x):
                    features = x # keep the last tensor as features
                f = []
                for item in features:
                    f.append(self.feats_compression(item))
                features = None
                feats = np.array(f)
                dets_out = x
            else:
                divider = 10
                sub_batch_num, rem = int(x.shape[0]/divider), x.shape[0] % divider
                anch = 0
                x_list = []
                for i in range(sub_batch_num):
                    x_list.append(x[anch:anch+divider])
                    anch = (i+1)*divider
                if anch < x.shape[0] or x.shape[0] > 30:
                    print("Maximum batch supported right now is 30.")
                    print("Known bug: cant use batch size larger than 10, that isnt divisible by 10.")
                    raise NotImplementedError
                    #x_list.append(x[anch:anch+rem])
                x = None
                feats = None
                dets_out = None
                for T in x_list:
                    #x =  self.preprocess(x)
                    for m in self.model:
                        if m.f != -1:  # if not from previous layer
                            T = y[m.f] if isinstance(m.f, int) else [T if j == -1 else y[j] for j in m.f]  # from earlier layers
                        # if torch.is_tensor(x):
                        #     features = x # keep the last tensor as features
                        T = m(T)  # run
                        if torch.is_tensor(T):
                            features = T # keep the last tensor as features
                        y.append(T if m.i in self.save else None)  # save output
                    if torch.is_tensor(T):
                        features = T # keep the last tensor as features
                    f = []
                    for item in features:
                        f.append(self.feats_compression(item))
                    features = None
                    if feats is None:
                        feats = np.array(f)
                        dets_out = T
                    else:
                        feats = np.concatenate((feats, np.array(f)), axis=0)
                        dets_out = (torch.cat((dets_out[0], T[0]), axis = 0), [torch.cat((dets_out[1][0], T[1][0]), axis=0), torch.cat((dets_out[1][1], T[1][1]), axis=0), torch.cat((dets_out[1][2], T[1][2]), axis=0)])
                    T = None
                    f = None
                    torch.cuda.empty_cache()
            return feats, dets_out # return features and detection output
    
    def custom_forward(self, x):
        """
        This is a modified version of the original _forward_once() method in BaseModel,
        found in ultralytics/nn/tasks.py.
        The original method returns only the detection output, while this method returns
        both the detection output and the features extracted by the last convolutional layer.
        """
        y = []
        features = None
        if torch.is_tensor(x):
            x =  self.preprocess(x)
            for m in self.model:
                if m.f != -1:  # if not from previous layer
                    x = y[m.f] if isinstance(m.f, int) else [x if j == -1 else y[j] for j in m.f]  # from earlier layers
                if torch.is_tensor(x):
                    features = x # keep the last tensor as features
                x = m(x)  # run
                if torch.is_tensor(x):
                    features = x # keep the last tensor as features
                y.append(x if m.i in self.save else None)  # save output
            if torch.is_tensor(x):
                features = x # keep the last tensor as features
            return features, x # return features and detection output
        else:
            x = self.preprocess(x)['img']
            for m in self.model:
                if m.f != -1:  # if not from previous layer
                    x = y[m.f] if isinstance(m.f, int) else [x if j == -1 else y[j] for j in m.f]  # from earlier layers
                if torch.is_tensor(x):
                    features = x # keep the last tensor as features
                x = m(x)  # run
                if torch.is_tensor(x):
                    features = x # keep the last tensor as features
                y.append(x if m.i in self.save else None)  # save output
            if torch.is_tensor(x):
                features = x # keep the last tensor as features
            return features, x # return features and detection output
    
    def preprocess(self, batch):
        """Preprocesses batch of images for YOLO training."""
        if not(torch.is_tensor(batch)):
            batch['img'] = batch['img'].to(self.device, non_blocking=True)
            batch['img'] = (batch['img'].half() if self.half else batch['img'].float()) / 255
            for k in ['batch_idx', 'cls', 'bboxes']:
                batch[k] = batch[k].to(self.device)

            nb = len(batch['img'])
            self.lb = [torch.cat([batch['cls'], batch['bboxes']], dim=-1)[batch['batch_idx'] == i]
                    for i in range(nb)] if self.save_hybrid else []  # for autolabelling
        else:
            batch = batch.to(self.device, non_blocking=True)
            batch = (batch.half() if self.half else batch.float()) / 255
            nb = len(batch)
            # self.lb = [torch.cat([batch['cls'], batch['bboxes']], dim=-1)[batch['batch_idx'] == i]
            #         for i in range(nb)] if self.save_hybrid else []  # for autolabelling

        return batch
    
    
    
def create_yolov8_model(model_name_or_path, nc, class_names):
    from ultralytics.nn.tasks import attempt_load_one_weight
    from ultralytics.yolo.cfg import get_cfg
    ckpt = None
    if str(model_name_or_path).endswith('.pt'):
        weights, ckpt = attempt_load_one_weight(model_name_or_path)
        cfg = ckpt['model'].yaml
    else:
        cfg = model_name_or_path
    model = YOLOv8DetectionAndFeatureExtractorModel(cfg, nc=nc, verbose=True)
    if weights:
        model.load(weights)
    model.nc = nc
    model.names = class_names  # attach class names to model
    args = get_cfg(overrides={'model': model_name_or_path})
    model.args = args  # attach hyperparameters to model
    return model