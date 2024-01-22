# Ultralytics YOLO 🚀, AGPL-3.0 license
# Ultralytics version: 8.0.149
# Modified Akram @ IESE
"""
Check a model's accuracy on a test or val split of a dataset

Usage:
    $ yolo mode=val model=yolov8n.pt data=coco128.yaml imgsz=640

Usage - formats:
    $ yolo mode=val model=yolov8n.pt                 # PyTorch
                          yolov8n.torchscript        # TorchScript
                          yolov8n.onnx               # ONNX Runtime or OpenCV DNN with dnn=True
                          yolov8n_openvino_model     # OpenVINO
                          yolov8n.engine             # TensorRT
                          yolov8n.mlmodel            # CoreML (macOS-only)
                          yolov8n_saved_model        # TensorFlow SavedModel
                          yolov8n.pb                 # TensorFlow GraphDef
                          yolov8n.tflite             # TensorFlow Lite
                          yolov8n_edgetpu.tflite     # TensorFlow Edge TPU
                          yolov8n_paddle_model       # PaddlePaddle
"""
import json
import time
from pathlib import Path

import torch
from tqdm import tqdm

from ultralytics.cfg import get_cfg
from ultralytics.data.utils import check_cls_dataset, check_det_dataset
from ultralytics.nn.autobackend import AutoBackend
from ultralytics.utils import DEFAULT_CFG, LOGGER, RANK, SETTINGS, TQDM_BAR_FORMAT, callbacks, colorstr, emojis
from ultralytics.utils.checks import check_imgsz
from ultralytics.utils.files import increment_path
from ultralytics.utils.ops import Profile
from ultralytics.utils.torch_utils import de_parallel, select_device, smart_inference_mode

from ultralytics.utils.metrics import ConfusionMatrix, DetMetrics, box_iou
from ultralytics.utils import DEFAULT_CFG, LOGGER, ops
import numpy as np
from ultralytics.utils.metrics import ap_per_class

from .model import YOLOv8DetectionAndFeatureExtractorModel
#from .autobackend import AutoBackend


class BaseValidator:
    """
    BaseValidator

    A base class for creating validators.

    Attributes:
        dataloader (DataLoader): Dataloader to use for validation.
        pbar (tqdm): Progress bar to update during validation.
        args (SimpleNamespace): Configuration for the validator.
        model (nn.Module): Model to validate.
        data (dict): Data dictionary.
        device (torch.device): Device to use for validation.
        batch_i (int): Current batch index.
        training (bool): Whether the model is in training mode.
        speed (float): Batch processing speed in seconds.
        jdict (dict): Dictionary to store validation results.
        save_dir (Path): Directory to save results.
    """

    def __init__(self, dataloader=None, weights=None, save_dir=None, pbar=None, args=None, _callbacks=None, custom_model = None, get_feature=False, stop_at = None):
        """
        Initializes a BaseValidator instance.

        Args:
            dataloader (torch.utils.data.DataLoader): Dataloader to be used for validation.
            save_dir (Path): Directory to save results.
            pbar (tqdm.tqdm): Progress bar for displaying progress.
            args (SimpleNamespace): Configuration for the validator.
        """
        self.dataloader = dataloader
        self.pbar = pbar
        self.args = args or get_cfg(DEFAULT_CFG)
        self.weights = weights
        self.data = None
        self.device = None
        self.batch_i = None
        self.training = True
        self.speed = {'preprocess': 0.0, 'inference': 0.0, 'loss': 0.0, 'postprocess': 0.0}
        self.jdict = None
        self.custom_model = custom_model
        self.last_features = None
        self.lb = []
        self.stop_at = stop_at
        self.get_feature = get_feature

        project = self.args.project or Path(SETTINGS['runs_dir']) / self.args.task
        name = self.args.name or f'{self.args.mode}'
        self.save_dir = save_dir or increment_path(Path(project) / name,
                                                   exist_ok=self.args.exist_ok if RANK in (-1, 0) else True)
        (self.save_dir / 'labels' if self.args.save_txt else self.save_dir).mkdir(parents=True, exist_ok=True)

        if self.args.conf is None:
            self.args.conf = 0.001  # default conf=0.001

        self.plots = {}
        self.callbacks = _callbacks or callbacks.get_default_callbacks()

        self.metrics = DetMetrics(save_dir=self.save_dir, on_plot=self.on_plot)
        self.metrics.plot = self.args.plots
        self.seen = 0
        self.iouv = torch.linspace(0.5, 0.95, 10)  # iou vector for mAP@0.5:0.95
        self.niou = self.iouv.numel()

        self.res_each = []
        self.correctness = np.array([])

    def init_metrics(self, model):
        """Initialize evaluation metrics for YOLO."""
        #val = self.data.get(self.args.split, '')  # validation path
        #self.is_coco = isinstance(val, str) and 'coco' in val and val.endswith(f'{os.sep}val2017.txt')  # is COCO
        self.is_coco = False  # is COCO
        self.class_map = ops.coco80_to_coco91_class() if self.is_coco else list(range(1000))
        self.args.save_json |= self.is_coco and not self.training  # run on final val if training COCO
        self.args.save_json = False # Not saving the plots
        self.names = model.names
        self.nc = len(model.names)
        self.metrics.names = self.names
        self.metrics.plot = self.args.plots
        self.confusion_matrix = ConfusionMatrix(nc=self.nc)
        self.seen = 0
        self.jdict = []
        self.stats = []

    @smart_inference_mode()
    def __call__(self, trainer=None):#, model=None):
        """
        Supports validation of a pre-trained model if passed or a model being trained
        if trainer is passed (trainer gets priority).
        """
        self.training = trainer is not None
        augment = self.args.augment and (not self.training)
        if self.training:
            self.device = trainer.device
            self.data = trainer.data
            model = trainer.ema.ema or trainer.model
            self.args.half = self.device.type != 'cpu'  # force FP16 val during training
            model = model.half() if self.args.half else model.float()
            self.model = model
            self.loss = torch.zeros_like(trainer.loss_items, device=trainer.device)
            self.args.plots = trainer.stopper.possible_stop or (trainer.epoch == trainer.epochs - 1)
            model.eval()
        else:
            callbacks.add_integration_callbacks(self)
            self.run_callbacks('on_val_start')
            model = self.weights
            assert model is not None, 'Either trainer or model is needed for validation'
            model = AutoBackend(model,
                                device=select_device(self.args.device, self.args.batch),
                                dnn=self.args.dnn,
                                data=self.args.data,
                                fp16=self.args.half)
            self.model = model
            self.device = model.device  # update device
            self.args.half = model.fp16  # update half
            stride, pt, jit, engine = model.stride, model.pt, model.jit, model.engine
            imgsz = check_imgsz(self.args.imgsz, stride=stride)
            if engine:
                self.args.batch = model.batch_size
            elif not pt and not jit:
                self.args.batch = 1  # export.py models default to batch-size 1
                LOGGER.info(f'Forcing batch=1 square inference (1,3,{imgsz},{imgsz}) for non-PyTorch models')

            # if isinstance(self.args.data, str) and self.args.data.split('.')[-1] in ('yaml', 'yml'):
            #     self.data = check_det_dataset(self.args.data)
            # elif self.args.task == 'classify':
            #     self.data = check_cls_dataset(self.args.data, split=self.args.split)
            # else:
            #     raise FileNotFoundError(emojis(f"Dataset '{self.args.data}' for task={self.args.task} not found ❌"))

            if self.device.type == 'cpu':
                self.args.workers = 0  # faster CPU val as time dominated by inference, not dataloading
            if not pt:
                self.args.rect = False
            self.dataloader = self.dataloader or self.get_dataloader(self.data.get(self.args.split), self.args.batch)

            model.eval()
            model.warmup(imgsz=(1 if pt else self.args.batch, 3, imgsz, imgsz))  # warmup

        dt = Profile(), Profile(), Profile(), Profile()
        n_batches = len(self.dataloader)
        desc = self.get_desc()
        # NOTE: keeping `not self.training` in tqdm will eliminate pbar after segmentation evaluation during training,
        # which may affect classification task since this arg is in yolov5/classify/val.py.
        # bar = tqdm(self.dataloader, desc, n_batches, not self.training, bar_format=TQDM_BAR_FORMAT)
        #bar = tqdm(self.dataloader, desc, n_batches, bar_format=TQDM_BAR_FORMAT)
        self.init_metrics(de_parallel(model))
        self.jdict = []  # empty before each val
        start = 0
        if not(self.stop_at is None):
            if n_batches < self.stop_at:
                print("Warning: Cant stop at sample greater than the total batches. Skipping stopping early.")
            else:
                #print("ZZZZ", len(bar), self.stop_at)
                n_batches = self.stop_at
                print("Trying validation on a sample of : ", self.stop_at)
        bar = tqdm(self.dataloader, desc, n_batches, bar_format=TQDM_BAR_FORMAT)
        for batch_i, batch in enumerate(bar):
            self.run_callbacks('on_val_batch_start')
            self.batch_i = batch_i
            # Preprocess
            with dt[0]:
                if self.custom_model is None:
                    batch = self.preprocess(batch)

            # Inference
            with dt[1]:
                if self.custom_model is None:
                    torch.cuda.empty_cache()
                    preds = model(batch['img'], augment=augment)
                else:
                    if not(self.get_feature):
                        _, detection_output = self.custom_model.custom_forward(batch)
                        preds, yolov8_features = detection_output[0], detection_output[1]

                        a = yolov8_features[2][0][None, :, :, :]
                        #print("hereeee", len(yolov8_features), a.shape)
                        if self.last_features is None:
                            #self.last_features = yolov8_features[0][0].cpu().numpy()[None, :, :, :]
                            self.last_features = a
                        else:
                            #self.last_features = np.concatenate((self.last_features, yolov8_features[0][0].cpu().numpy()[None, :, :, :]), axis=0)
                            self.last_features = torch.concatenate((self.last_features, a), axis=0)
                        # if self.last_features is None:
                        #     self.last_features = []
                    else:
                        feats, detection_output = self.custom_model.get_feats_forward(batch)
                        preds, _ = detection_output[0], detection_output[1]

                        if self.last_features is None:

                            self.last_features = feats
                        else:
                            #self.last_features = np.concatenate((self.last_features, yolov8_features[0][0].cpu().numpy()[None, :, :, :]), axis=0)
                            self.last_features = np.concatenate((self.last_features, feats), axis=0)

            with dt[2]:
                if self.training:
                    self.loss += model.loss(batch, preds)[1]

            # Postprocess
            with dt[3]:
                preds = self.postprocess(preds)

            self.update_metrics(preds, batch)
            if self.args.plots and batch_i < 3:
                self.plot_val_samples(batch, batch_i)
                self.plot_predictions(batch, preds, batch_i)

            self.run_callbacks('on_val_batch_end')
            start += 1
            if self.stop_at != None and start >= self.stop_at:
                break
        stats = self.get_stats()
        self.check_stats(stats)
        self.speed = dict(zip(self.speed.keys(), (x.t / len(self.dataloader.dataset) * 1E3 for x in dt)))
        self.finalize_metrics()
        self.print_results()
        self.run_callbacks('on_val_end')
        if self.training:
            model.float()
            results = {**stats, **trainer.label_loss_items(self.loss.cpu() / len(self.dataloader), prefix='val')}
            return {k: round(float(v), 5) for k, v in results.items()}  # return results as 5 decimal place floats
        else:
            LOGGER.info('Speed: %.1fms preprocess, %.1fms inference, %.1fms loss, %.1fms postprocess per image' %
                        tuple(self.speed.values()))
            if self.args.save_json and self.jdict:
                with open(str(self.save_dir / 'predictions.json'), 'w') as f:
                    LOGGER.info(f'Saving {f.name}...')
                    json.dump(self.jdict, f)  # flatten and save
                stats = self.eval_json(stats)  # update stats
            if self.args.plots or self.args.save_json:
                LOGGER.info(f"Results saved to {colorstr('bold', self.save_dir)}")
            return stats

    def add_callback(self, event: str, callback):
        """Appends the given callback."""
        self.callbacks[event].append(callback)

    def run_callbacks(self, event: str):
        """Runs all callbacks associated with a specified event."""
        for callback in self.callbacks.get(event, []):
            callback(self)

    def get_dataloader(self, dataset_path, batch_size):
        """Get data loader from dataset path and batch size."""
        raise NotImplementedError('get_dataloader function not implemented for this validator')

    def build_dataset(self, img_path):
        """Build dataset"""
        raise NotImplementedError('build_dataset function not implemented in validator')

    def preprocess(self, batch):
        """Preprocesses batch of images for YOLO training."""
        batch['img'] = batch['img'].to(self.device, non_blocking=True)
        batch['img'] = (batch['img'].half() if self.args.half else batch['img'].float()) / 255
        for k in ['batch_idx', 'cls', 'bboxes']:
            batch[k] = batch[k].to(self.device)

        nb = len(batch['img'])
        self.lb = [torch.cat([batch['cls'], batch['bboxes']], dim=-1)[batch['batch_idx'] == i]
                   for i in range(nb)] if self.args.save_hybrid else []  # for autolabelling

        return batch
    
    def postprocess(self, preds):
        """Apply Non-maximum suppression to prediction outputs."""

        return ops.non_max_suppression(preds,
                                       self.args.conf,
                                       self.args.iou,
                                       labels=self.lb,
                                       multi_label=True,
                                       agnostic=self.args.single_cls,
                                       max_det=self.args.max_det)

    def update_metrics(self, preds, batch):
        """Metrics."""
        res_each_map = []
        for si, pred in enumerate(preds):
            
            idx = batch['batch_idx'] == si
            cls = batch['cls'][idx]
            bbox = batch['bboxes'][idx]
            nl, npr = cls.shape[0], pred.shape[0]  # number of labels, predictions
            shape = batch['ori_shape'][si]
            correct_bboxes = torch.zeros(npr, self.niou, dtype=torch.bool, device=self.device)  # init
            self.seen += 1

            if npr == 0:
                if nl:
                    self.stats.append((correct_bboxes, *torch.zeros((2, 0), device=self.device), cls.squeeze(-1)))
                    if self.args.plots:
                        self.confusion_matrix.process_batch(detections=None, labels=cls.squeeze(-1))
                continue

            # Predictions
            if self.args.single_cls:
                pred[:, 5] = 0
            predn = pred.clone()
            ops.scale_boxes(batch['img'][si].shape[1:], predn[:, :4], shape,
                            ratio_pad=batch['ratio_pad'][si])  # native-space pred

            # Evaluate
            if nl:
                height, width = batch['img'].shape[2:]
                tbox = ops.xywh2xyxy(bbox) * torch.tensor(
                    (width, height, width, height), device=self.device)  # target boxes
                ops.scale_boxes(batch['img'][si].shape[1:], tbox, shape,
                                ratio_pad=batch['ratio_pad'][si])  # native-space labels
                labelsn = torch.cat((cls, tbox), 1)  # native-space labels
                correct_bboxes = self._process_batch(predn, labelsn)
                # TODO: maybe remove these `self.` arguments as they already are member variable
                if self.args.plots:
                    self.confusion_matrix.process_batch(predn, labelsn)
                fdg = ap_per_class(correct_bboxes.cpu().numpy(), pred[:, 4].cpu().numpy(), pred[:, 5].cpu().numpy(), labelsn[:,0].cpu().numpy(), names={0: 'human'})
                res_each_map.append(fdg[5][0])
            self.stats.append((correct_bboxes, pred[:, 4], pred[:, 5], cls.squeeze(-1)))  # (conf, pcls, tcls)
            # Save
            if self.args.save_json:
                self.pred_to_json(predn, batch['im_file'][si])
            if self.args.save_txt:
                file = self.save_dir / 'labels' / f'{Path(batch["im_file"][si]).stem}.txt'
                self.save_one_txt(predn, self.args.save_conf, shape, file)
        self.res_each.append(res_each_map)

    def finalize_metrics(self, *args, **kwargs):
        """Finalizes and returns all metrics."""
        pass
    
    def get_stats(self):
        """Returns metrics statistics and results dictionary."""
        stats = [torch.cat(x, 0).cpu().numpy() for x in zip(*self.stats)]  # to numpy
        if len(stats) and stats[0].any():
            self.metrics.process(*stats)
        self.nt_per_class = np.bincount(stats[-1].astype(int), minlength=self.nc)  # number of targets per class
        return self.metrics.results_dict


    def check_stats(self, stats):
        """Checks statistics."""
        pass

    def print_results(self):
        """Prints training/validation set metrics per class."""
        pf = '%22s' + '%11i' * 2 + '%11.3g' * len(self.metrics.keys)  # print format
        LOGGER.info(pf % ('all', self.seen, self.nt_per_class.sum(), *self.metrics.mean_results()))
        if self.nt_per_class.sum() == 0:
            LOGGER.warning(
                f'WARNING ⚠️ no labels found in {self.args.task} set, can not compute metrics without labels')

        # Print results per class
        if self.args.verbose and not self.training and self.nc > 1 and len(self.stats):
            for i, c in enumerate(self.metrics.ap_class_index):
                LOGGER.info(pf % (self.names[c], self.seen, self.nt_per_class[c], *self.metrics.class_result(i)))

        if self.args.plots:
            for normalize in True, False:
                self.confusion_matrix.plot(save_dir=self.save_dir,
                                           names=self.names.values(),
                                           normalize=normalize,
                                           on_plot=self.on_plot)
                
    def _process_batch(self, detections, labels):
        """
        Return correct prediction matrix
        Arguments:
            detections (array[N, 6]), x1, y1, x2, y2, conf, class
            labels (array[M, 5]), class, x1, y1, x2, y2
        Returns:
            correct (array[N, 10]), for 10 IoU levels
        """
        iou = box_iou(labels[:, 1:], detections[:, :4])
        correct = np.zeros((detections.shape[0], self.iouv.shape[0])).astype(bool)
        correct_class = labels[:, 0:1] == detections[:, 5]
        for i in range(len(self.iouv)):
            x = torch.where((iou >= self.iouv[i]) & correct_class)  # IoU > threshold and classes match
            if x[0].shape[0]:
                matches = torch.cat((torch.stack(x, 1), iou[x[0], x[1]][:, None]),
                                    1).cpu().numpy()  # [label, detect, iou]
                if x[0].shape[0] > 1:
                    matches = matches[matches[:, 2].argsort()[::-1]]
                    matches = matches[np.unique(matches[:, 1], return_index=True)[1]]
                    # matches = matches[matches[:, 2].argsort()[::-1]]
                    matches = matches[np.unique(matches[:, 0], return_index=True)[1]]
                correct[matches[:, 1].astype(int), i] = True
        return torch.tensor(correct, dtype=torch.bool, device=detections.device)

    def get_desc(self):
        """Get description of the YOLO model."""
        pass

    @property
    def metric_keys(self):
        """Returns the metric keys used in YOLO training/validation."""
        return []

    def on_plot(self, name, data=None):
        """Registers plots (e.g. to be consumed in callbacks)"""
        self.plots[name] = {'data': data, 'timestamp': time.time()}

    # TODO: may need to put these following functions into callback
    def plot_val_samples(self, batch, ni):
        """Plots validation samples during training."""
        pass

    def plot_predictions(self, batch, preds, ni):
        """Plots YOLO model predictions on batch images."""
        pass

    def pred_to_json(self, preds, batch):
        """Convert predictions to JSON format."""
        pass

    def eval_json(self, stats):
        """Evaluate and return JSON format of prediction statistics."""
        pass
