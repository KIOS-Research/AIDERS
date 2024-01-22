import cv2
import numpy as np
import numba as nb
import abc
from object_detection.src.utils.rect import to_tlbr, as_tlbr
import torch
import pandas as pd
from object_detection.src.utils.tiling import tilesToimg, split_image
import time
from object_detection.src.utils.rect import xyxy2xywh, xywh2xyxy
import torchvision
import torch.backends.cudnn as cudnn

# from object_detection.src.utils.torch_yv5.models.common import DetectMultiBackend
# from object_detection.src.utils.torch_yv5.utils.datasets import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
# from object_detection.src.utils.torch_yv5.utils.general import ( check_file, check_img_size, check_imshow, check_requirements, colorstr,
#                            increment_path, non_max_suppression, print_args, scale_coords, strip_optimizer, xyxy2xywh)
# from object_detection.src.utils.torch_yv5.utils.augmentations import letterbox
# from object_detection.src.utils.torch_yv5.utils.plots import Annotator, colors, save_one_box
# from object_detection.src.utils.torch_yv5.utils.torch_utils import select_device, time_sync

DET_DTYPE = np.dtype(
    [('tlbr', float, 4),
     ('label', int),
     ('conf', float)],
    align=True
)

class Detector(abc.ABC):
    @abc.abstractmethod
    def __init__(self, size):
        self.size = size


    @abc.abstractmethod
    def detect(self,frame):
        """
        Synchronizes, applies postprocessing, and returns a record array
        of detections (DET_DTYPE).
        This function should be called after `detect_async`.
        """
        raise NotImplementedError


class yolov5_torch(Detector):
    def __init__(self, size, model_file=None, conf_thresh=0.25, nms_thresh=0.4,
                 use_gpu=True, batch_size=32, tiling=False, overlap=0.2, device=0,
                 dnn=False, data='cfg/coco128.yaml', fp16=False, max_det=1000, classes=[]):

    # def __init__(self,size,model_file=None,conf_thresh=None,nms_thresh=None,
    #                     use_gpu=True, batch_size=32, tiling = False, overlap = 0.2):
        super().__init__(size)
        self.model_file = model_file
        self.conf_thresh = conf_thresh
        self.nms_thresh = nms_thresh

        # if use_gpu:
        #     self.device = select_device(device)
        # else:
        #     self.device= select_device('')
        self.batch_size = batch_size
        self.tiling = tiling
        self.detections = []
        # loading the model from torch hub
        # self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=self.model_file,
        #                             force_reload=True, device=0)
        self.model = torch.hub.load('src/utils/torch_yv5', 'custom', path=self.model_file,
                                    force_reload=True, device=0, source='local')


        self.model.conf = self.conf_thresh  # confidence threshold (0-1)
        self.model.iou =self.nms_thresh  # NMS IoU threshold (0-1)
        self.tiles_pts = None
        self.overlap = overlap

    def detect(self, images):
        if self.tiling:
            return self.detect_tiles(images)
        detects = []
        image = images[0][..., ::-1] # BGR TO RGB
        # img = torch.tensor(image, device= self.device)
        result = self.model(image, size=self.size)
        detects = result.pandas().xyxy[0]
        pred = detects[['xmin', 'ymin', 'xmax', 'ymax', 'confidence', 'class']]
        pred = pred.astype({'xmin': float, 'ymin': float, 'xmax': float, 'ymax': float})

        detections = self.postprocess(pred.to_numpy())

        detections = np.asarray(detections, dtype=DET_DTYPE).view(np.recarray)
        self.detections = detections
        return detections


    def detect_tiles(self, images):

        # if self.tiles_pts is None:
        t = time.time()
        image = images[0][..., ::-1]
        tiles,tile_pts = split_image(image, (self.size, self.size), 0.2)
        print('xronos', time.time()- t)
        detects = []

        for i in range(0, len(tiles), self.batch_size):
            t2 = time.time()
            tile = [tile for tile in tiles[i:i + self.batch_size]]
            print('xronos3', time.time() - t2)
            # print(tinle)
            t2 = time.time()
            # tiles_torch =  np.stack(tile)
            # imgs= torch.from_numpy(np.moveaxis(np.asarray(tile),-1,1)).float()
            # print(imgs,imgs.shape)
            tiles_np = np.moveaxis(np.array(tile),-1,1)
            dets = np.zeros((self.batch_size,3,512,512))
            dets[0:len(tiles_np)] = tiles_np

            print(dets[0].shape)
            # print(dets)
            result = self.model(dets,  augment=False)
            print('xronos4', time.time() - t2)
            # print('result', result[0].shape)
            t2 = time.time()
            dects = [tilesToimg(tile_pts[i + ind], res) for ind, res in enumerate(result.pandas().xyxy[:])]
            dects = pd.concat(dects, ignore_index=True)
            # concat with previous detects if exists
            detects = dects if len(detects) == 0 else pd.concat([detects, dects], ignore_index=True)
            print('xronos5', time.time() - t2)


        # perform custom NMS in case of batch
        pred = detects[['xmin', 'ymin', 'xmax', 'ymax', 'confidence', 'class']]
        pred = pred.astype({'xmin': float, 'ymin': float ,'xmax': float, 'ymax': float,
                            'confidence': float, 'class':int})
        if len(pred)>0:
            pred_ts = torch.tensor(np.expand_dims(pred.to_numpy(), axis=0), device=torch.device('cuda:0'))
            dets = self.custom_nms(pred_ts, is_xyxy=True)
            dets_np = dets[0].cpu().detach().numpy()
            # dets_df = pd.DataFrame(dets_np, columns=['xmin', 'ymin', 'xmax', 'ymax', 'confidence', 'class'])
            # dets_df['class'] = [result.names[int(x['class'])] for i, x in dets_df.iterrows()]
            # dets_df = dets_df.astype({'xmin': int, 'ymin': int, 'xmax': int, 'ymax': int})
            detections = self.postprocess(dets_np)
            detections = np.asarray(detections, dtype=DET_DTYPE).view(np.recarray)
            self.detections = detections
        else:
            detections = np.asarray([], dtype=DET_DTYPE).view(np.recarray)

        return detections


    @staticmethod
    # @nb.njit(fastmath=True, cache=True)
    def postprocess(preds):
        dets = []
        for i in range(0,len(preds)):
            # print("preds",np.array(preds[i][0:4], dtype=np.float32))
            dets.append((preds[i][0:4],
                         preds[i][5], preds[i][4]))
        return dets


    def custom_nms(self, prediction, conf_thres=0.25, iou_thres=0.45, classes=None, agnostic=False, multi_label=False,
                            labels=(), max_det=300, is_xyxy=False):
        """Runs Non-Maximum Suppression (NMS) on inference results

        Returns:
             list of detections, on (n,6) tensor per image [xyxy, conf, cls]
        """

        nc = prediction.shape[2] - 5  # number of classes
        xc = prediction[..., 4] > conf_thres  # candidates
        # Checks
        assert 0 <= conf_thres <= 1, f'Invalid Confidence threshold {conf_thres}, valid values are between 0.0 and 1.0'
        assert 0 <= iou_thres <= 1, f'Invalid IoU {iou_thres}, valid values are between 0.0 and 1.0'

        # Settings
        min_wh, max_wh = 2, 7680  # (pixels) minimum and maximum box width and height
        max_nms = 30000  # maximum number of boxes into torchvision.ops.nms()
        time_limit = 10.0  # seconds to quit after
        redundant = True  # require redundant detections
        multi_label &= nc > 1  # multiple labels per box (adds 0.5ms/img)
        merge = False  # use merge-NMS

        t = time.time()
        output = [torch.zeros((0, 6), device=prediction.device)] * prediction.shape[0]
        for xi, x in enumerate(prediction):  # image index, image inference
            # Apply constraints
            if is_xyxy:
                x[:, :4] = xyxy2xywh(x[:, :4])

            x[((x[..., 2:4] < min_wh) | (x[..., 2:4] > max_wh)).any(1), 4] = 0  # width-height
            x = x[xc[xi]]  # confidence

            # If none remain process next image
            if not x.shape[0]:
                continue

            # Compute conf
            # x[:, 5:] *= x[:, 4:5]  # conf = obj_conf * cls_conf
            # Box (center x, center y, width, height) to (x1, y1, x2, y2)
            box = xywh2xyxy(x[:, :4])
            classes_ids = self.ret_classid(x[:, 5])
            # Detections matrix nx6 (xyxy, conf, cls)
            conf, j = x[:, 4:5].max(1, keepdim=True)

            x = torch.cat((box, conf, classes_ids), 1)[conf.view(-1) > conf_thres]
            # for ind in x[:]:
            #     if ind[4] >1:

            # Check shape
            n = x.shape[0]  # number of boxes
            if not n:  # no boxes
                continue
            elif n > max_nms:  # excess boxes
                x = x[x[:, 4].argsort(descending=True)[:max_nms]]  # sort by confidence

            # Batched NMS
            c = x[:, 5:6] * (0 if agnostic else max_wh)  # classes
            boxes, scores = x[:, :4], x[:, 4]  # boxes (offset by class), scores
            i = torchvision.ops.nms(boxes, scores, iou_thres)  # NMS
            if i.shape[0] > max_det:  # limit detections
                i = i[:max_det]

            output[xi] = x[i]
            if (time.time() - t) > time_limit:
                # LOGGER.warning(f'WARNING: NMS time limit {time_limit}s exceeded')
                break  # time limit exceeded
        return output

    @staticmethod
    def ret_classid(x):
        y = x.clone() if isinstance(x, torch.Tensor) else np.copy(x)
        return torch.unsqueeze(y, dim=-1)


class yolov5(Detector):
    def __init__(self,size,model_file=None,conf_thresh=0.25,nms_thresh=0.4,
                        use_gpu=True, batch_size=32, tiling = False, overlap = 0.2, device=0,
                 dnn=False, data='cfg/coco128.yaml', fp16=False, max_det=1000, classes=[]):
        super().__init__(size)
        self.model_file = model_file
        self.conf_thresh = conf_thresh
        self.nms_thresh = nms_thresh
        self.use_gpu = use_gpu
        if self.use_gpu:
            self.device = select_device(device)
        else:
            self.device= select_device('')
        self.dnn = dnn
        self.data = data
        self.fp16 = fp16
        self.tiling = tiling
        self.batch_size = 1 if not self.tiling else batch_size
        self.detections = []
        self.max_det = max_det

        # loading the model from yolov5 repo
        self.model = DetectMultiBackend(self.model_file, device=self.device, dnn=dnn, data=data,
                                        fp16=self.fp16)
        self.classes = None if len(classes)==0 else classes
        self.model.conf = self.conf_thresh  # confidence threshold (0-1)
        self.model.iou =self.nms_thresh  # NMS IoU threshold (0-1)
        self.tiles_pts = None
        self.overlap = overlap
        self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt
        self.imgsz = check_img_size((self.size,self.size), s=self.stride)  # check image size
        cudnn.benchmark = True  # set True to speed up constant image size inference

        # Run inference
        self.model.warmup(imgsz=(1 if self.pt else self.batch_size, 3, *self.imgsz))  # warmup

    @torch.no_grad()
    def detect(self, images):
        if self.tiling:
            return self.detect_tiles(images)

        detects = []
        # load image(s)
        # dataset = LoadImages(images, img_size=self.imgsz, stride=self.stride, auto=self.pt)
        n_n = 0
        dt, seen = [0.0, 0.0, 0.0], 0

        for im in images:
            # Convert
            t1 = time_sync()
            im0s = im.copy()
            # Padded resize
            im = letterbox(im0s, self.size, stride=self.stride, auto=self.pt)[0]

            im = im.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
            im = np.ascontiguousarray(im)
            im = torch.from_numpy(im).to(self.device)
            im = im.half() if self.model.fp16 else im.float()  # uint8 to fp16/32
            im /= 255  # 0 - 255 to 0.0 - 1.0
            if len(im.shape) == 3:
                im = im[None]  # expand for batch dim
            t2 = time_sync()
            dt[0] += t2 - t1
            # Inference
            result = self.model(im, augment=False, visualize=False)
            t3 = time_sync()
            dt[1] += t3 - t2

            # NMS
            result = non_max_suppression(result, self.conf_thresh, self.nms_thresh,
                         self.classes, False, max_det=self.max_det)
            dt[2] += time_sync() - t3
            t = tuple(x / len(result) * 1E3 for x in dt)  # speeds per image
            print(
                f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *self.imgsz)}' % t)

            im0 = im0s.copy()

            # Process predictions
            detects = self.post_process(result, im, im0)
            n_n +=1

        print('imgaes', n_n)

        # dets_np = detects.cpu().detach().numpy()
        detections = np.asarray(detects, dtype=DET_DTYPE).view(np.recarray)
        self.detections = detections
        return detections

    def post_process(self, result, im, im0):
        detects = []
        for i, det in enumerate(result):  # per image
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()
                for *xyxy, conf, cls in reversed(det):
                    xyxy_list = torch.tensor(xyxy).view(-1, 4).tolist()[0]
                    class_id = cls.clone().detach().view(-1, 1).tolist()[0][0]
                    confidence = conf.clone().detach().view(-1, 1).tolist()[0][0]
                    detects.append((xyxy_list, class_id,confidence))
        return detects


    def detect_tiles(self, images):

        # if self.tiles_pts is None:
        t = time.time()
        image = images[0][..., ::-1]
        tiles,tile_pts = split_image(image, (self.size, self.size), 0.2)
        print('xronos', time.time()- t)

        detects = []


        for i in range(0, len(tiles), self.batch_size):
            t2 = time.time()
            tile = [tile for tile in tiles[i:i + self.batch_size]]
            print('xronos3', time.time() - t2)

            t2 = time.time()
            result = self.model(tile)
            print('xronos4', time.time() - t2)
            t2 = time.time()
            dects = [tilesToimg(tile_pts[i + ind], res) for ind, res in enumerate(result.pandas().xyxy[:])]
            # dects = pd.concat(dects, ignore_index=True)
            # concat with previous detects if exists
            detects = dects if len(detects) == 0 else pd.concat([detects, dects], ignore_index=True)
            print('xronos5', time.time() - t2)


        # perform custom NMS in case of batch
        pred = detects[['xmin', 'ymin', 'xmax', 'ymax', 'confidence', 'class']]
        pred = pred.astype({'xmin': float, 'ymin': float ,'xmax': float, 'ymax': float,
                            'confidence': float, 'class':int})
        if len(pred)>0:
            pred_ts = torch.tensor(np.expand_dims(pred.to_numpy(), axis=0), device=torch.device('cuda:0'))
            dets = self.custom_nms(pred_ts, is_xyxy=True)
            dets_np = dets[0].cpu().detach().numpy()
            # dets_df = pd.DataFrame(dets_np, columns=['xmin', 'ymin', 'xmax', 'ymax', 'confidence', 'class'])
            # dets_df['class'] = [result.names[int(x['class'])] for i, x in dets_df.iterrows()]
            # dets_df = dets_df.astype({'xmin': int, 'ymin': int, 'xmax': int, 'ymax': int})
            detections = self.postprocess(dets_np)
            detections = np.asarray(detections, dtype=DET_DTYPE).view(np.recarray)
            self.detections = detections
        else:
            detections = np.asarray([], dtype=DET_DTYPE).view(np.recarray)

        return detections


    @staticmethod
    # @nb.njit(fastmath=True, cache=True)
    def postprocess(preds):
        dets = []
        for i in range(0,len(preds)):
            # print("preds",np.array(preds[i][0:4], dtype=np.float32))
            dets.append((preds[i][0:4],
                         preds[i][5], preds[i][4]))
        return dets


# from object_detection.src.utils.torch_yv5.models.experimental import attempt_load


class OBJ_DETECTION():
    def __init__(self, model_path='', size=512, tiling=False, use_gpu=True, batch_size=64):

        self.tiling = tiling
        if use_gpu:
            self.device = select_device(0)
        else:
            self.device = select_device('')
        # self.yolo_model = torch.hub.load('ultralytics/yolov5', 'custom',
        #                                  path=model_path, device=self.device)
        self.batch_size = batch_size
        self.yolo_model = attempt_load(weights=model_path, map_location=self.device)
        self.size = size

    def detect(self, main_img):
        if self.tiling:
            return self.detect_tiles(main_img)

        height, width = main_img.shape[:2]

        time_rz = time.time()
        img = cv2.resize(main_img, (self.size, self.size))

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = np.moveaxis(img, -1, 0)
        img = torch.from_numpy(img).to( self.device)
        img = img.float() / 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        pred = self.yolo_model(img, augment=False)[0]
        pred = non_max_suppression(pred, conf_thres=0.25, iou_thres=0.45, classes=None)
        items = []

        if pred[0] is not None and len(pred):
            for p in pred[0]:
                score = np.round(p[4].cpu().detach().numpy(), 2)
                label = int(p[5])
                xmin = int(p[0] * main_img.shape[1] / self.size)
                ymin = int(p[1] * main_img.shape[0] / self.size)
                xmax = int(p[2] * main_img.shape[1] / self.size)
                ymax = int(p[3] * main_img.shape[0] / self.size)

                # item = [((xmin, ymin), (xmax, ymax)), label, score]

                items.append(((xmin,ymin,xmax,ymax), label, score))

        detections = np.asarray(items, dtype=DET_DTYPE).view(np.recarray)

        return detections

    def detect_tiles(self, images):
        img = cv2.cvtColor(images, cv2.COLOR_BGR2RGB)
        tiles, tile_pts = split_image(img, (self.size, self.size), 0.2)
        tiling = []

        # for tile in tiles:
        #
        #     # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        #     img = np.moveaxis(tile, -1, 0)
        #     img = torch.from_numpy(img).to(self.device)
        #     img = img.float() / 255.0  # 0 - 255 to 0.0 - 1.0
        #     if img.ndimension() == 3:
        #         img = img.unsqueeze(0)
        #     tiling.append(img)
        # tiling = torch.squeeze(torch.stack(tiling))
        # normilize 0.0 - 1.0
        # img_div = tiles / 255.0
        # # move axis
        # img_axis = np.moveaxis(np.stack(img_div),-1, 1)
        # imgs = torch.FloatTensor(img_axis).to(self.device)
        # print(imgs,imgs.shape)
        #
        # # tiles = tiles / 255.0  # 0 - 255 to 0.0 - 1.0
        # if imgs.ndimension() == 3:
        #     imgs = imgs.unsqueeze(0)
        # print(tiles.shape)
        # tiling = [tile for tile in tiles]

        # pred = self.yolo_model(torch.FloatTensor(np.moveaxis(tiles,-1,1)).to(self.device),
        pred = self.yolo_model(torch.tensor(np.moveaxis(np.asarray(tiles),-1,1))
                               .to(self.device).float())
        # for i in range(0, len(tiles), self.batch_size):
        #     tile = [tile for tile in tiles[i:i + self.batch_size]]
        #     pred = self.predict(np.moveaxis(np.asarray(tile),-1,1)
        #                        , self.yolo_model)
        print(pred[0].shape)
        # for res in pred[0] :
        #     res = torch.unsqueeze(res, 0)
        #     print(len(res2))
        #     print(res2)
        res2 = non_max_suppression(pred[0], conf_thres=0.25, iou_thres=0.45, classes=None)
        print(res2)
    def predict(self, tensor, model):
        print(tensor.shape)
        tensor = torch.tensor(tensor).to(self.device)
        print(tensor.shape)
        yhat = model(tensor.float())
        print(yhat)
        yhat = yhat.clone().detach()
        return yhat
