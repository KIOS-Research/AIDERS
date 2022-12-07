import math
from collections import defaultdict
from pathlib import Path
import configparser
import abc
import numpy as np
import numba as nb
import cupy as cp
import cupyx.scipy.ndimage
import cv2
import time

from logic.algorithms.object_detection.src.models.ssd import SSD
from logic.algorithms.object_detection.src.models.ssd import SSD
from logic.algorithms.object_detection.src.models.yolo import YOLO
from logic.algorithms.object_detection.src.models.reid import ReID
from logic.algorithms.object_detection.src.models.label import get_label_name, set_label_map
from logic.algorithms.object_detection.src.utils.inference import TRTInference
from logic.algorithms.object_detection.src.utils.rect import as_tlbr, aspect_ratio, to_tlbr, get_size, area
from logic.algorithms.object_detection.src.utils.rect import enclosing, multi_crop, iom, diou_nms, nms
from logic.algorithms.object_detection.src.utils.numba import find_split_indices
from logic.algorithms.object_detection.src.utils.tiling import tilesToimg, split_image

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

    def __call__(self, frame):
        """Detect objects synchronously."""
        self.detect_async(frame)
        return self.postprocess()

    @abc.abstractmethod
    def detect_async(self, frame):
        raise NotImplementedError

    @abc.abstractmethod
    def postprocess(self):
        raise NotImplementedError


class SSDDetector(Detector):
    def __init__(self, size,
                 class_ids,
                 model='SSDInceptionV2',
                 tile_overlap=0.25,
                 tiling_grid=(4, 2),
                 conf_thresh=0.5,
                 merge_thresh=0.6,
                 max_area=120000):
        """An object detector for SSD models.

        Parameters
        ----------
        size : tuple
            Width and height of each frame.
        class_ids : sequence
            Class IDs to detect. Note class ID starts at zero.
        model : str, optional
            SSD model to use.
            Must be the name of a class that inherits `models.SSD`.
        tile_overlap : float, optional
            Ratio of overlap to width and height of each tile.
        tiling_grid : tuple, optional
            Width and height of tile layout to split each frame for batch inference.
        conf_thresh : float, optional
            Detection confidence threshold.
        merge_thresh : float, optional
            Overlap threshold to merge bounding boxes across tiles.
        max_area : int, optional
            Max area of bounding boxes to detect.
        """
        super().__init__(size)
        self.model = SSD.get_model(model)
        assert 0 <= tile_overlap <= 1
        self.tile_overlap = tile_overlap
        assert tiling_grid[0] >= 1 and tiling_grid[1] >= 1
        self.tiling_grid = tiling_grid
        assert 0 <= conf_thresh <= 1
        self.conf_thresh = conf_thresh
        assert 0 <= merge_thresh <= 1
        self.merge_thresh = merge_thresh
        assert max_area >= 0
        self.max_area = max_area

        self.label_mask = np.zeros(self.model.NUM_CLASSES, dtype=np.bool_)
        try:
            self.label_mask[tuple(class_ids),] = True
        except IndexError as err:
            raise ValueError('Unsupported class IDs') from err

        self.batch_size = int(np.prod(self.tiling_grid))
        self.tiles, self.tiling_region_sz = self._generate_tiles()
        self.scale_factor = tuple(np.array(self.size) / self.tiling_region_sz)
        self.backend = TRTInference(self.model, self.batch_size)
        self.inp_handle = self.backend.input.host.reshape(self.batch_size, *self.model.INPUT_SHAPE)

    def detect_async(self, frame):
        """Detects objects asynchronously."""
        self._preprocess(frame)
        self.backend.infer_async()

    def postprocess(self):
        """Synchronizes, applies postprocessing, and returns a record array
        of detections (DET_DTYPE).
        This API should be called after `detect_async`.
        Detections are sorted in ascending order by class ID.
        """
        det_out = self.backend.synchronize()[0]
        detections, tile_ids = self._filter_dets(det_out, self.tiles, self.model.TOPK,
                                                 self.label_mask, self.max_area,
                                                 self.conf_thresh, self.scale_factor)
        detections = self._merge_dets(detections, tile_ids)
        return detections

    def _preprocess(self, frame):
        frame = cv2.resize(frame, self.tiling_region_sz)
        self._normalize(frame, self.tiles, self.inp_handle)

    def _generate_tiles(self):
        tile_size = np.array(self.model.INPUT_SHAPE[:0:-1])
        tiling_grid = np.array(self.tiling_grid)
        step_size = (1 - self.tile_overlap) * tile_size
        total_size = (tiling_grid - 1) * step_size + tile_size
        total_size = np.rint(total_size).astype(int)
        tiles = np.array([to_tlbr((c * step_size[0], r * step_size[1], *tile_size))
                          for r in range(tiling_grid[1]) for c in range(tiling_grid[0])])
        return tiles, tuple(total_size)

    def _merge_dets(self, detections, tile_ids):
        detections = np.fromiter(detections, DET_DTYPE, len(detections)).view(np.recarray)
        tile_ids = np.fromiter(tile_ids, int, len(tile_ids))
        if len(detections) == 0:
            return detections
        detections = self._merge(detections, tile_ids, self.batch_size, self.merge_thresh)
        return detections.view(np.recarray)

    @staticmethod
    @nb.njit(parallel=True, fastmath=True, cache=True)
    def _normalize(frame, tiles, out):
        imgs = multi_crop(frame, tiles)
        for i in nb.prange(len(imgs)):
            bgr = imgs[i]
            # BGR to RGB
            rgb = bgr[..., ::-1]
            # HWC -> CHW
            chw = rgb.transpose(2, 0, 1)
            # Normalize to [-1.0, 1.0] interval
            out[i] = chw * (2 / 255.) - 1.

    @staticmethod
    @nb.njit(fastmath=True, cache=True)
    def _filter_dets(det_out, tiles, topk, label_mask, max_area, thresh, scale_factor):
        detections = []
        tile_ids = []
        for tile_idx in range(len(tiles)):
            tile = tiles[tile_idx]
            w, h = get_size(tile)
            tile_offset = tile_idx * topk
            for det_idx in range(topk):
                offset = (tile_offset + det_idx) * 7
                label = int(det_out[offset + 1])
                conf = det_out[offset + 2]
                if conf < thresh:
                    break
                if label_mask[label]:
                    xmin = (det_out[offset + 3] * w + tile[0]) * scale_factor[0]
                    ymin = (det_out[offset + 4] * h + tile[1]) * scale_factor[1]
                    xmax = (det_out[offset + 5] * w + tile[0]) * scale_factor[0]
                    ymax = (det_out[offset + 6] * h + tile[1]) * scale_factor[1]
                    tlbr = as_tlbr((xmin, ymin, xmax, ymax))
                    if 0 < area(tlbr) <= max_area:
                        detections.append((tlbr, label, conf))
                        tile_ids.append(tile_idx)
        return detections, tile_ids

    @staticmethod
    @nb.njit(fastmath=True, cache=True)
    def _merge(dets, tile_ids, num_tile, thresh):
        # find duplicate neighbors across tiles
        neighbors = [[0 for _ in range(0)] for _ in range(len(dets))]
        for i, det in enumerate(dets):
            max_ioms = np.zeros(num_tile)
            for j, other in enumerate(dets):
                if tile_ids[i] != tile_ids[j] and det.label == other.label:
                    overlap = iom(det.tlbr, other.tlbr)
                    # use the detection with the greatest IoM from each tile
                    if overlap >= thresh and overlap > max_ioms[tile_ids[j]]:
                        max_ioms[tile_ids[j]] = overlap
                        neighbors[i].append(j)

        # merge neighbors using depth-first search
        keep = set(range(len(dets)))
        stack = []
        for i in range(len(dets)):
            if len(neighbors[i]) > 0 and tile_ids[i] != -1:
                tile_ids[i] = -1
                stack.append(i)
                candidates = []
                while len(stack) > 0:
                    for j in neighbors[stack.pop()]:
                        if tile_ids[j] != -1:
                            candidates.append(j)
                            tile_ids[j] = -1
                            stack.append(j)
                for k in candidates:
                    dets[i].tlbr[:] = enclosing(dets[i].tlbr, dets[k].tlbr)
                    dets[i].conf = max(dets[i].conf, dets[k].conf)
                    keep.discard(k)
        dets = dets[np.array(list(keep))]

        # sort detections by class
        dets = dets[np.argsort(dets.label)]
        return dets


class YOLODetector(Detector):
    def __init__(self, size,
                 class_ids,
                 model='YOLOv4',
                 conf_thresh=0.25,
                 nms_thresh=0.5,
                 max_area=800000,
                 min_aspect_ratio=0.1):
        """An object detector for YOLO models.

        Parameters
        ----------
        size : tuple
            Width and height of each frame.
        class_ids : sequence
            Class IDs to detect. Note class ID starts at zero.
        model : str, optional
            YOLO model to use.
            Must be the name of a class that inherits `models.YOLO`.
        conf_thresh : float, optional
            Detection confidence threshold.
        nms_thresh : float, optional
            Nonmaximum suppression overlap threshold.
            Set higher to detect crowded objects.
        max_area : int, optional
            Max area of bounding boxes to detect.
        min_aspect_ratio : float, optional
            Min aspect ratio (height over width) of bounding boxes to detect.
            Set to 0.1 for square shaped objects.
        """
        super().__init__(size)
        self.model = YOLO.get_model(model)
        assert 0 <= conf_thresh <= 1
        self.conf_thresh = conf_thresh
        assert 0 <= nms_thresh <= 1
        self.nms_thresh = nms_thresh
        assert max_area >= 0
        self.max_area = max_area
        assert min_aspect_ratio >= 0
        self.min_aspect_ratio = min_aspect_ratio

        self.label_mask = np.zeros(self.model.NUM_CLASSES, dtype=np.bool_)
        try:
            self.label_mask[tuple(class_ids),] = True
        except IndexError as err:
            raise ValueError('Unsupported class IDs') from err

        self.backend = TRTInference(self.model, 1)
        self.inp_handle, self.upscaled_sz, self.bbox_offset = self._create_letterbox()

    def detect_async(self, frame):
        """Detects objects asynchronously."""
        self._preprocess(frame)
        self.backend.infer_async(from_device=True)

    def postprocess(self):
        """Synchronizes, applies postprocessing, and returns a record array
        of detections (DET_DTYPE).
        This API should be called after `detect_async`.
        Detections are sorted in ascending order by class ID.
        """
        det_out = self.backend.synchronize()
        det_out = np.concatenate(det_out).reshape(-1, 7)
        detections = self._filter_dets(det_out, self.upscaled_sz, self.bbox_offset,
                                       self.label_mask, self.conf_thresh, self.nms_thresh,
                                       self.max_area, self.min_aspect_ratio)
        detections = np.fromiter(detections, DET_DTYPE, len(detections)).view(np.recarray)
        return detections

    def _preprocess(self, frame):
        zoom = np.roll(self.inp_handle.shape, -1) / frame.shape
        with self.backend.stream:
            frame_dev = cp.asarray(frame)
            # resize
            small_dev = cupyx.scipy.ndimage.zoom(frame_dev, zoom, order=1, mode='opencv', grid_mode=True)
            # BGR to RGB
            rgb_dev = small_dev[..., ::-1]
            # HWC -> CHW
            chw_dev = rgb_dev.transpose(2, 0, 1)
            # normalize to [0, 1] interval
            cp.multiply(chw_dev, 1 / 255., out=self.inp_handle)

    def _create_letterbox(self):
        src_size = np.array(self.size)
        dst_size = np.array(self.model.INPUT_SHAPE[:0:-1])
        if self.model.LETTERBOX:
            scale_factor = min(dst_size / src_size)
            scaled_size = np.rint(src_size * scale_factor).astype(int)
            img_offset = (dst_size - scaled_size) / 2
            roi = np.s_[:, img_offset[1]:img_offset[1] + scaled_size[1],
                        img_offset[0]:img_offset[0] + scaled_size[0]]
            upscaled_sz = np.rint(dst_size / scale_factor).astype(int)
            bbox_offset = (upscaled_sz - src_size) / 2
        else:
            roi = np.s_[:]
            upscaled_sz = src_size
            bbox_offset = np.zeros(2)
        inp_reshaped = self.backend.input.device.reshape(self.model.INPUT_SHAPE)
        inp_reshaped[:] = 0.5 # initial value for letterbox
        inp_handle = inp_reshaped[roi]
        return inp_handle, upscaled_sz, bbox_offset

    @staticmethod
    @nb.njit(fastmath=True, cache=True)
    def _filter_dets(det_out, size, offset, label_mask, conf_thresh, nms_thresh, max_area, min_ar):
        """
        det_out: a list of 3 tensors, where each tensor
                 contains a multiple of 7 float32 numbers in
                 the order of [x, y, w, h, box_confidence, class_id, class_prob]
        """
        # filter by class and score
        keep = []
        for i in range(len(det_out)):
            if label_mask[int(det_out[i, 5])]:
                score = det_out[i, 4] * det_out[i, 6]
                if score >= conf_thresh:
                    keep.append(i)
        det_out = det_out[np.array(keep)]

        # scale to pixel values
        det_out[:, :4] *= np.append(size, size)
        det_out[:, :2] -= offset

        # per-class NMS
        det_out = det_out[np.argsort(det_out[:, 5])]
        split_indices = find_split_indices(det_out[:, 5])
        all_indices = np.arange(len(det_out))

        keep = []
        for i in range(len(split_indices) + 1):
            begin = 0 if i == 0 else split_indices[i - 1]
            end = len(det_out) if i == len(split_indices) else split_indices[i]
            cls_dets = det_out[begin:end]
            cls_keep = diou_nms(cls_dets[:, :4], cls_dets[:, 4], nms_thresh)
            keep.extend(all_indices[begin:end][cls_keep])
        nms_dets = det_out[np.array(keep)]

        # create detections
        detections = []
        for i in range(len(nms_dets)):
            tlbr = to_tlbr(nms_dets[i, :4])
            label = int(nms_dets[i, 5])
            conf = nms_dets[i, 4] * nms_dets[i, 6]
            if 0 < area(tlbr) <= max_area and aspect_ratio(tlbr) >= min_ar:
                detections.append((tlbr, label, conf))
        return detections

class YOLODetector_batch(Detector):
    def __init__(self, size,
                 class_ids,
                 model='YOLOv4',
                 conf_thresh=0.25,
                 nms_thresh=0.5,
                 max_area=800000,
                 min_aspect_ratio=0.1,
                 tile_size = 512
                 ):
        """An object detector for YOLO models.

        Parameters
        ----------
        size : tuple
            Width and height of each frame.
        class_ids : sequence
            Class IDs to detect. Note class ID starts at zero.
        model : str, optional
            YOLO model to use.
            Must be the name of a class that inherits `models.YOLO`.
        conf_thresh : float, optional
            Detection confidence threshold.
        nms_thresh : float, optional
            Nonmaximum suppression overlap threshold.
            Set higher to detect crowded objects.
        max_area : int, optional
            Max area of bounding boxes to detect.
        min_aspect_ratio : float, optional
            Min aspect ratio (height over width) of bounding boxes to detect.
            Set to 0.1 for square shaped objects.
        """
        super().__init__(size)
        self.model = YOLO.get_model(model)
        assert 0 <= conf_thresh <= 1
        self.conf_thresh = conf_thresh
        assert 0 <= nms_thresh <= 1
        self.nms_thresh = nms_thresh
        assert max_area >= 0
        self.max_area = max_area
        assert min_aspect_ratio >= 0
        self.min_aspect_ratio = min_aspect_ratio

        self.label_mask = np.zeros(self.model.NUM_CLASSES, dtype=np.bool_)
        try:
            self.label_mask[tuple(class_ids),] = True
        except IndexError as err:
            raise ValueError('Unsupported class IDs') from err
        self.batch_size = self.model.BATCHES
        self.tile_size = tile_size
        self.xywhs = np.zeros(self.batch_size, dtype=np.int)
        self.backend = TRTInference(self.model, self.batch_size)
        self.inp_handle, self.upscaled_sz, self.bbox_offset = self._create_letterbox()


    def detect_async(self, frame):
        # """Detects objects asynchronously."""
        self.tiles,self.tile_points = [], []

        timeit1 = time.time()
        self.tiles, self.tile_points = split_image(frame, self.model.INPUT_SHAPE[1:], 0.2)
        n_patch = math.ceil(len(self.tiles)/self.batch_size) * self.batch_size
        detects = np.empty(n_patch, dtype=object)
        timeit2 = time.time()
        for i in range(0, len(self.tiles), self.batch_size):
            frames = [tile for tile in self.tiles[i:i + self.batch_size]]
            frmid = 0
            # for frame in frames:
            #     cv2.imshow(f'frame{frmid}',frame)
            #     cv2.waitKey(0)
            #     frmid+=1
            self.det_index = i
            timepre = time.time()
            self._preprocess(frames)
            timeinfer = time.time()
            self.backend.infer_async(from_device=True)
            timepost = time.time()
            detections = self.postprocess()
            for j in range(0, len(detections)):
                detects[i+j] = detections[j]
        timeit3=time.time()
        dets = [det for img in detects for det in img]
        # print(dets)
        result = np.fromiter(dets, DET_DTYPE, len(dets)).view(np.recarray)
        det_indx = nms(result.tlbr, result.conf, 0.4)
        detections = np.take(result, det_indx)
        return detections

        # print(len(detects))
    def postprocess(self):
        timestack = time.time()
        """Synchronizes, applies postprocessing, and returns a record array
        of detections (DET_DTYPE).
        This API should be called after `detect_async`.
        Detections are sorted in ascending order by class ID.
        """
        det_out = self.backend.synchronize()
        tile_pts = []
        dets_out = []
        for i in range(0, len(det_out)):
            dets = np.array(det_out[i]).reshape(self.batch_size, -1, 7)
            for j in range(0, self.batch_size):
                if i>0:
                    dets_out[j] = np.vstack((dets_out[j],dets[j]))
                else:
                    dets_out.append(dets[j])
        timefilter = time.time()
        detections = self._filter_dets(np.array(dets_out), self.upscaled_sz, self.bbox_offset,
                                       self.label_mask, self.conf_thresh, self.nms_thresh,
                                       self.max_area, self.min_aspect_ratio, self.det_index,
                                       np.array(self.tile_points))
        # detections = self._normalize_dets(detections, self.tile_points, self.det_index)
        # det_imgs = np.fromiter(detections, DET_DTYPE, len(detections)).view(np.recarray)
        # for i in range(0,len(detections)):
        #     det_imgs.append()
        return detections

    def _preprocess(self, frames):
        with self.backend.stream:
            # imgs = frames[:,0]
            # self.xywhs = frames[:,1]
            for i in nb.prange(len(frames)):
                frame_dev = cp.asarray(frames[i])
                # BGR to RGB
                rgb_dev = frame_dev[..., ::-1]
                # HWC -> CHW
                chw_dev = rgb_dev.transpose(2, 0, 1)
                # normalize to [0, 1] interval
                cp.multiply(chw_dev, 1 / 255., out=self.inp_handle[i])

    # @staticmethod
    # @nb.njit(parallel=True, fastmath=True, cache=True)
    def _normalize_dets( dets, xywhs, idx):
        # result = []
        for n in range(0, len(dets)):
            # det = np.fromiter(dets[n], DET_DTYPE, len(dets[n])).view(np.recarray)
            if (idx + n) < len(xywhs):
                xywh = xywhs[idx + n]
                # TODO ADD NUMBA FUNCTION convert(dets[n][i])
                # print(dets[n])
                # dets[n] = self.tiles_to_img(dets, n,
                #                             xywh)
                for i in range(0, len(dets[n])):
                    dets[n][i][0][0] = dets[n][i][0][0] + xywh[0]
                    dets[n][i][0][1] = dets[n][i][0][1] + xywh[1]
                    dets[n][i][0][2] = dets[n][i][0][2] + xywh[0]
                    dets[n][i][0][3] = dets[n][i][0][3] + xywh[1]

            # if len(result) == 0:
            #     if len(normalized)>0:
            #         result = normalized
            # else:
            #     if len(normalized)>0:
            #         result = np.concatenate((result, normalized), dtype=DET_DTYPE,  axis=0)

        return dets

    @staticmethod
    @nb.njit(parallel=True, fastmath=True, cache=True)
    def tiles_to_img(detets, n,  xywh):
        dets = detets[n]
        for i in range(0, len(dets)):
            dets[i][0][0] = dets[i][0][0] + xywh[0]
            dets[i][0][1] = dets[i][0][1] + xywh[1]
            dets[i][0][2] = dets[i][0][2] + xywh[0]
            dets[i][0][3] = dets[i][0][3] + xywh[1]
        return dets

    def _post_process(self, dets):
        for i in range(0,len(self.tile_points)):
            print(self.tile_points)
            det = dets[i]
            if len(det) >0:
                # print('dets0',dets[i][:])

                # print('dets',dets[i][:][0,0][0])
                dets[i][:][:,0][0] = dets[i][:][:,0][0] + self.tile_points[i][0]
                dets[i][:][:,0][0] = dets[i][:][:,0][0] + self.tile_points[i][1]
                dets[i][:][:,0][0] = dets[i][:][:,0][0] + self.tile_points[i][0]
                dets[i][:][:,0][0] = dets[i][:][:,0][0] + self.tile_points[i][1]
                # print('dets new',dets[i][:][0,0][0])

        return dets

    def _create_letterbox(self):
        src_size = np.array(self.tile_size)
        roi = np.s_[:]
        upscaled_sz = src_size
        bbox_offset = np.zeros(2)
        # self.inp_handle = self.backend.input.host.reshape(self.batch_size, *self.model.INPUT_SHAPE)

        inp_reshaped = self.backend.input.device.reshape(self.batch_size, *self.model.INPUT_SHAPE)
        inp_reshaped[:] = 0.5 # initial value for letterbox
        inp_handle = inp_reshaped[roi]
        return inp_handle, upscaled_sz, bbox_offset

    @staticmethod
    @nb.njit(fastmath=True, cache=True)
    def _filter_dets(det_out, size, offset, label_mask, conf_thresh,
                     nms_thresh, max_area, min_ar, det_index, tile_points):
        """
        det_out: a list of 3 tensors, where each tensor
                 contains a multiple of 7 float32 numbers in
                 the order of [x, y, w, h, box_confidence, class_id, class_prob]
        """
        # filter by class and score
        result = []
        for i in range(len(det_out)):
            keep = []
            # scale to pixel values
            det = det_out[i]
            for n in range(len(det)):
                if label_mask[int(det[n, 5])]:
                    score = det[n, 4] * det[n, 6]
                    if score >= conf_thresh:
                        keep.append(n)
            det = det[np.array(keep)]
            det[:, :4] *= np.append(size, size)
            det[:, :2] -= offset

            # per-class NMS
            det = det[np.argsort(det[:, 5])]
            split_indices = find_split_indices(det[:, 5])
            all_indices = np.arange(len(det))

            keep2 = []
            for j in range(len(split_indices) + 1):
                begin = 0 if j == 0 else split_indices[j - 1]
                end = len(det) if j == len(split_indices) else split_indices[i]
                cls_dets = det[begin:end]
                cls_keep = diou_nms(cls_dets[:, :4], cls_dets[:, 4], nms_thresh)
                keep2.extend(all_indices[begin:end][cls_keep])
            nms_dets = det[np.array(keep2)]

            detects = []
            # create detections
            if (i+ det_index) < len(tile_points):
                xywh = tile_points[i+det_index]
            else:
                xywj = [0, 0, 0, 0]
            for j in range(len(nms_dets)):
                tlbr = to_tlbr(nms_dets[j, :4])

                tlbr[0] = tlbr[0] + xywh[0]
                tlbr[1] = tlbr[1] + xywh[1]
                tlbr[2] = tlbr[2] + xywh[0]
                tlbr[3] = tlbr[3] + xywh[1]
                # print(tlbr,tile_points[i])
                label = int(nms_dets[j, 5])
                conf = nms_dets[j, 4] * nms_dets[j, 6]
                if 0 < area(tlbr) <= max_area and aspect_ratio(tlbr) >= min_ar:
                    # convert tile detection to img dimensions
                    detects.append((tlbr, label, conf))
            result.append(detects)
        return result


class PublicDetector(Detector):
    def __init__(self, size,
                 class_ids,
                 frame_skip,
                 sequence_path=None,
                 conf_thresh=0.5,
                 max_area=800000):
        """Class to use MOT Challenge's public detections.

        Parameters
        ----------
        size : tuple
            Width and height of each frame.
        class_ids : sequence
            Class IDs to detect. Only 1 (i.e. person) is supported.
        frame_skip : int
            Detector frame skip.
        sequence_path : str, optional
            Relative path to MOT Challenge's sequence directory.
        conf_thresh : float, optional
            Detection confidence threshold.
        max_area : int, optional
            Max area of bounding boxes to detect.
        """
        super().__init__(size)
        assert tuple(class_ids) == (1,)
        self.frame_skip = frame_skip
        assert sequence_path is not None
        self.seq_root = Path(__file__).parents[1] / sequence_path
        assert 0 <= conf_thresh <= 1
        self.conf_thresh = conf_thresh
        assert max_area >= 0
        self.max_area = max_area

        assert self.seq_root.exists()
        seqinfo = configparser.ConfigParser()
        seqinfo.read(self.seq_root / 'seqinfo.ini')
        self.seq_size = (int(seqinfo['Sequence']['imWidth']), int(seqinfo['Sequence']['imHeight']))

        self.detections = defaultdict(list)
        self.frame_id = 0

        det_txt = self.seq_root / 'det' / 'det.txt'
        for mot_challenge_det in np.loadtxt(det_txt, delimiter=','):
            frame_id = int(mot_challenge_det[0]) - 1
            tlbr = to_tlbr(mot_challenge_det[2:6])
            # mot_challenge_det[6]
            conf = 1.0
            # mot_challenge_det[7]
            label = 1 # person
            # scale inside frame
            tlbr[:2] = tlbr[:2] / self.seq_size * self.size
            tlbr[2:] = tlbr[2:] / self.seq_size * self.size
            tlbr = np.rint(tlbr)
            if conf >= self.conf_thresh and area(tlbr) <= self.max_area:
                self.detections[frame_id].append((tlbr, label, conf))

    def detect_async(self, frame):
        pass

    def postprocess(self):
        detections = np.array(self.detections[self.frame_id], DET_DTYPE).view(np.recarray)
        self.frame_id += self.frame_skip
        return detections



