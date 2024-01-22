from collections import deque
import numpy as np
import numba as nb

from .models.label import get_label_name
from .utils.distance import cdist, cosine
from .utils.numba import apply_along_axis, normalize_vec
from .utils.rect import get_center
from .utils.vehicle_algos import calc_velocity, get_directions,calc_acceleration
import math

class Traffic_Data:
    def __init__(self,ppkm,buffer_size):
        self.velocities = deque([], maxlen=buffer_size)
        self.accelerations = deque([], maxlen=buffer_size)
        self.directions = deque([''], maxlen=buffer_size)
        init_acc = [0, 0, 0]
        self.accelerations.append(init_acc)
        self.park_status = 0  # vehicle status, 0 = moving - default value, 1 = stopped, 2 = parkedclass ClusterFeature:
        self.park_duration = 0.0
        self.ppkm = ppkm

class ClusterFeature:
    def __init__(self, num_clusters, metric):
        self.num_clusters = num_clusters
        self.metric = metric
        self.clusters = None
        self.cluster_sizes = None
        self._next_idx = 0

    def __len__(self):
        return self._next_idx

    def __call__(self):
        return self.clusters[:self._next_idx]

    def update(self, embedding):
        if self._next_idx < self.num_clusters:
            if self.clusters is None:
                self.clusters = np.empty((self.num_clusters, len(embedding)), embedding.dtype)
                self.cluster_sizes = np.zeros(self.num_clusters, int)
            self.clusters[self._next_idx] = embedding
            self.cluster_sizes[self._next_idx] += 1
            self._next_idx += 1
        else:
            nearest_idx = self._get_nearest_cluster(self.clusters, embedding)
            self.cluster_sizes[nearest_idx] += 1
            self._seq_kmeans(self.clusters, self.cluster_sizes, embedding, nearest_idx)

    def distance(self, embeddings):
        if self.clusters is None:
            return np.ones(len(embeddings))
        clusters = normalize_vec(self.clusters[:self._next_idx])
        return apply_along_axis(np.min, cdist(clusters, embeddings, self.metric), axis=0)

    def merge(self, features, other, other_features):
        if len(features) > len(other_features):
            for feature in other_features:
                if feature is not None:
                    self.update(feature)
        else:
            for feature in features:
                if feature is not None:
                    other.update(feature)
            self.clusters = other.clusters.copy()
            self.clusters_sizes = other.cluster_sizes.copy()
            self._next_idx = other._next_idx

    @staticmethod
    @nb.njit(fastmath=True, cache=True)
    def _get_nearest_cluster(clusters, embedding):
        return np.argmin(cosine(np.atleast_2d(embedding), clusters))

    @staticmethod
    @nb.njit(fastmath=True, cache=True)
    def _seq_kmeans(clusters, cluster_sizes, embedding, idx):
        div_size = 1. / cluster_sizes[idx]
        clusters[idx] += (embedding - clusters[idx]) * div_size


class SmoothFeature:
    def __init__(self, learning_rate):
        self.lr = learning_rate
        self.smooth = None

    def __call__(self):
        return self.smooth

    def update(self, embedding):
        if self.smooth is None:
            self.smooth = embedding.copy()
        else:
            self._rolling(self.smooth, embedding, self.lr)

    @staticmethod
    @nb.njit(fastmath=True, cache=True)
    def _rolling(smooth, embedding, lr):
        smooth[:] = (1. - lr) * smooth + lr * embedding
        norm_factor = 1. / np.linalg.norm(smooth)
        smooth *= norm_factor


class AverageFeature:
    def __init__(self):
        self.sum = None
        self.avg = None
        self.count = 0

    def __call__(self):
        return self.avg

    def is_valid(self):
        return self.count > 0

    def update(self, embedding):
        self.count += 1
        if self.sum is None:
            self.sum = embedding.copy()
            self.avg = embedding.copy()
        else:
            self._average(self.sum, self.avg, embedding, self.count)

    def merge(self, other):
        self.count += other.count
        if self.sum is None:
            self.sum = other.sum
            self.avg = other.avg
        elif other.sum is not None:
            self._average(self.sum, self.avg, other.sum, self.count)

    @staticmethod
    @nb.njit(fastmath=True, cache=True)
    def _average(sum, avg, vec, count):
        sum += vec
        div_cnt = 1. / count
        avg[:] = sum * div_cnt
        norm_factor = 1. / np.linalg.norm(avg)
        avg *= norm_factor


class Track:
    _count = 0

    def __init__(self, frame_id, tlbr, state, label, confirm_hits=1, im_size=[1280, 720], stream_fps=30,
                 ppkm=[0,0], buffer_size=150):

        self.trk_id = self.next_id()
        self.start_frame = frame_id
        self.frame_ids = deque([frame_id], maxlen=buffer_size)
        self.bboxes = deque([tlbr], maxlen=buffer_size)
        self.confirm_hits = confirm_hits
        self.state = state
        self.label = label

        self.tm_frame_ids = deque([frame_id], maxlen=buffer_size)
        self.tm_bboxes = deque([tlbr], maxlen=buffer_size)

        self.stream_fps = stream_fps
        self.age = 0
        self.hits = 0
        self.avg_feat = AverageFeature()
        self.last_feat = None

        self.inlier_ratio = 1.
        self.keypoints = np.empty((0, 2), np.float32)
        self.prev_keypoints = np.empty((0, 2), np.float32)
        self.im_size = im_size
        self.is_exported = 0

        self.traffic = Traffic_Data(ppkm=ppkm,buffer_size=buffer_size)
        # self.velocities = deque([], maxlen=buffer_size)
        # self.accelerations = deque([], maxlen=buffer_size)
        # self.directions = deque([''], maxlen=buffer_size)
        # init_acc = [0,0,0]
        # self.accelerations.append(init_acc)
        # self.park_status = 0 # vehicle status, 0 = moving - default value, 1 = stopped, 2 = parked

    def __str__(self):
        x, y = get_center(self.tlbr)
        return f'{get_label_name(self.label):<10} {self.trk_id:>3} at ({int(x):>4}, {int(y):>4})'

    def __repr__(self):
        return self.__str__()

    def __len__(self):
        return self.end_frame - self.start_frame

    def __lt__(self, other):
        # ordered by approximate distance to the image plane, closer is greater
        return (self.tlbr[-1], -self.age) < (other.tlbr[-1], -other.age)

    @property
    def tlbr(self):
        return self.bboxes[-1]

    @property
    def lbl_str(self):
        return f'{get_label_name(self.label)}'

    @property
    def veloc(self):
        return self.traffic.velocities[-1][0]

    @property
    def accel(self):
        if len(self.traffic.accelerations)>1:
            return self.traffic.accelerations[-1][0]
        else:
            return 0

    @property
    def direction(self):
        return self.traffic.directions[-1]

    @property
    def end_frame(self):
        return self.tm_frame_ids[-1]

    @property
    def active(self):
        return self.age < 2

    @property
    def park_status(self):
        if self.traffic.park_status == 0:
            return "moving"
        elif self.traffic.park_status == 1:
            return f"stopped {self.traffic.park_duration:.2f}"
        elif self.traffic.park_status == 2:
            return f"parked {self.traffic.park_duration:.2f}"

    @property
    def confirmed(self):
        return self.hits >= self.confirm_hits

    def update(self, tlbr, state, frame_id, skip=False):
        self.bboxes.append(tlbr)
        self.state = state
        if skip:
            self.tm_frame_ids.append(frame_id)
            self.tm_bboxes.append(tlbr)
            self.calc_traffic_data(frame_id)

    def add_detection(self, frame_id, tlbr, state, embedding, is_valid=True):
        self.frame_ids.append(frame_id)
        self.tm_frame_ids.append(frame_id)
        self.bboxes.append(tlbr)
        self.tm_bboxes.append(tlbr)
        self.state = state
        if is_valid:
            self.last_feat = embedding
            self.avg_feat.update(embedding)
        self.age = 0
        self.hits += 1
        self.calc_traffic_data(frame_id)

    def calc_traffic_data(self,frame_id):
        # calculate velocity
        velocity, velocityX, velocityY = calc_velocity(self.tm_bboxes, self.stream_fps, self.tm_frame_ids,
                                                       self.im_size, self.traffic.ppkm)
        # calculate direction
        direction = get_directions(frame_id, self.tm_bboxes)
        direction = direction if direction != '' else self.traffic.directions[-1]
        self.traffic.directions.append(direction)
        self.traffic.velocities.append((velocity, velocityX, velocityY, frame_id))
        if len(self.traffic.velocities) > 2:
            # calculate acceleration
            acceleration, accelerationX, accelerationY = calc_acceleration(self.traffic.velocities[-1],
                                                                           self.traffic.velocities[-2],
                                                                           np.asarray(self.traffic.accelerations[-1], dtype=np.float32),
                                                                           self.stream_fps)
            self.traffic.accelerations.append((acceleration, accelerationX, accelerationY))
            # check if the vehicle is parked
            # calculate parked data
            if velocity == 0:
                self.traffic.park_duration += (frame_id - self.tm_frame_ids[-2]) / self.stream_fps
                if self.traffic.park_duration > 30:
                    self.traffic.park_status = 2
                else:
                    self.traffic.park_status = 1
            else:
                if self.traffic.park_status == 2:
                    if velocity > 15:
                        self.traffic.park_status = 0
                        self.traffic.park_duration = 0
                else:
                    self.traffic.park_status = 0
                    self.traffic.park_duration = 0

    def reinstate(self, frame_id, tlbr, state, embedding):
        self.start_frame = frame_id
        self.frame_ids.append(frame_id)
        self.bboxes.append(tlbr)
        self.state = state
        self.last_feat = embedding
        self.avg_feat.update(embedding)
        self.age = 0
        self.keypoints = np.empty((0, 2), np.float32)
        self.prev_keypoints = np.empty((0, 2), np.float32)

    def mark_missed(self):
        self.age += 1

    def file_exported(self):
        self.is_exported = 1

    def merge_continuation(self, other):
        self.frame_ids.extend(other.frame_ids)
        self.bboxes.extend(other.bboxes)
        self.state = other.state
        self.age = other.age
        self.hits += other.hits

        self.keypoints = other.keypoints
        self.prev_keypoints = other.prev_keypoints

        if other.last_feat is not None:
            self.last_feat = other.last_feat
        self.avg_feat.merge(other.avg_feat)


    @staticmethod
    def pointRadialDistance( lat1, lon1, bearing, distance):
        """
        Return final coordinates (lat2,lon2) [in degrees] given initial coordinates
        (lat1,lon1) [in degrees] and a bearing [in degrees] and distance [in km]
        """
        rEarth = 6371.01  # Earth's average radius in km
        epsilon = 0.000001  # threshold for floating-point equality
        rlat1 = math.radians(lat1)
        rlon1 = math.radians(lon1)
        rbearing = math.radians(bearing)
        rdistance = (distance / 1000) / rEarth  # normalize linear distance to radian angle
        rlat = math.asin(
            math.sin(rlat1) * math.cos(rdistance) + math.cos(rlat1) * math.sin(rdistance) * math.cos(rbearing))
        if math.cos(rlat) == 0 or abs(math.cos(rlat)) < epsilon:  # Endpoint a pole
            rlon = rlon1
        else:
            # rlon = ((rlon1 - math.asin(math.sin(rbearing) * math.sin(rdistance) / math.cos(rlat)) + math.pi) % (
            #         2 * math.pi)) - math.pi
            rlon = ((rlon1 + math.asin(math.sin(rbearing) * math.sin(rdistance) / math.cos(rlat)) + math.pi) % (
                    2 * math.pi)) - math.pi
            # rlon = rlon1 + math.atan2(math.sin(rbearing) * math.sin(rdistance) * math.cos(rlat1),
            #                          math.cos(rdistance) - math.sin(rlat1) * math.sin(rlat))

        lat = math.degrees(rlat)
        lon = math.degrees(rlon)
        # print(lat,lon)
        return (lat, lon)

    @staticmethod
    def calc_dist_gps_coords(box, altitude, gps, bearing, cam_angle, imH, imW):

        # initially taken from https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6266436
        # DEPTH AND GEOMETRY FROM A SINGLE 2D IMAGE USING TRIANGULATION , paper
        # camera's parameters
        # cam_angl = 45 # camera angle may vary depending on user's input
        # Field of view of the camera of the drone for Mavic enterprise 2
        # FOVv = 54.5 # spark
        # FOVv = 55.11 #mavic ent. 2
        # FOVh = 69.86 # spark
        # print("CAM ANGLE: " + str(cam_angle))
        cam_angle = (-1) * cam_angle


        FOVh = 68  # FOR MAVIC
        FOVv = 40  # mavic
        # FOVh = 70.57 #mavic ent. 2
        # gps coords
        lat1, long1 = gps
        bbX, bbY, bbW, bbH = box
        bbYl = int(bbY + bbH)
        if bbYl == 0:
            bbYl = 1
        y = cam_angle + ((imH / 2) - bbYl) * (FOVv / imH)  # ψ angle
        f = ((bbX + (bbW / 2)) - (imW / 2)) * (FOVh / imW)  # φ angle ( from the center of the image to the ground)
        yrad = math.radians(y)
        frad = math.radians(f)
        gdistY = altitude * (math.tan(yrad))  # Y ground distance to the object
        gdistX = gdistY * (math.tan(frad))  # X ground distance to the object
        bearing2 = bearing + f  # bearing minus the angle of the object from the center

        # if bearing2 < 0:
        # bearing = bearing2 + 360 # turn negative angles to positive

        # bearing2 = 180 + bearing2
        # if bearing2 < 0:
        # bearing2 = 360 + bearing2
        # else:
        # bearing2 = 360-bearing2
        # print(lat1,long1,bearing, bearing2, yrad, frad, gdistY, y, f)
        lat2, long2 = Track.pointRadialDistance(lat1, long1, (bearing2), gdistY)

        # print("alt %f bear %f lat %f lon %f dist %f"%(altitude,bearing,lat2,long2,gdistY))
        return gdistY, long2, lat2



    @staticmethod
    def next_id():
        Track._count += 1
        return Track._count
