class ECDFDistanceMeasureConfiguration:
    # For the Distance Measure Computation Filtering using p-Values
    filtering_active = True
    p_value_alpha_threshold: float = 0.05
    number_bootstrap_samples = 1000

Conf = ECDFDistanceMeasureConfiguration()

from enum import Enum
from typing import Tuple
from tqdm.autonotebook import tqdm

import random
import numpy as np

# For suppressing tensorflow warnings
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

try:
    import torch
    pyTensor = torch.tensor
except ImportError:
    pyTensor = None
    print("Unable to import PyTorch.")

try:
    import tensorflow as tf
    import tensorflow.experimental.numpy as tnp
    tfTensor = "tf.tensor"
except ImportError:
    tfTensor = None
    print("Unable to import Tensorflow.")


class ECDFDistanceMeasures(Enum):
    WASSERSTEIN_DISTANCE = 1
    CRAMER_VON_MISES_DISTANCE = 2
    KUIPER_DISTANCE = 3
    ANDERSON_DARLING_DISTANCE = 4
    KOLMOGOROV_SMIRNOV_DISTANCE = 5
    DTS_DISTANCE = 6


class ECDFDistanceMeasure:
    DISTANCE_TYPE = None

    def measure_metric_p_value(self, data_1: np.ndarray, data_2: np.ndarray, filter_p_value: bool = True,
                               bootstrap_samples: int = 1000, verbose=False) -> Tuple[np.ndarray, np.ndarray]:
        """The function returns measures on a set of input arrays. The
        arrays are assumed in the shape of (number, width, height,
        channels).

        :param data_1: the first dataset to be compared in the form of numpy ndarray
        :param data_2: the second dataset to be compared in the form numpy ndarray
        :param filter_p_value: Flag to Filter distance based on pVal
            less than 0.05. Default value is True.
        :param bootstrap_samples: is the optional input for pVal
            accuracy. Default value is 1000.
        :return: The outputs are Distance metric and P value.
        """
        if len(data_1.shape) == 4:
            [nA, w, h, c] = data_1.shape
            nB = data_2.shape[0]
            D = np.zeros(w * h * c)
            pVal = np.zeros(w * h * c)
        elif len(data_1.shape) == 3:
            [nA, w, h] = data_1.shape
            c = 1
            nB = data_2.shape[0]
            D = np.zeros(w * h)
            pVal = np.zeros(w * h)
        # TODO: the == 2 case seems to be required for 1d multi feature
        #  data -> required below in the gpu-version method as well
        #  Not certain whether this is the intended way to handle such
        #  input data.
        elif len(data_1.shape) == 2:
            nA, w = data_1.shape
            h = c = 1
            nB = data_2.shape[0]
            D = np.zeros(w * h)
            pVal = np.zeros(w * h)
        else:
            nA = data_1.shape[0]
            w = 1
            h = 1
            c = 1
            nB = data_2.shape[0]
            D = np.zeros(w*h)
            pVal = np.zeros(w*h)

        xxx_2 = np.array([yy.flatten() for yy in data_1])
        yyy_2 = np.array([yy.flatten() for yy in data_2])

        if w == h == c == 1:
            pVal, D = self._compute_distance_p_value(xxx_2.flatten(), yyy_2.flatten(), bootstrap_samples)
        else:
            for kk in tqdm(range(1, w * h * c), disable=verbose):
                pVal[kk], D[kk] = self._compute_distance_p_value(xxx_2[:nA, kk], yyy_2[:nB, kk], bootstrap_samples)

            if filter_p_value:
                D[pVal > Conf.p_value_alpha_threshold] = 0
                pVal[pVal > Conf.p_value_alpha_threshold] = 0

        return pVal, D

    def measure_metric_p_value_gpu(self, data_1: np.ndarray, data_2: np.ndarray, filter_p_value: bool = True,
                                   bootstrap_samples: int = 1000, verbose: bool = False) -> Tuple[pyTensor, pyTensor]:
        """The function returns measures on a set of input arrays. The
        arrays are assumed in the shape of (number, width, height,
        channels). The GPU is used for the distance computations.

        :param data_1: the first dataset to be compared in the form of numpy ndarray
        :param data_2: the second dataset to be compared in the form numpy ndarray
        :param filter_p_value: Flag to Filter distance based on pVal
            less than 0.05. Default value is True.
        :param bootstrap_samples: is the optional input for pVal
            accuracy. Default value is 1000.
        :return: Tuple of the p-Value (is a measure of suitability of
            the metric) and the Tensor including the distances.
        """
        if torch.cuda.is_available():
            DEVICE = 'cuda'
        else:
            DEVICE = 'cpu'
            print("WARNING: cant find cuda. Computation will not be accelerated.")
        
        if len(data_1.shape) == 4:
            [nA, w, h, c] = data_1.shape
            nB = data_2.shape[0]
            D = torch.zeros(w * h * c)
            pVal = torch.zeros(w * h * c)
        elif len(data_1.shape) == 3:
            [nA, w, h] = data_1.shape
            c = 1
            nB = data_2.shape[0]
            D = torch.zeros(w * h)
            pVal = torch.zeros(w * h)
        else:
            nA = data_1.shape[0]
            w = 1
            h = 1
            c = 1
            nB = data_2.shape[0]
            D = torch.zeros(w * h)
            pVal = torch.zeros(w * h)

        xxx_2 = torch.tensor(np.array([yy.flatten() for yy in data_1])).to(DEVICE)
        yyy_2 = torch.tensor(np.array([yy.flatten() for yy in data_2])).to(DEVICE)

        if w == h == c == 1:
            pVal, D = self._compute_distance_p_value_gpu(xxx_2.flatten(), yyy_2.flatten(), w, h, c, DEVICE, bootstrap_samples, verbose= verbose)
        else:
            pVal, D = self._compute_distance_p_value_gpu(xxx_2, yyy_2, w, h, c, DEVICE, bootstrap_samples, verbose= verbose)

            if filter_p_value:
                compare = pVal > Conf.p_value_alpha_threshold
                if not (w == h == c == 1):
                    D[compare.nonzero()] = 0
                    pVal[compare.nonzero()] = 0

        return pVal, D

    def measure_metric_p_value_gpu_tf(self, data_1: np.ndarray, data_2: np.ndarray, filter_p_value: bool = True,
                                   bootstrap_samples: int = 1000, verbose = False) -> Tuple[tfTensor, tfTensor]:
        """The function returns measures on a set of input arrays. The
        arrays are assumed in the shape of (number, width, height,
        channels). The GPU is used for the distance computations. 
        This version of the function uses tensorflow.

        :param data_1: the first dataset to be compared in the form of numpy ndarray
        :param data_2: the second dataset to be compared in the form numpy ndarray
        :param filter_p_value: Flag to Filter distance based on pVal
            less than 0.05. Default value is True.
        :param bootstrap_samples: is the optional input for pVal
            accuracy. Default value is 1000.
        :return: Tuple of the p-Value (is a measure of suitability of
            the metric) and the Tensor including the distances.
        """
        if len(tf.config.experimental.list_physical_devices('GPU')) > 0:
            DEVICE = '/GPU:0'
        else:
            print("WARNING: cant find CUDA. Computation will not be accelerated. ")
            DEVICE = '/CPU:0'
        
        if len(data_1.shape) == 4:
            [nA, w, h, c] = data_1.shape
            nB = data_2.shape[0]
            D = tf.zeros(w * h * c)
            pVal = tf.zeros(w * h * c)
        elif len(data_1.shape) == 3:
            [nA, w, h] = data_1.shape
            c = 1
            nB = data_2.shape[0]
            D = tf.zeros(w * h)
            pVal = tf.zeros(w * h)
        else:
            nA = data_1.shape[0]
            w = 1
            h = 1
            c = 1
            nB = data_2.shape[0]
            D = tf.zeros(w * h)
            pVal = tf.zeros(w * h)

        with tf.device(DEVICE):
            xxx_2 = tf.convert_to_tensor(np.array([yy.flatten() for yy in data_1]))
            yyy_2 = tf.convert_to_tensor(np.array([yy.flatten() for yy in data_2]))

        if w == h == c == 1:
            xxx_2 = tf.reshape(xxx_2, [-1])
            yyy_2 = tf.reshape(yyy_2, [-1])
            pVal, D = self._compute_distance_p_value_gpu_tf(xxx_2, yyy_2, w, h, c, DEVICE, bootstrap_samples, verbose = verbose)
        else:
            pVal, D = self._compute_distance_p_value_gpu_tf(xxx_2, yyy_2, w, h, c, DEVICE, bootstrap_samples, verbose = verbose)

            if filter_p_value:
                D = D.numpy()
                compare = pVal > Conf.p_value_alpha_threshold
                if not (w == h == c == 1):
                    D[compare.nonzero()] = 0
                    pVal[compare.nonzero()] = 0

        return pVal, D

    def _compute_distance(self, data_1: np.ndarray, data_2: np.ndarray) -> float:
        """This function returns the distance between the two given
        numpy arrays.

        :param data_1: flattened numpy ndarray 
        :param data_2: flattened numpy ndarray 
        :return: distance measures.
        """
        raise NotImplementedError

    def _compute_distance_p_value(self, data_1: np.ndarray, data_2: np.ndarray, bootstrap_samples: int = 1000
                                  ) -> Tuple[float, float]:
        """This function returns the distance between given numpy arrays
        and corresponding pVal.

        :param data_1: the first dataset to be compared in the form of numpy ndarray
        :param data_2: the second dataset to be compared in the form numpy ndarray
        :param bootstrap_samples: is the optional input for pVal
            accuracy. Default value is 1000.
        :return: Tuple of the p-Value as a measure of suitability of the
            metric and the distance measure.
        """
        raise NotImplementedError

    def _compute_distance_gpu(self, data_1: pyTensor, data_2: pyTensor, width: int, height: int,
                              color: int, device: str, verbose: bool = False) -> pyTensor:
        """This function returns the distance between the two given
        PyTorch Tensors.

        :param data_1: the first dataset to be compared in the form of torch tensor
        :param data_2: the second dataset to be compared in the form torch tensor
        :param width: width of datapoint (in case of image shaped data)
        :param height: height of datapoint (in case of image shaped data)
        :param color: color of datapoint (in case of image shaped data)
        :param device: device of torch tensor ('cpu' or 'cuda')
        :return: Tensor of the distances.
        """
        raise NotImplementedError

    def _compute_distance_p_value_gpu(self, data_1: pyTensor, data_2: pyTensor, width: int, height: int,
                                      color: int, device: str, bootstrap_samples: int, verbose: bool = False
                                      ) -> Tuple[pyTensor, pyTensor]:
        """This function returns the distance between the two given
        PyTorch Tensors and the corresponding p-Values.

        :param data_1: the first dataset to be compared in the form of torch tensor
        :param data_2: the second dataset to be compared in the form torch tensor
        :param width: width of datapoint (in case of image shaped data)
        :param height: height of datapoint (in case of image shaped data)
        :param color: color of datapoint (in case of image shaped data)
        :param device: device of torch tensor ('cpu' or 'cuda')
        :param bootstrap_samples: is the optional input for pVal
            accuracy. Default value is 1000.
        :return: Tuple of the p-Value (is a measure of suitability of
            the metric) and the Tensor including the distances.
        """
        raise NotImplementedError


class WassersteinDistance(ECDFDistanceMeasure):
    DISTANCE_TYPE = ECDFDistanceMeasures.WASSERSTEIN_DISTANCE

    def _compute_distance(self, data_1: np.ndarray, data_2: np.ndarray) -> float:
        nx = len(data_1)
        ny = len(data_2)
        n = nx + ny

        # print(XX)
        XY = np.concatenate([data_1, data_2])
        X2 = np.concatenate([np.repeat(1 / nx, nx), np.repeat(0, ny)])
        Y2 = np.concatenate([np.repeat(0, nx), np.repeat(1 / ny, ny)])

        S_Ind = np.argsort(XY)
        XY_Sorted = XY[S_Ind]
        X2_Sorted = X2[S_Ind]
        Y2_Sorted = Y2[S_Ind]

        Res = 0
        E_CDF = 0
        F_CDF = 0
        power = 1

        for ii in range(0, n - 2):
            E_CDF = E_CDF + X2_Sorted[ii]
            F_CDF = F_CDF + Y2_Sorted[ii]
            height = abs(F_CDF - E_CDF)
            width = XY_Sorted[ii + 1] - XY_Sorted[ii]
            Res = Res + (height ** power) * width

        return Res

    def _compute_distance_p_value(self, data_1: np.ndarray, data_2: np.ndarray, bootstrap_samples: int = 1000, verbose: bool = False
                                  ) -> Tuple[float, float]:
        WD = self._compute_distance(data_1, data_2)
        na = len(data_1)
        nb = len(data_2)
        n = na + nb
        comb = np.concatenate([data_1, data_2])
        reps = 0
        bigger = 0
        for ii in range(1, bootstrap_samples):
            e = random.sample(range(n), na)
            f = random.sample(range(n), nb)
            boost_WD = self._compute_distance(comb[e], comb[f])
            if (boost_WD > WD):
                bigger = 1 + bigger

        pVal = bigger / bootstrap_samples

        return pVal, WD

    def _compute_distance_gpu(self, data_1: pyTensor, data_2: pyTensor, width: int, height: int,
                              color: int, device: str) -> pyTensor:
        nx = len(data_1)
        ny = len(data_2)
        n = nx + ny

        XY = torch.cat([data_1, data_2]).to(device)
        X2 = np.concatenate([np.repeat(1 / nx, nx), np.repeat(0, ny)])
        Y2 = np.concatenate([np.repeat(0, nx), np.repeat(1 / ny, ny)])

        if width == height == color == 1:
            ar_x = torch.tensor(X2).to(device)
            ar_y = torch.tensor(Y2).to(device)
            S_Ind_torch = torch.argsort(XY).to(device)
            XY_sorted = XY[S_Ind_torch]
            ar_x_sorted = ar_x[S_Ind_torch]
            ar_y_sorted = ar_y[S_Ind_torch]
        else:
            ar_x = np.tile([X2], (width * height * color, 1))
            ar_x = torch.tensor(ar_x).to(device)
            ar_x = torch.transpose(ar_x, 1, 0)

            ar_y = np.tile([Y2], (width * height * color, 1))
            ar_y = torch.tensor(ar_y).to(device)
            ar_y = torch.transpose(ar_y, 1, 0)

            S_Ind_torch = torch.argsort(XY, dim=0).to(device)
            XY_sorted = torch.gather(XY, 0, S_Ind_torch)
            ar_x_sorted = torch.gather(ar_x, 0, S_Ind_torch)
            ar_y_sorted = torch.gather(ar_y, 0, S_Ind_torch)

        power = 1

        #print("DaVIIIIIIIIIDDDDDD \n\n",XY.shape, ar_x.shape, ar_y.shape , ar_y_sorted.shape, ar_x_sorted.shape, XY_sorted.shape)

        E_CDF = torch.cumsum(ar_x_sorted[:-1], dim=0)
        F_CDF = torch.cumsum(ar_y_sorted[:-1], dim=0)
        #print("\n\n\nAAAAAAAAAA\n\n\n", torch.sum(E_CDF), E_CDF.shape, torch.sum(F_CDF), F_CDF.shape)
        height_wd = abs(F_CDF - E_CDF)
        width_wd = XY_sorted[1:] - XY_sorted[0:-1]
        Res = (height_wd ** power) * width_wd
        Res = torch.sum(Res, dim=0)

        #print("ADDDDDDDDDDDD", Res.shape)

        return Res

    def _compute_distance_p_value_gpu_tf(self, data_1:tfTensor, data_2: tfTensor, width: int, height: int,
                                      color: int, device: str, bootstrap_samples: int, verbose: bool = False
                                      ) -> Tuple[tfTensor, tfTensor]:
        WD = self._compute_distance_gpu_tf(data_1, data_2, width, height, color, device)
        na = len(data_1)
        nb = len(data_2)
        n = na + nb
        with tf.device(device):
            comb = tf.concat((data_1, data_2), 0)
            reps = 0
            bigger = np.zeros([width * height * color])
        for ii in tqdm(range(1, bootstrap_samples), disable=verbose):
            e = random.sample(range(n), na)
            f = random.sample(range(n), nb)

            if len(comb.shape) > 1:
                xa = tf.nn.embedding_lookup(comb, e)
                xb = tf.nn.embedding_lookup(comb, f)
            else:
                xa = tf.nn.embedding_lookup(comb, e)
                xb = tf.nn.embedding_lookup(comb, f)

            boost_WD = self._compute_distance_gpu_tf(xa, xb, width, height, color, device)
            compare = boost_WD.numpy() > WD.numpy()
            #bigger[np.nonzero(compare)] += 1
            bigger[compare] += 1

        pVal = bigger / bootstrap_samples

        return pVal, WD

    def tf_gather(self, x, indices, gather_axis):
        # if pytorch gather indices are
        # [[[0, 10, 20], [0, 10, 20], [0, 10, 20]],
        #  [[0, 10, 20], [0, 10, 20], [0, 10, 20]]]
        # tf nd_gather needs to be
        # [[0,0,0], [0,0,10], [0,0,20], [0,1,0], [0,1,10], [0,1,20], [0,2,0], [0,2,10], [0,2,20],
        #  [1,0,0], [1,0,10], [1,0,20], [1,1,0], [1,1,10], [1,1,20], [1,2,0], [1,2,10], [1,2,20]]

        # create a tensor containing indices of each element
        all_indices = tf.where(tf.fill(indices.shape, True))
        gather_locations = tf.reshape(indices, [indices.shape.num_elements()])

        #print("AGGGGGGGGGGGGGG", x.shape, indices.shape)

        # splice in our pytorch style index at the correct axis
        gather_indices = []
        for axis in range(len(indices.shape)):
            if axis == gather_axis:
                gather_indices.append(tf.cast(gather_locations, tf.int32))
            else:
                gather_indices.append(tf.cast(all_indices[:, axis], tf.int32))

        gather_indices = tf.stack(gather_indices, axis=-1)
        gathered = tf.gather_nd(x, gather_indices)
        reshaped = tf.reshape(gathered, indices.shape)
        return reshaped

    def _compute_distance_gpu_tf(self, data_1: tfTensor, data_2: tfTensor, width: int, height: int,
                              color: int, device: str) -> tfTensor:
        nx = len(data_1)
        ny = len(data_2)
        n = nx + ny

        with tf.device(device):
            XY = tf.concat([data_1, data_2], 0)
            X2 = np.concatenate([np.repeat(1 / nx, nx), np.repeat(0, ny)])
            Y2 = np.concatenate([np.repeat(0, nx), np.repeat(1 / ny, ny)])

        if width == height == color == 1:
            with tf.device(device):
                ar_x = tf.convert_to_tensor(X2)
                ar_y =  tf.convert_to_tensor(Y2)
                S_Ind_tf = tf.argsort(XY)
                XY_sorted = tf.nn.embedding_lookup(XY, S_Ind_tf)
                ar_x_sorted = tf.nn.embedding_lookup(ar_x, S_Ind_tf)
                ar_y_sorted = tf.nn.embedding_lookup(ar_y, S_Ind_tf)
        else:
            with tf.device(device):
                ar_x = np.tile([X2], (width * height * color, 1))
                ar_x = tf.convert_to_tensor(ar_x)
                ar_x = tf.transpose(ar_x, [1, 0])

                ar_y = np.tile([Y2], (width * height * color, 1))
                ar_y = tf.convert_to_tensor(ar_y)
                ar_y = tf.transpose(ar_y, [1, 0])

                S_Ind_tf = tf.argsort(XY, axis=0)

                XY_sorted = self.tf_gather(XY, S_Ind_tf, 0)
                ar_x_sorted = self.tf_gather(ar_x, S_Ind_tf, 0)
                ar_y_sorted = self.tf_gather(ar_y, S_Ind_tf, 0)

        power = 1

        with tf.device(device):
            E_CDF = tf.cumsum(ar_x_sorted[:-1], axis=0)
            F_CDF = tf.cumsum(ar_y_sorted[:-1], axis=0)
            height_wd = abs(F_CDF - E_CDF)
            width_wd = XY_sorted[1:] - XY_sorted[0:-1]
            Res = (height_wd ** power) * width_wd
            Res = tf.math.reduce_sum(Res, axis=0)

        return Res

    def _compute_distance_p_value_gpu(self, data_1:pyTensor, data_2: pyTensor, width: int, height: int,
                                      color: int, device: str, bootstrap_samples: int, verbose: bool = False
                                      ) -> Tuple[pyTensor, pyTensor]:
        WD = self._compute_distance_gpu(data_1, data_2, width, height, color, device)
        na = len(data_1)
        nb = len(data_2)
        n = na + nb
        comb = torch.cat((data_1, data_2)).to(device)
        reps = 0
        bigger = torch.zeros([width * height * color]).to(device)
        for ii in tqdm(range(1, bootstrap_samples), disable=verbose):
            e = random.sample(range(n), na)
            f = random.sample(range(n), nb)

            if len(comb.shape) > 1:
                xa = comb[e, :]
                xb = comb[f, :]
            else:
                xa = comb[e]
                xb = comb[f]

            boost_WD = self._compute_distance_gpu(xa, xb, width, height, color, device)
            compare = boost_WD > WD
            bigger[compare.nonzero()] += 1

        pVal = bigger / bootstrap_samples

        return pVal, WD
