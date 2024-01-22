from ultralytics.data import build_dataloader
from ultralytics.data import YOLODataset
import numpy as np
import tensorflow as tf
from tensorflow import keras
from tqdm import tqdm
from tensorflow.keras.utils import to_categorical
from tensorflow.keras.datasets import mnist, fashion_mnist, cifar10, cifar100
from tensorflow.keras import utils
from PIL import Image
from sklearn.model_selection import train_test_split
from tensorflow.keras.preprocessing.image import ImageDataGenerator
import cv2
import tensorflow as tf
import tensorflow_datasets as tfds
import pandas as pd
import os
import tensorflow as tf
import numpy as np
from sklearn.preprocessing import LabelBinarizer
import numpy as np
# import matplotlib.pyplot as plt
# import seaborn as sns
from scipy.io import loadmat
# from skimage import color
# from skimage import io
from tensorflow.keras.datasets import cifar10
from sklearn.model_selection import train_test_split
from scipy.io import loadmat
import tensorflow as tf
import numpy as np

import uuid
import pickle
import numpy as np
# import sagemaker
# import boto3
from tqdm import tqdm
import tensorflow as tf
import pathlib
import matplotlib.pyplot as plt
import tensorflow_datasets as tfds
from keras.preprocessing.image import ImageDataGenerator
def split_data(split, X, Y):
    per=1-split
    # x_test, x_val, y_test, y_val = train_test_split(X,Y, test_size=per, shuffle=False)  # 0.25 x 0.8 = 0.2
    x_test = X[:int(len(X)*per)]
    y_test=Y[:int(len(X)*per)]
    return x_test, y_test

def load_MNIST_32():
    (x_train, y_train), (x_test, y_test) = tf.keras.datasets.mnist.load_data()
    x_test, X_val, y_test, y_val = train_test_split(x_test, y_test, test_size=0.64,
                                                    random_state=1)  # 0.25 x 0.8 = 0.2

    # #expand new axis, channel axis
    x_train = np.expand_dims(x_train, axis=-1)
    x_test = np.expand_dims(x_test, axis=-1)
    X_val = np.expand_dims(x_test, axis=-1)
    # [optional]: we may need 3 channel (instead of 1)
    x_train = np.repeat(x_train, 3, axis=-1)
    x_test = np.repeat(x_test, 3, axis=-1)
    X_val = np.repeat(x_test, 3, axis=-1)
    # it's always better to normalize
    x_train = x_train.astype('float32') / 255
    x_test = x_test.astype('float32') / 255
    X_val = x_test.astype('float32') / 255
    # resize the input shape , i.e. old shape: 28, new shape: 32
    x_train = tf.image.resize(x_train, [32, 32])  # if we want to resize
    x_test = tf.image.resize(x_test, [32, 32])
    X_val = tf.image.resize(x_test, [32, 32])
    # one hot
    y_train = tf.keras.utils.to_categorical(y_train, num_classes=10)
    y_test = tf.keras.utils.to_categorical(y_test, num_classes=10)
    y_val = tf.keras.utils.to_categorical(y_test, num_classes=10)


    return x_train, y_train, x_test, y_test,X_val, y_val

def load_SVHN_(typedt):
    train_raw = loadmat('dataset/SVHN/train_32x32.mat')
    test_raw = loadmat('dataset/SVHN/test_32x32.mat')
    train_images = np.array(train_raw['X'])
    test_images = np.array(test_raw['X'])
    # print("DATA SVHN is loaded")

    train_labels = train_raw['y']
    test_labels = test_raw['y']
    # Fix the axes of the images

    train_images = np.moveaxis(train_images, -1, 0)
    test_images = np.moveaxis(test_images, -1, 0)
    lb = LabelBinarizer()
    train_labels = lb.fit_transform(train_labels)
    test_labels = lb.fit_transform(test_labels)

    # print(train_images.shape)
    # print(test_images.shape)
    # Convert train and test images into 'float64' type

    train_images = train_images.astype('float64')
    test_images = test_images.astype('float64')
    train_images /= 255.0
    test_images /= 255.0
    X_train = train_images
    y_train = train_labels
    X_test = test_images
    y_test = test_labels
    # print("DATA SVHN is Transformed")
    X_train, X_val, y_train, y_val = train_test_split(train_images, train_labels,
                                                      test_size=0.05, random_state=1)
    # print(X_train.shape)
    # print(X_val.shape)
    # print(X_test.shape)



    return X_train, y_train, X_test, y_test,X_val, y_val

def SVHN_data(typedt,channel_first=True):
    train_raw = loadmat('dataset/SVHN/train_32x32.mat')
    test_raw = loadmat('dataset/SVHN/test_32x32.mat')
    train_images = np.array(train_raw['X'])
    test_images = np.array(test_raw['X'])
    print("DATA SVHN is loaded")

    train_labels = train_raw['y']
    test_labels = test_raw['y']
    # Fix the axes of the images

    train_images = np.moveaxis(train_images, -1, 0)
    test_images = np.moveaxis(test_images, -1, 0)
    lb = LabelBinarizer()
    train_labels = lb.fit_transform(train_labels)
    test_labels = lb.fit_transform(test_labels)

    # print(train_images.shape)
    # print(test_images.shape)
    # Convert train and test images into 'float64' type

    train_images = train_images.astype('float64')
    test_images = test_images.astype('float64')
    train_images /= 255.0
    test_images /= 255.0
    X_train = train_images
    y_train = train_labels
    X_test = test_images
    y_test = test_labels
    print("DATA SVHN is Transformed")
    X_train, X_val, y_train, y_val = train_test_split(train_images, train_labels,
                                                      test_size=0.05, random_state=1)
    # print(X_train.shape)
    # print(X_val.shape)
    # print(X_test.shape)

    # if channel_first:
    #     X_train = X_train.reshape(X_train.shape[0], 1, 32, 32)
    #
    #     X_test = X_test.reshape(X_test.shape[0], 1, 32, 32)

    return X_train, y_train, X_test, y_test,X_val, y_val


def load_EMNIST(one_hot=True, channel_first=True):
    testing_letter = pd.read_csv('dataset/emnist/emnist-letters-test.csv')
    training_letter = pd.read_csv('dataset/emnist/emnist-letters-train.csv')
    # print(training_letter.shape)
    # print(testing_letter.shape)
    # training_letters
    y1 = np.array(training_letter.iloc[:, 0].values)
    x1 = np.array(training_letter.iloc[:, 1:].values)
    # testing_labels
    y2 = np.array(testing_letter.iloc[:, 0].values)
    x2 = np.array(testing_letter.iloc[:, 1:].values)
    # print(y1.shape)
    # print(x1.shape)
    # Normalise and reshape data
    train_images = x1 / 255.0
    test_images = x2 / 255.0

    train_images_number = train_images.shape[0]
    train_images_height = 28
    train_images_width = 28
    train_images_size = train_images_height * train_images_width

    train_images = train_images.reshape(train_images_number, train_images_height, train_images_width, 1)

    test_images_number = test_images.shape[0]
    test_images_height = 28
    test_images_width = 28
    test_images_size = test_images_height * test_images_width

    test_images = test_images.reshape(test_images_number, test_images_height, test_images_width, 1)
    # Transform labels
    number_of_classes = 37

    y1 = tf.keras.utils.to_categorical(y1, number_of_classes)
    y2 = tf.keras.utils.to_categorical(y2, number_of_classes)
    train_x, test_x, train_y, test_y = train_test_split(train_images, y1, test_size=0.1, random_state=42)

    return train_x, train_y, test_x, test_y






def load_CIFAR(one_hot=True):
    (X_train, y_train), (X_test, y_test) = cifar10.load_data()
    X_train, X_val, y_train, y_val = train_test_split(X_train, y_train, test_size=0.05, random_state=1) # 0.25 x 0.8 = 0.2

    if one_hot:
        y_train = utils.to_categorical(y_train, num_classes=10)
        y_test = utils.to_categorical(y_test, num_classes=10)
        y_val= utils.to_categorical(y_test, num_classes=10)

    return X_train, y_train, X_test, y_test, X_val, y_val

def resize_with_crop(image, label):
    i = image
    i = tf.cast(i, tf.float32)
    i = tf.image.resize_with_crop_or_pad(i, 224, 224)
    i = tf.keras.applications.mobilenet_v2.preprocess_input(i)
    return (i, label)
def load_Imagnet(type, channel_first=False):
    # Get imagenet labels
    labels_path = tf.keras.utils.get_file('ImageNetLabels.txt',
                                          'https://storage.googleapis.com/download.tensorflow.org/data/ImageNetLabels.txt')
    imagenet_labels = np.array(open(labels_path).read().splitlines())

    # Set data_dir to a read-only storage of .tar files
    # Set write_dir to a w/r storage
    data_dir = './dataset/imagenet/'
    write_dir_train = 'psando/tf-imagenet-dirs'
    write_dir_test = 'test/tf-imagenet-dirs'

    # Construct a tf.data.Dataset
    download_config = tfds.download.DownloadConfig(
        extract_dir=os.path.join(write_dir_train , 'extracted'),
        manual_dir=data_dir
    )
    download_and_prepare_kwargs = {
        'download_dir': os.path.join(write_dir_train , 'downloaded'),
        'download_config': download_config,
    }
    ds_train = tfds.load('imagenet2012_subset',
                   data_dir=os.path.join(write_dir_train , 'data'),
                   split='train',
                   shuffle_files=False,
                   download=True,
                   as_supervised=True,
                   download_and_prepare_kwargs=download_and_prepare_kwargs)
    download_configTest = tfds.download.DownloadConfig(
        extract_dir=os.path.join(write_dir_test, 'extracted'),
        manual_dir=data_dir
    )
    download_and_prepare_kwargsTest = {
        'download_dir': os.path.join(write_dir_test, 'downloaded'),
        'download_config': download_configTest,
    }
    ds_test = tfds.load('imagenet2012_subset',
                         data_dir=os.path.join(write_dir_test, 'data'),
                         split='test',
                         shuffle_files=False,
                         download=True,
                         as_supervised=True,
                         download_and_prepare_kwargs=download_and_prepare_kwargsTest)
    X_train, y_train, X_test, y_test = train_test_split(ds_train, imagenet_labels, test_size=0.2, random_state=1)
    print(type(X_train))
    return  X_train, y_train, X_test, y_test
def load_MNIST(one_hot=True, channel_first=True):
    """
    Load MNIST data
    :param channel_first:
    :param one_hot:
    :return:
    """
    # Load data
    (X_train, y_train), (X_test, y_test) = mnist.load_data()

    # Preprocess dataset
    # Normalization and reshaping of input.
    if channel_first:
        X_train = X_train.reshape(X_train.shape[0], 1, 28, 28)

        X_test = X_test.reshape(X_test.shape[0], 1, 28, 28)

    else:
        # X_train = X_train.astype('float32').reshape(-1, 32, 32, 1)
        # X_test = X_test.astype('float32').reshape(-1, 32, 32, 1)
        X_train = X_train.reshape(X_train.shape[0], 28, 28, 1)

        X_test = X_test.reshape(X_test.shape[0], 28, 28, 1)


    X_train = X_train.astype('float32')
    X_test = X_test.astype('float32')
    X_train /= 255
    X_test /= 255

    if one_hot:
        # For output, it is important to change number to one-hot vector.
        y_train = utils.to_categorical(y_train, num_classes=10)
        y_test = utils.to_categorical(y_test, num_classes=10)


    return X_train, y_train, X_test, y_test
def normalization(train_images, test_images):
    mean = np.mean(train_images, axis=(0, 1, 2, 3))
    std = np.std(train_images, axis=(0, 1, 2, 3))
    train_images = (train_images - mean) / (std + 1e-7)
    test_images = (test_images - mean) / (std + 1e-7)
    return train_images, test_images
def resize_data(data):
    data_upscaled = np.zeros((data.shape[0], 224, 224, 3))
    for i, img in enumerate(data):
        large_img = cv2.resize(img, dsize=(224, 224), interpolation=cv2.INTER_CUBIC)
        data_upscaled[i] = large_img

    return data_upscaled

# resize train and  test data

def load_cifar_vgg():
        (train_images, train_labels), (test_images, test_labels) = cifar10.load_data()
        train_images, X_val, train_labels, y_val = train_test_split(train_images, train_labels, test_size=0.2, random_state=1)

        # train_images = train_images.astype(np.float32)
        # test_images = test_images.astype(np.float32)

        # (train_images, test_images) = normalization(train_images, test_images)

        train_labels = to_categorical(train_labels, 10)
        test_labels = to_categorical(test_labels, 10)
        y_val=to_categorical(test_labels, 10)
        # train_dataset = tf.data.Dataset.from_tensor_slices((train_images, train_labels)).shuffle(
        #     buffer_size=10000).batch(batch_size)
        # test_dataset = tf.data.Dataset.from_tensor_slices((test_images, test_labels)).batch(batch_size)


        # X_train = resize_data(X_train)
        # test_images = resize_data(test_images)
        # X_val=resize_data(X_val)
        # print("resizing is done")
        return  train_images, train_labels, test_images, test_labels, X_val, y_val
def load_GTSRB():
    data_path = './dataset/GTSRB/'
    train_path = './dataset/GTSRB/Train'
    test_path = './dataset/GTSRB/Test'
    folders = os.listdir(train_path)
    # it will include all the classes fold name into the 'folders'
    classes = {0: 'Speed limit (20km/h)',
               1: 'Speed limit (30km/h)',
               2: 'Speed limit (50km/h)',
               3: 'Speed limit (60km/h)',
               4: 'Speed limit (70km/h)',
               5: 'Speed limit (80km/h)',
               6: 'End of speed limit (80km/h)',
               7: 'Speed limit (100km/h)',
               8: 'Speed limit (120km/h)',
               9: 'No passing',
               10: 'No passing veh over 3.5 tons',
               11: 'Right-of-way at intersection',
               12: 'Priority road',
               13: 'Yield',
               14: 'Stop',
               15: 'No vehicles',
               16: 'Veh > 3.5 tons prohibited',
               17: 'No entry',
               18: 'General caution',
               19: 'Dangerous curve left',
               20: 'Dangerous curve right',
               21: 'Double curve',
               22: 'Bumpy road',
               23: 'Slippery road',
               24: 'Road narrows on the right',
               25: 'Road work',
               26: 'Traffic signals',
               27: 'Pedestrians',
               28: 'Children crossing',
               29: 'Bicycles crossing',
               30: 'Beware of ice/snow',
               31: 'Wild animals crossing',
               32: 'End speed + passing limits',
               33: 'Turn right ahead',
               34: 'Turn left ahead',
               35: 'Ahead only',
               36: 'Go straight or right',
               37: 'Go straight or left',
               38: 'Keep right',
               39: 'Keep left',
               40: 'Roundabout mandatory',
               41: 'End of no passing',
               42: 'End no passing veh > 3.5 tons'}

    train_number = []
    class_num = []

    for folder in folders:
        train_files = os.listdir(train_path + '/' + folder)
        # add the number of each class into train_number
        train_number.append(len(train_files))
        class_num.append(classes[int(folder)])

    image_data = []
    image_labels = []

    all_classes = len(os.listdir(train_path))
    for i in range(all_classes):
        path = data_path + '/Train/' + str(i)
        images = os.listdir(path)

        for img in images:
            image = cv2.imread(path + '/' + img)
            image_fromarray = Image.fromarray(image, 'RGB')
            # tips: change the size can affect the result
            # img = cv2.resize(img, (32, 32))
            resize_image = image_fromarray.resize((32, 32))
            image_data.append(np.array(resize_image))
            image_labels.append(i)

    # Changing the list to numpy array
    image_data = np.array(image_data)
    image_labels = np.array(image_labels)

    # print(image_data.shape, image_labels.shape)
    shuffle_indexes = np.arange(image_data.shape[0])
    np.random.shuffle(shuffle_indexes)
    image_data = image_data[shuffle_indexes]
    image_labels = image_labels[shuffle_indexes]

    X_train, X_val, y_train, y_val = train_test_split(image_data, image_labels, shuffle=True, test_size=0.2,
                                                      random_state=1)

    X_train = X_train / 255
    X_val = X_val / 255

    # print("X_train.shape", X_train.shape)
    # print("X_valid.shape", X_val.shape)
    # print("y_train.shape", y_train.shape)
    # print("y_valid.shape", y_val.shape)
    y_train = keras.utils.to_categorical(y_train, all_classes)
    y_val = keras.utils.to_categorical(y_val, all_classes)
    return X_train, X_val, y_train, y_val

def load_MNISTVAL(typedt, one_hot=True, channel_first=True):
        """
        Load MNIST data
        :param channel_first:
        :param one_hot:
        :return:
        """
        if typedt=="emnist":
          # Load data
          X_train, y_train, X_test, y_test = load_EMNIST()
          # train_x, train_y, test_x, test_y
          X_train, X_val, y_train, y_val = train_test_split(X_train, y_train, test_size=0.1, random_state=1)
        if typedt=="fashion":
          # Load data
          (X_train, y_train), (X_test, y_test) = fashion_mnist.load_data()#fashion_mnist.load_data()
          X_train, X_val, y_train, y_val = train_test_split(X_train, y_train, test_size=0.1, random_state=1) # 0.25 x 0.8 = 0.2
        # Load data
        elif  typedt=="ini":
          (X_train, y_train), (X_test, y_test) = mnist.load_data()
          print ("Loading....")
          X_train, X_val, y_train, y_val = train_test_split(X_train, y_train, test_size=0.1, random_state=1) # 0.25 x 0.8 = 0.2

        # Preprocess dataset
        # Normalization and reshaping of input.
        if channel_first:
            X_train = X_train.reshape(X_train.shape[0], 1, 28, 28)
            X_test = X_test.reshape(X_test.shape[0], 1, 28, 28)
            X_val= X_val.reshape(X_val.shape[0], 1, 28, 28)
        else:
            X_train = X_train.reshape(X_train.shape[0], 28, 28, 1)
            X_test = X_test.reshape(X_test.shape[0], 28, 28, 1)
            X_val =  X_val.reshape(X_val.shape[0], 28, 28, 1)

        X_train = X_train.astype('float32')
        X_test = X_test.astype('float32')
        X_val = X_val.astype('float32')
        X_train /= 255
        X_test /= 255
        X_val /= 255

        if one_hot:
            # For output, it is important to change number to one-hot vector.
            y_train = utils.to_categorical(y_train, num_classes=10)
            y_test = utils.to_categorical(y_test, num_classes=10)
            y_val = utils.to_categorical(y_val, num_classes=10)

        return X_train, y_train, X_test, y_test, X_val, y_val
def load_CIFARVAL(typedt, one_hot=True):
    if typedt == "zeroshot":
        # Load data
        (X_train, y_train), (X_test, y_test) =cifar100.load_data(label_mode="fine")
        X_train, X_val, y_train, y_val = train_test_split(X_train, y_train, test_size=0.05,
                                                          random_state=1)  # 0.25 x 0.8 = 0.2

        if one_hot:
            y_train = utils.to_categorical(y_train, num_classes=100)
            y_test = utils.to_categorical(y_test, num_classes=100)
            y_val = utils.to_categorical(y_test, num_classes=100)
        return X_train, y_train, X_test, y_test, X_val, y_val

    elif typedt == "ini":
        (X_train, y_train), (X_test, y_test) = cifar10.load_data()
        print("Loading....")
        X_train, X_val, y_train, y_val = train_test_split(X_train, y_train, test_size=0.05,
                                                          random_state=1)  # 0.25 x 0.8 = 0.2

        if one_hot:
            y_train = utils.to_categorical(y_train, num_classes=10)
            y_test = utils.to_categorical(y_test, num_classes=10)
            y_val = utils.to_categorical(y_val, num_classes=10)
        # print("from load!!!!!")
        # print(X_val.shape, y_val.shape)
        return X_train, y_train, X_test, y_test, X_val, y_val
    elif typedt == "augmented":
        image_folder = tfds.ImageFolder("/home/sondess/sm2672/DataAugmentation_pipline/data/cifar/Augmented")
        trdata = ImageDataGenerator()
        traindata = trdata.flow_from_directory(
            directory="/home/sondess/sm2672/DataAugmentation_pipline/data/cifar/Augmented/Inpainting",
            target_size=(32, 32))
        tsdata = ImageDataGenerator()
        testdata = tsdata.flow_from_directory(
            directory="/home/sondess/sm2672/DataAugmentation_pipline/data/cifar/Augmented/Erasing",
            target_size=(32, 32))
        valdata = ImageDataGenerator()
        validdata = valdata.flow_from_directory(
            directory="/home/sondess/sm2672/DataAugmentation_pipline/data/cifar/Augmented/Noise",
            target_size=(32, 32))
        knwdata = ImageDataGenerator()
        knw_data = knwdata.flow_from_directory(
            directory="/home/sondess/sm2672/DataAugmentation_pipline/data/cifar/Augmented/Inpaintingknw",
            target_size=(32, 32))

        X_train1, y_train1 = next(traindata)  # next gives one batch images 32 in this case of coco
        X_test1, y_test = next(testdata)
        X_val1, y_val = next(validdata)
        X_knw_d, y_knw_d= next(knw_data)
        size_batch = X_train1.shape[0]
        size_test = X_test1.shape[0]
        size_val = X_val1.shape[0]
        sizeknw= X_knw_d.shape[0]
        X_trainI, y_trainI = get_xy_generator_folder(traindata, size_batch)
        X_testE, y_testE = get_xy_generator_folder(testdata, size_test)
        X_valN, y_valN = get_xy_generator_folder(validdata, size_val)
        X_knw, y_knw = get_xy_generator_folder(knw_data, sizeknw)
        return X_trainI, y_trainI,X_testE, y_testE,X_valN, y_valN,X_knw, y_knw



def load_driving_data(path='driving_data/', batch_size=64, shape=(100, 100)):
    xs = []
    ys = []
    start_load_time = time.time()
    with open(path + 'final_example.csv', 'r') as f:
        for i, line in enumerate(f):
            if i == 0:
                continue
            xs.append(path + 'center/' + line.split(',')[0] + '.jpg')
            ys.append(float(line.split(',')[1]))
    # shuffle list of images
    c = list(zip(xs, ys))
    random.shuffle(c)
    xs, ys = zip(*c)

    train_xs = xs
    train_ys = ys

    return train_xs, train_ys
def filter_val_set(desired_class, X, Y):
    """
    Filter the given sets and return only those that match the desired_class value
    :param desired_class:
    :param X:
    :param Y:
    :return:
    """
    X_class = []
    Y_class = []
    for x, y in zip(X, Y):
        if y[desired_class] == 1:
            X_class.append(x)
            Y_class.append(y)
    # print("Validation set filtered for desired class: " + str(desired_class))
    return np.array(X_class), np.array(Y_class)
def get_xy_generator_folder(train_generator, size_batch):
    train_generator.reset()
    X_train, y_train = next(train_generator)
    for i in tqdm(range(int((train_generator.samples)/size_batch))):
        img, label = next(train_generator)
        X_train = np.append(X_train, img, axis=0)
        y_train = np.append(y_train, label, axis=0)
    # print(X_train.shape, y_train.shape)
    return X_train,y_train
def get_xy_generator_flow(train_generator, size_batch):
    train_generator.reset()
    X_train, y_train = next(train_generator)
    for i in tqdm(range(int(len(train_generator)/size_batch))):
        img, label = next(train_generator)
        X_train = np.append(X_train, img, axis=0)
        y_train = np.append(y_train, label, axis=0)
    # print(X_train.shape, y_train.shape)
    return X_train,y_train
def load_cifar10_resize(img_rows, img_cols):
        # Load cifar10 training and validation sets
        (X_train, Y_train), (X_valid, Y_valid) = cifar10.load_data()

        # Resize training images
        # X_train = np.array([cv2.resize(img, (img_rows, img_cols)) for img in X_train[:, :, :, :]])
        X_valid = np.array([cv2.resize(img, (img_rows, img_cols)) for img in X_valid[:, :, :, :]])

        # Transform targets to keras compatible format
        # Y_train = utils.to_categorical(Y_train, num_classes=10)
        Y_valid = utils.to_categorical(Y_valid,  num_classes=10)

        # X_train = X_train.astype('float32')
        X_valid = X_valid.astype('float32')

        # preprocess data
        # X_train = X_train / 255.0
        X_valid = X_valid / 255.0

        return  X_valid, Y_valid
def load_coco():
    # change the folder directory depends server/local
    image_folder = tfds.ImageFolder("/home/sondess/sm2672/DataAugmentation_pipline/dataset/coco/data_structured")
    # image_folder.info
    trdata = ImageDataGenerator()
    traindata = trdata.flow_from_directory(directory="/home/sondess/sm2672/DataAugmentation_pipline/dataset/coco/data_structured/train", target_size=(32,32))
    tsdata = ImageDataGenerator()
    testdata = tsdata.flow_from_directory(directory="/home/sondess/sm2672/DataAugmentation_pipline/dataset/coco/data_structured/test", target_size=(32,32))
    vldata = ImageDataGenerator()
    valdata = vldata.flow_from_directory(directory="/home/sondess/sm2672/DataAugmentation_pipline/dataset/coco/data_structured/val", target_size=(32,32))
    # print("traindata", traindata.samples)
    X_train1, y_train1 = next(traindata)# next gives one batch images 32 in this case of coco
    # X_test, y_test = next(testdata)
    # X_val, y_val = next(testdata)
    size_batch=X_train1.shape[0]

    X_train,y_train=get_xy_generator_folder(traindata,size_batch)
    X_test, y_test = get_xy_generator_folder(testdata,size_batch)
    X_valin, y_valin = get_xy_generator_folder(valdata,size_batch)
    X_train, X_val, y_train, y_val = train_test_split(X_train, y_train, test_size=0.5,
                                                      random_state=1)
    # X_test = np.concatenate([X_test, X_valin], axis=0)
    # y_test = np.concatenate([y_test, y_valin], axis=0)
    # print("the concatenate results:",X_test.shape)
    # print("x goes to generator:",X_val.shape, y_val.shape)

    print("shapes ",X_train.shape, y_train.shape, X_test.shape, y_test.shape, X_val.shape, y_val.shape)

    # print('Using real-time data augmentation.')
    #     # This will do preprocessing and realtime data augmentation:
    # datagen = ImageDataGenerator(
    #         featurewise_center=True,  # set input mean to 0 over the dataset
    #         samplewise_center=False,  # set each sample mean to 0
    #         featurewise_std_normalization=True,  # divide inputs by std of the dataset
    #         samplewise_std_normalization=False,  # divide each input by its std
    #         zca_whitening=False,  # apply ZCA whitening
    #         zca_epsilon=1e-06,  # epsilon for ZCA whitening
    #         rotation_range=20,  # randomly rotate images in the range (degrees, 0 to 180)
    #         # randomly shift images horizontally (fraction of total width)
    #         width_shift_range=0.2,
    #         # randomly shift images vertically (fraction of total height)
    #         height_shift_range=0.2,
    #         shear_range=0.,  # set range for random shear
    #         zoom_range=0.,  # set range for random zoom
    #         channel_shift_range=0.,  # set range for random channel shifts
    #         # set mode for filling points outside the input boundaries
    #         fill_mode='nearest',
    #         cval=0.,  # value used for fill_mode = "constant"
    #         horizontal_flip=True,  # randomly flip images
    #         vertical_flip=False,  # randomly flip images
    #         # set rescaling factor (applied before any other transformation)
    #         rescale=None,
    #         # set function that will be applied on each input
    #         preprocessing_function=None,
    #         # image data format, either "channels_first" or "channels_last"
    #         data_format=None,
    #         # fraction of images reserved for validation (strictly between 0 and 1)
    #         validation_split=0.0)
    # generator for reading data from folder
    """the following worked perfectly
        val_generator = datagen.flow_from_directory(
            directory="./dataset/coco/data_structured/val",
            target_size=(224, 224),
            color_mode='rgb',
            batch_size=8,
            class_mode='categorical',
            shuffle=False,
            seed=45
        )"""
    # print("reasure:",X_val.shape)
    # # val_generator = datagen.flow(X_val, y_val,
    # #     batch_size = 8)
    # val_generatorX=[]
    # val_generatorY=[]
    # for X_batch, y_batch in datagen.flow(X_val, y_val, batch_size=9, shuffle=False):
    #     # print(X_batch.min(), X_batch.mean(), X_batch.max())
    #     val_generatorX.append(X_batch)
    #     val_generatorY.append(y_batch)
    #
    # # X_valaug1, y_valaug1 = next(val_generator)
    # # batchsize = X_valaug1.shape[0]
    # # print (X_valaug1.shape[0])
    # print ("valgenrator",val_generatorX.shape)
    # # X_valaug, y_valaug= get_xy_generator_flow(val_generator,batchsize)


    val_X, val_Y = load_cifar10_resize(32, 32)

    return X_train, y_train, X_test, y_test, X_val, y_val, val_X[:2000], val_Y[:2000]
def load_data(data_file):
    pickle_in=open(data_file,"rb")
    x_train=pickle.load(pickle_in)
    return x_train
def load_leaves_V(type):
    if type=='original':
        image_folder = tfds.ImageFolder("/home/sondess/sm2672/DataAugmentation_pipline/dataset/Grapes")
        trdata = ImageDataGenerator()
        traindata = trdata.flow_from_directory(directory="/home/sondess/sm2672/DataAugmentation_pipline/dataset/Grapes/train", target_size=(256, 256))
        tsdata = ImageDataGenerator()
        testdata = tsdata.flow_from_directory(directory="/home/sondess/sm2672/DataAugmentation_pipline/dataset/Grapes/test", target_size=(256, 256))


        X_train1, y_train1 = next(traindata)# next gives one batch images 32 in this case of coco
        X_test1, y_test = next(testdata)
        # X_val, y_val = next(testdata)
        size_batch=X_train1.shape[0]
        size = X_test1.shape[0]

        X_train,y_train=get_xy_generator_folder(traindata,size_batch)
        X_test, y_test = get_xy_generator_folder(testdata,size)

        X_train, X_val, y_train, y_val = train_test_split(X_train, y_train, test_size=0.5,
                                                          random_state=1)
        # print("size", X_train.shape)
        "gray scale images "
        # train_z = '/home/sondess/sm2672/DataAugmentation_pipline/dataset/Grapes/Grey/x_test_gray.pickle'
        # test_z = '/home/sondess/sm2672/DataAugmentation_pipline/dataset/Grapes/Grey/y_test_gray.pickle'
        # X_test_z = load_data(train_z)
        # y_test_z = load_data(test_z)
        # print(X_test_z.shape)
        # y_test_z = to_categorical(y_test_z, 4)
        # X_test_z = np.array(X_test_z).reshape(-1, 256, 256, 3)
        zdata = ImageDataGenerator()
        valzdata = zdata.flow_from_directory(
            directory="/home/sondess/sm2672/DataAugmentation_pipline/dataset/Grapes/Jatropha", target_size=(256, 256))
        X_z, y_z = next(valzdata)
        sizez = X_z.shape[0]
        X_test_z, y_test_z = get_xy_generator_folder(valzdata, sizez)
        # print("!!!!!!!",  y_test_z.shape,X_test_z.shape)
        return X_train, y_train, X_test, y_test, X_val, y_val, X_test_z,y_test_z
    elif type=='augmented':
        image_folder = tfds.ImageFolder("/home/sondess/sm2672/DataAugmentation_pipline/data/Grapes/Augmented")
        trdata = ImageDataGenerator()
        traindata = trdata.flow_from_directory(
            directory="/home/sondess/sm2672/DataAugmentation_pipline/data/Grapes/Augmented/Inpainting", target_size=(256, 256))
        tsdata = ImageDataGenerator()
        testdata = tsdata.flow_from_directory(
            directory="/home/sondess/sm2672/DataAugmentation_pipline/data/Grapes/Augmented/Erasing", target_size=(256, 256))
        valdata = ImageDataGenerator()
        validdata = valdata.flow_from_directory(
            directory="/home/sondess/sm2672/DataAugmentation_pipline/data/Grapes/Augmented/Noise",
            target_size=(256, 256))

        Knw_data = ImageDataGenerator()
        Knw_inpdata = Knw_data.flow_from_directory(
            directory="/home/sondess/sm2672/DataAugmentation_pipline/data/Grapes/Augmented/Inpaintingknw",
            target_size=(256, 256))
        X_knw, y_knw = next(Knw_inpdata)
        X_train1, y_train1 = next(traindata)  # next gives one batch images 32 in this case of coco
        X_test1, y_test = next(testdata)
        X_val1, y_val = next(validdata)
        size_batch = X_train1.shape[0]
        size_test = X_test1.shape[0]
        size_val = X_val1.shape[0]
        sizeknw=X_knw.shape[0]

        X_train, y_train = get_xy_generator_folder(traindata, size_batch)
        X_test, y_test = get_xy_generator_folder(testdata, size_test)
        X_val, y_val = get_xy_generator_folder(validdata, size_val)
        X_knw, y_knw = get_xy_generator_folder(Knw_inpdata, sizeknw)



        # print("shapes ", X_train.shape, y_train.shape, X_test.shape, y_test.shape, X_val.shape, y_val.shape)


        return X_train, y_train, X_test, y_test, X_val, y_val,X_knw, y_knw


def load_coco_augmented():
    print("load augmented data ...")
    """ shall works if we have categories folders inside each augmented folder"""
    image_folder = tfds.ImageFolder("/home/sondess/sm2672/DataAugmentation_pipline/data/coco/Augmented")
    image_folder.info
    trdata = ImageDataGenerator()
    print("change reshape")
    inpaintdata = trdata.flow_from_directory(directory="/home/sondess/sm2672/DataAugmentation_pipline/data/coco/Augmented/Inpainting", target_size=(32,32))
    tsdata = ImageDataGenerator()
    erasedata = tsdata.flow_from_directory(directory="/home/sondess/sm2672/DataAugmentation_pipline/data/coco/Augmented/Erasing", target_size=(32,32))
    vldata = ImageDataGenerator()
    noisedata = vldata.flow_from_directory(directory="/home/sondess/sm2672/DataAugmentation_pipline/data/coco/Augmented/Noise", target_size=(32,32))
    # print("traindata", traindata.samples)
    X_train1, y_train1 = next(inpaintdata)# next gives one batch images 32 in this case of coco
    X_test1, y_test1 = next(erasedata)
    X_val1, y_val1 = next(noisedata)
    size_batch=X_train1.shape[0]
    print('size_batch', size_batch)
    batchE = X_test1.shape[0]
    # print('size_batch erase', batchE)
    batchN = X_val1.shape[0]
    # print('size_batch noise', batchN)
    X_inpaint,y_inpaint=get_xy_generator_folder(inpaintdata,size_batch)
    X_erase, y_erase = get_xy_generator_folder(erasedata,batchE)
    X_noise, y_noise = get_xy_generator_folder(noisedata,batchN)






    # xinpaint = 'x_test_aug_inpainting.pickle'
    # yinpaint = 'y_test_aug_inpainting.pickle'
    # xerase = 'x_test_aug_erase.pickle'
    # yerase = 'y_test_aug_erase.pickle'
    # pickle_in1 = open(xinpaint , "rb")
    # X_inpaint = pickle.load(pickle_in1)
    # X_inpaint = np.array(X_inpaint).reshape(-1, 256, 256, 3)
    # pickle_in2 = open(yinpaint, "rb")
    # y_inpaint = pickle.load(pickle_in2)
    # pickle_in3 = open(xerase, "rb")
    # X_erase = pickle.load(pickle_in3)
    # # X_erase  = np.array(X_erase ).reshape(-1, 256, 256, 3)
    # pickle_in4 = open(yerase, "rb")
    # y_erase = pickle.load(pickle_in4)


    print("shapes ",X_inpaint.shape, y_inpaint.shape, X_erase.shape, y_erase.shape)

    return X_inpaint,y_inpaint,X_erase, y_erase,X_noise, y_noise



def get_yolo_dataset(weight_path, data_path, img_path, batch, mode='val'):
    from ultralytics.utils import DEFAULT_CFG
    from ultralytics.data import build_yolo_dataset
    import yaml

    DEFAULT_CFG.mode = 'val'
    DEFAULT_CFG.model = weight_path
    DEFAULT_CFG.save = False
    DEFAULT_CFG.batch = batch
    DEFAULT_CFG.single_cls = True # False # for all data
    DEFAULT_CFG.cache = None
    DEFAULT_CFG.plot = False
    with open(data_path, 'r') as file:
        yfile = yaml.safe_load(file)
    DEFAULT_CFG.data = yfile

    dataset = build_yolo_dataset(DEFAULT_CFG, img_path, DEFAULT_CFG.batch, DEFAULT_CFG.data, mode=mode)
    return(dataset)
