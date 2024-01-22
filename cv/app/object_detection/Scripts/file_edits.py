import os, sys
from shutil import copy2, move
import random
import numpy as np
import cv2

# === Cars: 248413===
# === Motorbikes: 535===
# === Busses: 2002===
# === Trucks: 7744===
folder = os.getcwd()


def check_balance(txt):
    ccars, cbus, ctruck, cmotorbikes = 0, 0, 0, 0

    with open(txt, 'r+') as fl:
        imgfiles = fl.readlines()
        for img in imgfiles:
            filename_base = img.split('.')[-2] + '.txt'
            if not os.path.isfile(filename_base): continue
            with open(filename_base, 'r+') as f:
                lines = f.readlines()
                f.seek(0)
                for line in lines:
                    if line[0] == '1':
                        cbus += 1
                    elif line[0] == '2':
                        ctruck += 1
                    elif line[0] == '3':
                        cmotorbikes += 1
                    elif line[0] == '0':
                        ccars += 1
    print(f"=== Cars: {ccars}===\n")
    print(f"=== Motorbikes: {cmotorbikes}===\n")
    print(f"=== Busses: {cbus}===\n")
    print(f"=== Trucks: {ctruck}===\n")


def get_filenames():
    txt_filenames = []
    for path, subdirs, files in os.walk(folder):
        for filename in files:
            infilename = os.path.join(path, filename)
            if not os.path.isfile(infilename): continue
            if infilename.endswith('.txt'):  # check if ifle is txt format
                filename_base = infilename.split('.txt')[-2]
                imgname = None
                if os.path.exists(filename_base + '.jpg') or os.path.exists(filename_base + '.JPG') \
                        or os.path.exists(filename_base + '.png') or os.path.exists(filename_base + '.PNG'):
                    txt_filenames.append(infilename)
    return txt_filenames


def delete_random_elems(input_list, to_delete):
    return [x for x in input_list if not x in to_delete]


# with this function you can change first letter of each txt file (folder + subfolders)
def change_lines():
    # folder = r"D:\Kios\Dataset\Vehicles\Validation\kitti/"
    # folder = r"D:\Kios\Dataset\Vehicles\Train/"
    ccars, cbus, ctruck, cmotorbikes = 0, 0, 0, 0
    no_img = []

    busses, trucks, motorbikes, cars = [], [], [], []
    bus_truck, bus_motorbike, bus_car = [], [], []
    truck_motorbike, truck_car, car_motorbike, all = [], [], [], []
    img_num = 0

    txt_filenames = get_filenames()
    print(len(set(txt_filenames)))
    while (len(txt_filenames) > 0):
        rndm_txt = random.choice(txt_filenames)
        filename_base = rndm_txt.split('.txt')[-2]
        # get the actual image name
        if os.path.exists(filename_base + '.jpg'):
            imgname = filename_base + '.jpg'
        elif os.path.exists(filename_base + '.JPG'):
            imgname = filename_base + '.JPG'
        elif os.path.exists(filename_base + '.png'):
            imgname = filename_base + '.png'
        elif os.path.exists(filename_base + '.PNG'):
            imgname = filename_base + '.PNG'

        with open(rndm_txt, 'r+') as f:
            img_num += 1
            lines = f.readlines()
            f.seek(0)
            flag0, flag1, flag2, flag3 = 0, 0, 0, 0
            for line in lines:
                if line[0] == '1':
                    cbus += 1
                    flag1 = 1
                elif line[0] == '2':
                    ctruck += 1
                    flag2 = 1
                elif line[0] == '3':
                    cmotorbikes += 1
                elif line[0] == '0':
                    ccars += 1
                    flag0 = 1

            if flag1 and not (flag2 or flag3 or flag0):
                busses.append(imgname)
            elif flag2 and not (flag1 or flag3 or flag0):
                trucks.append(imgname)
            elif flag0 and not (flag1 or flag3 or flag2):
                cars.append(imgname)
            elif flag3 and not (flag1 or flag2 or flag0):
                motorbikes.append(imgname)
            elif flag1 and flag2 and not (flag3 or flag0):
                bus_truck.append(imgname)
            elif flag1 and flag3 and not (flag2 or flag0):
                bus_motorbike.append(imgname)
            elif flag2 and flag3 and not (flag1 or flag0):
                truck_motorbike.append(imgname)
            elif flag1 and flag0 and not (flag2 or flag3):
                bus_car.append(imgname)
            elif flag0 and flag2 and not (flag1 or flag3):
                truck_car.append(imgname)
            elif flag3 and flag0 and not (flag2 or flag1):
                car_motorbike.append(imgname)
            else:  # if flag1 and flag2 and flag3 and flag0:
                all.append(imgname)
        txt_filenames.remove(rndm_txt)
    train, test, val = [], [], []

    for i in range(0, 3):
        dataset_part = []

        if i == 1:  # test
            npart = 0.50
        elif i == 0:
            npart = 0.6
        else:
            npart = 1.0

        car_part = random.sample(cars, int(len(cars) * npart))
        cars = delete_random_elems(cars, car_part)

        bus_part = random.sample(busses, int(len(busses) * npart))
        busses = delete_random_elems(busses, bus_part)

        truck_part = random.sample(trucks, int(len(trucks) * npart))
        trucks = delete_random_elems(trucks, truck_part)

        motor_part = random.sample(motorbikes, int(len(motorbikes) * npart))
        motorbikes = delete_random_elems(motorbikes, motor_part)

        bus_truck_part = random.sample(bus_truck, int(len(bus_truck) * npart))
        bus_truck = delete_random_elems(bus_truck, bus_truck_part)

        bus_motorbike_part = random.sample(bus_motorbike, int(len(bus_motorbike) * npart))
        bus_motorbike = delete_random_elems(bus_motorbike, bus_motorbike_part)

        truck_motorbike_part = random.sample(truck_motorbike, int(len(truck_motorbike) * npart))
        truck_motorbike = delete_random_elems(truck_motorbike, truck_motorbike_part)

        bus_car_part = random.sample(bus_car, int(len(bus_car) * npart))
        bus_car = delete_random_elems(bus_car, bus_car_part)

        truck_car_part = random.sample(truck_car, int(len(truck_car) * npart))
        truck_car = delete_random_elems(truck_car, truck_car_part)

        car_motorbike_part = random.sample(car_motorbike, int(len(car_motorbike) * npart))
        car_motorbike = delete_random_elems(car_motorbike, car_motorbike_part)

        all_part = random.sample(all, int(len(all) * npart))
        all = delete_random_elems(all, all_part)

        dataset_part = bus_part + truck_part + motor_part + bus_truck_part + bus_motorbike_part + truck_motorbike_part + \
                       all_part + car_part + bus_car_part + truck_car_part + car_motorbike_part

        if i == 2:
            test = set(dataset_part)
        elif i == 1:
            val = set(dataset_part)
        elif i == 0:
            train = set(dataset_part)

    trainfile = open("train.txt", "w")
    for element in train:
        trainfile.write(element + "\n")
    trainfile.close()

    testfile = open("test.txt", "w")
    for element in test:
        testfile.write(element + "\n")
    testfile.close()

    valfile = open("valid.txt", "w")
    for element in val:
        valfile.write(element + "\n")
    valfile.close()

    # dataset_15pc = list(set(dataset_part))
    # f = open("kitti_mixdataset.txt", "a")
    # for txt in dataset_15pc:
    #     f.write(txt+"\n")
    #     imgname = txt.strip('.txt')+'.jpg'
    #     if not os.path.exists(imgname):
    #         imgname = imgname.strip('.jpg')+'.png'
    #     if not os.path.exists(imgname):
    #         continue
    #     move(imgname,r'D:\Kios\Dataset\Vehicles\Train\kitti/'+os.path.basename(imgname))
    #     copy2(txt,r'D:\Kios\Dataset\Vehicles\Train\kitti/'+os.path.basename(txt))
    #     # move(imgname,r'D:\Kios\Dataset\Vehicles\Validation\Custom_mix/'+os.path.basename(imgname))
    #     # copy2(txt,r'D:\Kios\Dataset\Vehicles\Validation\Custom_mix/'+os.path.basename(txt))
    # f.close()
    # print("lol")
    # os.chdir("res_valid/")
    # f = open("motors.txt", "a")
    # for motor in motorbikes:
    #     f.write(motor+"\n")
    # f.close()
    # f = open("busses.txt", "a")
    # for bus in busses:
    #     f.write(bus+"\n")
    # f.close()
    # f = open("trucks.txt",'a')
    # for truck in trucks:
    #     f.write(truck + "\n")
    # f.close()
    # f = open("bus_truck.txt","a")
    # for bt in bus_truck:
    #     f.write(bt + "\n")
    # f.close()
    # f = open("bus_motor.txt", "a")
    # for bm in bus_motorbike:
    #     f.write(bm + "\n")
    # f.close()
    # f = open("truck_motor.txt", "a")
    # for tm in truck_motorbike:
    #     f.write(tm + "\n")
    # f.close()
    # f = open("all3.txt", "a")
    # for veh in all:
    #     f.write(veh + "\n")
    # f.close()

    print(f"=== Cars: {ccars}===\n")
    print(f"=== Motorbikes: {cmotorbikes}===\n")
    print(f"=== Busses: {cbus}===\n")
    print(f"=== Trucks: {ctruck}===\n")
    print(f"=== Total Images: {img_num}===\n")
    # f.close()


def classes_statistics():
    txt = './train.txt'
    ccars, cbus, ctruck, cmotorbikes = [], [], [], []

    with open(txt, 'r+') as fl:
        imgfiles = fl.readlines()
        for img in imgfiles:
            frame = cv2.imread(img.strip('\n'))
            imH, imW = frame.shape[:2]
            filename_base = img.split('.')[-2] + '.txt'
            if not os.path.isfile(filename_base): continue
            with open(filename_base, 'r+') as f:
                lines = f.readlines()
                f.seek(0)
                for line in lines:
                    c, x, y, w, h = line.split()
                    print(x, y, w, h, (float(w) * imW) * (float(h) * imH), imH, imW)
                    if line[0] == '1':
                        cbus.append(float(w) * float(h))
                    elif line[0] == '2':
                        ctruck.append(float(w) * float(h))
                    elif line[0] == '3':
                        cmotorbikes.append(float(w) * float(h))
                    elif line[0] == '0':
                        ccars.append(float(w) * float(h))

    cars = np.array(ccars)
    motors = np.array(cmotorbikes)
    buss = np.array(cbus)
    truckss = np.array(ctruck)
    output = f"{txt} - Full sized imgs\n"
    output = output + f"Cars: \n  avg: {cars.mean():.5f}\n  min: {cars.min():.5f}\n  max: {cars.max():.5f}\n"
    output = output + f"Motors: \n  avg: {motors.mean():.5f}\n  min: {motors.min():.5f}\n  max: {motors.max():.5f}\n"
    output = output + f"Bus: \n  avg: {buss.mean():.5f}\n  min: {buss.min():.5f}\n  max: {buss.max():.5f}\n"
    output = output + f"Trucks: \n  avg: {truckss.mean():.5f}\n  min: {truckss.min():.5f}\n  max: {truckss.max():.5f}\n\n"

    f = open("results_sizesavg.txt", "a")
    print(output)
    f.write(output)
    f.close


def create_test_train_txt():
    # Current directory
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # Percentage of images to be used for the test set
    percentage_test = 100

    # Create and/or truncate train.txt and test.txt
    file_train = open('train.txt', 'w')
    file_test = open('test.txt', 'w')

    # Populate train.txt and test.txt
    counter = 1
    index_test = round(100 / percentage_test)
    for path, subdirs, files in os.walk(current_dir):
        for filename in files:
            infilename = os.path.join(path, filename)
            if not os.path.isfile(infilename): continue
            if infilename.endswith('.jpg') or infilename.endswith('.JPG') or infilename.endswith(
                    '.png') or infilename.endswith('.PNG'):  # check if ifle is txt format
                if counter == index_test:
                    counter = 1
                    file_test.write(infilename + "\n")
                else:
                    file_train.write(infilename + "\n")
                    counter = counter + 1
                index_test = round(100 / percentage_test)


def check_labeled():
    # folder = r"D:\Kios\Dataset\Vehicles\Validation\kitti/"
    folder = r"D:\Kios\Dataset\Vehicles\Train/"
    for path, subdirs, files in os.walk(folder):
        if path.find("Augmented") > 0:
            continue
        print(path, subdirs)

        # for filename in files:
        #     infilename = os.path.join(path,filename)
        #     if not os.path.isfile(infilename): continue
        #     if infilename.endswith('.png') and not os.path.exists(infilename.strip(".png")+'.txt'):
        #         os.remove(infilename)


# check_labeled()
# change_lines()
# create_test_train_txt()
# check_balance('val.txt')
# classes_statistics()
