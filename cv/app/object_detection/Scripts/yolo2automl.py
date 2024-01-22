import os
from PIL import Image
from pathlib import Path


# directory
# train_dir = os.path.dirname('/home/rmakri01/datasets/KIOS-VEH/608/trained/')
# valid_dir = os.path.dirname('/home/rmakri01/datasets/KIOS-VEH/608/val/')
# test_dir = os.path.dirname('/home/rmakri01/datasets/KIOS-VEH/608/test/')
def get_xyx2y2(class_id, xcnt, ycnt, w, h):
    class_id, xcnt, ycnt, w, h = int(class_id), float(xcnt), float(ycnt), float(w), float(h)
    # calculate values
    xmin, ymin, xmax, ymax = xcnt - (w / 2), ycnt - (h / 2), xcnt + (w / 2), ycnt + (h / 2)
    # check if below zero
    xmin, ymin, xmax, ymax = max(xmin,0), max(ymin,0), max(xmax,0), max(ymax,0)
    # check if above 1
    xmin, ymin, xmax, ymax = min(xmin,1), min(ymin,1), min(xmax,1), min(ymax,1)
    return class_id, xmin, ymin, xmax, ymax

file_train = open('train.csv', 'w', encoding='utf-8')
# png_folder_train = '/home/rmakri01/datasets/KIOS-VEH/608/trained/png'
# png_folder_valid = '/home/rmakri01/datasets/KIOS-VEH/608/val/png'
# png_folder_test = '/home/rmakri01/datasets/KIOS-VEH/608/test/png'
#
# Path(png_folder_train).mkdir(parents=True, exist_ok=True)
# Path(png_folder_valid).mkdir(parents=True, exist_ok=True)
# Path(png_folder_test).mkdir(parents=True, exist_ok=True)

_label_map = (
    'car',
    'bus',
    'truck',
    'motorbike',
)
folder_dir = '/home/rmakri01/datasets/KIOS-VEH/608/'
folders = {'TRAIN':'trained', 'VAL':'val', 'TEST':'test'}

for operation in folders:
    fold_path = folder_dir + folders[operation]
    png_folder = fold_path + '/png'
    Path(png_folder).mkdir(parents=True, exist_ok=True)
    print(f"Execution:{operation}\nFolder:{fold_path}\nPNG Folder:{png_folder}")
    for path, subdirs, files in os.walk(fold_path):
        for filename in files:
            infilename = os.path.join(path, filename)
            if not os.path.isfile(infilename): continue
            if infilename.endswith('.jpg') or infilename.endswith('.JPG') or infilename.endswith(
                    '.JPEG') or infilename.endswith('.jpeg'):  # check if file is an image
                annotation = infilename.split('.')[0] + '.txt'
                if not os.path.exists(annotation): continue
                # read each bounding box
                with open(annotation, 'r+') as f:
                    lines = f.readlines()
                    f.seek(0)
                    for line in lines:
                        # 'set,path,label,x_min,y_min,,,x_max,y_max,,'
                        class_id, xcnt, ycnt, w, h = line.split()
                        class_id, xmin, ymin, xmax, ymax = get_xyx2y2(class_id, xcnt, ycnt, w, h)
                        file_train.write(
                            f'{operation},{infilename},{_label_map[class_id]},{xmin},{ymin},,,{xmax},{ymax},,' + "\n")
            elif infilename.endswith('.png') or infilename.endswith('.PNG'):
                annotation = infilename.split('.')[0] + '.txt'
                if not os.path.exists(annotation): continue
                im = Image.open(infilename)
                rgb_im = im.convert('RGB')
                new_name = png_folder + '/' + filename.split('.')[0] + '.jpg'
                rgb_im.save(new_name)
                # read each bounding box
                with open(annotation, 'r+') as f:
                    lines = f.readlines()
                    f.seek(0)
                    for line in lines:
                        # 'set,path,label,x_min,y_min,,,x_max,y_max,,'
                        class_id, xcnt, ycnt, w, h = line.split()
                        class_id, xmin, ymin, xmax, ymax = get_xyx2y2(class_id, xcnt, ycnt, w, h)
                        file_train.write(
                            f'{operation},{new_name},{_label_map[class_id]},{xmin},{ymin},,,{xmax},{ymax},,' + "\n")
#
# for path, subdirs, files in os.walk(train_dir):
#     for filename in files:
#         infilename = os.path.join(path, filename)
#         if not os.path.isfile(infilename): continue
#         if infilename.endswith('.jpg') or infilename.endswith('.JPG') or infilename.endswith(
#                 '.JPEG') or infilename.endswith('.jpeg'):  # check if file is an image
#             annotation = infilename.split('.')[0] + '.txt'
#             if not os.path.exists(annotation): continue
#             # read each bounding box
#             with open(annotation, 'r+') as f:
#                 lines = f.readlines()
#                 f.seek(0)
#                 for line in lines:
#                     # 'set,path,label,x_min,y_min,,,x_max,y_max,,'
#                     class_id, xcnt, ycnt, w, h = line.split()
#                     class_id, xcnt, ycnt, w, h = int(class_id), float(xcnt), float(ycnt), float(w), float(h)
#                     xmin, ymin, xmax, ymax = xcnt-w/2, ycnt-h/2, xcnt + w/2, ycnt + h/2
#                     file_train.write(
#                         f'TRAIN,{infilename},{_label_map[class_id]},{xmin},{ymin},,,{xmax},{ymax},,' + "\n")
#         elif infilename.endswith('.png') or infilename.endswith('.PNG'):
#             annotation = infilename.split('.')[0] + '.txt'
#             if not os.path.exists(annotation): continue
#             im = Image.open(infilename)
#             rgb_im = im.convert('RGB')
#             new_name = png_folder_train+'/'+filename.split('.')[0]+'.jpg'
#             rgb_im.save(new_name)
#             # read each bounding box
#             with open(annotation, 'r+') as f:
#                 lines = f.readlines()
#                 f.seek(0)
#                 for line in lines:
#                     # 'set,path,label,x_min,y_min,,,x_max,y_max,,'
#                     class_id, xcnt, ycnt, w, h = line.split()
#                     class_id, xcnt, ycnt, w, h = int(class_id), float(xcnt), float(ycnt), float(w), float(h)
#                     xmin, ymin, xmax, ymax = xcnt-w/2, ycnt-h/2, xcnt + w/2, ycnt + h/2
#                     file_train.write(
#                         f'TRAIN,{new_name},{_label_map[class_id]},{xmin},{ymin},,,{xmax},{ymax},,' + "\n")
#
#
# for path, subdirs, files in os.walk(valid_dir):
#     for filename in files:
#         infilename = os.path.join(path, filename)
#         if not os.path.isfile(infilename): continue
#         if infilename.endswith('.jpg') or infilename.endswith('.JPG') or infilename.endswith(
#                 '.JPEG') or infilename.endswith('.jpeg'):  # check if file is an image
#             annotation = infilename.split('.')[0] + '.txt'
#             if not os.path.isfile(annotation): continue
#             # read each bounding box
#             with open(annotation, 'r+') as f:
#                 lines = f.readlines()
#                 f.seek(0)
#                 for line in lines:
#                     # 'set,path,label,x_min,y_min,,,x_max,y_max,,'
#                     class_id, xcnt, ycnt, w, h = line.split()
#                     class_id, xcnt, ycnt, w, h = int(class_id), float(xcnt), float(ycnt), float(w), float(h)
#                     xmin, ymin, xmax, ymax = xcnt-w/2, ycnt-h/2, xcnt + w/2, ycnt + h/2
#                     file_train.write(f'VAL,{infilename},{_label_map[class_id]},{xmin},{ymin},,,{xmax},{ymax},,' + "\n")
#         elif infilename.endswith('.png') or infilename.endswith('.PNG'):
#             annotation = infilename.split('.')[0] + '.txt'
#             if not os.path.exists(annotation): continue
#             im = Image.open(infilename)
#             rgb_im = im.convert('RGB')
#             new_name = png_folder_valid+'/'+filename.split('.')[0]+'.jpg'
#             rgb_im.save(new_name)
#             # read each bounding box
#             with open(annotation, 'r+') as f:
#                 lines = f.readlines()
#                 f.seek(0)
#                 for line in lines:
#                     # 'set,path,label,x_min,y_min,,,x_max,y_max,,'
#                     class_id, xcnt, ycnt, w, h = line.split()
#                     class_id, xcnt, ycnt, w, h = int(class_id), float(xcnt), float(ycnt), float(w), float(h)
#                     xmin, ymin, xmax, ymax = xcnt-w/2, ycnt-h/2, xcnt + w/2, ycnt + h/2
#                     file_train.write(
#                         f'VAL,{new_name},{_label_map[class_id]},{xmin},{ymin},,,{xmax},{ymax},,' + "\n")
#
# for path, subdirs, files in os.walk(test_dir):
#     for filename in files:
#         infilename = os.path.join(path, filename)
#         if not os.path.isfile(infilename): continue
#         if infilename.endswith('.jpg') or infilename.endswith('.JPG') or infilename.endswith(
#                 '.JPEG') or infilename.endswith('.jpeg'):  # check if file is an image
#             annotation = infilename.split('.')[0] + '.txt'
#             if not os.path.isfile(annotation): continue
#             # read each bounding box
#             with open(annotation, 'r+') as f:
#                 lines = f.readlines()
#                 f.seek(0)
#                 for line in lines:
#                     # 'set,path,label,x_min,y_min,,,x_max,y_max,,'
#                     class_id, xcnt, ycnt, w, h = line.split()
#                     class_id, xcnt, ycnt, w, h = int(class_id), float(xcnt), float(ycnt), float(w), float(h)
#                     xmin, ymin, xmax, ymax = xcnt-w/2, ycnt-h/2, xcnt + w/2, ycnt + h/2
#                     file_train.write(f'TEST,{infilename},{_label_map[class_id]},{xmin},{ymin},,,{xmax},{ymax},,' + "\n")
#         elif infilename.endswith('.png') or infilename.endswith('.PNG'):
#             annotation = infilename.split('.')[0] + '.txt'
#             if not os.path.exists(annotation): continue
#             im = Image.open(infilename)
#             rgb_im = im.convert('RGB')
#             new_name = png_folder_test+'/'+filename.split('.')[0]+'.jpg'
#             rgb_im.save(new_name)
#             # read each bounding box
#             with open(annotation, 'r+') as f:
#                 lines = f.readlines()
#                 f.seek(0)
#                 for line in lines:
#                     # 'set,path,label,x_min,y_min,,,x_max,y_max,,'
#                     class_id, xcnt, ycnt, w, h = line.split()
#                     class_id, xcnt, ycnt, w, h = int(class_id), float(xcnt), float(ycnt), float(w), float(h)
#                     xmin, ymin, xmax, ymax = xcnt-w/2, ycnt-h/2, xcnt + w/2, ycnt + h/2
#                     file_train.write(
#                         f'TEST,{new_name},{_label_map[class_id]},{xmin},{ymin},,,{xmax},{ymax},,' + "\n")
file_train.close()