import os

Pack_DIR = '/app/deepKnowledgeSrc/MetaData/'
DATASET_DIR=os.path.join(Pack_DIR, 'dataset')
Experiment_DIR=os.path.join(Pack_DIR, 'experiments')
DATA_DIR=os.path.join(Pack_DIR, 'data')
OOD_yaml = os.path.join(DATASET_DIR, "coco/coco.yaml")
ID_yaml= os.path.join(DATASET_DIR, "pd_dataset/pd_data.yaml")
val_path = os.path.join(DATASET_DIR, "pd_dataset/train/images")
test_path= os.path.join(DATASET_DIR, "pd_dataset/test/images")
OOD_data_path = os.path.join(DATASET_DIR, "coco/images/val2017")
trained_weights = os.path.join(Pack_DIR, "Networks/KIOS/best.pt")
dataset_format="yolo"
Batch_size=5000
sample_size = 1
