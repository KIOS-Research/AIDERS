import os

Pack_DIR = '/home/sondess/sm2672/DeepKnowledge_KIOS/MetaData/'
DATASET_DIR=os.path.join(Pack_DIR, 'dataset')
Experiment_DIR=os.path.join(Pack_DIR, 'experiments')
DATA_DIR=os.path.join(Pack_DIR, 'data')
OOD_yaml = "/home/sondess/sm2672/DeepKnowledge_KIOS/MetaData/dataset/coco/coco.yaml"
ID_yaml="/home/sondess/sm2672/DeepKnowledge_KIOS/MetaData/dataset/pd_dataset/pd_data.yaml"
val_path = "/home/sondess/sm2672/DeepKnowledge_KIOS/MetaData/dataset/pd_dataset/train/images"
test_path= "/home/sondess/sm2672/DeepKnowledge_KIOS/MetaData/dataset/PAL_simulator_data"
OOD_data_path = "/home/sondess/sm2672/DeepKnowledge_KIOS/MetaData/dataset/coco/images/val2017"
trained_weights = "/home/sondess/sm2672/DeepKnowledge_KIOS/MetaData/Networks/KIOS/best.pt"
dataset_format="yolo"
Batch_size=5000
sample_size = 1
