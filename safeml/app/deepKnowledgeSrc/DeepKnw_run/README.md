# Instructions how to use DeepKnowledge
## Install Required Packages
We recommend starting by creating a virtual environment and then installing the required packages.

#### Virtual Environement

```
python3 -m pip install --user virtualenv

python3 -m venv path/to/the/virtual/environment
```
###### Activate virtual environment

```
source path/to/the/virtual/environment/bin/activate
```



#### Linux
Install the required packages
```
pip install tensorflow==2.12.0
pip install numpy
pip install Pillow
pip install opencv-python
pip install pandas
pip install scikit-learn
pip install tqdm
pip install matplotlib
```

Run DeepKnowledge
```
knw=DeepKnw_run(Your_model,Your_dataset)
coverage=knw.estimate_coverage(Your_testset)
```
