##DeepKnowledge @Designtime  & @Runtime version
##### Author: M.Sondess @ UoY



#### Folder structure
```
parent
├── DeepKnw_pack ← downloads here (1 MB)
└── Metadata
    └── config.py ←  data & model paths here
    └── dataset   ←  download data here
    └── data   ←  Designtime execution (needed for runtime)
    └── experiments   ←  Designtime execution
    └── Networks  
        └── KIOS ←  trained model weights here

```


#### Installation

```
cd DIRECTORY/DeepKnw_pack/

python setup.py install

```
####Example usage for DesignTime Deployment:
```
import DeepKnw_run as knw

COV=knw.DeepKnw(PATH_To_CONFIG_FILE)

test_path=PATH_To_TestData_Images
size=5000 (← EXPL)

test_loader = COV.getTestloader(test_path,size)

RSLT=COV.estimate_coverage(test_loader)


```
####Example usage for RunTime Deployment:

```
import DeepKnw_run as knw

COV=knw.DeepKnw(PATH_To_CONFIG_FILE)

YOLOloader, model_features=COV.DesignDataAnalyzer()

Frames_path=PATH_To_RunTime_Data

Batch=5 (← EXPL)

Frame_Loader = COV.getTestloader(Frames_path,Batch)

Deepknowledge_Uncertainty=COV.Runtime_Estimate(Frame_Loader,YOLOloader)

```
