# %% [markdown]
# Training YOLOv8 and RF-DETR on the Tirme Waste Dataset

This notebook sets up training for:
1. **YOLOv8** with the RAdam optimizer and early stopping.
2. **RF-DETR** with the RAdam optimizer (fixed epochs for now).

Dataset structure:
```
Datasets/tirme_segmentation.yolo.v2/
  train/images
  train/labels
  valid/images
  valid/labels
```
One class: `waste`.

You can run each cell sequentially in a Jupyter environment.

# %% [markdown]
## 1. Install dependencies

```bash
!pip install ultralytics torch_optimizer detectron2 detrex pyyaml
```  

# %% [markdown]
## 2. Prepare the YOLO data config

Create a small YAML file pointing to your train/val folders and class name.

# %%
import yaml

data_cfg = {
    'train': 'Datasets/tirme_segmentation.yolo.v2/train/images',
    'val':   'Datasets/tirme_segmentation.yolo.v2/valid/images',
    'nc':    1,
    'names': ['waste']
}
with open('tirme_dataset.yaml', 'w') as f:
    yaml.dump(data_cfg, f)
print("Saved YOLO data config to 'tirme_dataset.yaml'")

# %% [markdown]
## 3. Train YOLOv8 with RAdam & Early Stopping

We use Ultralytics YOLOv8 Python API, swapping in RAdam and stopping if no val/mAP50 improvement for 10 epochs.

# %%
from ultralytics import YOLO
import torch_optimizer as optim

# Load a pretrained YOLOv8s model
model = YOLO('yolov8s.pt')

# Override default optimizer to RAdam
optimizer_cfg = {
    'optimizer': optim.RAdam,
    'lr0': 1e-3
}

# Start training with early stopping (patience=10)
metrics = model.train(
    data='tirme_dataset.yaml',
    epochs=100,
    batch=16,
    imgsz=640,
    **optimizer_cfg,
    patience=10,           # stop after 10 epochs with no mAP50 improvement
    save=True              # save best weights
)

print("YOLOv8 training completed.")

# %% [markdown]
## 4. Prepare RF-DETR with Detectron2 + Detrex

We'll use Detrex's RF-DETR config under the hood, and Detectron2's trainer.

**Note:** Early stopping for RF-DETR isn't built-in—in this initial setup we train for a fixed number of epochs (e.g. 50). You can add a similar hook later.

# %%
import os
import torch_optimizer as optim
from detectron2.config import get_cfg
from detrex.config import get_rfdetr_cfg
from detrex.engine import DefaultTrainer

# Base DETR config
cfg = get_cfg()
cfg.merge_from_file(get_rfdetr_cfg('rf_detr_R_50_DC5_1x'))

# Point to our dataset (you'll need to register these in Detectron2 beforehand)
cfg.DATASETS.TRAIN = ('tirme_train',)
cfg.DATASETS.TEST = ('tirme_valid',)

# DataLoader and solver settings
cfg.DATALOADER.NUM_WORKERS = 4
cfg.SOLVER.IMS_PER_BATCH = 8
cfg.SOLVER.BASE_LR = 1e-4
cfg.SOLVER.MAX_EPOCH = 50       # fixed number of epochs
tf

# Override the optimizer builder to use RAdam with weight decay
from detectron2.solver import build_optimizer as d2_build_opt
import detectron2.solver

def build_radam_optimizer(cfg, model):
    params = [p for p in model.parameters() if p.requires_grad]
    return optim.RAdam(params, lr=cfg.SOLVER.BASE_LR, weight_decay=cfg.SOLVER.WEIGHT_DECAY)

detectron2.solver.build_optimizer = build_radam_optimizer

# Create output directory
os.makedirs(cfg.OUTPUT_DIR, exist_ok=True)

# %% [markdown]
## 5. Train RF-DETR

# %%
trainer = DefaultTrainer(cfg)
trainer.resume_or_load(resume=False)
trainer.train()
print("RF-DETR training completed.")
