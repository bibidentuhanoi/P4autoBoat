# Maritime YOLOv26n QAT Kaggle Notebook — Design Spec

## Goal

Single Kaggle notebook that trains YOLOv26n on maritime obstacle detection data (MODD2 + MoDS), then quantizes the trained model via QAT for ESP32-P4 deployment. Output: `yolo26n_224.espdl`.

## Constraints

- Kaggle: 19GB disk, dual T4 (16GB each), 12hr GPU session, internet enabled
- 2 classes: obstacle (0), land (1)
- Train at 640, quantize at 224
- Must reuse existing `pipeline_source/` from `BoumedineBillal/yolo26n_esp` repo

## Cell Layout

```
Cell  0: [markdown] Title + overview
Cell  1: [code]     Kaggle setup (GPU check, pip install, clone repo)
Cell  2: [code]     System imports
Cell  3: [markdown] --- Phase 1: Maritime Data ---
Cell  4: [code]     Download MODD2 + MoDS (per-zip: download→extract→delete)
Cell  5: [code]     Convert to YOLO format (2 classes) + rewrite data.yaml with absolute paths
Cell  6: [markdown] --- Phase 2: Train YOLOv26n ---
Cell  7: [code]     Train yolo26n on maritime (640, 100 epochs, patience=5, device=[0,1], batch=64)
Cell  8: [code]     Disk cleanup (delete Maritime_Datasets/, keep yolo_p4_dataset/ + best.pt)
Cell  9: [markdown] --- Phase 3: QAT Setup ---
Cell 10: [code]     QAT config (IMG_SZ=224, PT_FILE=maritime best.pt, DATA_YAML=maritime)
Cell 11: [code]     Virtual module injection + local module imports
Cell 12: [code]     ESP-PPQ patches
Cell 13: [code]     Export maritime .pt → ONNX (batch=1, EXPORT_DYNAMIC=True)
Cell 14: [code]     Quantizer init + aux branch separation
Cell 15: [markdown] --- Phase 4: Calibration + QAT ---
Cell 16: [code]     Maritime data loaders + calibration pipeline
Cell 17: [code]     Baseline PTQ validation
Cell 18: [code]     QAT training loop (5 epochs)
Cell 19: [code]     Reload best graph
Cell 20: [markdown] --- Phase 5: Export ---
Cell 21: [code]     Graph surgery (prune aux, split concat) + export .espdl
```

## Critical Fixes (from Devil's Advocate review)

### 1. Graph surgery NC — CRITICAL

The COCO notebook hardcodes `elif 80 in dims` for cls tensor detection.
Maritime has NC=2. Must use `model_meta['nc']` instead:

```python
# BEFORE (broken for NC!=80):
elif 80 in dims: cls_var = input_var

# AFTER:
elif model_meta['nc'] in dims: cls_var = input_var
```

### 2. Disk management — CRITICAL

Peak disk during MODD2 download+extract can hit 8GB. Must download→extract→delete PER ZIP:

```python
# Per-zip pattern:
!wget -q -nc <url> -O {path}.zip
!unzip -q -o {path}.zip -d {dest}
!rm -f {path}.zip
```

Also delete `Maritime_Datasets/` immediately after conversion (before training), since `yolo_p4_dataset/` has copies.

### 3. Batch size — HIGH

yolo26n has attention modules (C2PSA) with O(N^2) memory. Default `batch=64` (32/GPU) at imgsz=640.
Comment: "increase to 128 if no OOM."

### 4. Absolute paths in data.yaml — HIGH

After conversion, overwrite data.yaml with absolute paths:

```python
yaml_content = f"""path: {os.path.abspath(OUTPUT_DIR)}
train: images/train
val: images/val
nc: 2
names: ['obstacle', 'land']
"""
```

### 5. PT_FILE must point to trained model — MEDIUM

QAT config must NOT use `yolo26n.pt` (COCO). Must point to maritime-trained output:

```python
PT_FILE = os.path.join(BASE_DIR, "yolo26n_maritime", "train", "weights", "best.pt")
```

### 6. Assert NC=2 before export — MEDIUM

After loading trained model, verify head was properly adjusted:

```python
tmp = ESP_YOLO(QATConfig.PT_FILE)
assert tmp.model.model[-1].nc == 2, f"Expected NC=2, got {tmp.model.model[-1].nc}"
```

## Dependencies (Kaggle install)

```python
!pip install -q --no-deps ultralytics==8.4.7
!pip install -q esp-ppq==1.2.4 onnx==1.17.0 onnxscript ultralytics-thop polars
```

DO NOT install torch/torchvision — Kaggle's CUDA versions must stay.

## Storage Timeline

| Phase | Cumulative Disk | Notes |
|-------|----------------|-------|
| Clone repo + pip install | ~300MB | |
| MODD2 download+extract (zip deleted) | ~4GB | Per-zip cleanup |
| MoDS download+extract | ~5.5GB | kagglehub cache auto-managed |
| Convert to yolo_p4_dataset | ~8GB | Raw + converted coexist briefly |
| Delete Maritime_Datasets/ | ~4GB | Freed ~4GB |
| Training output | ~4.5GB | weights, logs |
| QAT (ONNX, native, espdl) | ~4.7GB | |
| Final | ~4.7GB | Well within 19GB |

## Training Config

```python
model = YOLO("yolo26n.pt")  # COCO pretrained, head auto-adjusts to NC=2
results = model.train(
    data=dataset_yaml,       # maritime data.yaml (absolute paths)
    epochs=100,
    imgsz=640,
    batch=64,                # safe for yolo26n attention on dual T4
    workers=8,
    device=[0, 1],
    project="yolo26n_maritime",
    name="train",
    exist_ok=True,
    patience=5,
    lr0=0.001,
    optimizer='AdamW',
    warmup_epochs=5,
    cos_lr=True,
    verbose=True
)
```

## QAT Config

```python
class QATConfig:
    IMG_SZ = 224
    BATCH_SIZE = 32
    EPOCHS = 5
    DEVICE = "cuda"
    DATA_YAML_FILE = "<absolute path to yolo_p4_dataset/data.yaml>"
    PT_FILE = "yolo26n_maritime/train/weights/best.pt"
    EXPORT_DYNAMIC = True    # batch axis dynamic, spatial static
    EXPORT_OPSET = 13
    TARGET_PLATFORM = get_target_platform("esp32p4", 8)
    # ... rest same as COCO notebook
```

## Output

`output/size_224/yolo26n_224.espdl` — ready for ESP32-P4 deployment with NC=2 (obstacle + land).

## Source Files

- Base: `tools/yolo26n_esp/YOLOv26_QAT_Kaggle.ipynb` (COCO version, proven working)
- Maritime data logic: `tools/yolo26n_esp/ultralytics-yolo11-notebook.ipynb` (cells 3-5)
- Pipeline source: `tools/yolo26n_esp/pipeline_source/` (unchanged)
- Output: `tools/yolo26n_esp/YOLOv26_Maritime_Kaggle.ipynb`
