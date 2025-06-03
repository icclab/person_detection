import numpy as np
from collections import defaultdict

def iou(boxA, boxB):
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[0] + boxA[2], boxB[0] + boxB[2])
    yB = min(boxA[1] + boxA[3], boxB[1] + boxB[3])
    interArea = max(0, xB - xA) * max(0, yB - yA)
    boxAArea = boxA[2] * boxA[3]
    boxBArea = boxB[2] * boxB[3]
    return interArea / float(boxAArea + boxBArea - interArea)

# Load predictions
pred = defaultdict(list)
with open("yolov8n_deepsort_MOT20-01.txt") as f:
    for line in f:
        fid, tid, x, y, w, h, *_ = map(float, line.strip().split(','))
        if int(tid) == 4:
            pred[int(fid)].append((x, y, w, h))

# Load GT
gt = defaultdict(list)
with open("gt.txt") as f:
    for line in f:
        fid, gid, x, y, w, h, *_ = map(float, line.strip().split(','))
        gt[int(fid)].append((int(gid), (x, y, w, h)))

# Count matches
match_counts = defaultdict(int)

for fid in pred:
    if fid not in gt:
        continue
    pred_box = pred[fid][0]
    best_iou = 0
    best_gid = -1
    for gid, gt_box in gt[fid]:
        score = iou(pred_box, gt_box)
        if score > best_iou:
            best_iou = score
            best_gid = gid
    if best_iou > 0.5:
        match_counts[best_gid] += 1

# Print most matched GT ID
matched_gt_id = max(match_counts, key=match_counts.get)
print(f"Your track_id 4 most closely matches ground-truth ID: {matched_gt_id}")
