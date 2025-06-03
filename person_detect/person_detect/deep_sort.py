import os
import cv2
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort

import time

# === Configuration ===
sequence_name = "MOT20-01"  # Change to your sequence
dataset_path = "/home/icc-nano/energy_ws/src/"
img_folder = os.path.join(dataset_path, sequence_name, "img1")
output_txt = f"yolov8n_deepsort_{sequence_name}.txt"
gt_file = os.path.join(dataset_path, sequence_name, "gt", "gt.txt")
target_gt_id = 3  # ID from ground truth you want to visualize
iou_log_file = f"iou_log_{sequence_name}.txt"

# === Load Ground Truth ===
gt_dict = {}  # Format: {frame_id: [(id, x, y, w, h), ...]}

with open(gt_file, "r") as f:
    for line in f:
        parts = line.strip().split(",")
        frame_id = int(parts[0])
        track_id = int(parts[1])
        x, y, w, h = map(float, parts[2:6])
        mark = int(parts[6])

        if mark == 1:
            if frame_id not in gt_dict:
                gt_dict[frame_id] = []
            gt_dict[frame_id].append((track_id, x, y, w, h))

# === IoU Function ===
def compute_iou(boxA, boxB):
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[0] + boxA[2], boxB[0] + boxB[2])
    yB = min(boxA[1] + boxA[3], boxB[1] + boxB[3])
    interArea = max(0, xB - xA) * max(0, yB - yA)
    boxAArea = boxA[2] * boxA[3]
    boxBArea = boxB[2] * boxB[3]
    return interArea / float(boxAArea + boxBArea - interArea)

# === Initialize model and tracker ===
model = YOLO("yolov8n.pt")
tracker = DeepSort(max_age=30)

frame_id = 1
all_results = []
iou_logs = []

while True:
    
    # At top of loop:
    start_time = time.time()
    
    img_path = os.path.join(img_folder, f"{frame_id:06d}.jpg")
    if not os.path.exists(img_path):
        break

    frame = cv2.imread(img_path)
    detections = []

    # === Run YOLOv8n on the frame ===
    preds = model(frame, verbose=False)[0]
    for box in preds.boxes:
        cls_id = int(box.cls[0])
        if cls_id != 0:
            continue  # Only track 'person' class

        x1, y1, x2, y2 = map(int, box.xyxy[0])
        conf = float(box.conf[0])
        bbox_w, bbox_h = x2 - x1, y2 - y1
        detections.append(([x1, y1, bbox_w, bbox_h], conf, None))

    # === Update tracker ===
    tracks = tracker.update_tracks(detections, frame=frame)

    tracked_box = None

    for track in tracks:
        if not track.is_confirmed():
            continue

        track_id = track.track_id

        if int(track_id) != 4:
            continue  # Only follow person with ID 1

        l, t, r, b = track.to_ltrb()
        w, h = r - l, b - t
        tracked_box = [l, t, w, h]
        conf = 1.0

        # Format: frame, track_id, x, y, w, h, conf, -1, -1, -1
        all_results.append(f"{frame_id},{track_id},{int(l)},{int(t)},{int(w)},{int(h)},{conf:.2f},-1,-1,-1\n")

        # === Visualize ground truth for the target ID ===
        if frame_id in gt_dict:
            for gt_id, x, y, w, h in gt_dict[frame_id]:
                if gt_id == target_gt_id:
                    x1, y1, x2, y2 = int(x), int(y), int(x + w), int(y + h)
                    # cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    # cv2.putText(frame, f"GT ID {gt_id}", (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

                    if tracked_box:
                        iou = compute_iou(tracked_box, [x, y, w, h])
                        iou_logs.append(f"{frame_id},{iou:.4f}\n")
                        # cv2.putText(frame, f"IoU: {iou:.2f}", (x1, y2 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        # Visualize
        # cv2.rectangle(frame, (int(l), int(t)), (int(r), int(b)), (0, 255, 0), 2)
        # cv2.putText(frame, f"ID {track_id}", (int(l), int(t) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # Show frame
    # cv2.imshow("YOLOv8n + DeepSORT", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

    frame_id += 1

    # At bottom of loop:
    elapsed = time.time() - start_time

    fps = 1 / elapsed if elapsed > 0 else 0
    print(f"Frame {frame_id}: {fps:.2f} FPS")

cv2.destroyAllWindows()

# === Save results in MOT format ===
with open(output_txt, "w") as f:
    f.writelines(all_results)

with open(iou_log_file, "w") as f:
    f.writelines(iou_logs)

print(f"[✓] Tracking done. Results saved in {output_txt}")
