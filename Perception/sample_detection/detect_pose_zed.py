import argparse
import cv2
import numpy as np
import pyzed.sl as sl
from ultralytics import YOLO
from sklearn.decomposition import PCA

# class names from your dataset YAML
CLASS_NAMES = ["regolith", "rock", "unexpected object"]

def init_zed(resolution=sl.RESOLUTION.HD720, fps=30, depth_mode=sl.DEPTH_MODE.PERFORMANCE):
    """Initialize ZED camera"""
    init_params = sl.InitParameters()
    init_params.camera_resolution = resolution
    init_params.camera_fps = fps
    init_params.depth_mode = depth_mode
    init_params.coordinate_units = sl.UNIT.METER

    zed = sl.Camera()
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("ZED failed to open")
        exit(1)

    runtime_params = sl.RuntimeParameters()
    return zed, runtime_params


def get_3d_position(point_cloud, x1, y1, x2, y2):
    """Get 3D centroid from ZED point cloud inside the bounding box"""
    roi = []
    step_x = max(1, (x2 - x1) // 10)  # sample to reduce points
    step_y = max(1, (y2 - y1) // 10)

    for v in range(y1, y2, step_y):
        for u in range(x1, x2, step_x):
            err, point = point_cloud.get_value(u, v)
            if err == sl.ERROR_CODE.SUCCESS:
                x, y, z = point[:3]
                # filter out NaN, inf, and unreasonable depth values
                if (
                    not np.isnan([x, y, z]).any()
                    and not np.isinf([x, y, z]).any()
                    and -10 < x < 10 and -10 < y < 10 and 0 < z < 20
                ):
                    roi.append([x, y, z])

    if len(roi) < 3:  # need at least 3 points for PCA
        return None, None

    roi = np.array(roi)
    centroid = np.mean(roi, axis=0)

    # Orientation via PCA
    try:
        pca = PCA(n_components=3)
        pca.fit(roi)
        main_axis = pca.components_[0]  # main orientation axis
    except Exception as e:
        print(f"[WARN] PCA failed: {e}")
        main_axis = None

    return centroid, main_axis



def main(args):
    # Load YOLO model
    model = YOLO(args.model)

    # Init ZED
    zed, runtime_params = init_zed()

    image_zed = sl.Mat()
    point_cloud = sl.Mat()

    while True:
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            # Get left image
            zed.retrieve_image(image_zed, sl.VIEW.LEFT)
            frame = image_zed.get_data()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)  # Convert 4-channel → 3-channel

            # Run YOLO detection
            results = model(frame, verbose=False)[0]

            # Get point cloud
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ)

            for box in results.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = CLASS_NAMES[cls_id]   # <-- use your YAML-defined class names

                centroid, orientation = get_3d_position(point_cloud, x1, y1, x2, y2)

                if centroid is not None:
                    # Draw 2D box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    text = f"{label} {conf:.2f} Pos:({centroid[0]:.2f},{centroid[1]:.2f},{centroid[2]:.2f})m"
                    cv2.putText(frame, text, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

                    print(f"Detected {label} @ {centroid} m")
                    if orientation is not None:
                        print(f"  Orientation axis: {orientation}")


            # Show image
            cv2.imshow("YOLO + ZED Pose Estimation", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    zed.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=str, required=True,
                        help="Path to YOLOv8 model (e.g., yolov8n.pt or custom.pt)")
    args = parser.parse_args()
    main(args)
