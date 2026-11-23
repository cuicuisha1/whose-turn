"""Local diagnostic view, always showing whether the source is simulated."""
import numpy as np


def render(frame, observation, max_depth=6.0):
    import cv2
    color = frame.color.copy()
    depth_m = frame.depth.astype(np.float32) * frame.depth_scale
    depth = cv2.applyColorMap(np.uint8(np.clip(depth_m / max_depth, 0, 1) * 255), cv2.COLORMAP_TURBO)
    depth[frame.depth == 0] = 0
    for person in observation["people"]:
        x1, y1, x2, y2 = map(int, person["bbox_xyxy"])
        selected = observation["target"] and observation["target"]["track_id"] == person["track_id"]
        ink = (70, 220, 110) if selected else (255, 180, 80)
        distance = person["distance_m"]
        label = f"ID {person['track_id']} | " + (f"{distance:.2f} m" if distance is not None else "depth unknown")
        cv2.rectangle(color, (x1, y1), (x2, y2), ink, 2)
        cv2.putText(color, label, (x1, max(18, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, .5, ink, 1)
        xa, ya, xb, yb = person["depth"]["roi"]
        cv2.rectangle(depth, (xa, ya), (xb, yb), ink, 2)
    if observation["target"]:
        target = observation["target"]
        point = (int(target["center_x"] * color.shape[1]), int(target["center_y"] * color.shape[0]))
        cv2.drawMarker(color, point, (50, 240, 100), cv2.MARKER_CROSS, 20, 2)
    panels = np.hstack([color, depth])
    header = np.full((82, panels.shape[1], 3), 18, dtype=np.uint8)
    source = "SIMULATION / SYNTHETIC FIXTURES" if observation["simulated"] else "REALSENSE / LIVE"
    cv2.putText(header, source + " | " + observation["status"], (12, 24),
                cv2.FONT_HERSHEY_SIMPLEX, .6, (110, 220, 255), 1)
    age = observation["frame_age_ms"]
    text = f"{observation['detector']} | capture {observation['capture_fps']:.1f} FPS | age {age} ms | people {observation['person_count']}"
    cv2.putText(header, text, (12, 48), cv2.FONT_HERSHEY_SIMPLEX, .5, (220, 220, 220), 1)
    cv2.putText(header, "BGR + target (left) / aligned depth + torso ROI (right) | q: quit | ACTUATION OFF",
                (12, 70), cv2.FONT_HERSHEY_SIMPLEX, .45, (160, 180, 180), 1)
    if not observation["valid"]:
        cv2.putText(panels, "INVALID / NO CURRENT PERCEPTION", (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, .7, (60, 60, 255), 2)
    return np.vstack([header, panels])
