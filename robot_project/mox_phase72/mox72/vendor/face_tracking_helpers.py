"""复用现有人脸算法的纯视觉辅助函数；不打开相机或串口。"""
import math
import cv2
import numpy as np

def detect_faces(detector, bgr):
    """Letterbox 424x240 -> YuNet 320x320; return box and nose in original coords."""
    h, w = bgr.shape[:2]
    sz = 320
    scale = min(sz/w, sz/h)
    sw, sh = max(1, round(w*scale)), max(1, round(h*scale))
    sx, sy = sw / w, sh / h
    ox, oy = (sz-sw)//2, (sz-sh)//2
    image = np.full((sz, sz, 3), 114, np.uint8)
    image[oy:oy+sh, ox:ox+sw] = cv2.resize(bgr, (sw, sh))
    _, rows = detector.detect(image)
    found = []
    if rows is None:
        return found
    for row in rows:
        if len(row) < 15 or not np.isfinite(row[:15]).all():
            continue
        x, y, bw, bh = [float(v) for v in row[:4]]
        x1 = float(np.clip((x-ox)/sx, 0, w-1))
        y1 = float(np.clip((y-oy)/sy, 0, h-1))
        x2 = float(np.clip((x+bw-ox)/sx, 0, w-1))
        y2 = float(np.clip((y+bh-oy)/sy, 0, h-1))
        if x2-x1 < 10 or y2-y1 < 10:
            continue
        nx = (float(row[8]) - ox) / sx
        ny = (float(row[9]) - oy) / sy
        if not (0 <= nx < w and 0 <= ny < h):
            continue  # 鼻尖出画时停止以它作为控制目标，不把它伪造在图像边缘。
        found.append({"box": (x1, y1, x2, y2), "aim": (nx, ny), "score": float(row[14])})
    return found


def select_face(faces, previous, width):
    if not faces:
        return None
    if previous is not None:
        near = min(faces, key=lambda f: math.dist(f["aim"], previous))
        if math.dist(near["aim"], previous) <= width*.30:
            return near
        # No silent switching to another face when tracking one already.
        return None
    return max(faces, key=lambda f: f["score"] * math.sqrt(max(1, (f["box"][2]-f["box"][0])*(f["box"][3]-f["box"][1]))))


def depth_at_face(depth, face, scale):
    if depth.ndim != 2 or not math.isfinite(scale) or scale <= 0:
        return None
    h, w = depth.shape
    x1,y1,x2,y2 = face["box"]
    nx,ny = face["aim"]
    r = max(3, int(min(x2-x1,y2-y1)*.12))
    xa = max(0,int(nx-r),int(x1)); xb = min(w,int(nx+r+1),int(x2)+1)
    ya = max(0,int(ny-r),int(y1)); yb = min(h,int(ny+r+1),int(y2)+1)
    if xa>=xb or ya>=yb:
        return None
    patch = depth[ya:yb,xa:xb].astype(np.float32)*scale
    v = patch[np.isfinite(patch) & (patch>=.35) & (patch<=3.5)]
    if len(v) < max(4, math.ceil(patch.size*.3)):
        return None
    return float(np.median(v))


def corrected_step(normalized_error, focal, half_size, deadband, gain, max_step):
    """Angle delta is proportional to excess image error beyond a dead zone."""
    if abs(normalized_error) <= deadband:
        return 0.0
    excess = math.copysign(abs(normalized_error)-deadband, normalized_error)
    angle = math.degrees(math.atan2(excess*half_size, focal))
    return float(np.clip(gain*angle, -max_step, max_step))


def draw(bgr, faces, face, smoothed, pan, tilt, distance, mode, status):
    im = bgr.copy()
    h,w=im.shape[:2]
    cv2.drawMarker(im,(w//2,h//2),(255,255,255),cv2.MARKER_CROSS,12,1)
    for f in faces:
        x1,y1,x2,y2=map(int,f["box"])
        cv2.rectangle(im,(x1,y1),(x2,y2),(0,255,0) if f is face else (0,175,255),2)
    if face is not None:
        xx,yy=map(int,face["aim"])
        cv2.circle(im,(xx,yy),3,(255,0,255),-1)
    if smoothed is not None:
        xx,yy=map(int,smoothed)
        cv2.circle(im,(xx,yy),4,(0,255,255),-1)
        cv2.line(im,(w//2,h//2),(xx,yy),(255,255,0),1)
    z = f"{distance:.2f}m" if distance is not None else "N/A"
    cv2.putText(im,f"{mode} P:{pan:+.1f} T:{tilt:+.1f} Z:{z}",(4,15),cv2.FONT_HERSHEY_SIMPLEX,.35,(255,255,255),1)
    cv2.putText(im,status,(4,h-6),cv2.FONT_HERSHEY_SIMPLEX,.36,(255,255,255),1)
    return im

