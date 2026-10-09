"""Small illumination correction shared by the camera recognition nodes."""

import cv2
import numpy as np


def enhance_lighting(image, force=False):
    """Return the original frame unless its luminance needs correction."""
    height, width = image.shape[:2]
    if min(height, width) >= 180:
        sample = image[int(height * 0.4):int(height * 0.75),
                       int(width * 0.1):int(width * 0.9)]
    else:
        sample = image
    sample = sample[::max(1, sample.shape[0] // 120),
                    ::max(1, sample.shape[1] // 160)]
    gray = cv2.cvtColor(sample, cv2.COLOR_BGR2GRAY)
    dark, bright = np.percentile(gray, (25, 75))
    poor_lighting = (bright < 60 or
                     (dark > 170 and bright >= 220) or
                     (bright - dark < 35 and bright < 185))
    if not force and not poor_lighting:
        return image

    ycrcb = cv2.cvtColor(image, cv2.COLOR_BGR2YCrCb)
    gain = float(np.clip(115.0 / max(bright, 20.0), 0.65, 2.5))
    values = np.arange(256, dtype=np.float32)
    lut = np.clip(values * gain, 0, 255).astype(np.uint8)
    luminance = cv2.LUT(ycrcb[:, :, 0], lut)
    tiles = (4, 4) if min(height, width) < 128 else (8, 8)
    ycrcb[:, :, 0] = cv2.createCLAHE(clipLimit=2.0, tileGridSize=tiles).apply(luminance)
    return cv2.cvtColor(ycrcb, cv2.COLOR_YCrCb2BGR)
