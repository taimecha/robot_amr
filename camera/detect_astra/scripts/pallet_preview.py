"""Readable, lightweight bounding boxes for the recognition camera view."""
import cv2
import numpy as np


def draw_detections(image, boxes, classes, confidences, names):
    view = image.copy()
    h, w = view.shape[:2]
    labels = []
    # Draw every detected class, including Box; never just the seed pallet.
    for box, cls, confidence in zip(boxes, classes, confidences):
        name = names[int(cls)]
        key = name.strip().lower()
        title = {'box': 'Box', 'pallets': 'Pallet',
                 'emty pallet': 'Empty pallet'}.get(key, name)
        color = (60, 170, 245) if key == 'box' else (245, 175, 70)
        x1, y1, x2, y2 = np.rint(box).astype(int)
        x1, x2 = np.clip([x1, x2], 0, w-1)
        y1, y2 = np.clip([y1, y2], 0, h-1)
        cv2.rectangle(view, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)
        text = f'{title} {confidence:.0%}'
        (tw, th), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, .5, 1)
        width, height = min(w-2, tw+14), th+baseline+12
        x = max(1, min(int(x1), w-width-1))
        candidates = [max(1, y1-height-3), min(h-height-1, y2+3)]
        candidates += [max(1, candidates[0]-height-3), min(h-height-1, candidates[1]+height+3)]
        def overlap(y):
            return sum(max(0, min(x+width, b[2])-max(x, b[0])) *
                       max(0, min(y+height, b[3])-max(y, b[1])) for b in labels)
        y = min(candidates, key=overlap)
        labels.append((x, y, x+width, y+height))
        cv2.rectangle(view, (x, y), (x+width, y+height), color, cv2.FILLED)
        cv2.putText(view, text, (x+7, y+th+5), cv2.FONT_HERSHEY_SIMPLEX,
                    .5, (24, 30, 40), 1, cv2.LINE_AA)
    return view
