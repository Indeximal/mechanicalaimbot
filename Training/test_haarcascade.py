import os
import sys

import cv2
import numpy as np


xml_file = sys.argv[1]
directory = sys.argv[2]

haar_cascade = cv2.CascadeClassifier(xml_file)

for file in os.listdir(directory):
    file_path = os.path.join(directory, os.fsdecode(file))

    img = cv2.imread(file_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    heads = haar_cascade.detectMultiScale(gray, 1.2, 6)

    for x, y, w, h in heads:
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)

    cv2.imshow('Frame', img)
    key_pressed = cv2.waitKey(0)

    if key_pressed == ord("q"):
        break

cv2.destroyAllWindows()