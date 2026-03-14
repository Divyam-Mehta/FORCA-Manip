#!/bin/bash
# Download common models

python3 -c "import sys;
from ament_index_python import get_package_share_directory;
path = get_package_share_directory('sanet_onionsorting');
sys.path.append(path + '/thirdparty/yolov5/');
from utils.google_utils import *;
attempt_download('weights/yolov5s.pt');
attempt_download('weights/yolov5m.pt');
attempt_download('weights/yolov5l.pt');
attempt_download('weights/yolov5x.pt')"
