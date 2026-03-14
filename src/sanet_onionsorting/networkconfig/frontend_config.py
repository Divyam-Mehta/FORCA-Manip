#! /usr/bin/python
class configs_raw:
    # File Paths
    datapath = "sanet_onionsorting/data/Action_Data"
    model_path = "sanet_onionsorting/snapshot/nnn.h5"
    logs_dir = "sanet_onionsorting/logs/"
    snap_path = "sanet_onionsorting/snapshot/rcn_m.h5"
    config_path = "sanet_onionsorting/thirdparty/yolo/yonfig.json"
    yolo_weight_file = "sanet_onionsorting/snapshot/Tiny_yolo_VOC_1.h5"

    # mask related
    mask_rgb_0 = "sanet_onionsorting/data/mask/rgb_mask_0.npy"
    mask_rgb_1 = "sanet_onionsorting/Action_Data/nobot/nobot_1/rgb_images1/frame48.npy"
    mask_rgb_2 = "sanet_onionsorting/utils/mask/rgb_mask_2.npy"

    mask_depth_0 = "sanet_onionsorting/data/mask/d_mask_0.npy"
    mask_depth_1 = "sanet_onionsorting/Action_Data/nobot/nobot_1/depth_images1/dframe1.npy"
    mask_depth_2 = "sanet_onionsorting/utils/mask/d_mask_2.npy"

    #other
    labels_to_names = {0: 'Blemished', 1: 'Unblemished'}