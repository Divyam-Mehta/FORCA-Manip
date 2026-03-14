#! /usr/bin/python3

from ament_index_python.packages import get_package_share_directory
path = get_package_share_directory('sanet_onionsorting')

class configs_raw:
    # Paths
    datapath = "data/Action_Data"
    model_path = "sanet_onionsorting/snapshot/nnn_kfold_1.h5"
    model_path_kfold = "sanet_onionsorting/snapshot/kfold/nnn_kfold_"
    datapath_test = "sanet_onionsorting/data/Action_Data_Test"
    checkpoint_folder = path + "/checkpoint"
    logs_dir = "logs/"
    INPUT_SHAPE_STATE = (480, 640, 4)
    # Samples x Height x Width x Channel ||| Here Channel = Red,Green,Blue,Depth
    INPUT_SHAPE_TIME = (3, 224, 640, 4)
    INPUT_SHAPE_TIME_ACTION = (3, 150, 100, 4)

    # Network Hyper-Parameters
    BATCH_SIZE = 64
    BATCH = []
    TOTAL_EPOCHS = 30
    kfolds = 5
    # EPOCHS = int(TOTAL_EPOCHS / kfolds) + 1
    EPOCHS = 5000

    

    # RNN Parameters
    TIME_GAP = 5  # No of frames that has to be dropped between t,t-1,t-2

    # Kfold related
    kfolds_train_path = "./kfolds_store/train/"
    kfolds_test_path = "./kfolds_store/test/"

    # mask related
    mask_rgb_0 = "../../utils/mask/rgb_mask_0.npy"
    mask_rgb_1 = "Action_Data/nobot/nobot_1/rgb_images1/frame48.npy"
    mask_rgb_2 = "../../utils/mask/rgb_mask_2.npy"

    mask_depth_0 = "../../utils/mask/d_mask_0.npy"
    mask_depth_1 = "Action_Data/nobot/nobot_1/depth_images1/dframe1.npy"
    mask_depth_2 = "../../utils/mask/d_mask_2.npy"