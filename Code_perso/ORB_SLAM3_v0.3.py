import os
import yaml
from yaml import SafeLoader

with open("config_v0.3.yaml") as file:
    allconf = list(yaml.load_all(file, Loader=SafeLoader))


for conf in allconf:
    #if dataset on an external drive
    if (conf.get("path_data_ext")):
        print("Copying external dataset locally ...")
        os.system("mkdir ~/datasets/tmp/dataset")
        os.system("cp -r "+conf["path_data_ext"]+"/mav0 ~/datasets/tmp/dataset")
        path_data = "~/datasets/tmp/dataset"
        print("Copy done")
    else:
        path_data = conf["path_dataset"]

    # variables definition
    voc = "~/code_dir/ORB_SLAM3/Vocabulary/ORBvoc.txt"
    save_name = conf["save_name"]
    curr_path = "~/code_dir/ORB_SLAM3/Examples/"
    exec = "./../Examples/"

    # SLAM Type
    if (conf["slam_type"] == "monocular"):
        print("Monocular SLAM")
        exec = exec + "Monocular/mono_"
        curr_path = curr_path + "Monocular/"
    elif (conf["slam_type"]=="mono-inertial"):
        print("Monocular-Inertial SLAM")
        exec = exec + "Monocular-Inertial/mono_inertial_"
        curr_path = curr_path + "Monocular-Inertial/"
    elif (conf["slam_type"]=="stereo"):
        print("Stereo SLAM")
        exec = exec + "Stereo/stereo_"
        curr_path = curr_path + "Stereo/"
    else:
        print("Stereo-Inertial SLAM")
        exec = exec + "Stereo-Inertial/stereo_inertial_"
        curr_path = curr_path + "Stereo-Inertial/"

    # Dataset Used
    # TUM VI
    dataset_images = ""
    if (conf["dataset"]=="tum"):
        exec = exec + "euroc"
        for i in range(0,len(path_data)):
            dataset_images = dataset_images + path_data[i] + " " + curr_path + "TUM_TimeStamps/" + conf["timestamps_name"][i] + ".txt "
        settings = curr_path + "TUM-VI.yaml"
    # EuRoC MAV
    elif (conf["dataset"]=="euroc"):
        exec = exec + "euroc"
        for i in range(0,len(path_data)):
            dataset_images = dataset_images + path_data[i] + " " + curr_path + "EuRoC_TimeStamps/" + conf["timestamps_name"][i] + ".txt "
        settings = curr_path + "EuRoC.yaml"
    # Kitti odometry
    else:
        print("Kitti dataset")
        exec = exec + "kitti"
        for i in range(0,len(path_data)):
            dataset_images = dataset_images + path_data[i]
        settings = curr_path + "KITTI00-02.yaml"

    # making the shell command
    command = exec + " " + voc + " " + settings + " " + dataset_images + save_name
    
    print(command)
    
    # executing the shell command
    proc = os.system(command)
    
    # copying results on the local results folder
    for i in range(0,len(path_data)):
        os.makedirs("results/ORB_SLAM3_output/" + conf["dataset"] + "/" + conf["timestamps_name"][i],exist_ok=True)
        os.system("cp f_" + save_name + ".txt ~/code_dir/ORB_SLAM3/Code_perso/results/ORB_SLAM3_output/"  + conf["dataset"] + "/" + conf["timestamps_name"][i] + "/f_" + save_name + ".txt")
        
        os.system("cp kf_" + save_name + ".txt ~/code_dir/ORB_SLAM3/Code_perso/results/ORB_SLAM3_output/" + conf["dataset"] + "/" + conf["timestamps_name"][i] + "/kf_" + save_name + ".txt")
        
        os.system("cp t_" + save_name + ".txt ~/code_dir/ORB_SLAM3/Code_perso/results/ORB_SLAM3_output/" + conf["dataset"] + "/" + conf["timestamps_name"][i] + "/t_" + save_name + ".txt")
    try:
        os.remove("f_" + save_name + ".txt")
    except:
        print("fichier non existant")
    try:
        os.remove("kf_" + save_name + ".txt")
    except:
        print("fichier non existant")
    try:
        os.remove("t_" + save_name + ".txt")
    except:
        print("fichier non existant")

    # # Deleting temporary files if on an external drive
    # if (conf.get("path_data_ext")):
    #     print("Removing the local copy of the dataset...")
    #     os.system("rm -r ~/datasets/tmp/dataset")
    #     print("Removing done")

# end for
print("Done")