import os
import subprocess
import yaml
from yaml import SafeLoader

with open("config_v0.1.yaml") as file:
    allconf = list(yaml.load_all(file, Loader=SafeLoader))

for conf in allconf:
    #if dataset on an external drive
    if (conf.get("path_data_ext")):
        print("Copying external dataset locally ...")
        copy = os.system("mkdir ~/datasets/tmp/dataset && cp -r "+conf["path_data_ext"]+"/mav0 ~/datasets/tmp/dataset")
        path_data = "~/datasets/tmp/dataset"
        print("Copy done")
    else:
        path_data = conf["path_dataset"]

    print("Config file open "+conf["save_name"])
    command = conf["exec"]+" "+conf["vocabulary"]+" "+conf["settings"]+" "+ path_data +conf["image_1"]+" "
    #if stereo
    if (conf.get("image_2")):
        command = command+path_data+conf["image_2"]+" "
    command = command+conf["timestamps"]+" "
    #if inertial
    if (conf.get("imudata")):
        command = command+conf["imudata"]+" "
    command = command+conf["save_path"]+" "+conf["save_name"]
    
    print(command)
    
    proc = os.system(command)
    
    if (conf.get("path_data_ext")):
        print("Removing the local copy of the dataset...")
        remove = os.system("rm -r ~/datasets/tmp/dataset")
        print("Removing done")

print("Done")