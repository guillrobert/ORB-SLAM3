import os
import re

import numpy as np
import matplotlib.pyplot as plt

input_directory = "results/ORB_SLAM3_output/"


for dataset in os.listdir(input_directory):
    print("\n"+dataset+" :")
    for sequence in os.listdir(input_directory+dataset):
        if(sequence!="kitti"):
            trajdict = {}
            print(sequence + " :")
            for curr_traj in os.listdir(input_directory+dataset+"/"+sequence):
                if(re.search(r'^t_',curr_traj)): # if file name match "$p_*" (regex)
                    print(curr_traj)
                    # open file and make an array with its content
                    with open(input_directory+dataset+"/"+sequence+"/"+curr_traj,"r") as file:
                        compute_times = [ float(time) for time in file.read().splitlines() if time!='' ]
                    trajdict[curr_traj.replace("t_","").replace(".txt","").replace("512_","").replace("_"," ")]=compute_times
            fig,ax = plt.subplots()
            ax.boxplot(trajdict.values())
            ax.set_xticklabels(trajdict.keys())
            plt.ylabel("Temps de calcul pour une image (s)")
            plt.xlabel("Configuration de capteurs")
            plt.show()
print("Done")
