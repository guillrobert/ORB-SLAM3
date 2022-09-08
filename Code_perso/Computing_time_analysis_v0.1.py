import os
import re

import numpy as np
import matplotlib.pyplot as plt

input_directory = "results/ORB_SLAM3_output/"


for dataset in os.listdir(input_directory):
    print("\n"+dataset+" :")
    for sequence in os.listdir(input_directory+dataset):
        if(sequence!="kitti"):
            print(sequence + " :")
            for curr_traj in os.listdir(input_directory+dataset+"/"+sequence):
                if(re.search(r'^t_',curr_traj)): # if file name match "$p_*" (regex)
                    print(curr_traj)
                    # open file and make an array with its content
                    with open(input_directory+dataset+"/"+sequence+"/"+curr_traj,"r") as file:
                        compute_times = [ float(time) for time in file.read().splitlines() if time!='' ]
                    plt.hist(compute_times, 50)
                    plt.title("historgramme des temps de calculs pour "+curr_traj)
                    plt.show()
                    plt.boxplot(compute_times)
                    plt.title("boxplot des temps de calculs pour "+curr_traj)
                    plt.show()
                    # TODO légende + généralisation ou regrouppement par dataset + boxplot
                    
                    
                    
    

print("Done")