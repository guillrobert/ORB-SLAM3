import os
import re
import numpy as np

print("loading required evo modules")
from evo.core import trajectory, sync, metrics
from evo.tools import file_interface


print("loading plot modules")
from evo.tools import plot
import matplotlib.pyplot as plt

input_directory = "results/ORB_SLAM3_output/"
output_directory = "results/Data_analysis_output/"
GT_directory = "Ground_Truth/"

for dataset in os.listdir(input_directory):
    if(dataset == "kitti"):
        data_format = "kitti"
    else : 
        data_format = "tum"
    print("\n"+dataset+" :")
    for sequence in os.listdir(input_directory+dataset):
        print(sequence + " :")

        traj_dict = {}
        err_dict = {}

        traj_dict['ref'] = traj_ref = file_interface.read_tum_trajectory_file(
                    GT_directory+dataset+"/"+sequence+".tum")

        for curr_traj_name in os.listdir(input_directory+dataset+"/"+sequence):
            if(re.search(r'^f_',curr_traj_name)): # if file name match "^f_*" (regex) - if it is a camera trajectory file
                # if we are in mono only, rescale the results to make it comparable
                print(curr_traj_name)
                mono = False
                if(re.search(r'_mono.txt$',curr_traj_name)):
                    mono = True
                    
                curr_traj_name_2 = curr_traj_name.replace(".txt","").replace("f_","").replace("512_","").replace("_"," ")

                print("loading trajectories")
                curr_traj = file_interface.read_tum_trajectory_file(
                    input_directory+dataset+"/"+sequence+"/"+curr_traj_name)

                print("registering and aligning trajectories")
                traj_ref, curr_traj = sync.associate_trajectories(traj_dict['ref'], curr_traj)
                curr_traj.align(traj_ref, correct_scale=mono, n=300)

                print(traj_ref)
                print(curr_traj)

                traj_dict[curr_traj_name_2] = curr_traj

                print("calculating APE")
                data = (traj_ref, curr_traj)
                ape_metric = metrics.APE(metrics.PoseRelation.translation_part)
                ape_metric.process_data(data)
                ape_statistics = ape_metric.get_all_statistics()
                print("mean:", ape_statistics["mean"])
                
                err_dict[curr_traj_name_2] = ape_metric.error


        print("plotting")

        plot_collection = plot.PlotCollection("plots for "+sequence.replace("_","").replace("512","").replace("dataset","").replace("-",""))
        # All trajectories
        fig_1 = plt.figure()
        plot_mode = plot.PlotMode.xyz
        plot.trajectories(fig_1,traj_dict,plot_mode,title="Trajectories for the sequence")
        plot_collection.add_figure("all trajs", fig_1)

        # APE Error
        fig_2 = plt.figure()
        for k,v in err_dict.items():
            plt.plot(traj_dict[k].timestamps-traj_dict[k].timestamps[0],v,label=k)
            plt.legend()
            plt.xlabel("Temps écoulé (s)")
            plt.ylabel("Absolute Position Error (m)")
        plot_collection.add_figure("APE Error", fig_2)

        # trajectory colormapped with speed
        fig_3,ax = plt.subplots()
        ax.boxplot(err_dict.values())
        plt.xlabel("Estimation")
        plt.ylabel("Absolute Position Error (m)")
        ax.set_xticklabels(err_dict.keys())
        plot_collection.add_figure("Boxplots", fig_3)
        plot_collection.show()


print("Done")