import os
import re

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
        os.system("rm -r "+output_directory+dataset+"/"+sequence+"/")
        os.makedirs(output_directory+dataset+"/"+sequence+"/",exist_ok=True)
        print(sequence + " :")
        for curr_traj in os.listdir(input_directory+dataset+"/"+sequence):
            if(re.search(r'^f_',curr_traj)): # if file name match "$f_*" (regex)
                # if we are in mono only, rescale the results to make it comparable
                print(curr_traj)
                if(re.search(r'_mono.txt$',curr_traj)):
                    scale = "s"
                else:
                    scale = ""
                command = "evo_ape "+data_format+" "+GT_directory+dataset+"/"+sequence+".tum "+input_directory+dataset+"/"+sequence+"/"+curr_traj+" -a"+scale+" --n_to_align=300 --save_results "+output_directory+dataset+"/"+sequence+"/"+curr_traj.replace(".txt",".zip")
                # print(command)
                os.system(command)
        os.system("evo_res "+output_directory+dataset+"/"+sequence+"/f_*.zip --save_table "+output_directory+dataset+"/"+sequence+"/"+sequence+".csv --ignore_title --silent")


print("Done")