Ce git est basé sur ORB_SLAM3 légèrement modifié afin de résondre certains problèmes de crashs rencontrés avec le code oiginal. A également été ajoutés des scripts afin de simplifier son utilisation et d'effectuer une analyse des données rendues.

# 0.1 Installation

Il a été fait pour système d’exploitation Ubuntu 18.04 afin de pouvoir profiter du simulateur
de MuSHR également car celui-ci n’est pas encore fonctionnel en 20.04 à l’heure actuelle 
(Cela dit, nous ne l'installons pas ici, mais il peut être utile pour certains). 
Le reste peut cependant être installé également en Ubuntu 20.04

Il faudra remplacer "~/code_dir" par votre workspace (l'emplacement où vous comptez le placer) pour l'installation et l'utilisation.

Pour toute la partie installation, une connexion internet est requise.


Mettre à jour le manager de packets

```
sudo apt update
sudo apt upgrade
```

## 0.1.1 Python

Installer python et numpy

```
sudo apt install python python-pip python3-dev python3-numpy
pip3 install numpy
```

## 0.1.2 Dépendances

Installer les prérequis

```
sudo apt install build-essential cmake git pkg-config libgtk-3-dev \
  libavcodec-dev libavformat-dev libswscale-dev libv4l-dev nano \
  libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
  gfortran openexr libatlas-base-dev curl wget autoconf automake \
  libtbb2 libtbb-dev libdc1394-22-dev libssl-dev libboost-all-dev
```

## 0.1.3 Eigen3

Installer Eigen3

```
sudo apt install libeigen3-dev
```

## 0.1.4 Pangolin

Installer Pangolin

```
cd ~/code_dir
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
./scripts/install_prerequisites.sh recommended
mkdir build && cd build
#This could take a while
cmake ..
sudo make install
```

## 0.1.5 OpenCV

Installer OpenCV

```
mkdir ~/code_dir/opencv_build && cd ~/code_dir/opencv_build
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
cd ~/code_dir/opencv_build/opencv
mkdir build && cd build
#This could take a while
cmake -D CMAKE_BUILD_TYPE=RELEASE \
  -D CMAKE_INSTALL_PREFIX=/usr/local \
  -D INSTALL_C_EXAMPLES=ON \
  -D INSTALL_PYTHON_EXAMPLES=ON \
  -D OPENCV_GENERATE_PKGCONFIG=ON \
  -D OPENCV_EXTRA_MODULES_PATH=~/code_dir/opencv_build/opencv_contrib/modules \
  -D BUILD_EXAMPLES=ON ..
make -j8 # -j8 a changer selon le nombre de coeurs du processeur
sudo make install
```

## 0.1.6 Evo

Installer evo pour l’évaluation :

```
sudo apt-get install python3-tk
pip3 install evo --upgrade --no-binary evo
```

## 0.1.7 ORB SLAM3

Pour installer ce fork d'ORB SLAM3 : 
```
cd ~/code_dir
git clone https://github.com/guillrobert/ORB_SLAM3.git
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

# 0.2 Utilisation

Ici on va utiliser le dataset corridor4_512 de TUM-VI et slides2_512 en stéréo-inertiel et on considère la séquence
dans un dossier "dataset" dans le home directory
Exemple TUM-VI :

```
cd ~/code_dir/ORB_SLAM3/Exemples/Stereo-Inertial
# args : Voc CamParam Cam1 Cam2 TimeStamps IMU
  ./stereo_inertial_tum_vi ../../Vocabulary/ORBvoc.txt TUM-VI.yaml \
  ~/dataset/TUM-VI/dataset-corridor4_512/mav0/cam0/data \
  ~/dataset/TUM-VI/dataset-corridor4_512/mav0/cam1/data \
  TUM_Timestamps/dataset-corridor4_512.txt \
  TUM_IMU/dataset-corridor4_512.txt
```

Pour faire du multi-sequences, il faut faire ainsi :

```
cd ~/code_dir/ORB_SLAM3/Exemples/Stereo-Inertial
# args : Voc CamParam Cam1Seq1 Cam2Seq1 TimeStampsSeq1 IMUSeq1
# Cam1Seq2 Cam2Seq2 TimeStampsSeq2 IMUSeq2
  ./stereo_inertial_tum_vi ../../Vocabulary/ORBvoc.txt TUM-VI.yaml \
  ~/dataset/TUM-VI/dataset-corridor4_512/mav0/cam0/data \
  ~/dataset/TUM-VI/dataset-corridor4_512/mav0/cam1/data \
  TUM_Timestamps/dataset-corridor4_512.txt \
  TUM_IMU/dataset-corridor4_512.txt \
  ~/dataset/TUM-VI/dataset-slides2_512/mav0/cam0/data \
  ~/dataset/TUM-VI/dataset-slides2_512/mav0/cam1/data \
  TUM_Timestamps/dataset-slides2_512.txt \
  TUM_IMU/dataset-slides2_512.txt
```

J’ai également réalisé un script python permettant de facilement lancer un des ensembles de
donnés qui ont un exemple sur ORB SLAM3. Il faut pour cela mettre dans le fichier config_v0.x.yaml les
informations à propos du dataset utilisé et de sa localisation et il l’exécutera tout seul. Il permet
également de pouvoir lancer plusieurs séquences d’à filé et d’enregistrer les résultats dans le répertoire 'results'.

si on souhaite activer/désactiver l'interface utilisateur, il faut pour cela modifier le fichier .cc exécuté ('stereo_inertial_euroc.cc' à la ligne 120, si on veut lancer du stéréo-inertiel) et rebuild le projet ensuite (avec `./build.sh`). Il faut changer `ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO, true, 0);` (ou équivalent) en `ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO, false, 0);` pour désactiver l'interface, et l'inverse pour l'activer.

À noter que ce script utilise l'exécutable c++ de EuRoC pour les deux datasets mais qu'il fourni les trajectoires en format TUM vi car il est plus adapté pour des comparaisons de trajectoires (le format euroc contient trop d'informations) et que Evo ne peut comparer que des trajectoires en formats TUM vi (il peut cependant convertir une trajectoire du format euroc au format tum vi à l'aide de la commande `evo_traj euroc --save_as_tum <traj>`).



Ce script permet d'utiliser un dataset enregistré en local uniquement, il est aussi capable de chercher les images sur un disque externe mais cela fausse les résultats car dans le temps de calcul est compté le temps d'accès aux images (bien plus long dans le cas d'un disque externe). Un début d'adaptation pour contrer cela à été fait, mais il est pour l'instant non fonctionnel.



On le lance ensuite avec la commande :

```
python3 ORB_SLAM3_v0.x.py
```

Sur les multiples versions (0.1, 0.2 et 0.3) la 0.3 est plus simple d'utilisation mais un peu moins flexible que la 0.1. La 0.2 n'était qu'un passage entre les deux.
On utilise la même version du fichier de config que de l'exécutable.

À noter également que les vérités terrains sont dans le dossier 'Ground_Truth' au format TUM vi


Afin d’évaluer les résultats dans le cas des ensembles de donnés de TUM-vi et EuRoC,
le script Data_analysis.py permet d’enregistrer et d’afficher les résultats sous une forme
permettant une analyse simple de chaque séquence pour chaque paramètre qui a été lancés 
précédemment avec ORB_SLAM3.py

Il va effectuer séparément une analyse pour chaque sous-dossiers du dossier 'results/ORB_SLAM3_output'.

On le lance avec la commande :

```
python3 Data_analysis_v0.x.py
```

La version 0.1 va créer des fichiers contenant différentes informations sur les données (dans le dossier 'results/Data_analysis_output') quand le 0.2 va afficher différentes courbes à propos des trajectoires.

Pour ce qui est des résultats des temps de calculs, j'ai fait le script 'Computing_time_analysis.py' qui s'exécute comme les précédents.
Il va afficher un boxplot pour chaque sous-dossiers du dossier 'results/ORB_SLAM3_output'.

----------------------------------------------
----------------------------------------------

# ORB-SLAM3

### V1.0, December 22th, 2021
**Authors:** Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, [José M. M. Montiel](http://webdiis.unizar.es/~josemari/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/).

The [Changelog](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Changelog.md) describes the features of each version.

ORB-SLAM3 is the first real-time SLAM library able to perform **Visual, Visual-Inertial and Multi-Map SLAM** with **monocular, stereo and RGB-D** cameras, using **pin-hole and fisheye** lens models. In all sensor configurations, ORB-SLAM3 is as robust as the best systems available in the literature, and significantly more accurate. 

We provide examples to run ORB-SLAM3 in the [EuRoC dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) using stereo or monocular, with or without IMU, and in the [TUM-VI dataset](https://vision.in.tum.de/data/datasets/visual-inertial-dataset) using fisheye stereo or monocular, with or without IMU. Videos of some example executions can be found at [ORB-SLAM3 channel](https://www.youtube.com/channel/UCXVt-kXG6T95Z4tVaYlU80Q).

This software is based on [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) developed by [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2)).

<a href="https://youtu.be/HyLNq-98LRo" target="_blank"><img src="https://img.youtube.com/vi/HyLNq-98LRo/0.jpg" 
alt="ORB-SLAM3" width="240" height="180" border="10" /></a>

### Related Publications:

[ORB-SLAM3] Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M. M. Montiel and Juan D. Tardós, **ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM**, *IEEE Transactions on Robotics 37(6):1874-1890, Dec. 2021*. **[PDF](https://arxiv.org/abs/2007.11898)**.

[IMU-Initialization] Carlos Campos, J. M. M. Montiel and Juan D. Tardós, **Inertial-Only Optimization for Visual-Inertial Initialization**, *ICRA 2020*. **[PDF](https://arxiv.org/pdf/2003.05766.pdf)**

[ORBSLAM-Atlas] Richard Elvira, J. M. M. Montiel and Juan D. Tardós, **ORBSLAM-Atlas: a robust and accurate multi-map system**, *IROS 2019*. **[PDF](https://arxiv.org/pdf/1908.11585.pdf)**.

[ORBSLAM-VI] Raúl Mur-Artal, and Juan D. Tardós, **Visual-inertial monocular SLAM with map reuse**, IEEE Robotics and Automation Letters, vol. 2 no. 2, pp. 796-803, 2017. **[PDF](https://arxiv.org/pdf/1610.05949.pdf)**. 

[Stereo and RGB-D] Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *IEEE Transactions on Robotics,* vol. 33, no. 5, pp. 1255-1262, 2017. **[PDF](https://arxiv.org/pdf/1610.06475.pdf)**.

[Monocular] Raúl Mur-Artal, José M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (**2015 IEEE Transactions on Robotics Best Paper Award**). **[PDF](https://arxiv.org/pdf/1502.00956.pdf)**.

[DBoW2 Place Recognition] Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp. 1188-1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**

# 1. License

ORB-SLAM3 is released under [GPLv3 license](https://github.com/UZ-SLAMLab/ORB_SLAM3/LICENSE). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Dependencies.md).

For a closed-source version of ORB-SLAM3 for commercial purposes, please contact the authors: orbslam (at) unizar (dot) es.

If you use ORB-SLAM3 in an academic work, please cite:
  
    @article{ORBSLAM3_TRO,
      title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial 
               and Multi-Map {SLAM}},
      author={Campos, Carlos AND Elvira, Richard AND G\´omez, Juan J. AND Montiel, 
              Jos\'e M. M. AND Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics}, 
      volume={37},
      number={6},
      pages={1874-1890},
      year={2021}
     }

# 2. Prerequisites
We have tested the library in **Ubuntu 16.04** and **18.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 3.0. Tested with OpenCV 3.2.0 and 4.4.0**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

## Python
Required to calculate the alignment of the trajectory with the ground truth. **Required Numpy module**.

* (win) http://www.python.org/downloads/windows
* (deb) `sudo apt install libpython2.7-dev`
* (mac) preinstalled with osx

## ROS (optional)

We provide some examples to process input of a monocular, monocular-inertial, stereo, stereo-inertial or RGB-D camera using ROS. Building these examples is optional. These have been tested with ROS Melodic under Ubuntu 18.04.

# 3. Building ORB-SLAM3 library and examples

Clone the repository:
```
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *ORB-SLAM3*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM3.so**  at *lib* folder and the executables in *Examples* folder.

# 4. Running ORB-SLAM3 with your camera

Directory `Examples` contains several demo programs and calibration files to run ORB-SLAM3 in all sensor configurations with Intel Realsense cameras T265 and D435i. The steps needed to use your own camera are: 

1. Calibrate your camera following `Calibration_Tutorial.pdf` and write your calibration file `your_camera.yaml`

2. Modify one of the provided demos to suit your specific camera model, and build it

3. Connect the camera to your computer using USB3 or the appropriate interface

4. Run ORB-SLAM3. For example, for our D435i camera, we would execute:

```
./Examples/Stereo-Inertial/stereo_inertial_realsense_D435i Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/RealSense_D435i.yaml
```

# 5. EuRoC Examples
[EuRoC dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) was recorded with two pinhole cameras and an inertial sensor. We provide an example script to launch EuRoC sequences in all the sensor configurations.

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Open the script "euroc_examples.sh" in the root of the project. Change **pathDatasetEuroc** variable to point to the directory where the dataset has been uncompressed. 

3. Execute the following script to process all the sequences with all sensor configurations:
```
./euroc_examples
```

## Evaluation
EuRoC provides ground truth for each sequence in the IMU body reference. As pure visual executions report trajectories centered in the left camera, we provide in the "evaluation" folder the transformation of the ground truth to the left camera reference. Visual-inertial trajectories use the ground truth from the dataset.

Execute the following script to process sequences and compute the RMS ATE:
```
./euroc_eval_examples
```

# 6. TUM-VI Examples
[TUM-VI dataset](https://vision.in.tum.de/data/datasets/visual-inertial-dataset) was recorded with two fisheye cameras and an inertial sensor.

1. Download a sequence from https://vision.in.tum.de/data/datasets/visual-inertial-dataset and uncompress it.

2. Open the script "tum_vi_examples.sh" in the root of the project. Change **pathDatasetTUM_VI** variable to point to the directory where the dataset has been uncompressed. 

3. Execute the following script to process all the sequences with all sensor configurations:
```
./tum_vi_examples
```

## Evaluation
In TUM-VI ground truth is only available in the room where all sequences start and end. As a result the error measures the drift at the end of the sequence. 

Execute the following script to process sequences and compute the RMS ATE:
```
./tum_vi_eval_examples
```

# 7. ROS Examples

### Building the nodes for mono, mono-inertial, stereo, stereo-inertial and RGB-D
Tested with ROS Melodic and ubuntu 18.04.

1. Add the path including *Examples/ROS/ORB_SLAM3* to the ROS_PACKAGE_PATH environment variable. Open .bashrc file:
  ```
  gedit ~/.bashrc
  ```
and add at the end the following line. Replace PATH by the folder where you cloned ORB_SLAM3:

  ```
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM3/Examples/ROS
  ```
  
2. Execute `build_ros.sh` script:

  ```
  chmod +x build_ros.sh
  ./build_ros.sh
  ```
  
### Running Monocular Node
For a monocular input from topic `/camera/image_raw` run node ORB_SLAM3/Mono. You will need to provide the vocabulary file and a settings file. See the monocular examples above.

  ```
  rosrun ORB_SLAM3 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```

### Running Monocular-Inertial Node
For a monocular input from topic `/camera/image_raw` and an inertial input from topic `/imu`, run node ORB_SLAM3/Mono_Inertial. Setting the optional third argument to true will apply CLAHE equalization to images (Mainly for TUM-VI dataset).

  ```
  rosrun ORB_SLAM3 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE [EQUALIZATION]	
  ```

### Running Stereo Node
For a stereo input from topic `/camera/left/image_raw` and `/camera/right/image_raw` run node ORB_SLAM3/Stereo. You will need to provide the vocabulary file and a settings file. For Pinhole camera model, if you **provide rectification matrices** (see Examples/Stereo/EuRoC.yaml example), the node will recitify the images online, **otherwise images must be pre-rectified**. For FishEye camera model, rectification is not required since system works with original images:

  ```
  rosrun ORB_SLAM3 Stereo PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION
  ```

### Running Stereo-Inertial Node
For a stereo input from topics `/camera/left/image_raw` and `/camera/right/image_raw`, and an inertial input from topic `/imu`, run node ORB_SLAM3/Stereo_Inertial. You will need to provide the vocabulary file and a settings file, including rectification matrices if required in a similar way to Stereo case:

  ```
  rosrun ORB_SLAM3 Stereo_Inertial PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION [EQUALIZATION]	
  ```
  
### Running RGB_D Node
For an RGB-D input from topics `/camera/rgb/image_raw` and `/camera/depth_registered/image_raw`, run node ORB_SLAM3/RGBD. You will need to provide the vocabulary file and a settings file. See the RGB-D example above.

  ```
  rosrun ORB_SLAM3 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```

**Running ROS example:** Download a rosbag (e.g. V1_02_medium.bag) from the EuRoC dataset (http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Open 3 tabs on the terminal and run the following command at each tab for a Stereo-Inertial configuration:
  ```
  roscore
  ```
  
  ```
  rosrun ORB_SLAM3 Stereo_Inertial Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/EuRoC.yaml true
  ```
  
  ```
  rosbag play --pause V1_02_medium.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw /imu0:=/imu
  ```
  
Once ORB-SLAM3 has loaded the vocabulary, press space in the rosbag tab.

**Remark:** For rosbags from TUM-VI dataset, some play issue may appear due to chunk size. One possible solution is to rebag them with the default chunk size, for example:
  ```
  rosrun rosbag fastrebag.py dataset-room1_512_16.bag dataset-room1_512_16_small_chunks.bag
  ```

# 8. Running time analysis
A flag in `include\Config.h` activates time measurements. It is necessary to uncomment the line `#define REGISTER_TIMES` to obtain the time stats of one execution which is shown at the terminal and stored in a text file(`ExecTimeMean.txt`).

# 9. Calibration
You can find a tutorial for visual-inertial calibration and a detailed description of the contents of valid configuration files at  `Calibration_Tutorial.pdf`
