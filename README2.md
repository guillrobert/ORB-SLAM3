Nous partons sur un système d’exploitation Ubuntu 18.04 afin de pouvoir profiter du simulateur
de MuSHR également car celui-ci n’est pas encore fonctionnel en 20.04 à l’heure actuelle. Le reste
peut cependant être installé également en Ubuntu 20.04

# 0.1 Installation

On partira du principe que nous installeront toutes les librairies et dépendances dans un dossier
"code_dir" déjà existant dans le home directory.
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

## 0.1.6 ORB SLAM3

Installer ORB SLAM3 :

```
cd ~/code_dir
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
cd ORB_SLAM3
chmod +x build.sh
#This could take a while
./build.sh
```

## 0.1.7 Evo

Installer evo pour l’évaluation :

```
sudo apt-get install python3-tk
pip3 install evo --upgrade --no-binary evo
```

# 0.2 Utilisation

Ici on va utiliser le dataset corridor4_512 de TUM-VI et slides2_512 et on considère qu’il est
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
donnés qui ont un exemple sur ORB SLAM3. Il faut pour cela mettre dans le fichier config.yaml les
informations à propos du dataset utilisé et de sa localisation et il l’exécutera tout seul. Il permet
également de pouvoir lancer plusieurs runs d’à filé et d’enregistrer les résultats dans un répertoire
défini.

Afin de l’utiliser, il faut faire un fichier config.yaml dans lequel il faut mettre quelques informa-
tions sur le dataset utilisé ainsi que le type de SLAM à effectuer. Il lance ensuite ORB_SLAM3
avec les paramètres adaptés.


On le lance avec la commande :

```
python3 ORB_SLAM3.py
```

Afin d’évaluer les résultats dans le cas des ensembles de donnés de TUM-vi, Kitti ainsi que
EuRoC, le script Data_analysis.py permet d’enregistrer et d’afficher les résultats sous une forme
permettant une analyse simple de chaque séquence pour chaque paramètre qui a été lancés 
précédemment avec ORB_SLAM3.py

On le lance avec la commande :

```
python3 Data_analysis.py
```
