# Object Recognition with Python, ROS and PCL
This repository supplements the perception pipeline used for the 3D-Perception-Project. It is used to create a training model used for object recognition. Also included is an example of the same perception pipeline used for the 3D-Perception-Project which provides filtering, segmentation, Euclidean clustering, and object recognition.

## Setup
For this setup, roboperception is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly
If you do not have an active ROS workspace, you can create one by:

```sh
$ mkdir -p ~/roboperception/src
$ cd ~/roboperception/
$ catkin_make
```

* If you do not already have a `sensor_stick` directory, first copy/move the `sensor_stick` folder to the `~/roboperception/src` directory of your active ros workspace. 

* Make sure you have all the dependencies resolved by using the `rosdep install` tool and running `catkin_make`:  
 
```sh
$ cd ~/roboperception
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ catkin_make
```

* If it's not already there, add the following lines to your `.bashrc` file  

```
export GAZEBO_MODEL_PATH=~/roboperception/src/sensor_stick/models
source ~/roboperception/devel/setup.bash
```

## Preparing for training

Launch the `training.launch` file to bring up the Gazebo environment: 

```sh
$ roslaunch sensor_stick training.launch
```
You should see an empty scene in Gazebo with only the sensor stick robot.

## Capturing Features
Next, in a new terminal, run the `capture_features.py` script to capture and save features for each of the objects in the environment.  This script spawns each object in random orientations (default 5 orientations per object) and computes features based on the point clouds resulting from each of the random orientations.

```sh
$ rosrun sensor_stick capture_features.py
```

The features will now be captured and you can watch the objects being spawned in Gazebo. It should take 5-10 sec. for each random orientations (depending on your machine's resources). When it finishes running you should have a `training_set.sav` file.

## Training

Once your feature extraction has successfully completed, you're ready to train your model. First, however, if you don't already have them, you'll need to install the `sklearn` and `scipy` Python packages.  You can install these using `pip`:

```sh
pip install sklearn scipy
```

After that, you're ready to run the `train_svm.py` model to train an SVM classifier on your labeled set of features.

```sh
$ rosrun sensor_stick train_svm.py
```

## Classifying Segmented Objects

If everything went well you now have a trained classifier and you're ready to do object recognition!

