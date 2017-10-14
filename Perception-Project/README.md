# 3D Perception Project Training 

This section builds upon the Exercise 3 in this repository and applies them to create a training model for the [3D Perception Project](https://github.com/kevinfructuoso/3D-Perception-Project). A similar perception pipeline as in the other exercises will be used for the final project as well.. 

## Setup
* If you completed Exercises 1 and 2 you will already have a `sensor_stick` folder in your `~/perception_exercises/src` directory.  You should replace that folder with the `sensor_stick` folder contained in this repository and add the Python script you wrote for Exercise-2 to the `scripts` directory. 

* If you do not already have a `sensor_stick` directory, first copy/move the `sensor_stick` folder to the `~/perception_exercises/src` directory of your active ros workspace. 

* Make sure you have all the dependencies resolved by using the `rosdep install` tool and running `catkin_make`:  
 
```sh
$ cd ~/perception_exercises
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ catkin_make
```

* If it's not already there, add the following lines to your `.bashrc` file  

```
export GAZEBO_MODEL_PATH=~/perception_exercises/src/sensor_stick/models
source ~/perception_exercises/devel/setup.bash
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

The features will now be captured and you can watch the objects being spawned in Gazebo. It should take 5-10 sec. for each random orientations (depending on your machine's resources) so with 7 objects total it takes awhile to complete. When it finishes running you should have a `training_set.sav` file.

## Training

Once your feature extraction has successfully completed, you're ready to train your model. First, however, if you don't already have them, you'll need to install the `sklearn` and `scipy` Python packages.  You can install these using `pip`:

```sh
pip install sklearn scipy
```

After that, you're ready to run the `train_svm.py` model to train an SVM classifier on your labeled set of features.

```sh
$ rosrun sensor_stick train_svm.py
```
**Note:  Running this exercise out of the box your classifier will have poor performance because the functions `compute_color_histograms()` and `compute_normal_histograms()` (within `features.py` in /sensor_stick/src/sensor_stick) are generating random junk.  Fix them in order to generate meaningful features and train your classifier!**

## Classifying Segmented Objects

If everything went well you now have a trained classifier and you're ready to do object recognition for the 3D-Perception-Project!

