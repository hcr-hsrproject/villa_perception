# rude_carnie_ros

A TensorFlow port of Rude Carnie with the minimal changes made to allow it to work in ROS. Includes helpers to expose common functionality.

## Installation

1. [Install Tensorflow](https://www.tensorflow.org/install/install_linux)

2. Run `checkpoints/get_weights.sh` to pull down fine-tuned age and gender models.

## Usage

This repo just provides the framework. Please use nodes from `age_identification` and `gender_identification` to run models.
