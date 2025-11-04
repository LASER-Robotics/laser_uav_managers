#!/bin/bash

PACKAGE_PATH=$(ros2 pkg prefix --share laser_uav_managers)

cp $PACKAGE_PATH/plotjuggler/eval_control_layout.xml /tmp/eval_control_layout.xml

sed -i "s/uav[0-9]/$UAV_NAME/g" /tmp/eval_control_layout.xml
