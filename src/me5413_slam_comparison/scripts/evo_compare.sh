#!/bin/bash

evo_ape tum groundtruth.tum gmapping.tum \
cartographer.tum rtabmap.tum \
--plot --plot_mode xy \
--save_plot ../results/plots/slam_comparison.pdf
