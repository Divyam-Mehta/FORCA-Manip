#!/bin/sh
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | sed -e "s|${CONDA_PREFIX}/lib:||g" -e "s|:${CONDA_PREFIX}/lib||g" -e "s|${CONDA_PREFIX}/lib||g")

