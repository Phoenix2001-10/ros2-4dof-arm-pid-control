#!/usr/bin/env bash
set -e
mkdir -p portfolio/assets
cp results/joint1_tracking.png portfolio/assets/joint1_tracking.png
cp results/joint2_tracking.png portfolio/assets/joint2_tracking.png
cp results/joint3_tracking.png portfolio/assets/joint3_tracking.png
cp results/joint4_tracking.png portfolio/assets/joint4_tracking.png
printf 'Copied final tracking plots into portfolio/assets/\n'
