#!/usr/bin/env bash
ROUNDS=5000
PHOTONS=100000
CKPT_INTERVAL=5
METHOD=sppm
# delete old build
rm -rf build

# If project not ready, generate cmake file.
if [[ ! -d build ]]; then
    mkdir -p build
    cd build
    cmake -S .. -B .
    cd ..
fi

# Build project.
cd build
cmake --build . --parallel
cd ..

# Run sppm.
mkdir -p sppm_output/scene06_bunny_1k_vn
time bin/dzy testcases/scene06_bunny_1k_vn.txt sppm_output/scene06_bunny_1k_vn_x $METHOD $ROUNDS $PHOTONS $CKPT_INTERVAL
#mkdir -p sppm_output/scene10_wineglass
#time bin/dzy testcases/scene10_wineglass.txt sppm_output/scene10_wineglass_x $METHOD $ROUNDS $PHOTONS $CKPT_INTERVAL
#mkdir -p sppm_output/scene10_wineglass
#time bin/dzy_curve testcases/scene10_wineglass.txt sppm_output/scene10_wineglass_curve $METHOD $ROUNDS $PHOTONS $CKPT_INTERVAL