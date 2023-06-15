#!/usr/bin/env bash
ROUNDS=5000
PHOTONS=100000
CKPT_INTERVAL=10
METHOD=sppm
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
time bin/dzy testcases/scene06_bunny_1k_vn.txt sppm_output/scene06_bunny_1k_vn $METHOD $ROUNDS $PHOTONS $CKPT_INTERVAL