#!/bin/bash

set -e

if [[ "$1" == "clean" ]]; then
    rm -rf srr_perf_artifacts
    rm -rf build_*
    exit 0
fi

CROSS_COMPILER_PATH_FLAG="-DCMAKE_CROSS_COMPILER_PATH=/u/cs452/public/xdev/bin"
if [[ -d "/u/cs452/public/xdev/bin" ]]; then
    CROSS_COMPILER_PATH_FLAG="-DCMAKE_CROSS_COMPILER_PATH=/u/cs452/public/xdev/bin"
elif [[ -d "/Users/tongkun/Playground/arm-gnu-toolchain-14.2.rel1-darwin-arm64-aarch64-none-elf/bin" ]]; then
    CROSS_COMPILER_PATH_FLAG="-DCMAKE_CROSS_COMPILER_PATH=/Users/tongkun/Playground/arm-gnu-toolchain-14.2.rel1-darwin-arm64-aarch64-none-elf/bin"
fi

ARTIFACT_DIR="srr_perf_artifacts"

build_kernel() {
    local opt_flag=$1
    local cache_flag=$2
    local build_dir="build_${opt_flag}_${cache_flag}"

    echo "Building configuration: optimization=$opt_flag, cache=$cache_flag"

    rm -rf "$build_dir"
    mkdir "$build_dir"

    cd "$build_dir"

    local cmake_flags="-DCMAKE_BUILD_TYPE=Release -DMMU=on -DSRR_PERF=ON $CROSS_COMPILER_PATH_FLAG"

    if [[ "$opt_flag" == "opt" ]]; then
        cmake_flags="$cmake_flags -DOPT=ON"
    fi

    case "$cache_flag" in
        "icache")
            cmake_flags="$cmake_flags -DICACHE=ON"
            ;;
        "dcache")
            cmake_flags="$cmake_flags -DDCACHE=ON"
            ;;
        "bcache")
            cmake_flags="$cmake_flags -DBOTH_CACHE=ON"
            ;;
        "nocache")
            cmake_flags="$cmake_flags -DNODCACHE=ON"
            ;;
    esac

    cmake $cmake_flags ..
    make -j$(nproc 2>/dev/null || echo 4)

    cd ..

    echo "Built kernel.img in $build_dir/"
    cp "$build_dir/kernel.img" "$ARTIFACT_DIR/kernel_${opt_flag}_${cache_flag}.img"
}


mkdir -p "$ARTIFACT_DIR"

configurations=(
    "noopt:nocache"
    "noopt:icache"
    "noopt:dcache"
    "noopt:bcache"
    "opt:nocache"
    "opt:icache"
    "opt:dcache"
    "opt:bcache"
)

for config in "${configurations[@]}"; do
    IFS=':' read -r opt_flag cache_flag <<< "$config"
    build_dir="build_${opt_flag}_${cache_flag}"

    echo ""
    echo "Processing configuration: $opt_flag + $cache_flag"
    echo "-----------------------------------------------"

    build_kernel "$opt_flag" "$cache_flag"
done
