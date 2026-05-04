#!/bin/bash
BENCH_DIR="../ISPD 2008 Benchmarks-20230410T050756Z-001/ISPD 2008 Benchmarks"
benchmarks=("adaptec1" "adaptec2" "adaptec3" "adaptec4" "adaptec5" "bigblue1" "bigblue2" "bigblue3" "newblue1" "newblue2" "newblue5" "newblue6")

echo "Starting benchmark run..."
printf "%-15s %-10s %-10s %-15s\n" "Benchmark" "TOF" "MOF" "WL"
echo "--------------------------------------------------------"

for b in "${benchmarks[@]}"; do
    ./router "$BENCH_DIR/$b.gr" "$b.out" > "$b.log" 2>&1
    res=$(python3 verifier.py "$BENCH_DIR/$b.gr" "$b.out" --no-header)
    echo "$res"
done
