#!/bin/bash
set -e

# Compile
echo "Compiling..."
make -j

# Run Testcase 1
echo "Running Testcase 1..."
./Lab1 testcase/testcase1.txt out1.txt
echo "Converting Testcase 1 result to CSV..."
./output2csv out1.txt out1.csv

# Run Testcase 2
echo "Running Testcase 2..."
./Lab1 testcase/testcase2.txt out2.txt
echo "Converting Testcase 2 result to CSV..."
./output2csv out2.txt out2.csv

echo "Done! Results saved in out1.csv and out2.csv"
