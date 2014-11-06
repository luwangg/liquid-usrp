#!/bin/bash
for i in {1..2000}
do
    echo "Run"
    timeout 60 ./simo_tone
    ./log-simo.py
    echo "Ran"
    sleep 5
done
