#!/bin/bash
source ./.venv/bin/activate

# Ловушка для завершения всех процессов и запуска autoland.py при нажатии Ctrl+C
trap 'kill $(jobs -p)' SIGINT

python ./main.py -f &
python ./main.py -b &
wait
