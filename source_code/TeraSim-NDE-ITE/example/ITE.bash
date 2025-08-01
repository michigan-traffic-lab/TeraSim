#!/bin/bash
export USE_LIBSUMO="1"
DIR_NAME="./output"
del_mode="all"
for i in {1..1000}; do

    experiment_name="ITE_test_$(date +%Y%m%d-%H%M%S)"
    echo "Experiment name: ${experiment_name}"
    mkdir -p ${DIR_NAME}/${experiment_name}
    mkdir -p ${DIR_NAME}/${experiment_name}/raw_data
    mkdir -p ${DIR_NAME}/${experiment_name}/raw_data/final_state
    mkdir -p ${DIR_NAME}/${experiment_name}/raw_data/maneuver_challenges

    mkdir -p ${DIR_NAME}/${experiment_name}/raw_data/
    # test record
    python3 safetest_mcity_main.py --dir ${DIR_NAME} --name ${experiment_name} --nth '0' > ${DIR_NAME}/${experiment_name}/raw_data/res.txt
    if [ $(grep "victim" ${DIR_NAME}/${experiment_name}/raw_data/${experiment_name}_0/collision.xml | wc -l) -eq 0 ]; then
        echo "Removing ${DIR_NAME}/${experiment_name}"
        if [ "${del_mode}" = "verbose" ]; then
            rm -f ${DIR_NAME}/${experiment_name}/raw_data/${experiment_name}_0/fcd_all.xml
        else
            rm -rf ${DIR_NAME}/${experiment_name}
        fi
    fi
done