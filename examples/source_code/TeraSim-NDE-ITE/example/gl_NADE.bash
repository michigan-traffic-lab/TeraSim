#!/bin/bash

#SBATCH --job-name="safetest_ite"
#SBATCH --mail-user=haoweis@umich.edu
#SBATCH --mail-type=BEGIN,END,FAIL
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=1
#SBATCH --mem=500mb
#SBATCH --array=0-499 # how many workers you are using
#SBATCH --time=00-24:00:00 # time duration
#SBATCH --account=henryliu98
#SBATCH --partition=standard
#SBATCH --output=/home/haoweis/safe_test.log # change to your directory



ulimit -c 0
cd /home/haoweis
source .bashrc
cd /home/haoweis/ASAP/TeraSim-NDE-ITE

module purge
module load python3.10-anaconda/2023.03
conda activate mcity_nade

cd /home/haoweis/ASAP/TeraSim-NDE-ITE/example
export IS_MAGNITUDE_INTERSECTION=10
export IS_MAGNITUDE_ROUNDABOUT=40
export IS_MAGNITUDE_HIGHWAY=40
export AVOID_COLLISION_IS_PROB=0.6


DIR_NAME="/scratch/mcguireg_root/mcguireg98/shared_data/safetest-nade"
export USE_LIBSUMO=1

# add time stamp to experiment name
experiment_name="NADE_IS_I_${IS_MAGNITUDE_INTERSECTION}_R_${IS_MAGNITUDE_ROUNDABOUT}_H_${IS_MAGNITUDE_HIGHWAY}_P_${AVOID_COLLISION_IS_PROB}_60s_change_unavoidable_part_fix_highway_v5"
mkdir -p ${DIR_NAME}/${experiment_name}
mkdir -p ${DIR_NAME}/${experiment_name}/raw_data
mkdir -p ${DIR_NAME}/${experiment_name}/aggregated_data

del_mode="all" # all verbose off

for i in {1..1000}; do
    # write i to aggregated_data/SLURM_ARRAY_TASK_ID.txt
    # echo $i >> ${DIR_NAME}/${experiment_name}/aggregated_data/${SLURM_ARRAY_TASK_ID}.txt
    exp_nth=${SLURM_ARRAY_TASK_ID}_${i}
    mkdir -p ${DIR_NAME}/${experiment_name}/raw_data/${exp_nth}
    # test record
    python safetest_mcity_main.py --dir ${DIR_NAME} --name ${experiment_name} --nth ${exp_nth} --aggregateddir ${DIR_NAME}/${experiment_name}/aggregated_data/${SLURM_ARRAY_TASK_ID} > ${DIR_NAME}/${experiment_name}/raw_data/${exp_nth}/res.txt
    # python merge_json.py --dir ${DIR_NAME} --name ${experiment_name} --nth ${exp_nth} || true
    # remove if no collision happens (no victim) or the victim is in the wrong junction
    if [ "${del_mode}" = "off" ]; then # no deletion
        continue
    fi

    # else, check if collision happens and delete related information
    # if [ $(grep "victim" ${DIR_NAME}/${experiment_name}/raw_data/${exp_nth}/collision.xml -m 1 | wc -l) -eq 0 ] || [ $(grep "nd_34_1_6" ${DIR_NAME}/${experiment_name}/raw_data/${exp_nth}/collision.xml -m 1 | wc -l) -gt 0 ]; then
    if [ $(grep "victim" ${DIR_NAME}/${experiment_name}/raw_data/${exp_nth}/collision.xml -m 1 | wc -l) -eq 0 ] ; then
        echo "Removing ${DIR_NAME}/${experiment_name}/raw_data/${exp_nth}"
        if [ "${del_mode}" = "verbose" ]; then
            rm -f ${DIR_NAME}/${experiment_name}/raw_data/${exp_nth}/fcd_all.xml
        else
            rm -rf ${DIR_NAME}/${experiment_name}/raw_data/${exp_nth}
        fi
    fi
done