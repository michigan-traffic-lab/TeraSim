# run safetest_mcity_main_AV.py indefinitely

conda activate mcity_nade
for i in {1..1000}
do
    python safetest_mcity_main_AV.py
done
