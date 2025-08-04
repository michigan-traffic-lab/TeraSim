python build.py # generate .so file
pyarmor gen -O dist_pyarmor -e 30 -r -i ./terasim_nde_nade
cp pyproject.toml dist_pyarmor/
cp update_version.py dist_pyarmor/
cd dist_pyarmor
python update_version.py
poetry build -f wheel
poetry publish -r sdpi