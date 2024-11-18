from setuptools import setup

setup(
    name="terasim_cosim",
    version="2024.10.18",
    author="Zhijie Qiao, Haowei Sun, Haojie Zhu",
    author_email="zhijieq@umich.edu, haoweis@umich.edu, zhuhj@umich.edu",
    packages=[
        "terasim_cosim",
        "terasim_cosim.redis_client_wrapper",
        "terasim_cosim.redis_msgs",
        "terasim_cosim.terasim_plugin"
    ],
    scripts=[],
    url="https://github.com/michigan-traffic-lab/TeraSim-Cosim",
    license="MIT",
    description="SUMO co-simulation with other physics simulator",
    install_requires=["pygame==2.6.1", "opencv-python==4.10.0.84"],
)
