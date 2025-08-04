from setuptools import setup, find_packages

setup(
    name="terasim_cosim",
    version="2025.06.11",
    description="TeraSim redis co-simulation with other platforms",
    author="Zhijie Qiao, Haowei Sun, Haojie Zhu, Yihao Sun",
    author_email="zhijieq@umich.edu",
    license="MIT",
    packages=find_packages(
        where=".",
        include=[
            "terasim_cosim",
            "terasim_cosim.redis_client_wrapper",
            "terasim_cosim.redis_msgs",
        ],
    ),
    install_requires=[
        "numpy==1.26.4",
        "pydantic==2.6.3",
        "redis==5.0.2",
        "utm==0.8.0",
        "loguru==0.7.2",
    ],
)