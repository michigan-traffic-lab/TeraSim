from setuptools import setup, find_packages

setup(
    name="terasim_cosim",
    version="2.0",
    description="TeraSim co-simulation with other simulators",
    author="Zhijie Qiao, Haowei Sun, Haojie Zhu",
    author_email="zhijieq@umich.edu, haoweis@umich.edu, zhuhj@umich.edu",
    license="MIT",
    packages=find_packages(
        where=".",
        include=[
            "terasim_cosim",
            "terasim_cosim.redis_client_wrapper",
            "terasim_cosim.redis_msgs",
            "terasim_cosim.terasim_plugin",
            "terasim_cosim.mcityos",
        ],
    ),
    install_requires=[
        "pygame==2.6.1",
        "carla==0.9.15",
        "opencv-python==4.10.0.84",
        "typing_extensions==4.12.2",
        "numpy==1.26.4",
        "pydantic==2.6.3",
        "redis==5.0.2",
        "utm==0.8.0",
        "loguru==0.7.2",
        "boto3==1.38.5"
    ],
    python_requires=">=3.10",
)

