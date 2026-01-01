from setuptools import setup

package_name = "waiter_nlu_parser"

setup(
    name=package_name,
    version="0.0.1",
    packages=[
        package_name,
        f"{package_name}.prompts",
        f"{package_name}.schemas",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/nlu_parser.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robot_waiter",
    maintainer_email="",
    description="NLU + LLM parser converting STT transcripts into validated order JSON",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "nlu_node = waiter_nlu_parser.nlu_node:main",
        ],
    },
)
