from setuptools import setup, find_packages
import os
from glob import glob
from collections import defaultdict

package_name = 'waiter_speech_to_text'

# In your repo, the model lives here:
# waiter_speech_to_text/share/models/vosk-model-small-en-us-0.15/...
MODEL_SRC_ROOT = os.path.join('share', 'models', 'vosk-model-small-en-us-0.15')


def collect_tree_data_files(src_root: str, dst_root: str):
    """
    Collect all files under src_root and map them into data_files tuples
    rooted at dst_root in the install space.

    Example:
      src_root = 'share/models/vosk-model-small-en-us-0.15'
      dst_root = 'share/<pkg>/models/vosk-model-small-en-us-0.15'
    """
    if not os.path.isdir(src_root):
        raise FileNotFoundError(
            f"[{package_name}] Model directory not found at: {src_root}\n"
            f"Expected relative to package root: {src_root}\n"
            f"Fix: ensure the model exists at:\n"
            f"  ~/robot_waiter_ws/src/{package_name}/{src_root}"
        )

    grouped = defaultdict(list)

    for root, _, files in os.walk(src_root):
        for fname in files:
            src_path = os.path.join(root, fname)

            # relative path under src_root
            rel_dir = os.path.relpath(root, src_root)
            if rel_dir == '.':
                rel_dir = ''

            # install destination dir
            dst_dir = os.path.join(dst_root, rel_dir)
            grouped[dst_dir].append(src_path)

    return list(grouped.items())


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),

    # ✅ install launch files (this fixes ros2 launch "file not found")
    (os.path.join('share', package_name, 'launch'),
     glob('launch/*.launch.py')),
]

# ✅ Install the Vosk model under:
# install/share/<package_name>/models/vosk-model-small-en-us-0.15/...
data_files.extend(
    collect_tree_data_files(
        src_root=MODEL_SRC_ROOT,
        dst_root=os.path.join('share', package_name, 'models', 'vosk-model-small-en-us-0.15')
    )
)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='senithu',
    maintainer_email='senexdamx@gmail.com',
    description='Offline speech-to-text (Vosk) gated by wake word.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_to_text_node = waiter_speech_to_text.speech_to_text_node:main',
        ],
    },
)
