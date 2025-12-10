from setuptools import setup, find_packages
import os

# Read the contents of your README file (if you have one)
this_directory = os.path.abspath(os.path.dirname(__file__))
with open(os.path.join(this_directory, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

setup(
    name='vla_module',
    version='1.0.0',
    description='Vision-Language-Action (VLA) module for AI-native robotics',
    long_description=long_description if 'long_description' in locals() else '',
    long_description_content_type='text/markdown',
    author='AI-Native Robotics Team',
    author_email='info@example.com',
    url='https://github.com/your-org/ai-native-robotics',
    packages=find_packages(),
    install_requires=[
        'rclpy>=3.0.0',
        'openai>=1.0.0',
        'anthropic>=0.5.0',
        'opencv-python>=4.8.0',
        'torch>=2.0.0',
        'torchvision>=0.15.0',
        'pyyaml>=6.0',
        'python-dotenv>=1.0.0',
    ],
    tests_require=[
        'pytest>=7.2.0',
        'pytest-asyncio>=0.21.0',
        'pytest-cov>=4.0.0',
    ],
    entry_points={
        'console_scripts': [
            'vla-cli=vla_module.cli.vla_cli:main',
        ],
    },
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3.11',
    ],
    keywords='robotics, ai, vision, language, action, ros2',
    license='MIT',
    python_requires='>=3.11',
)