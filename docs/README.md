# Robotics Project 

---

This guide provides step-by-step instructions to create and configure a Conda environment with the required libraries for the project.

---

## Prerequisites

Ensure you have the following installed on your system:
- [Conda](https://docs.conda.io/en/latest/miniconda.html) (Anaconda or Miniconda)
- Python 3.8 or higher

---

## Steps to Set Up the Environment

### 1. Create a New Conda Environment
Run the following command in your terminal to create a new environment with Python 3.8:
```bash
conda create --name robotics_env python=3.8 -y
```
Replace `robotics_env` with your preferred environment name.

### 2. Activate the Environment
Activate the newly created environment:
```bash
conda activate robotics_env
```

### 3. Install Required Libraries
Install the required libraries using the following commands:

#### Install Libraries via Conda
Use the `conda-forge` channel to install Flask, NumPy, and Requests:
```bash
conda install -c conda-forge flask numpy requests
```

#### Install Libraries via Pip
Install Robotics Toolbox for Python and SpatialMath:
```bash
pip install roboticstoolbox-python spatialmath-python
```

---

## Verifying the Installation
To verify that all required libraries are installed and working, run the following command:
```bash
python -c "import flask, numpy, requests, roboticstoolbox, spatialmath; print('All libraries are installed successfully')"
```

If no errors appear, the setup is complete!

---

## Notes
- Always activate the environment before running your project:
  ```bash
  conda activate robotics_env
  ```
- If you encounter issues, ensure you are using the correct Python environment or reinstall the missing libraries.

---

## Additional Commands
- **List all Conda environments**:
  ```bash
  conda env list
  ```
- **Remove the environment**:
  ```bash
  conda remove --name robotics_env --all
  ```
