# [UAV Routing for Enhancing the Performance of a Classifier-in-the-loop](https://arxiv.org/abs/2310.08828)

Tested on Windows 11 with Python 3.12, Julia 1.10, and Gurobi 11.0.2

## Installation
* Install [Python](https://www.python.org/downloads/) and [Julia](https://julialang.org/downloads/)
* Obtain Gurobi Liscence, install [Gurobi](https://www.gurobi.com/)
* Clone this repository 
    ```bash
    git clone https://github.com/pranavraj575/UAV_routing_classification
    ```
* Install Python dependencies (run command in cloned folder)
    ```bash
    pip3 install -e .
    ```
* Add julia packages
    ```bash
    julia setup.jl
    ```
## Run experiments
* collect experiment data with 
    ```bash 
    julia run_experiments.jl
    ```

    this will run our algorithm on tsp files from a specified folder

    change the parameters in the file to mess with the input files, alpha/tau values, and the neighborhood paremters used
* collect memetic experiment data with 
    ```bash 
    julia memetic_eval.jl
    ```
    this will use the memetic routes and run gradient descent to obtain memetic values
* plot comparisions between our data and memetic data with `python memetic_plot.py`


