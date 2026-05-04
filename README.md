# [UAV Routing for Enhancing the Performance of a Classifier-in-the-loop](https://link.springer.com/article/10.1007/s10846-024-02169-1)

![](https://github.com/pranavraj575/UAV_routing_classification/blob/main/map_generation/tsp_real_world_examples.gif)

Full paper available [here](https://link.springer.com/article/10.1007/s10846-024-02169-1)

## Installation

Tested on Windows 11 with Python 3.12, Julia 1.10, and Gurobi 11.0.2

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


