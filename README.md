# Adaptive Traffic Signal Optimization for Emergency Vehicle Routing

## CIS 505 Group 10 Term Project

## Setup

* Install git - see [here](https://git-scm.com/downloads/win) for Windows
* Clone the repo from GitHub:
  `git clone https://github.com/sockduct/ATSO4EVR.git`
* Install Python 3.12, 64 bit - see
  [here](https://www.python.org/ftp/python/3.12.10/python-3.12.10-amd64.exe) for
  Windows as of April 8, 2025
* Setup and activate a Python virtual environment:
  * Windows:

    ```PowerShell
    PS> py -3.12 -m venv .venv
    PS> .venv\scripts\activate
    ```

* Install required libraries from within Python virtual environment from the root
  directory of the repository (where the master requirements.txt package specifications is):
  `pip install -r requirements.txt`

* Run the program:
  `python sim1.py`

## Traffic Grids

* Simple 4x4 traffic grid layout used in sim1.py:

  ![Traffic Grid Layout](./Traffic%20Grid-4x4.png)

* Medium 6x6 traffic grid layout used in sim1.py:

  ![Traffic Grid Layout](./Traffic%20Grid-6x6.png)

* Large 8x8 traffic grid layout used in sim1.py:

  ![Traffic Grid Layout](./Traffic%20Grid-8x8.png)
