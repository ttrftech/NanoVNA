# Control NanoVNA from python script

NanoVNA is able to be controlled via USB serial interface from PC. There are sample scripts in this directory.

## Preparation

    $ cd python
    $ pip install numpy
    $ pip install scikit-rf   

## Run

### Plot reflection LOGMAG.

    $ ./nanovna.py -p

### Plot transmission LOGMAG.

    $ ./nanovna.py -p -P 1

### Plot smithchart.

    $ ./nanovna.py -s

### Show usage.

    $ ./nanovna.py -h

## Using in Jupyter Notebook

To use NanoVNA from Jupyter notebook, see [this page](/python/NanoVNA-example.ipynb).
