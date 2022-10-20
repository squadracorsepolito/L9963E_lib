# L9963E_lib

This project implements the low level communication between a STM32 microcontroller and the ST L9963E Automotive Multicell battery monitoring and balancing IC.

For the moment only register interaction is implemented (burst commands will be implemented if it will be needed)

The ```registersgen``` folder contains a python script which generates the ```registers.h``` from the IC datasheet. Just run it and the ```registers.h``` file will be generated in the local folder. It depends on the [tabula-py](https://github.com/chezou/tabula-py/) module for the extraction of the registers' informations from the pdf.