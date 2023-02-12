# L9963E_lib

This project implements the low level communication between any microcontroller and the [ST L9963E Automotive Multicell battery monitoring and balancing IC](https://www.st.com/en/automotive-analog-and-power/l9963e.html).

Both *register level* access and *burst commands* are implemented.

*L9963E_DRV.** contains the low level interface with the IC while *L9963E.** contains the logic level interface.

The ```registersgen``` folder contains a python script which generates the ```registers.h``` from the IC datasheet. Just run it and the ```registers.h``` file will be generated in the local folder. It depends on the [tabula-py](https://github.com/chezou/tabula-py/) module for the extraction of the registers' informations from the pdf.