# TORCS_Simulink_Client

[![View TORCS Simulink Client on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://www.mathworks.com/matlabcentral/fileexchange/73744-torcs-simulink-client)

This repository contains an S-Function for Simulink featuring the client-server championship code for TORCS [1]. This is implemented to faciliate the developement of SIL control algorithms for drivers in TORCS.

This code is implemented on MATLAB R2018a for Windows and compiled via Microsoft Visual Studio 2017 Community Edition.

To compile the S-function, write the following command within MATLAB:
```
mex -g -R2018a -I.\ TORCSClient_S.cpp CarControl.cpp CarState.cpp SimpleDriver.cpp SimpleParser.cpp WrapperBaseDriver.cpp ws2_32.lib
```

The file TestModel.slx contains a simple demo for cruise control and lane keeping system. The controllers are just for testing basic functionalities of the S-function.

[1] Loiacono, Daniele, Luigi Cardamone, and Pier Luca Lanzi. "Simulated car racing championship: Competition software manual." arXiv preprint arXiv:1304.1672 (2013).
