# README for PD-controller Lokomat project

The research project, for the couse ME41125, titled: "Modulating PD-controller parameters allowing for differences in trajectory tracking error: LOKOMAT rehabilitation system.", was performed using the code and models in this repository. It derives a range of PD-parameters that result in an adequate step response (within 10% offset or overshoot) for the knee and hip joint of the LOKOMAT rehabilitation system. It then derives the Root Mean Square Error (RMSE) in trajectory tracking for exemplary trajectories of knee flexion and hip abduction and flexion, for the previously obtaind PD-parameters.

## Getting Started

These instructions will give you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites
First of all, a Matlab environment (preferably R2022b) with the Simulink extension is necessary to run these files. For non-TU Delft students/employees, Matlab can be downloaded here: 
- [Dowload Matlab](https://nl.mathworks.com/store/?gclid=CjwKCAjwvdajBhBEEiwAeMh1U9jA01JvpdchiN7sCJfYlFx0Q6Hv_Id0KgkZlk-dMQNlQ1nteVlmqBoCn3kQAvD_BwE&ef_id=CjwKCAjwvdajBhBEEiwAeMh1U9jA01JvpdchiN7sCJfYlFx0Q6Hv_Id0KgkZlk-dMQNlQ1nteVlmqBoCn3kQAvD_BwE:G:s&s_kwcid=AL!8664!3!552213010978!p!!g!!get%20matlab&s_eid=ppc_69452703753&q=get%20matlab)
Be sure to also check the 'Simulink' extension during installation.

For TU Delft students/employees a free version of Matlab R2022b and Simulink extension can be downloaded here: 
- [Download Matlab TU Delft](https://software.tudelft.nl/402/)
An installation manual is also provided. Be sure to also check the 'Simulink' extension during installation. 

### Downloading files
In this repository you will find the Matlab and Simulink files necessary for obtaining the presented results in the paper. It is best to download all the files before starting with running, since some call upon others. The following files are presented with the following functions: 
- Lokomat_Simulation.slx: This file contains the simulink model of the PD-controller and the Lokomat system. It is called upon and loaded within the necessary Matlab files, but some settings will need to be altered in the model while going through the other files (More on that in a later section). The Lokomat_Simulation file contains: 
    - A SimScape model (previously SimMechanics) of the right leg of the LOKOMAT orthosis. The model supports the rotations of the hip joint through two prismatic actuators and the rotation of the knee joint through a rotary actuator.
    - A Desired Trajectory block defines the desired angles of the knee flexion/extension and hip flexion/extension and abduction/adduction. 
    - An Inverse Kinematic block provides equations to determine the traveling amount of prismatic actuators (internal and external), given the hip angles for flexion/extension and abduction/adduction.
    - A Controller block with a PD-controller code within.
- Step_Response.m: This file contains the Matlab code for obtaining the PD-controller parameters that result in an adequate step response. It calls upon the two matlab function files: Step_Hip.m and Step_Knee.m
- RMSE.m: This file contains the Matlab code for obtaining the RMSE in trajectory tracking for the previously determined range of PD-parameters with adequate step response. Be aware output is in radians. In the figure_creation file it is converted to degrees, which is the unit used in the report. 
- Figure_Creation.m: This matlab file contains the script necessary for obtaining the figures presented in the report. It calls on the data files created by Step_Response.mat and RMSE.mat. 

!! Running of the Step_Response.m and RMSE.m files will take a long time (multiple hours). In case time is limited, the data files resulting from Step_Response.m and RMSE.m are already provided in this repository !! 

The pre-made result files necessary to run Figure_Creation.m are all of the type 'MATLAB data', do not confuse them with the matlab fuction files with similar names: 
- RMSE_Hip.mat
- RMSE_Knee.mat
- Stab_Reg_Hip.mat
- Stab_Reg_Knee.mat

To prevent overwriting of these files in a faulty simulation on accident, no autosaving is implemented in the Step_Response.m and RMSE.m files. **If you wish to regenerate the data (by following the steps in the next section), you need to save the data produced in the .m files under the exact same name, to get the Figure_Creation.m running** The files that need to be saved and their naming is also indicated in within the Matlab code. 

## Running the tests

If one is interested in recreating the raw data obtained from simulations, the RMSE.m and Step_Response.m files can be used. For using different sections of these files, different settings in the Lokomat_Simulation.slx need to be altered. The changes in settings are all provided at the top of the Matlab files that call on the Lokomat_Simulation. Some general remarks are useful to understand these settings. 

- Multiple manual switches have been implemented to either switch between a exemplary trajectory or step response (SWITCH 1-3), or to switch between a constant as input for the LOKOMAT MODEL or the actual controller output (SWITCH 4-6). In every Matlab file it is indicated to what these switches need to be set. **Switching a manual switch is done by double clicking**. 
- Three CONVERTER blocks are present that convert a controller output signal to an understandable unit for the LOKOMAT MODEL block. The unit can be altered by typing in the 'Input signal unit' box. **Opening this setting is done by double clicking the CONVERTER block**
- Within the LOKOMAT MODEL block, there are three blocks (Prismatic left and right actuator and Revolute Joint Shank Bar) that need to be altered for different runs. **Altering the necessary settings is done by double clicking the block, and under 'Z Prismatic Primitive (Pz')' going to 'Actuation'**. There you can alter the Force and Motion to either be Automatically Computed or Provided by Input, depending on what is stated in the Matlab files. **Sometimes when doing this, lines disconnect (indicated by a red dashed line). To fix this, delete the red dashed line by clicking on it specifically and pressing the delete button. Then drag the now missing line from the disconnected input number to the open input spot in the block**.



## Built With

  - [Matlab R2022b](https://nl.mathworks.com/store/?gclid=CjwKCAjwvdajBhBEEiwAeMh1U9jA01JvpdchiN7sCJfYlFx0Q6Hv_Id0KgkZlk-dMQNlQ1nteVlmqBoCn3kQAvD_BwE&ef_id=CjwKCAjwvdajBhBEEiwAeMh1U9jA01JvpdchiN7sCJfYlFx0Q6Hv_Id0KgkZlk-dMQNlQ1nteVlmqBoCn3kQAvD_BwE:G:s&s_kwcid=AL!8664!3!552213010978!p!!g!!get%20matlab&s_eid=ppc_69452703753&q=get%20matlab) - Used to write all code files and create the Simulink model.
  - [Creative Commons](https://creativecommons.org/) - Used to choose the correct license.


## Authors
- **Mostafa Mogharabi Manzari** - Created the Simulink model of the LOKOMAT system, the Desired trajectory block and the inverse kinematics block
- **Karien ter Welle** - Wrote all Matlab files, created the PD-Controller block in the Simulink model and wrote the README files. Made small alterations to the Simulink model to allow for interaction with the Matlab files.


## License

This project is licensed under the [CC-BY](LICENSE.md) Creative Commons License - see the [LICENSE.md](LICENSE.md) file for
details

## Acknowledgments

 The author would like to thank Stefano Dalla Gasperina for helping with complications in the Simulink model and writing the Matlab code.