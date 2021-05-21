# Extended Kalman Filter in C++


[//]: # (Image References)


[image1]: ./examples/Startposition.png "Startposition"
[image2]: ./examples/Time_Step_189.png
[image3]: ./examples/whole.png
[image4]: ./examples/Fusion_Flow_EKF.png


## 1. Introduction

In this project I implemented an EKF (Extended Kalman Filter) in C++. The task is to estimate the state of a moving object using LIDAR and RADAR data measurements in a UNITY programmed simulator.   
Lidar measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles. The object to track is a blue car moving in a figure eight around the sensor.   
Here are some images to see how the environment looks like:   

##### Car in start position with the sensor:  

![alt text][image1]

##### Car at a certain time step:  

![alt text][image2]

##### Overview at the end:  

![alt text][image3]


#### Tasks in this project

- Initialize variables and matrices (x, F, H_laser, H_jacobian, P, etc.)  
- Initialize the Kalman filter position vector with the first sensor measurements   
- Modify the F and Q matrices prior to the prediction step based on the elapsed time between measurements   
- Call the update step for either the lidar or radar sensor measurement

#### Resources for the project

The github repository includes the code in the src folder:
- `main.cpp` - communicates with the Term 2 Simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE
- `FusionEKF.cpp` - initializes the filter, calls the predict function, calls the update function
- `kalman_filter.cpp`- defines the predict function, the update function for lidar, and the update function for radar
- `tools.cpp`- function to calculate RMSE and the Jacobian matrix  

A Docs folder, which contains details about the structure of the code templates  
The `CMakeLists.txt` file that will be used compiling the code  
A data file for testing the extended Kalman filter which the simulator interface provides  

#### Explanation of the C++ files:

- `Main.cpp` reads in the data and sends a sensor measurement to `FusionEKF.cpp`  
- `FusionEKF.cpp` takes the sensor data and initializes variables and updates variables. `FusionEKF.cpp` has a variable called `ekf_`, which is an instance of a KalmanFilter class. `ekf_` will hold the matrix and vector values and is used to call the predict and update equations.
- The KalmanFilter class is defined in `kalman_filter.cpp` and `kalman_filter.h`, which contains functions for the prediction and update steps.
- `tools.ccp` holds the equation for calculating the Root Mean Squared Error (RMSE) between the location determined by the EKF and the ground truth

#### Dependencies

    cmake: 3.5
    make: 4.1
    gcc/g++: 5.4


## 2. Results & Discussion

The Kalman Filter consists of an endless loop of Prediction and Measurement Update steps, as shown in the image below:   

![alt text][image4]

The first step - at the beginning - is to initialize the states and covariance matrices.   

The prediciton step includes:
1. Compute the elapsed time deltaT
2. Use deltaT to compute the new EKF, F and Q matrices
3. Predict x and P   

For Measurement Update it is important to distinguish between
1. Laser Measurement: Position, no Velocity in cartesian coordinates
2. Radar Measurement: Postion and Velocity in polar coordinates, linear approximation necessary

Finally the state update, based on prediction and measurement.   

I used the RMSE (Root Mean Squared Error) to evaluate the EKF performance. The smaller the value of the RMSE the nearer is the estimated position to the true result (ground truth).

As a result the position can be estimated better by using both Radar and Lidar contrary to just using one of them. Radar seems to be more accurate for estimating the velocity and Lidar more accurate for estimating the position of the car.

It can be seen that Kalman Filters combine somewhat inaccurate sensor measurements with somewhat inaccurate predictions of motion to get a filtered location estimate, that is better than any estimate that comes from only sensor reading or only knowledge about movement.
