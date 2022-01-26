![banner](https://user-images.githubusercontent.com/43790840/151192386-c24d9120-548b-497f-9bc4-48ada55ca864.png)

# Learning by Imitation FS

This program controls the actuators of a formula student driverless racing car with a machine learning technique named **learning by imitation**. 

Firstly, you have to drive the car, preferably with a racing wheel, in order to take data of your driving. With enought collected data, next step is to train a machine learning algorithm. The testing takes place in the **Webots** simulator. 


For further information, please check the documentation of the degree final project (awaiting publication). 


# Set-Up
The set-up used to run the executions will be shown below.
- Windows 10
- [Webots R2021b](https://github.com/cyberbotics/webots/releases/tag/R2021b)
- [Weka](https://waikato.github.io/weka-wiki/downloading_weka/#stable-version)
- Python3

# How to run
1. Replicate the set-up
2. Clone this repository
3. Open ``` Webots >> worlds >> city_mad.wbt ```
4. Open with the ```Text Editor``` ``` Webots >> controllers >> autonomous_vehicle >> autonomous_vehicle.c ``` 
5. Press ```Build the current project```
6. Press ```Run the simulation in real-time```

# Features
- Different circuits and a creation tool
  - Oval
  - Skidpad
  - FSG 
- Racing Wheel PC implementation to take data
- Basic example running and the data

# Example
The car is able to complete a lap in the oval circuit with the MLP algorithm, you can easily improve the results by using different machine learning algorithms. 

https://user-images.githubusercontent.com/43790840/151197768-73d32bcf-de7f-4173-98b5-7e98f6323e01.mp4




# Credits

This project was developed by Alejandro Parrado Arribas as part of his work at [MAD Formula Team](https://www.madformulateam.com/).

Note: This public version might have some features removed 
![MADFT_P_Main_crop](https://user-images.githubusercontent.com/43790840/151197531-a73f05a1-e954-483b-9b58-474350db0ef6.png)
