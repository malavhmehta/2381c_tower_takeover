# Tower Takeover 2019-2020

This project was created as part of the the VEX VRC Tower Takeover Competition in the 2019-2020 competition season. Mainly, the programming aspect of the competition consists of coding:
 - driver control (including macros for driving); and
 - a reliable drive pathway for the autonomous sequence.

The purpose of this project is to achieve the above criteria for the code successfully, which means to ensure reliability, simplicity and ensure that everything works well during the competition.

This repository belongs to team 2381C from Colonel By Secondary School (Ottawa, Canada).

#### Table of Contents  
[Driver Control](#driver-control)  
[Contributing](#contributing)  
<a name="headers"/>

<p align="center">
	<a href="http://www.youtube.com/watch?feature=player_embedded&v=MGRXpUMptEE" target="_blank">
		<img src="http://img.youtube.com/vi/MGRXpUMptEE/0.jpg" 
		alt="IMAGE ALT TEXT HERE" width="600" height="450" border="10" />
	</a>
</p>

## Driver Control

The driving directions are called from within the `opcontrol()` function in the [main.cpp](https://github.com/malav-mehta/Tower-Takeover-2019-2020/blob/master/src/main.cpp) file.

### Basic Driving Directions
The drivebase of the robot was made using an H-drive structure. From the code, the basic driving directions are called as shown below:

- **Drive Forward**
	```cpp
	leftBack.move(+);
	leftFront.move(-);
	rightBack.move(-);
	rightFront.move(+);
	```
- **Drive Backwards**
	```cpp
	leftBack.move(-);
	leftFront.move(+);
	rightBack.move(+);
	rightFront.move(-);
	```
- **Turn Left**
	```cpp
	leftBack.move(-);
	leftFront.move(+);
	rightBack.move(-);
	rightFront.move(+);
	```
- **Turn Right**
	```cpp
	leftBack.move(+);
	leftFront.move(-);
	rightBack.move(+);
	rightFront.move(-);
	```

### Goofy Tower Macros

The macros created for the goofy arm work as shown below. There are 3 buttons for the Goofy Tower Macros.
- `left_digital` triggers the *small* tower macro.
- `right_digital` triggers the *medium* tower macro.
- `down_digital` resets the transmission and the goofy arm to their neutral positions.

The _basic program structure_ for the macros is as follows:

**If statements** are used for checking when and which buttons are pressed. Each if statement has its own conditions based off a variable called `control`, when each button is pressed, the value of `control` will be updated, and will determine which macro need be called.

The **encoder values** of the goofy arm were recorded at the specific heights designated for the small medium towers. The goofy arm is then moved until it reaches the prescribed encoder value.

>Due to the extreme weight of the goofy arm, the motors need to be running at a speed of 5rpm in order to keep  it still. Built-in functions, such as `brake_mode` are unable to balance the weight of the arm, so an opposing force is required.

## Contributing

To contribute to this repository you should obviously be in 2381C. Navigate to the _Projects_ tab from the menu at the top of the page. Under the projects page, select the project entitled _Organizational Tasks_. Under this project, you will see a list of todo items. Pick an item, move it to the *In Progress* column and start working.

### Creating a branch

To keep the repository clean, do the following when you start contributing:

 1. If you have not already, ensure that you have cloned the repository and navigate to that folder from your terminal program.
 2. Create a new branch for your work.
	 ```git
	 $ git checkout -b "<descriptive branch name>" master
	 ```

Once this is done, you can start working.

### Pull Requests and Merging

Once you've finished what you were working on, **DO NOT** directly commit and push your edits to the `master` branch immediately. First, your code must go under review. To start this process, you must make a pull request before your code is merged.

### Project Management and Issues

If there are issues, then you must add it accordingly, describing the issue and refer to its location in the code.
