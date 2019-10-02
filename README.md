# ROS_Service_Robot

[Video](https://youtu.be/vOX4fplDB2w) of Turtlebot 2.0 picking up a package and then dropping it off at pre-defined locations.

## IMPORTANT
I don't know if it is negligence on the part of Udacity, or if they are trying to make us all better coders.  Either way, there is a catch in the Home Service Robot project in the Udacity Robotics Software Engineer NanoDegree.

If you are using the Udacity Workspace, it defaults with Python 3.  There are no ROS versions that work with Python 3.  ROS only works with Python 2.  Soooo, you gotta fix that before you can do this project.

## What I Did!
### Conda
You are going to need to use Anaconda and create a virtual environment that runs Python 2.  After that, every time you bring up a terminal you need to switch it over to the Python 2 virtual environment.

You are going to want to check and see if miniconda is actually installed in the Udacity Workspace.  If it is, then you can go ahead and run conda commands.

If you don't already have conda or miniconda in your Udacity Workspace, you can install Anaconda.  To install Anaconda packages:

**conda upgrade conda**
  
**conda upgrade --all**

### The .student_bashrc File
Once you get into the Udacity Workspace environment and pull up the Desktop, you will have an icon for invoking a terminal.  It's called "Terminator".  Every time you click on "Terminator" (at least on my system), the OS jumps to the .student_bashrc file.  In my system, that file did not exist, so I created one to help me with my bring-up efforts.

Go to /home/workspace and check if .student_bashrc exists.

NOTE:
You can check for the .student_bashrc file by entering:

**ls -ls**

If, like me, it does not exist, you can create one with gedit.

**cd /home/workspace/**

**gedit .student_bashrc**

Looking at the .student_bashrc file, you can see that it runs another file called **move2py.sh**.  This file runs:

**conda init bash**

And once that is run, you have to shut down the terminal that ran it before the init process is complete.

Before any of these .sh files will run, you have to set them up as executable.  You do this by entering:

**chmod +x .student_bashrc**

**chmod +x move2py.sh**

I tried to get the script to do everything, but for some reason I can't seem to trick the script into properly running the "conda activate /opt/robond/py2" command (back in the .student_bashrc file).  So, everytime I opened a terminal, the first thing I would need to type in is:

**conda activate /opt/robond/py2**

And this would put the terminal into the proper Python 2 virtual environment.  You will know this because at the front of the cursor line you will now see this added:

(/opt/robond/py2)

### Get ROSCORE Running
O.K.  So now we are in a terminal that has a Python 2 virtual environment.

Now we need to get roscore up and running.  So enter

**source /opt/ros/kinetic/setup.bash; roscore**

This will get you set up just fine.  And you can terminate this window now if you want to get it out of the way (or just shrink it to below).  

NOTE: Every time the Udacity Workspace kills your session (because you walked away for too long), you have to go through this entire process again before you can get back to work.

### Rock & Roll
So, to run the Home Service Robot run these commands:

**cd /home/workspace/catkin_ws/**

**source devel/setup.bash**

**./src/scripts/home_service.sh**

The home_service.sh script does five things.  Here you can see the terminals that handle the localization, mapping, and navigation.

1. Opens up the Gazebo simulator and loads it with the turtlebot robot and turtlebot world.

2. Fires up the amcl code to be run with Gazebo (enables us to locate around without running into things)

3. Then we crank up the RViz with our pre-built navigation content (enables us to see all of the data in action)

4. We start the Markers code (puts a marker at pick-up location, and will place a marker at drop-off location once the robot gets there)

5. We send the robot the pick-up location, wait for the robot to pick up the cube, then send the robot the drop-off location


