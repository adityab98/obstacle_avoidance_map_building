The relevant files are at controllers/create_avoid_obstacles with
create_avoid_obstacles.c being the most important.

# Abstract

We modify the working of iRobot’s Create such that the robot 
creates an internal map of the environment, using it for navigation. 
This internal map would aid the robot in its cleaning and pathfinding 
and would make it more efficient in doing its tasks, as it can 
simply avoid points on the map that contain obstacles. The mechanism 
for building the map is also very simple. While doing its 
task, i.e, cleaning the house, the robot would randomly collide 
with various obstacles. When it does collide with an obstacle, 
it adds the obstacle along with the point on the map where the 
collision took place. While moving around its environment,
it can now avoid the obstacle marked in its map.
In this way, we have made iRobot’s create more efficient 
in its task of cleaning its environment.
