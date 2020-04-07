# Compulsory tasks

The launch procedure it's the quite the same for the compulsory part and the bonus part. I assume that the folders structure of the system you are testing the homework on is the same as the one where I worked on.\

For the compulsory:
```
roslaunch thymio_course_skeleton thymio_gazebo_bringup.launch name:=thymio10 world:=wall
```
Then you can adjust the wall angle w.r.t the MyT by hand. You can now run the following command in onother terminal and see the results of task3:
```
roslaunch homework2 compulsory.launch 

```
For the bonus:

```
roslaunch thymio_course_skeleton thymio_gazebo_bringup.launch name:=thymio10 world:=arena
```
Then open another terminal and run:
```
roslaunch homework2 bonus.launch 

```
This is gonna show the task number 4 even though I did the task 5 as well. I'm gonna provide the launh file for task 5 as well but, as I posted on the forum, my mac freezes when I try to run Gazebo with more than one Thymio. Just in case I list the command to run task 5 below.\
Again start Gazebo with:
```
roslaunch thymio_course_skeleton thymio_gazebo_bringup.launch name:=thymio10 world:=arena
```
Then try this launch file:
```
roslaunch homework2 bonus_unlucky_task5.launch 
```
Of course I cannot post any video of task 5.

NOTE: sometimes Gazebo launches a warning about time and the sensors start to behave randomly, so in thas case please restart Gazebo.

[Link to Videos and screenshots cited in this file and requested](https://drive.google.com/open?id=1ua5XhY3mhEp6ykb1VsVIZni8zdI6YpWO)

## Task1

### Implemented: yes

### Problems/Notes: 
None

## Task2

### Implemented: yes

### Problems/Notes: 

Initially in the wall detecting part I wasn't considering any distance threshold, the MyT stopped as soon as it detected the obstacle. So, in some cases during the rotation, the center_left sensor and the center_right sensor didn't perceive the obstacle (even though the center sensor was perceiving the obstacle). Since my approach is to minimize the difference between the center_left sensor and the center_right sensor, in those cases the difference was minimun but the MyT wasn't facing properly the wall.\
 In order to fix this problem I introduced a distance threshold (and a bit of parameters tuning). In this way the MyT got a little bit closer to the wall, perceiving the ostacle with center, center_left and center_right sensors.

## Task3

### Implemented: yes

### Problems/Notes: 

Since this controller is a modify version of controller 2, I faced the same issues basically.\
To measure the 2m distance I used two 1m edges cubes provided by Gazebo (stupid but effective). 

# Bonus tasks
## Task1 bonus

### Implemented: yes

### Problems/Notes: 

This controller is a modify version of controller3. In particular, in task 3 the MyT got perpendicular to the wall before restarting and reaching the point 2m away from the wall. In this task the MyT starts moving before getting perpendicular to the obstacle  (as soon as it detects the wall with one of the rear sensors). To make sure that the bot doens't stuck, I run a simulation for more than 1 hour in the arena scenario.

## Task2 bonus

### Implemented: yes

### Problems/Notes: 

As i said in the introduction, my simulation got stuck every time a try this task. In any case I know that there's a problem in some cases. Here:
```
while (self.sensors_values['rear_right'] >= self.max_sensor_distance and
                    self.sensors_values['rear_left'] >= self.max_sensor_distance):
        self.velocity_publisher.publish(velocity)
        self.rate.sleep()

```
Can happen that one Thymio stops too far from a obstacle even though it perceives the obastacle (included other MyT). Then the bot starts to spin. In this case the condition of the while above never becomes false, so the bot spins endessly. In task 4 this problem is very unlikely to happen, in task 5 I cannot try.