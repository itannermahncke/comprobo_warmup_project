# CompRobo Warmup Project
By Ivy Mahncke and Jiayi Shao

## Behavior #1: Teleoperated

### Description and process

The goal of the teleoperated node is to allow the user to pilot the Neato using keyboard input. To approach this challenge, I started with the base code provided for getting key inputs:
```
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
```

I left this function essentially unchanged in my node and called it to collect key input each time my timer went off. Once the key input was collected, I used a rudimentary match/case to associate various key inputs with linear and angular velocities to be sent to the Neato in a Twist message:
```
match current_key:
    case "i":
        twist_msg.linear.x = self.lin
    case "k":
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
    case ",":
        twist_msg.linear.x = self.lin * -1
# and so on...
```

In this system, keys `j` and `l` turn the Neato without moving it forward. Keys `i` and `,` command the Neato to move forward or backward without turning. The `k` key stops all motion. Finally, the corner keys `u, o, m,` and `.` all combine linear and angular motion in some way (e.g. driving forward while turning left).

Because my timer never resets the Neato's motion with an empty Twist message, the latest key pressed always determines the current behavior. As such, the only way to stop the Neato from driving at all is by using the `k` key.

### Challenges and reflection

If I spent more time on the teleoperated node, I would have put some thought into making a more intuitive keyboard. The corner keys, for example, are not particularly intuitive to use -- they were more for me to develop a sense of what turning while driving would look like on the Neato. Beyond improving the teleop node itself, I would have liked to include a way to swap to teleoperated mode in my other nodes (probably with a finite state controller?). It could be grouped with the emergency stop node as a safety feature, and probably would have allowed me to save my Neato from wall collisions more often!

## Behavior #2: Drive in a Square

### Decription and process

For this challenge, the drive_square node commands the Neato to drive in 1m by 1m square. I tailored my approach to this challenge based on the learning I wanted to get out of it -- in this case, I wanted to learn how to use internal sensor readings to adjust the robot's behaviors, which is a classic robot feedback loop.

In this challenge, the robot is always in one of two behaviors -- driving forwards or turning. Whenever the robot has achieved some goal, it switches to the alternate behavior and acts; this loop continues until the square is completed. It's easy to think of the robot as having four goals -- one for each square corner -- but it really has eight: four for reaching a corner, and four for orienting itself to face the next corner.

To implement this idea of behavior alternation, I first designed a sequence of position and orientation goals for the robot to meet:
```
self.state_goals = [
    # x, y, theta (zeroed at startup)
    [1.0, 0.0, 0.0],
    [1.0, 0.0, pi / 2],
    [1.0, 1.0, pi / 2],
    [1.0, 1.0, pi],
    [0.0, 1.0, pi],
    [0.0, 1.0, -1 * pi / 2],
    [0.0, 0.0, -1 * pi / 2],
    [0.0, 0.0, 0.0],
]
```

As the robot works towards its current goal, the drive_square node continually provides it motion commands while simultaneously receiving odometry information that indicates the robot's estimated position and heading. When all three of the robot's odometry coordinates (x, y, and heading) match its current goal within a margin of acceptable error, the robot moves on to its next goal (and switches to its alternate movement state). When the robot finally finishes all of its goals, it receives the command to stop moving -- this prevents it from staying stuck on its latest command, and spinning in a circle forever!

While working on this node, I found it useful to create a publisher that continually published the robot's latest odometry information alongside its current goal in a human-readable format. This helped me understand faulty behavior -- in particular, when the robot didn't recognize that a goal had been achieved. This helped me to know when I should adjust my error tolerance, and when the robot simply wasn't getting good odometry information.

### Challenges and reflection

The greatest challenge of this node, in my opinion, was selecting an error tolerance that kept the robot as close as possible to its actual goals without defining too narrow a range of acceptable positions. When I first broke down this problem, I thought that if the first two goals were met (the first meter of driving, and the first left turn), the subsequent ones would be fine, because it was all the same behavior. However, I noticed that each subsequent goal required a larger and larger error tolerance for the robot to recognize that it has met it.

My current theory on this is that as the robot progressed through the square, its estimated positions deviated further and further from reality; thus, it required a greater and greater error tolerance to recognize that its actual position was somewhere acceptable. If I had more time to work on this node, I would have developed code to tune this error up with each state goal. This feels like a better system than just having a large static error tolerance, which allows for imprecision when precision could have been achieved.

## Behavior #3: Wall Following

### Description and process

In this challenge, the wall_follower node instructs the Neato to approach a wall in front of it and then drive parallel to that wall.

I broke my wall follower into two states -- approaching a wall until the Neato reached a certain distance, and then driving parallel to the wall in a stable way. I used porportional control for the first state, which worked well: the Neato drove quickly, but slowed down as it approached the wall, and when it finally came to a stop it began to turn.

I found the second state much more difficult conceptually, and even now I feel I don't have a very good grasp on the trigonometry of the problem. My goal was for the robot to turn until the range scan from the right side of the robot reached a certain distance from the wall, which would indicate that the robot was parallel. However, I had a hard time discerning how to know what this value should be based on other known information. The code I ended up with worked fairly well, but the Neato had a tendency to begin driving away from the wall after enough time had passed.

I did, however, use the wall follower challenge as practice using rviz and Marker messages as a debugging tool. This skill was a great help because it allowed me to identify some likely areas in the code that could be culprits for my messy behavior, and in future challenges rviz helped me debug my code even more! I feel that this was probably my biggest takeaway from wall follower -- create models of the world, and use them to understand what your robot is "seeing".

### Visualization

![Wall_Follower_Rviz](https://github.com/user-attachments/assets/5276855b-540a-4797-89ca-231ba17aebb3)

Shown here is an Rviz gif of what the Neato "sees". The flickering blue dots represent the two ranges that the Neato uses to approximate its angle relative to the wall. As you can see, the dots are far beyond where they should be, which implies that something is faulty with the Neato's ability to truly identify those points.

### Challenges and reflection

My wall follower system is very minimal -- it has a lot of environmental constaints, such as only detecting walls on the right, and needing to start roughly pointed at a wall. If I had more time, I would develop code to handle these other cases such that the system was more robust overall. I also feel that I never had a very good understanding of the trigonometry behind the problem, so I would spend more time with the mathematical concepts to ensure I really understood what was going on.

I feel that the wall follower would have been a good opportunity to learn how to use the tf module, so that would have been my strecth goal given more time. I would have liked to understand how to define reference frames in ROS and transition points more comfortably between them.

## Behavior #4: Person Following

### Description and process

In this challenge, the person_follower node instructs the Neato to identify and follow a person around. I saw two central pieces to this challenge: Collecting LaserScan data to make an estimation of where the person is, and then using a control mechanism with that estimation to determine motion commands. I will discuss the process for each step now.

Learning how to collect and filter the LaserScan data was a big challenge for me because I did this node before I did wall_follower, which I found later to be a gentler introduction into LIDAR data. After each scan message, I filtered out all values of r that were either 0.0 or infinity (in short, bad scans). I also filtered out good scans that I simply wasn't interested in: r values greater than 1m, and scans taken at angles outside of an artificial "field of view". This additional filtering helped the Neato stay focused on its person, rather than being distracted by walls and other distant objects. I packed all of this filtering into a small helper function, like so:
```
def determine_range_quality(self, r):
    """
    Return True if range is a usable number; false if not.
    """
    if r != 0.0 and not math.isinf(r) and r < 1.5:
        return True
    return False
``` 

After all of this filtering, my robot was left with a set of "reasonable guess" points that represented places that the person was at or near. I found it helpful to mark these points in Rviz, so I could see what data the robot was choosing to keep rather than filter out. Once the Neato had these points, my node took an average across all of them to make a final estimate for where the person likely was. I included this in my visualization as well, in a different color, so I could compare it to the original set of points.

One of my goals for the person follower was to implement PID control, and I did so at this step. PID control has three components: the porportional component simply multiplies the error by some constant; the integral component grows or shrinks depending on how long it has taken to resolve the error; and the derivative component seeks to minimize how the error changes with time. The integral component leads to quicker error minimizing, but tends to overshoot if lots of time has passed. Meanwhile, the derivative component can dampen this effect, but can also get stuck if the error is not changing but is still not at the goal.

Here is a sample of my code, which implements PID control:
```
# find error btwn set point and process variable
self.current_error = abs(self.set_point - self.process_variable[0])

# determine P, I, D values
p_val = self.current_error
i_val = self.integral + self.current_error * self.timestep
d_val = (self.current_error - self.previous_error) / self.timestep

# update old values
self.integral = i_val
self.previous_error = self.current_error

# determine output
output = float(
    self.k_vals[0] * p_val + self.k_vals[1] * i_val + self.k_vals[2] * d_val
)
return output
```

As I understand it, people can spend years and PHD programs on learning to tune the constants of these components. I had less time, so I chose some basic values to play with and chose the ones that worked best. I ended up essentially just doing porportional control (as my integral and derivative constants remained at 0.0); however, my program computes an output as if it were doing PID control. This output is used to set the robot's linear velocity, while its angular velocity is purely a constant whose sign is determined by the estimated angle error.

### Visualization

https://github.com/user-attachments/assets/adc1935a-4b86-43f6-be9f-4f7d2b002155

Shown here is a representation of how the Neato is processing the environment around it. The fine red points are its LaserScan LIDAR readings. The blue spheres represent each point that the Neato is accounting for in its estimation of where the person is. Finally, the green sphere represents the average of the blue spheres, and the point that the Neato pilots towards.

As you watch the video, you can see that the Neato is pretty good at estimating its destimation on top of the two LIDAR feet that are present. However, towards the end of the video, you can see the Neato become "distracted" by its scans of the wall. It becomes unable to identify the person at the location of the feet, in favor of the tremendous weight present at the location of the wall.

### Challenges and reflection

The greatest challenge for me was recognizing the importance of filtering. I encountered a lot of roadblocks with the LaserScan data because I didn't yet understand why I was getting so much faulty data -- I thought something in my software processing was wrong, and only later did I realize I needed to identify and filter out bad scans (0.0 and infinity values). I also learned that it is often worthwhile to filter out even "good" data in favor of only keeping the data you need -- for example, giving the Neato an artificial "field of view" and a limit to the distance of its ranges, such that it would not be distracted by any nearby walls.

The biggest learning I got out of this project was definitely learning about PID control. I had heard the term often, but didn't know what it meant (conceptually or mathematically). During this project, I challenged myself to implement PID control for scaling the Neato's linear speeds. I did a lot of reading on Wikipedia about how each term affected the speed of a robotic behavior, and found examples online of psuedocode that implemented the math in Python. However, I didn't end up having enough time to play with the coefficient values beyond a very basic level.

While my code technically uses PID control, the integral and derivative coefficients are both set to zero, making it essentially a souped-up porportional control. If I had more time, I would experiment further with the integral and derivative coefficients to see if I could balance out the overshooting and dampening effects. Even though I didn't truly get to implement PID control, I still feel like I learned a lot about the math behind the process, and thus I am happy with what I got out of this challenge.

## Behavior #5: Obstacle Avoidance

### Description and process

In this challenge, 

I broke this down conceptually like so: as the Neato approaches a set of obstacles, it receives a set of scans that are either on its left or right side. The quantity of these scans, as well as their angle and range values, should contribute to a left or right "weight" that encourages the Neato to turn left or right. For example, if the Neato has a lot of scans on its left side, or those scans have very low range values, the Neato should prefer to turn away from those scans. If the Neato is receiving little to no scans, it should continue driving like normal. Finally, if the Neato was receiving scans that were very close by, it should stop driving forward and only turn (as not to crash).

In practice, I implemented this by including code in my node's LaserScan callback function to filter and sort scans based on whether they were on the left or right:
```
# if scan is on the left side
if 0.0 < angle < self.fov:
    self.left_scans[0].append(angle)
    self.left_scans[1].append(r)
    # ...
# if scan is on the right side
elif 360.0 - self.fov < angle < 360.0:
    self.right_scans[0].append(angle)
    self.right_scans[1].append(r)
```

Later, during the timer callback, my node uses the scan values on each side to weight a particular coefficient that affects the Neato's overall angular speed. There are a couple of ways that make sense to do this: namely number of scans, closest range value, or smallest angle away from 0 degrees. I initially started by just counting the total number of scans, but it didn't work very well, so I then pivoted to using the smallest (and therefore closest) range value to determine the coefficient size:
```
# flip max range (so small values create big coefficients, and vice versa)
left_k = 1.0 / min(self.left_scans[1])
right_k = 1.0 / min(self.right_scans[1]) * -1
vel_msg.angular.z = 0.1 * (left_k + right_k)
```

While this is sound in theory, my obstacle avoider was very poor in practice. My node inconsistently identified obstacles, and when it did identify them it only reacted somewhat well to avoiding them. One common problem I had was that when the Neato got too close, it wouldn't turn at all. This makes sense conceptually -- when facing an obstacle, the Neato had comporable amounts of left and right scans, and could not make a decision. However, I believe that its problem was much more deep rooted. One clue I noticed was that the Neato wouldn't stop driving even when too close to an obstacle, as it was meant to due to this logic:
```
# don't turn if too close
if self.closest_r > 0.3:
    vel_msg.linear.x = 0.1
else:
    print(f"Too close to drive forward: {self.closest_r}.")~~~
```

If the Neato was properly processing its scan data, this would not be a problem. However, its failure to even stop driving when an obstacle is too close leads me to my final theory: the way that the Neato was sorting and processing its scans was fundamentally flawed. While I unfortunately ran out of time to investigate this further, I was able to gather more clues by employing the use of Rviz.

### Visualization

![Obstacle Avoider Rviz](https://github.com/itannermahncke/comprobo_warmup_project/blob/main/images/Obstacle_Avoider.png)

Shown here is an Rviz screenshot of how the Neato is logging left and right scans (right is green, left is blue). As you can see, the Neato is doing a poor job of sorting the scans into left and right groups, and even identifying where those scans actually are -- for example, the Neato is marking points outside of its field of view. This indicates to me that the flaw in my obstacle avoider is present in the Neato's method of identifying and sorting points, rather than what it does with those values when catalogued.

### Challenges and reflection

Had I more time, I would have further investigated my problems with sorting LaserScan data. Once my node was working properly, I would have liked to improve its avoidance skills by encoporating the range and even angle data in a more intelligent way than just looking at the shortest range. For example, I could give the most weight to obstacles scanned at an angle close to the 0-degree heading, and less weight to obstacles closer to 90-degree and 270-degree headings. I also would have included code to handle getting stuck when the left and right scans were comparable (i.e. when the robot is directly facing an obstacle) by choosing a direction to hard-code as the Neato's turning preference in this case. Finally, my stretch goal would be to include a way to provide the Neato a goal destination to have to try and reach. Path planning is something I am very interested in, and if I had more time, I would have explored rudimentary path planning via this challenge.

## Behavior #6: Finite State Controller

### A Note

Unfortunately, my teammate for this project dropped the class. While I made it pretty far by myself, I didn't have time to write even rudimentary code for the Finite State Controller challenge, much less develop any working software. Instead, I have created a short conceptual writeup explaining how I would approach the problem. I also have a Python file finite_state_controller.py, though it is empty.

### Conceptual process

When planning this node, I chose to combine the person follower behavior and the drive square behavior for two reasons. Firstly, I only wanted one behavior performing obstacle detection, such that I wouldn't need to differentiate between a person, an obstacle, and a wall. Secondly, I didn't want to incorporate the teleoperated behavior, because it takes in no world information besides key presses, and therefore I felt the spirit of the challenge (controlling robot behavior based off of worldstate) would be somewhat lost.

Rather than having a small controller node that "toggled" two other nodes on and off, I decided it would be better to include both behaviors in the same node. This is because I felt it is somewhat antithetical to the ROS ideology to have a node "call" another node -- having a heirarchical structure to nodes in which there is a central "decision maker" node that all other nodes exclusively talk to goes against the idea that nodes should generally be decentralized and unranked. Also, writing a node that is capable of directly disabling another node is difficult to do in ROS (and I believe that this is by design).

Firstly, I would modify the drive_square node such that it reset its internal state (e.g. its progress on the square) on a callback function rather than only at the start. The node would provide Neato commands to drive in the square pattern as normal, until it detected some kind of object that it thought was a person. Like my person follower, I wouldn't attempt to differentiate a person from an obstacle, and instead would simply limit the field of view of the Neato to prevent it from getting "distracted". When the Neato felt it could no longer identify a person nearby, it would call its reset function and begin driving in a new square.

Injecting the person follower code into the existing drive_square feels somewhat tricky, but one way to cleanly handle it is to have both behaviors run on separate timer callbacks. I believe there is a way in ROS to destroy, reset, and restart timers; I would experiment with methods of initializing the opposing timer when the current one was being destroyed (i.e. when a person is spotted, destroy the drive_square timer and start the person_follower timer). Like how I reset the drive_square attributes, I would also want to reset the person follower's PID control values. This is because PID control requires a short history of error values and integral values, and I would not want these values to be kept from a prior instance of person following (nor would I want them to accumulate during drive_square behavior, which would happen if I did not destroy the timer).

### Anticipated challenges

Some difficulties from the prior challenges would persist here and would require fixes or workarounds. For example, the drive_square behavior's accumulation of error would still be present, and may even be worsened if I am unable to re-zero the robot's initial position each time it begins a new square. Additionally, the person_follower behavior requires a series of "blinders" on the Neato (field of view for angles, and filters for range values) that would continue to be necessary to avoid Neato attraction to obstacles and walls.

In addition to persisting problems, I can also see some new controller-specific problems appearing. Not being able to reset a state upon toggling states is a very major problem -- in particular, PID controls keeping old error values and even letting them build up while not in the relevant state would make it such that upon entering the person_follower state, the Neato could behave very erratically in an attempt to correct ancient errors. I also predict that if the Neato were unable to consistently predict a person's presence (i.e. the person seeming to flicker in and out of existence depending on scan quality), it would quickly jump back and forth between states, making it impossible to meaningfully progress on a square and unable to accumulate useful error values for the PID control. Resolving this would require dedication to high-quality and well-filtered scan data that was also consistent.