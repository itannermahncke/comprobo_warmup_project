# CompRobo Warmup Project
By Ivy Mahncke and Jiayi Shao

## Introduction

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

### Visualization

![](https://github.com/itannermahncke/comprobo_warmup_project/blob/main/images/Wall_Follower_Rviz.gif)

Shown here is an Rviz gif of what the Neato "sees". The flickering blue dots represent the two ranges that the Neato uses to approximate its angle relative to the wall. As you can see, the dots are far beyond where they should be, which implies that something is faulty with the Neato's ability to truly identify those points.

### Challenges and reflection

- investigate strange visual behavior (and understand trig better)
- Has to start pointed at a wall - remove this constraint
- Incoporate the tf module to learn about reference frames
- Code that reacts to a corner, turns and keeps following (third state)
- React to left walls, not just right walls

## Behavior #4: Person Following

### Description and process

In this challenge, the person_follower node instructs the Neato to identify and follow a person around.

### Visualization

### Challenges and reflection

- averaging needed to happen in cartesian, not polar
- filtering bad r values made everything much more functional
- more time: tune PID values rather than using fancy porportional control