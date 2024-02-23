# Emergency Braking

The Safety Node implements Automatic Emergency Breaking (AEB) through Time to Collision (TTC) calculations. This the time it would take for the car to collide with an obstacle if it maintained its current heading and velocity.

We approximate the time to collision using Instantaneous Time to Collision (iTTC), which is the ratio of instantaneous range to range rate calculated from current range measurements and velocity measurements of the vehicle:

$$iTTC_i(t) = \frac{r_i(t)}{[-\dot{r}i(t)]+}$$

where 
 is the instantaneous range measurements and 
 is the current range rates. The operator 
 is defined as 
.

We obtain:

$$r_i$$ by using the current measurements from the LaserScan message
 $$r_i$$ by mapping the vehicle's current longitudinal velocity onto each scan beam's angle by using 
, where 
 is the forward speed in the vehicle's frame of reference (obtained from Odometry message), and 
 is the beam angle obtained from LaserScan messages.
The code has been implemented in both Python3 (under scripts/wall_follow_node.py) and C++ (under src/safety_node.cpp). Both have the same functionalities.
