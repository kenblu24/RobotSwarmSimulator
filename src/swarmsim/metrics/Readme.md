# Logan's Behaviors

The format of this list will be: Metric Name -> Description -> Parameters 

Since I have mainly tested my metrics with different numbers of agents, I will only include parameters that are relevant to the metric succeeding. Otherwise, I will just leave it blank.


## Circle Packing

This metric is based on the circle packing problem. It trains the agents to disperse by calculating a circle around each agent that is as large as possible. The circle size is calculated by taking the distance of the agent to whichever is closer, another agent or a wall. They are trained on the smallest circle of the agents.

### Parameters

- Number of Agents: the number of agents directly influences the fitness value. The smaller the number of agents, the higher the fitness value and vice versa.

### Expected Outcome

The expected outcome is that the agents will disperse around the world. The agents will not overlap and will be spread out evenly. The fitness value should be expected to increase as they disperse. There is a maximum fitness that is dependent on the number of agents. You can probably use an online calculator to find the maximum radius for the number of circles in the box (world).


## Distance Size Ratio

Distance Size Ration is the same as Circle Packing, but it returns the size of the smallest circle over the largest circle. This metric might be completely useless, but it is included for completeness

### Parameters

- Number of Agents: the number of agents directly influences the fitness value.

### Expected Outcome

The expected outcome is disperal of the agents around the world. Like the Circle Packing metric, there is a maximum fitness value that is dependent on the number of agents. It isn't as easy to calculate because the fitness value is a ratio of the smallest circle over the largest circle.


## Random Box Sampling

This metric is just badly implemented. If you want to know why just ask me.

~~ The idea behind this metric is to train the agents to disperse by sampling random boxes for agents and trying to have them all be in seperate boxes. The number of boxes is determined by the population size and the size of the boxes is determined by the world size and the population size. In reality, my code isn't random and just trains the agents to be one per box. ~~

### Parameters

- Number of Agents: 9 seems to be the best number of agents for 

### Expected Outcome

The agents will disperse one per box. The fitness value should be expected to increase as the agents get in one per box.

## Coordinate Test 

This is a test metric to train the agents to aggregate to a specific coordinate. The metric finds the sum of the distances of all the agents to the origin. It returns the negative of this value.

### Parameters

-

### Expected Outcome

The expected outcome is that the agents will aggregate to the chose coordinates. The fitness value should decrease toward zero as the agents get closer to the coordinate.


## Aggregation

This metric spawns the agents spread out acrross the world and then trains them on the decreasing largest distance between any two agents.

### Parameters

-

### Expected Outcome

The agents will aggregate to random locations depending on where they spawned. The fitness value should decrease toward zero as they get closer together.


## Random Box Aggregation

Makes the agents try to aggregate to a random box. This is good for training agents to aggregate to random locations. The number of boxes is determined by the population size. The size of the boxes is determined by the world size and the population size.

### Parameters

-

### Expected Outcome

The agents will aggregate to a random box per run. The fitness value should decrease toward zero as the agents get closer to the center of the box.
