# Project 2: Chemotaxis

## Group 7

### Current strategy (09/22/2021)

#### Controller

- refine the BFS solution by taking the previous DirectionType into account, so that we won't turn unexpectly the next step
- when the agent has only 2 ways, if the controller calculated that it won't go back, it won't assign a chemical so that the agent can turn on its own
- when the agent has been blocked 3 ways (2 ways arround and 1 way ahead), which means it needs to go back, we won't assign the blue chemical to let it back, the agent knows that it should go the opposite way.



Bug fix:(still has 1 unfixed)

1. always waste one blue chemical when starting the agent



Remains to do in the future:

1. if the agent generates in 1-4 turns, the diffusion may work better than the current startegy for that the blue chemical can attract bunch of agents to one way.
2. we can change the meaning of bits which store in the agent, and we want the agent to remember it has just passed a chemical so as not to go back



