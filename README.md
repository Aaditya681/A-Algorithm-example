# A-Algorithm-example
The A* algorithm is a widely used search algorithm that finds the shortest path between two nodes in a graph.It is often used in various fields such as robotics and game development for path finding and graph transversal.his project is aimed at studying how the agent designs the real-time AI 
system based on the use of search algorithm methods.This project is 
done with the help of C++ language in visual studio.In this project the 
goal is to draw a maze in such a way that the robot finds a way to reach 
the path destination.Using A* algorithm as a base search method the 
robot visualize the paths and makes sure it reaches to the target using 
a closest path possible ignoring the obstacles found on the way.A* 
search algorithm is used here because it is often used in many fields of 
computer science due to it’s completeness, optimality and optimal 
efficiency. Also it uses cost path g(n) and a cost estimate h(n) to reach 
the goal.
 I.e.f(n)=g(n)+h(n).
The maze is designed in 2d array such that there are 12 rows and 24 
coloumns;288 blocks on the map.Since it is represented in 2D array ,’1’ 
represents wall,’0’ represent walkable space ‘A’ represents the robot 
and ‘B’ represents the target.Here,The agent uses a Global path 
planning that requires prior knowledge of the robot’s environment and 
using this knowledge it creates a stimulated environment where the 
methods can plan a path.
The envrionment created is static where the environment is unvarying, 
the source and destination position are fixed, and obstacles do not vary 
location over time.
What went wrong
Even the best- thought-out program can have unexpected issues. Some 
are easy to track down ;others can drive you crazy trying to figure out 
the issue. We might tempted to blame the robot as being faulty but 
that is not the case. The robot is doing exactly what it is told to do by 
the program.
The main error in the program is the robot passes through the wall 
which are present inside the mapArray scope.The (*) sign is used as a 
path created by the robot(A) to the target(B).Though the robot 
ultimately reaches the target and achieved it’s goal the obstacles are 
being neglected. Maybe this was due to the wrong step in the program 
,the robot seems to passes through it as a walkable space. In my 
conclusion the robot doesn’t count the inner walls as obstacles but as a 
free space.
Also the program has other errors like console engine was too big.The 
colors representating the wall, target and the source was not used so it 
is little difficult to visualize.

In this work project, the path planning strategies for a robot path 
planner was proposed and implemented sucessfully on a real life 
activities using different mathematical equations, heuritsic algorithm 
and a uniform search method I.e A -star algorithm.In a*, we determine 
the order by a lower bound on the total length of the path,which 
includes the distance from the robot,to which is added to a lower 
bound on the distance still to be traveled.Thus,A* is suitable search 
algorithm for the proposed path. The proposed path planning are 
realized by appropriate code using visual studio c++. 
The fundamental goal of this project was to find a path from a source to 
the target location. The robot(target)finds the target location using A* 
search and heuristic values. The path planning was sucessful as the 
trajectory was found as a shortest path to the target.
Though there seems to be the problem where robot doesn’t recognize 
the interior wall as obstacle,it passes right through them.This may 
cause the error result.
Proper use of search method and practical work in the real life ,the 
agent can learn a lot from it.
In summary,this project work is carried out for the development of path 
planning strategies which are to be implemented for mobile robot task 
like pick-and –place, material distribution, automated storage and 
retrieval operation etc using heuristic algorithms.
