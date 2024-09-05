
#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <utility>
#include <algorithm>
#include <queue> 

using namespace std;

class CNode
{
public:

    // the location of source or a robot and the travel cost for the heuristic function.
    CNode() : xPos(3), yPos(3), travelCost(2) {}
    CNode(int x, int y) : xPos(x), yPos(y), travelCost(2) {}
    CNode(int x, int y, int cost) : xPos(x), yPos(y), travelCost(cost) {}

    //Now locating the target NOde or destination.....
    inline CNode& operator=(const CNode& target)
    {
        if (*this != target)
        {
            xPos = target.xPos;
            yPos = target.yPos;
            travelCost = target.travelCost;
        }

        return *this;
    }

    inline bool operator==(const CNode& target) const
    {
        return xPos == target.xPos && yPos == target.yPos;
    }

    inline bool operator!=(const CNode& target) const
    {
        return !(*this == target);
    }

    inline bool operator<(const CNode& target) const
    {
        return target.travelCost < travelCost;
    }

    int xPos, yPos, travelCost;
};

class CPath
{
public:
    //Using vector to join the path from its parents node.......
    typedef vector<CNode> nodeList;

    nodeList Find(const CNode& startNode, const CNode& endNode, int mapArray[][24])
    {
        //open list is a list of all locations adjacent to areas that has been already explored and evaluated.
        //The closed list is a record of all location which have been explored and evaluated by the algorithm
        nodeList finalPath, openList, closedList;

        finalPath.push_back(startNode);
        openList.push_back(startNode);
        closedList.push_back(startNode);

        //First selects the start node (open list).
        while (!openList.empty())
        {
            // Check each node in the open list
            for (size_t i = 0; i < openList.size(); ++i)
            {
                if (openList[i].xPos == endNode.xPos && openList[i].yPos == endNode.yPos)
                    return finalPath;

                priority_queue<CNode> nodeQueue;

                // Get surrounding nodes
                for (int x = -1; x <= 1; ++x)
                {
                    for (int y = -1; y <= 1; ++y)
                    {
                        const int current_x = openList[i].xPos + x;
                        const int current_y = openList[i].yPos + y;

                        bool alreadyCheckedNode = false;
                        for (size_t i = 0; i < closedList.size(); ++i)
                        {
                            if (current_x == closedList[i].xPos && current_y == closedList[i].yPos) //Selects the goal node (closed list).
                            {
                                alreadyCheckedNode = true;
                                break;
                            }
                        }
                        // if the node space is already checked then continue the path processing.
                        if (alreadyCheckedNode)
                            continue;

                        // Ignore current coordinate and don't go out of array scope
                        if (current_x < 0 || current_x > 24 || current_y < 0 || current_y >24 || (openList[i].xPos == current_x && openList[i].yPos == current_y))
                            continue;

                        // Avoid walls
                        if (mapArray[current_x][current_y] == '#')
                            continue;


                        const int xNodeDifference = abs(current_x - (openList[i].xPos));
                        const int yNodeDifference = abs(current_y - (openList[i].yPos));

                        // Diagonal?
                        const int direction = xNodeDifference == 1 && yNodeDifference == 1;
                        const int xDistance = abs(current_x - endNode.xPos);
                        const int yDistance = abs(current_y - endNode.yPos);

                        //adding the heuristic 
                        //heuristic represents the distance between the selected  adjacent nodes with intersect point at references line.

                        int heuristic = (xDistance + yDistance);

                        nodeQueue.push(CNode(current_x, current_y, heuristic));
                    }
                }

                if (!nodeQueue.empty())
                {
                    // Add the nearest node to the current node.
                    openList.push_back(nodeQueue.top());
                    finalPath.push_back(nodeQueue.top());

                    // Put into closed list
                    while (!nodeQueue.empty())
                    {
                        closedList.push_back(nodeQueue.top());
                        nodeQueue.pop();
                    }
                }
            }
        }

        return finalPath;
    }
};
//creating a 2D array map of 12 rows 24 columns.

int mapArray[12][24] =
{
        { '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1',' 1', '1', '1', '1', '1', '1', '1', '1', '1', '1' },
        { '1', '0', '0', '0', '0', '0', '0', '0', '0', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',' 0', '1' },
        { '1', '0', 'A', '0', '0', '0', '0', '0', '0', '1', '0', '0', '0', '0', '0', '0', '0', '1', '1', '1', '1', '1', '0', '1' },
        { '1', '0', '0', '0', '0', '0', '0', '0', '0', '1', '0', '0', '0', '0', '0', '0', '0', '1', '0', '0', '0', '1', '0', '1' },
        { '1', '0', '0', '0', '0', '0', '0', '0', '0', '1', '0', '0', '0', '1', '1', '1', '1', '1', '0', '0', 'B', '1', '0', '1' },
        { '1', '0', '0', '0', '0', '0', '0', '0', '0', '1', '0', '0', '0', '1', '0', '0', '0', '1', '0', '0', '0', '1', '0', '1' },
        { '1', '0',' 1', '1', '1', '1', '1', '1', '1', '1', '0', '0', '0', '1', '0', '0', '0', '1', '0', '0', '0', '1', '0', '1' },
        { '1', '0', '0', '0', '0', '0', '0', '0', '0', '1', '0', '0', '0', '1', '0', '1', '0', '1', '0', '0', '0', '1', '0', '1' },
        { '1', '0', '0', '0', '0', '0', '0', '0', '0', '1', '0', '0', '0', '1', '1', '1', '0', '1', '0', '0', '0', '1', '0', '1' },
        { '1', '0',' 1', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1', '0', '0', '0', '1', '0', '1' },
        { '1', '0', '0', '0', '1', '0', '1', '1', '1', '1', '0', '0', '0', '0', '0', '0', '0', '1', '0', '0', '0', '0', '0', '1' },
        { '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1' },



};
//mathematical formulation to start the project from the source code A to the target code B.
int main(int argc, char** argv)
{
    CNode start, end;

    for (int width = 0; width < 24; ++width)
    {
        for (int height = 0; height < 24; ++height)
        {
            if (mapArray[width][height] == 'A')
            {
                start.xPos = width;
                start.yPos = height;
            }
            else if (mapArray[width][height] == 'B')
            {
                end.xPos = width;
                end.yPos = height;
            }
        }
    }

    CPath pathFinder;

    // A* finds a path from start to goal.
    // h is the heuristic function. h(n) estimates the cost to reach goal from node n.
    CPath::nodeList n = pathFinder.Find(start, end, mapArray);

    for (int i = 0; i < n.size(); ++i)
        if (mapArray[n[i].xPos][n[i].yPos] != 'A' && mapArray[n[i].xPos][n[i].yPos] != 'B')
            mapArray[n[i].xPos][n[i].yPos] = '*';




    for (int height = 0; height < 24; ++height)
    {
        for (int width = 0; width < 24; ++width)
        {
            if (width == 0) {
               cout << endl;
            }
              

            cout << (char)mapArray[height][width] << "  ";
        }
    }

    cin.get();

    return 0;
}

//////////_____________--------------RUN THE PROGRAM BY HITTING CTRL+F5 KEY --------------________________///////////////