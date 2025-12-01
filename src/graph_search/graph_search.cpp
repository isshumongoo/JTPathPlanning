#include <iostream>
#include <cmath>
#include <queue>

#include <path_planning/utils/math_helpers.h>
#include <path_planning/utils/graph_utils.h>

#include <path_planning/graph_search/graph_search.h>

/**
 * General graph search instructions:
 *
 * First, define the correct data type to keep track of your visited cells
 * and add the start cell to it. If you need to initialize any properties
 * of the start cell, do that too.
 *
 * Next, implement the graph search function. Save the result in the path
 * variable defined for you.
 *
 * To visualize which cells are visited in the navigation webapp, save each
 * visited cell in the vector in the graph struct as follows:
 *      graph.visited_cells.push_back(c);
 * where c is a Cell struct corresponding to the visited cell you want to
 * visualize.
 *
 * The tracePath() function will return a path (which you should assign to
 * the path variable above) given the goal index, if you have kept track
 * of the parent of each node correctly and have implemented the
 * getParent() function. If you do not find a path, return an empty path
 * vector.
*/

std::vector<Cell> depthFirstSearch(GridGraph& graph, const Cell& start, const Cell& goal)
{
    std::vector<Cell> path;  // The final path should be placed here.

    initGraph(graph);  // Make sure all the node values are reset.

    int start_idx = cellToIdx(start.i, start.j, graph);

    /**
     * TODO (P3): Implement DFS.
     */

    return path;
}

std::vector<Cell> breadthFirstSearch(GridGraph& graph, const Cell& start, const Cell& goal)
{
    std::vector<Cell> path;  // The final path should be placed here.

    initGraph(graph);  // Make sure all the node values are reset.

    int start_idx = cellToIdx(start.i, start.j, graph);
    int goal_idx  = cellToIdx(goal.i, goal.j, graph);

    /**
     * TODO (P3): Implement BFS.
     */

    // If start or goal is in collision, there is no valid path.
    if (checkCollision(start_idx, graph) || checkCollision(goal_idx, graph))
    {
        return path;  // empty
    }

    std::queue<int> frontier;

    // Initialize start node
    graph.nodes[start_idx].visited  = true;
    graph.nodes[start_idx].distance = 0.0f;
    graph.nodes[start_idx].parent   = -1;

    frontier.push(start_idx);

    bool found = false;

    while (!frontier.empty())
    {
        int current = frontier.front();
        frontier.pop();

        // Record this visited cell for visualization
        Cell c = idxToCell(current, graph);
        graph.visited_cells.push_back(c);

        // Check if we reached the goal
        if (current == goal_idx)
        {
            found = true;
            break;
        }

        // Explore neighbors
        std::vector<int> neighbors = findNeighbors(current, graph);
        for (int n_idx : neighbors)
        {
            // Skip if already visited
            if (graph.nodes[n_idx].visited)
            {
                continue;
            }

            // Skip if this neighbor would collide with an obstacle
            if (checkCollision(n_idx, graph))
            {
                continue;
            }

            graph.nodes[n_idx].visited  = true;
            graph.nodes[n_idx].parent   = current;
            graph.nodes[n_idx].distance = graph.nodes[current].distance + 1.0f;

            frontier.push(n_idx);
        }
    }

    if (found)
    {
        // Build the path from goal back to start using parents
        path = tracePath(goal_idx, graph);
    }

    return path;
}

std::vector<Cell> aStarSearch(GridGraph& graph, const Cell& start, const Cell& goal)
{
    std::vector<Cell> path;  // The final path should be placed here.

    initGraph(graph);  // Make sure all the node values are reset.

    int start_idx = cellToIdx(start.i, start.j, graph);

    /**
     * TODO (P3): Implement A-star search.
     */

    return path;
}
