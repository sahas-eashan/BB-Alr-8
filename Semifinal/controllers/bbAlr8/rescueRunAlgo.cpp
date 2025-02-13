#include "rescueRunAlgo.hpp"

RescueRunAlgo::RescueRunAlgo()
{
    log("RescueRunAlgo constructor called");
}

RescueRunAlgo::~RescueRunAlgo()
{
    log("RescueRunAlgo destructor called");
}

bool RescueRunAlgo::canMove(int x, int y, int direction)
{
    if (x < 0 || x >= MAZE_S || y < 0 || y >= MAZE_S)
    {
        return false;
    }

    int cell = maze[y][x];
    return !(cell & direction);
}

void RescueRunAlgo::log(const std::string &text)
{
    std::cerr << text << std::endl;
}

// Implements Dijkstra's algorithm with danger zone penalties
PathInfo RescueRunAlgo::findShortestPath(Point start, Point end)
{
    std::vector<std::vector<int>> distances(MAZE_S, std::vector<int>(MAZE_S, std::numeric_limits<int>::max()));
    std::vector<std::vector<Point>> previous(MAZE_S, std::vector<Point>(MAZE_S, Point{-1, -1}));

    // Priority queue with custom comparison
    auto compare = [](const std::pair<int, Point> &a, const std::pair<int, Point> &b)
    {
        return a.first > b.first;
    };
    std::priority_queue<std::pair<int, Point>,
                        std::vector<std::pair<int, Point>>,
                        decltype(compare)>
        pq(compare);

    distances[start.y][start.x] = 0;
    pq.push({0, start});

    // Helper function to check if a point is in a vector of points
    auto isPointIn = [](const Point &p, const std::vector<Point> &points)
    {
        return std::find(points.begin(), points.end(), p) != points.end();
    };

    while (!pq.empty())
    {
        Point current = pq.top().second;
        int currentDist = pq.top().first;
        pq.pop();

        if (current == end)
        {
            break;
        }

        if (currentDist > distances[current.y][current.x])
        {
            continue;
        }

        // Check all four directions
        const int dx[] = {0, 0, 1, -1}; // N, S, E, W
        const int dy[] = {1, -1, 0, 0};
        const int directions[] = {NORTH, SOUTH, EAST, WEST};

        for (int i = 0; i < 4; i++)
        {
            if (!canMove(current.x, current.y, directions[i]))
            {
                continue;
            }

            int newX = current.x + dx[i];
            int newY = current.y + dy[i];

            if (newX < 0 || newX >= MAZE_S || newY < 0 || newY >= MAZE_S)
            {
                continue;
            }

            Point newPoint{newX, newY};

            // Calculate step cost based on danger level
            int stepCost = 1; // Base cost for a step

            // Apply penalties for danger zones (unless it's the start or end point)
            if (!(newPoint == start) && !(newPoint == end))
            {
                if (isPointIn(newPoint, redNodes))
                {
                    stepCost *= 100;
                }
                if (isPointIn(newPoint, orangeNodes))
                {
                    stepCost *= 10;
                }
                else if (isPointIn(newPoint, yellowNodes))
                {
                    stepCost *= 1; // No penalty
                }
            }

            int newDist = distances[current.y][current.x] + stepCost;
            if (newDist < distances[newY][newX])
            {
                distances[newY][newX] = newDist;
                previous[newY][newX] = current;
                pq.push({newDist, newPoint});
            }
        }
    }

    // Reconstruct path
    PathInfo result;
    result.distance = distances[end.y][end.x];

    if (result.distance == std::numeric_limits<int>::max())
    {
        return result; // No path found
    }

    Point current = end;
    while (!(current == start))
    {
        result.path.push_back(current);
        current = previous[current.y][current.x];
    }
    result.path.push_back(start);
    std::reverse(result.path.begin(), result.path.end());

    return result;
}

// Calculate distances between all points (start + survivors)
std::vector<std::vector<int>> RescueRunAlgo::calculateAllPairDistances()
{
    std::vector<Point> allPoints = {startPoint};
    allPoints.insert(allPoints.end(), survivors.begin(), survivors.end());
    int n = allPoints.size();

    std::vector<std::vector<int>> distances(n, std::vector<int>(n));

    log("\nCalculating distances between all points:");
    for (int i = 0; i < n; i++)
    {
        for (int j = i + 1; j < n; j++)
        {
            PathInfo path = findShortestPath(allPoints[i], allPoints[j]);
            distances[i][j] = distances[j][i] = path.distance;

            std::string pointA = (i == 0) ? "Start" : "S" + std::to_string(i);
            std::string pointB = (j == 0) ? "Start" : "S" + std::to_string(j);
            log(pointA + " to " + pointB + ": " + std::to_string(path.distance) + " steps");
        }
    }
    return distances;
}

// Solve TSP using bruteforce - optimal for small number of points
PathInfo RescueRunAlgo::solveTSP(const std::vector<std::vector<int>> &distances)
{
    int n = distances.size();
    std::vector<int> vertices;
    for (int i = 1; i < n; i++) // Skip 0 (start point)
    {
        vertices.push_back(i);
    }

    PathInfo bestResult;
    bestResult.distance = std::numeric_limits<int>::max();

    // Try all possible permutations of vertices (excluding start point)
    do
    {
        int currentDist = distances[0][vertices[0]]; // Distance from start to first point

        // Add distances between consecutive points
        for (size_t i = 0; i < vertices.size() - 1; i++)
        {
            currentDist += distances[vertices[i]][vertices[i + 1]];
        }

        // Add distance from last point back to start
        currentDist += distances[vertices.back()][0];

        // Update best result if current path is shorter
        if (currentDist < bestResult.distance)
        {
            bestResult.distance = currentDist;
            bestResult.path.clear();

            // Reconstruct the path
            std::vector<Point> allPoints = {startPoint};
            allPoints.insert(allPoints.end(), survivors.begin(), survivors.end());

            bestResult.path.push_back(allPoints[0]); // Start point
            for (int vertex : vertices)
            {
                bestResult.path.push_back(allPoints[vertex]);
            }
            bestResult.path.push_back(allPoints[0]); // Return to start
        }
    } while (std::next_permutation(vertices.begin(), vertices.end()));

    return bestResult;
}

// Modify findPathsToSurvivors to draw only the final optimal route
void RescueRunAlgo::findOptimalRoute()
{
    auto distances = calculateAllPairDistances();
    PathInfo tspResult = solveTSP(distances);

    log("\nOptimal route found!");
    log("Total distance: " + std::to_string(tspResult.distance) + " steps");
    log("Route: ");

    // Clear previous path
    optimalPath.clear();

    // Build the complete path including intermediate points
    for (size_t i = 0; i < tspResult.path.size() - 1; i++)
    {
        Point current = tspResult.path[i];
        Point next = tspResult.path[i + 1];

        // Find path between consecutive points
        PathInfo pathSegment = findShortestPath(current, next);

        // Add all points except the last one (to avoid duplicates)
        optimalPath.insert(optimalPath.end(),
                           pathSegment.path.begin(),
                           pathSegment.path.end() - 1);

        // Log the path
        for (const auto &point : pathSegment.path)
        {
            log("(" + std::to_string(point.x) + "," + std::to_string(point.y) + ") ");
        }
        log("\n");
    }
    // Add the final point
    if (!tspResult.path.empty())
    {
        optimalPath.push_back(tspResult.path.back());
    }
}

RescueRunAlgo::Movement RescueRunAlgo::getNextMovement(const Point &currentPos, const Point &nextPos, int currentHeading) const
{
    Movement movement;
    movement.nextPosition = nextPos;

    // Determine target heading based on movement direction
    int targetHeading;
    if (nextPos.x > currentPos.x)
        targetHeading = 1; // EAST
    else if (nextPos.x < currentPos.x)
        targetHeading = 3; // WEST
    else if (nextPos.y < currentPos.y)
        targetHeading = 2; // SOUTH
    else
        targetHeading = 0; // NORTH

    // Calculate turn needed
    int turnNeeded = (targetHeading - currentHeading + 4) % 4;

    // Determine command based on turn needed
    switch (turnNeeded)
    {
    case 0: // Already facing correct direction
        movement.command = Command::MOVE_FORWARD;
        break;
    case 1: // Need to turn right
        movement.command = Command::TURN_RIGHT;
        break;
    case 2: // Need to turn 180
        movement.command = Command::TURN_180;
        break;
    case 3: // Need to turn left
        movement.command = Command::TURN_LEFT;
        break;
    }

    return movement;
}