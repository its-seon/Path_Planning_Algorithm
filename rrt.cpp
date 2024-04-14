// create rrt algorithm for path planning
#include <iostream>
#include <random>
#include <vector>
#include <cmath>

using namespace std;

struct Point{
    double x;
    double y;
    int parentIndex = -1;
};

struct Obstacle{
    Point leftTop;
    Point rightBottom;
};

const int width = 500;
const int heigth = 500;
const int jump = 5;

vector<Point> path;
vector<Obstacle> obstacles;

Point start;
Point destination;

random_device rd;
mt19937 gen(rd());
uniform_int_distribution<> dis(0, width);

void initRRT(int startX, int startY, int endX, int endY) {
    start.x = startX;
    start.y = startY;
    destination.x = endX;
    destination.y = endY;

    path.clear();
    path.push_back(start);
}

void initObstacles() {
    obstacles.clear();
    obstacles = {
        {{100, 100}, {200, 200}},
        {{300, 300}, {400, 400}},
        {{100, 300}, {200, 400}}
    };
}

Point generateRandomPoint() {
    Point randomPoint{static_cast<double>(dis(gen)), static_cast<double>(dis(gen))};
    return randomPoint;
}

Point& nearestPoint(vector<Point>& path, const Point& randomPoint) {
    double minDistance = numeric_limits<double>::max();
    int nearestIndex = 0;
    
    for (int i = 0; i < path.size(); i++) {
        double distance = sqrt(pow(path[i].x - randomPoint.x, 2) + pow(path[i].y - randomPoint.y, 2));
        if (distance < minDistance) {
            minDistance = distance;
            nearestIndex = i;
        }
    }

    return path[nearestIndex];
}

void generatePath() {
    Point randomPoint = generateRandomPoint();
    Point& nearest = nearestPoint(path, randomPoint);

    double angle = atan2(randomPoint.y - nearest.y, randomPoint.x - nearest.x);

    Point newPoint;
    newPoint.x = nearest.x + jump * cos(angle);
    newPoint.y = nearest.y + jump * sin(angle);
    newPoint.parentIndex = &nearest - &path[0]; // Store index of the parent
    
    path.push_back(newPoint);
}

bool isInObstacle(const Point& p) {
    for (const auto& obs : obstacles) {
        if (p.x > obs.leftTop.x && p.x < obs.rightBottom.x &&
            p.y > obs.leftTop.y && p.y < obs.rightBottom.y) {
            return true;
        }
    }
    return false;
}

void obstacleCheck() {
    while (isInObstacle(path.back())) {
        path.pop_back(); // If in obstacle, backtrack until safe
        if (path.empty()) {
            path.push_back(start); // Reset to start if all path points are invalid
            break;
        }
    }
}

int main() {
    initRRT(0, 0, 500, 500);
    initObstacles();

    int iterations = 0;
    const int maxIterations = 50000; // Limit the number of iterations to avoid infinite loop

    while (true) {
        if (iterations++ > maxIterations) {
            cout << "Max iterations reached." << endl;
            break;
        }

        generatePath();
        obstacleCheck();

        if (sqrt(pow(path.back().x - destination.x, 2) + pow(path.back().y - destination.y, 2)) < jump) {
            cout << "Destination reached." << endl;
            break;
        }
    }

    cout << "Path generated with " << path.size() << " points." << endl;
    return 0;
}

