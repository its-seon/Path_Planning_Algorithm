#include <iostream>
#include <random>
#include <vector>
#include <cmath>
#include "opencv2/opencv.hpp"

using namespace std;

struct Point {
    double x;
    double y;
    int parentIndex = -1;
    Point(double x = 0, double y = 0) : x(x), y(y) {}
};

struct Obstacle {
    Point leftTop;
    Point rightBottom;
    Obstacle(const Point& lt, const Point& rb) : leftTop(lt), rightBottom(rb) {}
};

const int width = 800;
const int height = 800;
const int jump = 30;

vector<Point> path;
vector<Obstacle> obstacles;

Point newPoint;

/* start and destionation setup */
Point start;
Point destination;

void initRRT(int startX, int startY, int endX, int endY) {
    start = Point(startX, startY);
    destination = Point(endX, endY);

    path.clear();
    path.push_back(start);
}

/* create obstacles */
void initObstacles() {
    obstacles.clear();
    obstacles.push_back(Obstacle({100, 100}, {200, 200}));
    obstacles.push_back(Obstacle({300, 300}, {400, 400}));
    obstacles.push_back(Obstacle({100, 300}, {200, 400}));
}

/* generate random point */
random_device rd;
mt19937 gen(rd());
uniform_int_distribution<> dis_x(0, width);
uniform_int_distribution<> dis_y(0, height);

Point generateRandomPoint() {
    return Point(dis_x(gen), dis_y(gen));
}

/* find the nearest point */
Point& nearestPoint(const Point& randomPoint) {
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

/* check if the random point is within the obstacles */
bool isInObstacle(const Point& p) {
    for (const auto& obs : obstacles) {
        if (p.x > obs.leftTop.x && p.x < obs.rightBottom.x &&
            p.y > obs.leftTop.y && p.y < obs.rightBottom.y) {
            return true;
        }
    }
    return false;
}

/* check if the path intersects with the obstacles */
bool checkPathIntersectionWithObstacles(Point& nearest, Point& newPoint){
    for (const auto& obs : obstacles) {
        Point topRight = {obs.rightBottom.x, obs.leftTop.y};
        Point bottomLeft = {obs.leftTop.x, obs.rightBottom.y};

        std::vector<std::pair<Point, Point>> edges = {
            {obs.leftTop, topRight},
            {topRight, obs.rightBottom},
            {obs.rightBottom, bottomLeft},
            {bottomLeft, obs.leftTop}
        };

        for (const auto& edge : edges) {
            double o1 = (newPoint.y - nearest.y) * (edge.first.x - nearest.x) - (newPoint.x - nearest.x) * (edge.first.y - nearest.y);
            double o2 = (newPoint.y - nearest.y) * (edge.second.x - nearest.x) - (newPoint.x - nearest.x) * (edge.second.y - nearest.y);
            double o3 = (edge.second.y - edge.first.y) * (nearest.x - edge.first.x) - (edge.second.x - edge.first.x) * (nearest.y - edge.first.y);
            double o4 = (edge.second.y - edge.first.y) * (newPoint.x - edge.first.x) - (edge.second.x - edge.first.x) * (newPoint.y - edge.first.y);

            if ((o1 * o2 < 0) && (o3 * o4 < 0)) {
                return true;
            }
        }
    }
    return false;
}

/* generate path */
void generatePath() {
    Point randomPoint = generateRandomPoint();
    Point& nearest = nearestPoint(randomPoint);

    double angle = atan2(randomPoint.y - nearest.y, randomPoint.x - nearest.x);
    newPoint = Point(nearest.x + jump * cos(angle), nearest.y + jump * sin(angle));
    cout << "New point: (" << newPoint.x << ", " << newPoint.y << ")" << endl;
    
    if (!isInObstacle(newPoint) && !checkPathIntersectionWithObstacles(nearest, newPoint)) {
        newPoint.parentIndex = &nearest - &path[0];
        path.push_back(newPoint);
    }

    if (sqrt(pow(path.back().x - destination.x, 2) + pow(path.back().y - destination.y, 2)) < jump) {
        path.push_back(destination);
        cout << "Destination reached." << endl;
    }
}

void updateVisualization(cv::Mat& img) {
    img.setTo(cv::Scalar(0, 0, 0));

    for (const auto& obs : obstacles) {
        cv::rectangle(img, cv::Point(obs.leftTop.x, obs.leftTop.y), cv::Point(obs.rightBottom.x, obs.rightBottom.y), cv::Scalar(0, 0, 255), -1);
    }

    // Draw lines from each point to its parent
    for (size_t i = 0; i < path.size(); i++) {
        if (path[i].parentIndex != -1) {
            cv::line(img, cv::Point(path[i].x, path[i].y), cv::Point(path[path[i].parentIndex].x, path[path[i].parentIndex].y), cv::Scalar(255, 255, 255), 2);
        }
        else{
            cv::line(img, cv::Point(path[i-1].x, path[i-1].y), cv::Point(path[i].x, path[i].y), cv::Scalar(255, 255, 255), 2);
        }
    }


    cv::circle(img, cv::Point(start.x, start.y), 5, cv::Scalar(0, 255, 0), cv::FILLED); // Green for start
    cv::circle(img, cv::Point(destination.x, destination.y), 5, cv::Scalar(255, 0, 0), cv::FILLED); // Blue for destination
    cv::circle(img, cv::Point(newPoint.x, newPoint.y), 5, cv::Scalar(0, 165, 255), cv::FILLED);  // Orange for newPoint

    cv::imshow("RRT Path Planning", img);
    cv::waitKey(20); // Short delay to update the display
}

int main() {
    initRRT(30, 30, 770, 770); // startX, startY, endX, endY point
    initObstacles(); // Initialize obstacles

    int iterations = 0;
    const int maxIterations = 50000; // Limit the number of iterations to avoid infinite loop

    cv::Mat img(width, height, CV_8UC3, cv::Scalar(0, 0, 0));

    while (true) {
        if (iterations++ > maxIterations) {
            cout << "Max iterations reached." << endl;
            break;
        }

        generatePath();
        updateVisualization(img);

        if (sqrt(pow(path.back().x - destination.x, 2) + pow(path.back().y - destination.y, 2)) < jump) {
            break;
        }
    }

    cv::waitKey(0); // Wait for a key press
    cout << "Path generated with " << path.size() << " points." << endl;

    return 0;
}
