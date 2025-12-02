#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <cmath>
#include <algorithm>

// ----------------- Vehicle / Ackerman Steering -----------------
struct State {
    double x, y, theta;
};

struct Motion {
    double v;      // السرعة
    double delta;  // زاوية التوجيه
};

class AckermanSteering {
public:
    double length, width, wheel_base;
    std::vector<Motion> motions;

    AckermanSteering(double l = 19.0, double w = 9.5, double wb = 0.6122) {
        length = l;
        width = w;
        wheel_base = wb;
        motions.push_back({1.0, 0.0});   // مستقيم
        motions.push_back({1.0, 0.2});   // دوران صغير
        motions.push_back({1.0, -0.2});
    }

    State next_state(const State& current, const Motion& m, double dt) {
        State next;
        next.x = current.x + m.v * cos(current.theta) * dt;
        next.y = current.y + m.v * sin(current.theta) * dt;
        next.theta = current.theta + (m.v / wheel_base) * tan(m.delta) * dt;
        return next;
    }
};

// ----------------- Environment -----------------
class Environment {
public:
    int side;
    std::vector<std::vector<int>> grid; // 0 = فارغ, 1 = عائق

    Environment(int s = 100) {
        side = s;
        grid.resize(side, std::vector<int>(side, 0));
        // مثال: بعض العقبات
        for(int i=40; i<60; ++i)
            for(int j=40; j<60; ++j)
                grid[i][j] = 1;
    }

    bool is_colliding(double x, double y) {
        int ix = (int)x;
        int iy = (int)y;
        if(ix < 0  iy < 0  ix >= side  iy >= side) return true;
        if(grid[ix][iy] == 1) return true;
        return false;
    }
};

// ----------------- Hybrid A* -----------------
struct Node {
    State s;
    double cost;
    bool operator>(const Node& other) const { return cost > other.cost; }
};

class Astar {
public:
    Environment env;
    AckermanSteering car;
    State start, goal;
    double dt;
    double goal_threshold;

    std::map<std::pair<int,int>, State> parent;
    std::map<std::pair<int,int>, double> g;

    Astar(State start_, State goal_, double dt_ = 0.05)
        : start(start_), goal(goal_), dt(dt_) {
        goal_threshold = 4.0;
    }

    double distance(const State& a, const State& b) {
        return hypot(a.x - b.x, a.y - b.y);
    }

    double heuristic(const State& s) {
        return distance(s, goal);
    }

    bool is_goal(const State& s) {
        return distance(s, goal) < goal_threshold;
    }

    // توليد neighbors مع primitives صغيرة لكل Motion
    std::vector<std::pair<State, Motion>> get_valid_neigh(const State& current) {
        std::vector<std::pair<State, Motion>> valid;
        for(auto& m : car.motions) {
            State temp = current;
            bool valid_motion = true;
            int steps = 5;            // عدد خطوات صغيرة داخل dt
            double dt_small = dt / steps;

            for(int i=0; i<steps; i++) {
                temp = car.next_state(temp, m, dt_small);
                if(env.is_colliding(temp.x, temp.y)) {
                    valid_motion = false;
                    break;
                }
            }

            if(valid_motion)
                valid.push_back({temp, m}); // نضيف نهاية الحركة بعد primitives
        }
        return valid;
    }

    std::vector<State> search() {
        auto cmp = [](Node left, Node right){ return left.cost > right.cost; };
        std::priority_queue<Node, std::vector<Node>, decltype(cmp)> OPEN(cmp);

        OPEN.push({start, heuristic(start)});
        g[{(int)start.x,(int)start.y}] = 0;

        while(!OPEN.empty()) {
            Node current = OPEN.top(); OPEN.pop();
            if(is_goal(current.s)) break;

            for(auto& [next, motion] : get_valid_neigh(current.s)) {
                auto key = std::make_pair((int)next.x,(int)next.y);
                double new_cost = g[{(int)current.s.x,(int)current.s.y}] + distance(current.s,next);
                if(g.find(key) == g.end()  new_cost < g[key]) {
                    g[key] = new_cost;
                    parent[key] = current.s;

Abrar Elnawi, [11/30/2025 11:38 AM]
double f = new_cost + heuristic(next);
                    OPEN.push({next, f});
                }
            }
        }

        // استخراج المسار خطوة بخطوة
        std::vector<State> path;
        State s = goal;
        while(distance(s, start) > 0.001) {
            path.push_back(s);
            auto it = parent.find({(int)s.x,(int)s.y});
            if(it == parent.end()) break;
            s = it->second;
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());
        return path;
    }
};

// ----------------- Main -----------------
int main() {
    State start{10,10,0};
    State goal{80,90,0};

    Astar planner(start, goal);
    std::vector<State> path = planner.search();

    std::cout << "Detailed Path found:\n";
    int step = 0;
    for(auto& s : path) {
        std::cout << "Step " << step++ << ": (" << s.x << "," << s.y << ")\n";
    }

    return 0;
}
هدا الكود النهائي
