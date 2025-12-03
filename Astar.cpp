#include <bits/stdc++.h>
using namespace std;

// ========================= Utility ============================
double PI = 3.141592653589793;
double deg2rad(double d){ return d * PI / 180.0; }
double rad2deg(double r){ return r * 180.0 / PI; }

double normalizeDeg(float d){
    d = fmod(d, 360.0);
    if(d < 0) d += 360.0;
    return d;
}

// ========================= State Class ============================
class State {
public:
    int x, y;       // position as integer
    float theta;    // orientation as float (degrees)

    State(int _x=0, int _y=0, float _t=0.0){
        x = _x; y = _y; theta = normalizeDeg(_t);
    }

    bool operator==(State const& s) const {
        return (x == s.x && y == s.y && fabs(theta - s.theta) < 0.01);
    }
};

// Needed for unordered_map key
struct StateHash {
    size_t operator()(State const& s) const {
        long long X = s.x;
        long long Y = s.y;
        long long T = (long long)(s.theta*100); // precision for theta
        return X*73856093 ^ Y*19349663 ^ T*83492791;
    }
};

// ========================= Environment Class ============================
class Environment {
public:
    int W, H;
    vector<vector<int>> grid;

    Environment(int w, int h){
        W=w; H=h;
        grid.assign(H, vector<int>(W, 0));
    }

    void addObstacle(int x, int y){
        if(x>=0 && x<W && y>=0 && y<H)
            grid[y][x] = 1;
    }

    bool freeCell(int x, int y) const {
        if(x<0 || y<0 || x>=W || y>=H) return false;
        return (grid[y][x] == 0);
    }
};

// ========================= Car Class ============================
class Car {
public:
    double L;        // length
    double step;     // forward step

    Car(double length=2.0, double step_size=1.0){
        L=length;
        step=step_size;
    }

    // Bicycle model motion
    State move(const State& s, float steeringDeg) const {
        float thetaRad = deg2rad(s.theta);
        float steerRad = deg2rad(steeringDeg);

        float nx = s.x + step * cos(thetaRad);
        float ny = s.y + step * sin(thetaRad);
        float ntheta = thetaRad + (step / L) * tan(steerRad);

        return State((int)round(nx), (int)round(ny), normalizeDeg(rad2deg(ntheta)));
    }
};

// ========================= A* Class ============================
class AStar {
public:
    Environment env;
    Car car;

    AStar(Environment e, Car c) : env(e), car(c) {}

    double heuristic(const State& a, const State& b){
        return fabs(a.x - b.x) + fabs(a.y - b.y);
    }

    bool search(State start, State goal, vector<State>& path){
        unordered_map<State, double, StateHash> gScore;
        unordered_map<State, State, StateHash> parent;
        unordered_map<State, bool, StateHash> closed;

        struct Node {
            State s; double f;
            bool operator>(Node const& o) const { return f > o.f; }
        };

        priority_queue<Node, vector<Node>, greater<Node>> open;

        gScore[start] = 0;
        open.push({start, heuristic(start, goal)});

        vector<float> steeringOptions = {-30.0, 0.0, 30.0};

        while(!open.empty()){
            Node cur = open.top(); open.pop();
            State cs = cur.s;

            if(closed[cs]) continue;
            closed[cs] = true;

            // Goal Check
            if(heuristic(cs, goal) < 1.0){
                reconstruct_path(parent, cs, start, path);
                return true;
            }

            // Expand
            for(float sd : steeringOptions){
                State ns = car.move(cs, sd);

                if(!env.freeCell(ns.x, ns.y)) continue;

                double newCost = gScore[cs] + car.step;

                if(!gScore.count(ns) || newCost < gScore[ns]){
                    gScore[ns] = newCost;
                    parent[ns] = cs;
                    double f = newCost + heuristic(ns, goal);
                    open.push({ns, f});
                }
            }
        }
        return false;
    }

    void reconstruct_path(
        unordered_map<State,State,StateHash>& parent,
        State goal, State start,
        vector<State>& path)
    {
        State cur = goal;
        while(!(cur == start)){
            path.push_back(cur);
            cur = parent[cur];
        }
        path.push_back(start);
        reverse(path.begin(), path.end());
    }
};

// ========================= MAIN ============================
int main(){
    Environment env(30, 30);

    // مثال عوائق
    env.addObstacle(10,10);
    env.addObstacle(11,10);

    Car car(2.0, 1.0);

    State start(2, 2, 0);
    State goal(20, 20, 0);

    AStar astar(env, car);

    vector<State> path;
    if(astar.search(start, goal, path)){
        cout << "✔ مسار مُكتشف:\n";
        for(auto&s : path){
            cout << "(" << s.x << ", " << s.y << ", θ=" << s.theta << "°)\n";
        }
        cout << "\nعدد النقاط = " << path.size() << "\n";
    } else {
        cout << "❌ لا يوجد مسار\n";
    }

    return 0;
}
