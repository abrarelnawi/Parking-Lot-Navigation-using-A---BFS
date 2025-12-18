#include <bits/stdc++.h>
using namespace std;

// ========================= Utility ============================
double PI = 3.14;

double deg2rad(double d){ return d * PI / 180.0; }
double rad2deg(double r){ return r * 180.0 / PI; }

double normalizeDeg(double d){
    d = fmod(d, 360.0);
    if(d < 0) d += 360.0;
    return d;
}

// ========================= State ============================
class State {
public:
    int x, y;
    double theta;

    State(int _x=0, int _y=0, double _t=0.0){
        x = _x;
        y = _y;
        theta = normalizeDeg(_t);
    }

    bool operator==(State const& s) const {
        return x == s.x && y == s.y && fabs(theta - s.theta) < 0.01;
    }
};

// ========================= Hash ============================
struct StateHash {
    size_t operator()(State const& s) const {
        long long X = s.x;
        long long Y = s.y;
        long long T = (long long)(s.theta * 100);
        return X * 73856093 ^ Y * 19349663 ^ T * 83492791;
    }
};

// ========================= Environment ============================
class Environment {
public:
    int W, H;
    vector<vector<int>> grid;

    Environment(int w, int h){
        W = w;
        H = h;
        grid.assign(H, vector<int>(W, 0));
    }

    void addObstacle(int x, int y){
        if(x >= 0 && x < W && y >= 0 && y < H)
            grid[y][x] = 1;
    }

    bool freeCell(int x, int y) const {
        if(x < 0 || y < 0 || x >= W || y >= H) return false;
        return grid[y][x] == 0;
    }
};

// ========================= Car ============================
class Car {
public:
    double L;
    double step;

    Car(double length=2.0, double step_size=1.0){
        L = length;
        step = step_size;
    }

    State move(const State& s, double steeringDeg) const {
        double thetaRad = deg2rad(s.theta);
        double steerRad = deg2rad(steeringDeg);

        double nx = s.x + step * cos(thetaRad);
        double ny = s.y + step * sin(thetaRad);
        double ntheta = thetaRad + (step / L) * tan(steerRad);

        return State((int)round(nx),
                     (int)round(ny),
                     rad2deg(ntheta));
    }
};

// ========================= BFS ============================
class BFS {
public:
    Environment env;
    Car car;

    BFS(Environment e, Car c) : env(e), car(c) {}

    bool search(State start, State goal,
                vector<State>& path,
                double &execTimeMs)
    {
        auto t0 = chrono::high_resolution_clock::now();

        queue<State> q;
        unordered_map<State,bool,StateHash> visited;
        unordered_map<State,State,StateHash> parent;

        q.push(start);
        visited[start] = true;

        vector<double> steering = {-30.0, 0.0, 30.0};

        bool found = false;
        State goalReached;

        while(!q.empty()){
            State cs = q.front(); q.pop();

            // شرط الوصول للهدف (منطقة قريبة)
            if(abs(cs.x - goal.x) + abs(cs.y - goal.y) < 1){
                found = true;
                goalReached = cs;
                break;
            }

            for(double sd : steering){
                State ns = car.move(cs, sd);

                if(!env.freeCell(ns.x, ns.y)) continue;
                if(visited[ns]) continue;

                visited[ns] = true;
                parent[ns] = cs;
                q.push(ns);
            }
        }

        if(!found){
            auto t1 = chrono::high_resolution_clock::now();
            execTimeMs = chrono::duration<double,milli>(t1 - t0).count();
            return false;
        }

        // إعادة بناء المسار
        State cur = goalReached;
        while(!(cur == start)){
            path.push_back(cur);
            cur = parent[cur];
        }
        path.push_back(start);
        reverse(path.begin(), path.end());

        auto t1 = chrono::high_resolution_clock::now();
        execTimeMs = chrono::duration<double,milli>(t1 - t0).count();
        return true;
    }
};

// ========================= MAIN ============================
int main(){
    Environment env(30,30);
    env.addObstacle(10,10);
    env.addObstacle(11,10);

    Car car(2.0,1.0);

    State start(2,2,0);
    State goal(20,20,0);
    // State start(2, 2, 0);
    // State goal(20, 20, 0);
    //State start(5, 5, 90);
    // State goal(25, 25, 350);
    // State start(8, 8, 45);
    // State goal(10, 9, 11);
    // State start(0, 0, 1);
    //  State goal(2, 0, 0);

    

    BFS bfs(env, car);

    vector<State> path;
    double execMs;

    if(bfs.search(start, goal, path, execMs)){
        cout << "المسار المكتشف باستخدام BFS:\n";
        for(auto &s : path){
            cout << "(" << s.x << ", " << s.y
                 << ", θ=" << s.theta << "°)\n";
        }

        cout << "\nعدد نقاط المسار = " << path.size() << "\n";
        cout << "زمن التنفيذ = " << execMs << " ms\n";
    }
    else{
        cout << "لا يوجد مسار باستخدام BFS\n";
    }

    return 0;
}
