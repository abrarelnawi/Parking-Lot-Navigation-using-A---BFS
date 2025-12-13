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

// ========================= Heuristic Type ============================
enum HeuristicType {
    MANHATTAN,
    EUCLIDEAN
};

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

// ========================= A* ============================
class AStar {
public:
    Environment env;
    Car car;
    HeuristicType hType;

    AStar(Environment e, Car c, HeuristicType ht)
        : env(e), car(c), hType(ht) {}

    double heuristic(const State& a, const State& b){
        if(hType == MANHATTAN)
            return fabs(a.x - b.x) + fabs(a.y - b.y);
        else
            return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }

    bool search(State start, State goal, vector<State>& path,
                bool &isAdmissible, bool &isConsistent,
                string &admReason, string &conReason,
                double &execTimeMs)
    {
        auto t0 = chrono::high_resolution_clock::now();

        isAdmissible = true;
        isConsistent = true;

        unordered_map<State,double,StateHash> gScore;
        unordered_map<State,State,StateHash> parent;
        unordered_map<State,bool,StateHash> closed;

        struct Node{
            State s;
            double f;
            bool operator>(Node const& o) const {
                return f > o.f;
            }
        };

        priority_queue<Node,vector<Node>,greater<Node>> open;

        gScore[start] = 0.0;
        open.push({start, heuristic(start, goal)});

        vector<double> steering = {-30.0, 0.0, 30.0};

        bool found = false;
        State goalReached;

        while(!open.empty()){
            Node cur = open.top(); open.pop();
            State cs = cur.s;

            if(closed[cs]) continue;
            closed[cs] = true;

            if(heuristic(cs, goal) < 1.0){
                found = true;
                goalReached = cs;
                break;
            }

            for(double sd : steering){
                State ns = car.move(cs, sd);
                if(!env.freeCell(ns.x, ns.y)) continue;

                double newCost = gScore[cs] + car.step;

                double h_cs = heuristic(cs, goal);
                double h_ns = heuristic(ns, goal);
                if(h_cs > car.step + h_ns + 1e-6)
                    isConsistent = false;

                if(!gScore.count(ns) || newCost < gScore[ns]){
                    gScore[ns] = newCost;
                    parent[ns] = cs;
                    open.push({ns, newCost + h_ns});
                }
            }
        }

        if(!found){
            auto t1 = chrono::high_resolution_clock::now();
            execTimeMs = chrono::duration<double,milli>(t1 - t0).count();
            admReason = "لم يتم العثور على مسار.";
            conReason = "لم يكتمل الفحص.";
            return false;
        }

        double goalCost = gScore[goalReached];

        for(auto &kv : gScore){
            double g = kv.second;
            double h = heuristic(kv.first, goal);
            if(h > goalCost - g + 1e-6){
                isAdmissible = false;
                break;
            }
        }

        conReason = isConsistent ?
            "لم يتم رصد خرق للـ consistency." :
            "تم رصد خرق للـ consistency.";

        admReason = isAdmissible ?
            "لم يتم رصد خرق للـ admissibility (فحص تجريبي)." :
            "تم رصد خرق للـ admissibility (فحص تجريبي).";

        auto t1 = chrono::high_resolution_clock::now();
        execTimeMs = chrono::duration<double,milli>(t1 - t0).count();

        reconstruct(parent, goalReached, start, path);
        return true;
    }

    void reconstruct(unordered_map<State,State,StateHash>& parent,
                     State goal, State start, vector<State>& path)
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
    Environment env(30,30);
    env.addObstacle(10,10);
    env.addObstacle(11,10);

    Car car(2.0,1.0);
    State start(2,2,0);
    State goal(20,20,0);

    for(int i=0;i<2;i++){
        HeuristicType ht = (i==0 ? MANHATTAN : EUCLIDEAN);

        cout << "\n=============================\n";
        cout << "Heuristic: "
             << (ht==MANHATTAN ? "Manhattan" : "Euclidean") << "\n";

        AStar astar(env, car, ht);

        vector<State> path;
        bool admissible, consistent;
        string admReason, conReason;
        double execMs;

        if(astar.search(start, goal, path,
                        admissible, consistent,
                        admReason, conReason, execMs)){

            cout << "\nالمسار المكتشف:\n";
            for(auto &s : path){
                cout << "(" << s.x << ", " << s.y
                     << ", θ=" << s.theta << "°)\n";
            }

            cout << "\nعدد نقاط المسار = " << path.size() << "\n";
            cout << "زمن التنفيذ = " << execMs << " ms\n\n";

            cout << "=== تحليل الـ Heuristic ===\n";
            cout << "Consistent? " << (consistent ? "YES" : "NO") << "\n";
            cout << conReason << "\n\n";

            cout << "Admissible? " << (admissible ? "YES" : "NO") << "\n";
            cout << admReason << "\n";
        }
        else{
            cout << "لا يوجد مسار\n";
        }
    }
    return 0;
}
