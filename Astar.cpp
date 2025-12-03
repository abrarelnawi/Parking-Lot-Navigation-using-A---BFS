#include <bits/stdc++.h>
using namespace std;

// ========================= Utility ============================
double PI = 3.14;
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

    // heuristic على (x,y) فقط
    double heuristic(const State& a, const State& b){
        return fabs(a.x - b.x) + fabs(a.y - b.y);
    }

    bool search(State start, State goal, vector<State>& path,
                bool &isAdmissible, bool &isConsistent,
                string &admReason, string &conReason)
    {
        isAdmissible = true;
        isConsistent = true;
        admReason.clear();
        conReason.clear();

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

        bool found = false;
        State goalReached;

        // لتخزين أول مثال لعدم الـ consistency
        bool consExampleStored = false;
        State cons_s_from, cons_s_to;
        double cons_h_from=0, cons_h_to=0, cons_edge_cost=0;

        while(!open.empty()){
            Node cur = open.top(); open.pop();
            State cs = cur.s;

            if(closed[cs]) continue;
            closed[cs] = true;

            // Goal Check (منطقة قرب الهدف)
            if(heuristic(cs, goal) < 1.0){
                found = true;
                goalReached = cs;
                break;
            }

            // Expand
            for(float sd : steeringOptions){
                State ns = car.move(cs, sd);

                if(!env.freeCell(ns.x, ns.y)) continue;

                double newCost = gScore[cs] + car.step;

                // --------- فحص الـ consistency على القوس cs -> ns ---------
                double h_cs = heuristic(cs, goal);
                double h_ns = heuristic(ns, goal);
                if (h_cs > car.step + h_ns + 1e-6) {
                    isConsistent = false;
                    if (!consExampleStored) {
                        consExampleStored = true;
                        cons_s_from = cs;
                        cons_s_to   = ns;
                        cons_h_from = h_cs;
                        cons_h_to   = h_ns;
                        cons_edge_cost = car.step;
                    }
                }
                // -----------------------------------------------------------

                if(!gScore.count(ns) || newCost < gScore[ns]){
                    gScore[ns] = newCost;
                    parent[ns] = cs;
                    double f = newCost + heuristic(ns, goal);
                    open.push({ns, f});
                }
            }
        }

        if(!found){
            admReason = "لم يتم العثور على مسار، لذلك لم يمكن تحليل الـ admissibility على مسار حل.";
            conReason = (isConsistent ? 
                         "لم يتم العثور على خرق للـ consistency أثناء التوسيع." :
                         "تم رصد خروقات للـ consistency أثناء التوسيع (لكن لم يوجد مسار كامل).");
            return false;
        }

        // =========== فحص الـ admissibility تقريبياً ===========
        double goalCost = gScore[goalReached];  // تكلفة المسار الذي وجدناه

        bool admExampleStored = false;
        State adm_s;
        double adm_g=0, adm_h=0, adm_bound=0;

        for (auto &kv : gScore) {
            const State &s = kv.first;
            double g = kv.second;
            double h = heuristic(s, goal);

            double bound = goalCost - g;
            // لو h(s) > C_found - g(s) ⇒ heuristic بالتأكيد تعدت الحد → non-admissible
            if (h > bound + 1e-6) {
                isAdmissible = false;
                if (!admExampleStored) {
                    admExampleStored = true;
                    adm_s = s;
                    adm_g = g;
                    adm_h = h;
                    adm_bound = bound;
                }
                break;
            }
        }
        // =====================================================

        // بناء رسائل التفسير
        if (!isConsistent && consExampleStored) {
            ostringstream oss;
            oss << "تم رصد خرق لخاصية الـ consistency عند الانتقال من الحالة "
                << "(" << cons_s_from.x << "," << cons_s_from.y << ", θ=" << cons_s_from.theta << "°)"
                << " إلى "
                << "(" << cons_s_to.x << "," << cons_s_to.y << ", θ=" << cons_s_to.theta << "°).\n"
                << "h(s) = " << cons_h_from
                << " > c(s,s') + h(s') = " << cons_edge_cost << " + " << cons_h_to
                << " = " << (cons_edge_cost + cons_h_to) << ".";
            conReason = oss.str();
        } else if (isConsistent) {
            conReason = "لم يتم رصد أي حالة يكون فيها h(s) > c(s,s') + h(s')، وبالتالي الهيوريستك متوافقة (consistent) على الحالات المزارة.";
        }

        if (!isAdmissible && admExampleStored) {
            ostringstream oss;
            oss << "تم رصد حالة تجعل الهيوريستك غير admissible:\n"
                << "الحالة s = (" << adm_s.x << "," << adm_s.y << ", θ=" << adm_s.theta << "°),\n"
                << "g(s) = " << adm_g << ", h(s) = " << adm_h
                << ", تكلفة المسار الذي وجدناه = " << goalCost << ".\n"
                << "يجب أن يتحقق h(s) ≤ C_found - g(s) = " << goalCost << " - " << adm_g
                << " = " << adm_bound << "،\n"
                << "لكن h(s) = " << adm_h << " > " << adm_bound << ".";
            admReason = oss.str();
        } else if (isAdmissible) {
            admReason = "لم يتم رصد حالة يكون فيها h(s) > C_found - g(s)، لذلك لم يظهر خرق للـ admissibility على الحالات المزارة (لكن هذا لا يُعد إثباتًا رياضيًا كاملاً).";
        }

        // إعادة بناء المسار من goalReached إلى start
        reconstruct_path(parent, goalReached, start, path);
        return true;
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

    // مثال عوائق بسيطة
    env.addObstacle(10,10);
    env.addObstacle(11,10);

    Car car(2.0, 1.0);

    State start(2, 2, 0);
    State goal(20, 20, 0);

    AStar astar(env, car);

    vector<State> path;
    bool admissible, consistent;
    string admReason, conReason;

    if(astar.search(start, goal, path, admissible, consistent, admReason, conReason)){
        cout << " مسار مُكتشف:\n";
        for(auto&s : path){
            cout << "(" << s.x << ", " << s.y << ", θ=" << s.theta << "°)\n";
        }
        cout << "\nعدد النقاط في المسار = " << path.size() << "\n\n";

        cout << "=== تحليل الـ Heuristic ===\n";
        cout << "Consistent ? " << (consistent ? "YES" : "NO") << "\n";
        cout << conReason << "\n\n";

        cout << "Admissible? " << (admissible ? "YES" : "NO") << "\n";
        cout << admReason << "\n";
    } else {
        cout << "  لا يوجد مسار\n";
        cout << "لم يتمكن البرنامج من اختبار الـ heuristic بدقة لأن A* لم يجد حلًا.\n";
    }

    return 0;
}
