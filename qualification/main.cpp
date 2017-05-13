#include <set>
#include <utility>
#include <vector>
#include <queue>
#include <map>
#include <iostream>
#include <fstream>
#include <cassert>
#include <cmath>
#include <algorithm>
#include <limits>

using namespace std;

//////////////////////////////////////////////////////////////////////////
// DATA STRUCTURES
//////////////////////////////////////////////////////////////////////////

char *input_name;
ifstream input_file;

// Problem
typedef pair<int, int> pii;
const int NM_MAX = 4000;
int N, M, DRONES, TURNS, PAYLOAD, TYPES, WAREHOUSES, ORDERS;

struct Warehouse {
    int i; 
    int j;
    vector<int> prod_available;
    int wh_num;
};

struct Order {
    int i; 
    int j;
    vector<int> items_type;
    int ord_num;
    int tie;
};

enum Action { LOAD = 0, DELIVER };

struct Command {
    int drone;
    Action action;
    int id; 
    int prod_type; 
    int prod_num;
};

struct Localisation {
    int i;
    int j;
};

struct Charge {
    int type;
    int num;
};

// STATE

long score_total;
vector<int> weight_type;
vector<int> max_items;
vector<vector<int>> order_mat;
vector<Warehouse> warehouses;
vector<Order> orders;
vector<int> total_items;
vector<int> min_completion_time;
vector<Localisation> localisations;
vector<Charge> charges;
vector<int> time_dispo;
vector<Command> sol;

long save_score_total;
vector<int> save_weight_type;
vector<int> save_max_items;
vector<vector<int>> save_order_mat;
vector<Warehouse> save_warehouses;
vector<Order> save_orders;
vector<int> save_total_items;
vector<int> save_min_completion_time;
vector<Localisation> save_localisations;
vector<Charge> save_charges;
vector<int> save_time_dispo;
vector<Command> save_sol;

void save() {
    save_score_total = score_total;
    save_weight_type = weight_type;
    save_max_items = max_items;
    save_order_mat = order_mat;
    save_warehouses = warehouses;
    save_orders = orders;
    save_total_items = total_items;
    save_min_completion_time = min_completion_time;
    save_localisations = localisations;
    save_charges = charges;
    save_time_dispo = time_dispo;
    save_sol = sol;
}

void restore() {
    score_total = save_score_total;
    weight_type = save_weight_type;
    max_items = save_max_items;
    order_mat = save_order_mat;
    warehouses = save_warehouses;
    orders = save_orders;
    total_items = save_total_items;
    min_completion_time = save_min_completion_time;
    localisations = save_localisations;
    charges = save_charges;
    time_dispo = save_time_dispo;
    sol = save_sol;
}

void print(ofstream &out, const Command &c) {
    out << c.drone << " ";
    out << (c.action?"D":"L") << " ";
    out << c.id << " ";
    out << c.prod_type << " ";
    out << c.prod_num << endl;
}

int distance(int i, int j, int ii, int jj) {
    int d2 = (i - ii) * (i - ii) + (j - jj) * (j - jj);
    int T = ceil(sqrt(d2));
    return T;
}

//////////////////////////////////////////////////////////////////////////
// PARSING 
//////////////////////////////////////////////////////////////////////////

void parse() {
    // m = number of col, n = number of lines
    input_file >> N;
    input_file >> M;
    assert(N <= NM_MAX && M <= NM_MAX);

    input_file >> DRONES;
    input_file >> TURNS;
    input_file >> PAYLOAD;
    input_file >> TYPES;

    for (int i = 0; i < TYPES; i++) {
        int tmp; 
        input_file >> tmp;
        weight_type.push_back(tmp);
    }

    input_file >> WAREHOUSES;
    for (int i = 0; i < WAREHOUSES; i++) {
        Warehouse w;
        w.wh_num = i;
        input_file >> w.i;
        input_file >> w.j;
        for (int j = 0; j < TYPES; j++) {
            int tmp; 
            input_file >> tmp;
            w.prod_available.push_back(tmp);
        }
        warehouses.push_back(w);
    }

    input_file >> ORDERS;
    for (int i = 0; i < ORDERS; i++) {
        Order o;
        o.ord_num = i;
        o.tie = rand() % 10;
        input_file >> o.i;
        input_file >> o.j;
        int num_items;
        input_file >> num_items;
        for (int j = 0; j < num_items; j++) {
            int tmp; 
            input_file >> tmp;
            o.items_type.push_back(tmp);
        }
        orders.push_back(o);
    }
}

//////////////////////////////////////////////////////////////////////////
// INITIALIZATION
//////////////////////////////////////////////////////////////////////////

void init_structures() {
    localisations = vector<Localisation>(DRONES);
    charges = vector<Charge>(DRONES);
    time_dispo = vector<int>(DRONES);
    for (int i = 0; i < DRONES; i++) {
        localisations[i].i = warehouses[0].i;
        localisations[i].j = warehouses[0].j;
        charges[i] = {0, 0};
        time_dispo[i] = 0;
    }

    order_mat = vector<vector<int>>(ORDERS, vector<int>(TYPES));
    for (int i = 0; i < ORDERS; i++) {
        for (int j = 0; j < TYPES; j++) {
            order_mat[i][j] = 0;
        }
    }

    for (int i = 0; i < ORDERS; i++) {
        for (int type : orders[i].items_type)  {
            order_mat[i][type]++;
        }
    }

    min_completion_time = vector<int>(ORDERS, 0);

    max_items = vector<int>(TYPES);
    for (int i = 0; i < TYPES; i++) {
        max_items[i] = PAYLOAD / weight_type[i];
        assert(max_items[i] > 0);
    }

    total_items = vector<int>(ORDERS);
    for (int i = 0; i < ORDERS; i++) {
        int remaining_items = 0;
        for (int k = 0; k < TYPES; k++) {
            remaining_items += order_mat[i][k];
        }
        total_items[i] = remaining_items;
        assert(remaining_items > 0);
    } 
}

int total_stock(int prod_type) {
    int res = 0;
    for (int i = 0; i < WAREHOUSES; i++) {
        Warehouse &w = warehouses[i];
        for (auto n : w.prod_available) {
            if (n == prod_type) res += n;
        }
    } 
    return res;
}

//////////////////////////////////////////////////////////////////////////
// SOLVING
//////////////////////////////////////////////////////////////////////////

struct Move {
    int ord;
    int it_ty;
    int wh;
    int drone; 
    int num; 
    int T;
};

// apply a move and return the score
int apply(const Move &m) {
    Command c1, c2;
    c1.drone = m.drone; c1.action = LOAD; c1.id = m.wh; c1.prod_type = m.it_ty; c1.prod_num = m.num;
    c2.drone = m.drone; c2.action = DELIVER; c2.id = m.ord; c2.prod_type = m.it_ty; c2.prod_num = m.num;

//    cout << order_mat[m.ord][m.it_ty] << " " << m.num << endl;

    assert(order_mat[m.ord][m.it_ty] > 0);
    order_mat[m.ord][m.it_ty] -= m.num;
    assert(order_mat[m.ord][m.it_ty] >= 0);

    localisations[m.drone].i = orders[m.ord].i;
    localisations[m.drone].j = orders[m.ord].j;
    time_dispo[m.drone] = m.T;
    assert(time_dispo[m.drone] <= TURNS);

    assert(warehouses[m.wh].prod_available[m.it_ty] > 0); 
    warehouses[m.wh].prod_available[m.it_ty] -= m.num;
    assert(warehouses[m.wh].prod_available[m.it_ty] >= 0); 
    total_items[m.ord] -= m.num;
    assert(total_items[m.ord] >= 0);
    min_completion_time[m.ord] = max(min_completion_time[m.ord], m.T);

    sol.push_back(c1);
    sol.push_back(c2);

    int score = 0;
    if (total_items[m.ord] == 0) {
     score = ceil( ((TURNS - min_completion_time[m.ord]) * 100) / (double) TURNS); 
   }
   return score;
}

void solve(int ord, int it_ty, int dr, int wh, Move &m, bool &found, long &score) {
    const Order &order = orders[ord];
    const Warehouse &whouse = warehouses[wh];

    assert(total_items[ord] > 0); 
    assert(order_mat[ord][it_ty] > 0);

    found = false;
    score = 0;

    int availatwh = whouse.prod_available[it_ty]; 
    int requested = order_mat[ord][it_ty];

    if (availatwh == 0) {
        found = false;
        return;
    }

    int possible = min(min(availatwh, requested), max_items[it_ty]);
    assert(possible >= 1);

    int dist1 = distance(whouse.i, whouse.j, localisations[dr].i, localisations[dr].j) + 1;
    int dist2 = distance(whouse.i, whouse.j, order.i, order.j) + 1;
    long new_time = (time_dispo[dr] + dist1 + dist2);
    if (new_time >= TURNS) {
        found = false;
        return;
    }

    m.ord = ord;
    m.it_ty = it_ty;
    m.wh = wh;
    m.drone = dr;
    m.num = possible;
    m.T = new_time;
    score = new_time;

    found = true;
    return;
}

bool solve_order(int order) {
    int start_it = rand() % TYPES;
    for (int it_ty_index = 0; it_ty_index < TYPES; it_ty_index++) {
        int it_ty = (start_it + it_ty_index) % TYPES;
        if (order_mat[order][it_ty] != 0) {
            Move best_move;
            bool found = false;
            long best_score = numeric_limits<long> :: max();
            for (int dr = 0; dr < DRONES; dr++) {
                for (int wh = 0; wh < WAREHOUSES; wh++) {
                    Move move;
                    long score;
                    bool this_found;
                    solve(order, it_ty, dr, wh, move, this_found, score);
                    if (!this_found) {
                        continue;
                    }
                    if (score < best_score) {
                        found = true;
                        best_move = move;
                        best_score = score;
                    }
                }
            }

            if (!found) { // unable to treat one item
                return false;
            }

            score_total += apply(best_move);
        }
    }

    return true;
}

void solve() {
    // bool found = true;
    vector<Order> orders_sorted = orders;

    auto compare = [](const Order &x1, const Order &x2) { return 10 * total_items[x1.ord_num] + x1.tie < 10 * total_items[x2.ord_num] + x2.tie; };
    sort(orders_sorted.begin(), orders_sorted.end(), compare);

    for (Order &o : orders_sorted) {
        solve_order(o.ord_num);
    }
}

void solve_greedy() {
    vector<bool> treated(ORDERS, false);

    vector<Order> orders_sorted = orders;
    auto compare = [](const Order &x1, const Order &x2) { return total_items[x1.ord_num] < total_items[x2.ord_num]; };
    sort(orders_sorted.begin(), orders_sorted.end(), compare);

    while (true) {
        int best_score = -1;
        int best_order = 0;
        bool found = false;
        for (Order o : orders_sorted) {
            if (treated[o.ord_num]) {
                continue;
            }
            save();
            bool res = solve_order(o.ord_num);
            if (res && score_total > best_score) {
                best_score = score_total;
                best_order = o.ord_num;
                found = true;
            }
            restore();
        }
        if (!found) {
            return;
        } else {
            solve_order(best_order);
            cout << best_order << " " << score_total << endl;
            treated[best_order] = true;
        }
    }

}

//////////////////////////////////////////////////////////////////////////
// IMPROVING
//////////////////////////////////////////////////////////////////////////

void improve() {

}

//////////////////////////////////////////////////////////////////////////
// PRINT SOLUTION
//////////////////////////////////////////////////////////////////////////

void print_sol() {
    int score = score_total;

    string output_name = string("sol-") + string(input_name) + "-" + to_string(score);
    ofstream output_file;
    output_file.open(output_name);
    if (!output_file.is_open()) {
        cerr << "can't open output file" << endl;
        exit(-1);
    }

    output_file << sol.size() << endl;
    for (int i = 0; i < sol.size(); i++) {
        print(output_file, sol[i]);
    }

    output_file.close();
}

//////////////////////////////////////////////////////////////////////////
// MAIN
//////////////////////////////////////////////////////////////////////////

int main(int argn, char* args[]) {
    assert(argn > 1);
    input_name = args[1];
    input_file.open (input_name);
    if (!input_file.is_open()) { 
        cout << "can't open input file " << input_name << endl;
        exit(-1);
    }

    srand(time(NULL));
    parse();

    int current_best = 0;
    vector<Command> best_sol;

    const int repeat = 1;
    for (int x = 0; x < repeat; x++) {
        score_total = 0;
        init_structures();
        solve();
        if (score_total > current_best) {
            current_best = score_total;
            cout << current_best << endl;
            best_sol = sol;
        }
    }
    sol = best_sol;
    print_sol();

    return 0;
}

