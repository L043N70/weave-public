from time import perf_counter

from ortools.sat.python import cp_model

def solve_uav_problem_with_cp_sat(data, log, use_all_uav=True, additional_uav_cost: list = None):
    D = data['D']
    N = data['N']
    V1 = data['V1']
    B = data['V3']
    E1 = data['A1']

    c = data['c']
    t = data['t']

    K1 = data['K1']
    Q = data['Q']
    T = data['T']

    if additional_uav_cost is None:
        additional_uav_cost = [1 for _ in range(K1)]

    num_totale_nodi = len(V1)
    max_arco_energia = max(c.values())
    min_arco_energia = min(c.values())
    max_arco_tempo = max(t.values())
    num_nodi_swap = len(B)

    max_costo_uav_senza_ricarica = num_totale_nodi * max_arco_energia
    max_tempo_uav_senza_swap = num_totale_nodi * max_arco_tempo

    if max_costo_uav_senza_ricarica <= Q:
        MAX_TIME = max_tempo_uav_senza_swap
        MAX_SPRECO = 0
    else:
        MAX_TIME = max_tempo_uav_senza_swap + (num_nodi_swap * T)
        MAX_SPRECO = Q - min_arco_energia
    MAX_ENERGY = max_costo_uav_senza_ricarica + (num_nodi_swap * MAX_SPRECO)

    bsps_from_depot_to = {}

    neighbours_of_depot = {}
    for d in D:
        neighbours_of_depot[d] = {n for (i, n) in E1 if i == d}

    all_depot_neighbours = []
    for d in neighbours_of_depot:
        all_depot_neighbours.extend(neighbours_of_depot[d])

    ps = list(powerset(B, exclude_empty_set=True))
    for i in V1:
        bsps_from_depot_to[i] = ps

    string_sub_power_set_of_bsps_up_to_node = dict()
    for i in V1:
        string_sub_power_set_of_bsps_up_to_node[i] = []
        for bsps in bsps_from_depot_to[i]:
            if bsps:
                U = "-".join(map(str, [*bsps]))
                string_sub_power_set_of_bsps_up_to_node[i].append(U)

    model = cp_model.CpModel()

    x = {}
    for k in range(K1):
        for i in V1:
            for j in V1:
                if (i, j) in E1:
                    x[(i, j, k)] = model.NewBoolVar(f"x({i},{j})[{k}]")

    w = {}
    for k in range(K1):
        for i in V1:
            w[(i, k)] = model.NewIntVar(0, MAX_TIME, f"w({i})[{k}]")

    q = {}
    for k in range(K1):
        for i in V1:
            q[(i, k)] = model.NewIntVar(0, MAX_ENERGY, f"q({i})[{k}]")

    z = {}
    for k in range(K1):
        for s in B:
            z[(s, k)] = model.NewBoolVar(f"z({s})[{k}]")

    r = {}
    for k in range(K1):
        for s in B:
            r[(s, k)] = model.NewIntVar(0, MAX_SPRECO, f"r({s})[{k}]")

    nv = {}
    for k in range(K1):
        for i in V1:
            nv[(i, k)] = model.NewBoolVar(f"nv({i})[{k}]")

    fnv = {}
    for k in range(K1):
        for i in all_depot_neighbours:
            fnv[(i, k)] = model.NewBoolVar(f"fnv({i})[{k}]")

    fsv = {}
    for k in range(K1):
        for s in B:
            fsv[(s, k)] = model.NewBoolVar(f"fsv({s})[{k}]")

    sfsv = {}
    for k in range(K1):
        for s in B:
            sfsv[(s, k)] = model.NewBoolVar(f"sfsv({s})[{k}]")

    b_bar = {}
    for k in range(K1):
        for i in V1:
            for j in V1:
                b_bar[(i, j, k)] = model.NewBoolVar(f"b_bar({i})({j})[{k}]")

    b = {}
    for k in range(K1):
        for i in V1:
            for j in V1:
                b[(i, j, k)] = model.NewBoolVar(f"b({i})({j})[{k}]")

    a_bar = {}
    for k in range(K1):
        for i in V1:
            for U in string_sub_power_set_of_bsps_up_to_node[i]:
                a_bar[(i, U, k)] = model.NewBoolVar(f"a_bar({i})({U})[{k}]")

    ac = {}
    for k in range(K1):
        for i in V1:
            for U in string_sub_power_set_of_bsps_up_to_node[i]:
                ac[(i, U, k)] = model.NewIntVar(0, len(B), f"ac({i})({U})[{k}]")

    acm = {}
    for k in range(K1):
        for i in V1:
            acm[(i, k)] = model.NewIntVar(0, len(B), f"acm({i})[{k}]")

    acmp = {}
    for k in range(K1):
        for i in V1:
            acmp[(i, k)] = model.NewBoolVar(f"acmp({i})[{k}]")

    a = {}
    for k in range(K1):
        for i in V1:
            for U in string_sub_power_set_of_bsps_up_to_node[i]:
                a[(i, U, k)] = model.NewBoolVar(f"a({i})({U})[{k}]")

    nsv = {}
    for k in range(K1):
        nsv[k] = model.NewBoolVar(f"nsv[{k}]")

    nvsc = {}
    for k in range(K1):
        for s in B:
            nvsc[(s, k)] = model.NewIntVar(0, len(V1), f"nvsc({s})[{k}]")

    nvfsc = {}
    for k in range(K1):
        nvfsc[k] = model.NewIntVar(0, len(V1), f"nvfsc[{k}]")

    svc = {}
    for k in range(K1):
        svc[k] = model.NewIntVar(0, len(B), f"svc[{k}]")

    for k in range(K1):
        for s in B:
            model.Add(
                z[(s, k)]
                == 0
            ).OnlyEnforceIf(nv[(s, k)].Not())

    for k in range(K1):
        bsps_visited = [nv[(i, k)] for i in B]
        model.Add(
            svc[k]
            == sum(bsps_visited)
        )

    for k in range(K1):
        model.Add(
            sum(acmp[(i, k)] for i in V1)
            == 0
        ).OnlyEnforceIf(nsv[k])

    for k in range(K1):
        model.Add(
            sum(acmp[(i, k)] for i in V1)
            != 0
        ).OnlyEnforceIf(nsv[k].Not())

    for k in range(K1):
        for s in B:
            model.Add(
                nvsc[(s, k)]
                == 0
            ).OnlyEnforceIf(nv[(s, k)].Not())

    for k in range(K1):
        for s in B:
            model.Add(
                nvsc[(s, k)]
                == sum(b[(i, s, k)] for i in V1)
            ).OnlyEnforceIf(nv[(s, k)])

    for k in range(K1):
        num_nodes_visited_after_bsp = [nvsc[(s, k)] for s in B]
        if num_nodes_visited_after_bsp:
            model.AddMaxEquality(
                nvfsc[k],
                num_nodes_visited_after_bsp
            )
        else:
            model.Add(
                nvfsc[k]
                == 0
            )

    for k in range(K1):
        model.Add(
            nvfsc[k]
            == 0
        ).OnlyEnforceIf(nsv[k])

    for k in range(K1):
        for i in V1:
            model.Add(
                sum(x[(j, i, k)] for j in V1 if (j, i) in E1)
                >= 1
            ).OnlyEnforceIf(nv[(i, k)])

    for k in range(K1):
        for i in V1:
            model.Add(
                sum(x[(j, i, k)] for j in V1 if (j, i) in E1)
                == 0
            ).OnlyEnforceIf(nv[(i, k)].Not())

    for k in range(K1):
        for j in D:
            for i in neighbours_of_depot[j]:
                model.Add(
                    x[(j, i, k)]
                    == fnv[(i, k)]
                )

    for k in range(K1):
        for s in B:
            model.Add(
                sum(b[(i, s, k)] for i in B if i != s)
                == svc[k] - 1
            ).OnlyEnforceIf(fsv[(s, k)])

    for k in range(K1):
        for s in B:
            model.Add(
                sum(b[(i, s, k)] for i in B if i != s)
                != svc[k] - 1
            ).OnlyEnforceIf(fsv[(s, k)].Not())

    for k in range(K1):
        for s in B:
            model.Add(
                fsv[(s, k)] + z[(s, k)]
                == 2
            ).OnlyEnforceIf(sfsv[(s, k)])

    for k in range(K1):
        for s in B:
            model.Add(
                fsv[(s, k)] + z[(s, k)]
                != 2
            ).OnlyEnforceIf(sfsv[(s, k)].Not())

    for k in range(K1):
        for i in V1:
            for j in V1:
                model.Add(
                    w[(i, k)]
                    > w[(j, k)]
                ).OnlyEnforceIf(b_bar[(i, j, k)])

    for k in range(K1):
        for i in V1:
            for j in V1:
                model.Add(
                    w[(i, k)]
                    <= w[(j, k)]
                ).OnlyEnforceIf(b_bar[(i, j, k)].Not())

    for k in range(K1):
        for i in V1:
            for j in V1:
                model.Add(
                    b_bar[(i, j, k)] + nv[(j, k)]
                    == 2
                ).OnlyEnforceIf(b[(i, j, k)])

    for k in range(K1):
        for i in V1:
            for j in V1:
                model.Add(
                    b_bar[(i, j, k)] + nv[(j, k)]
                    != 2
                ).OnlyEnforceIf(b[(i, j, k)].Not())

    for k in range(K1):
        for i in V1:
            for U in string_sub_power_set_of_bsps_up_to_node[i]:
                sub_power_set = [int(u) for u in U.split("-")]
                model.Add(
                    sum((b[(i, s, k)] + nv[(s, k)]) for s in sub_power_set)
                    == len(sub_power_set) * 2
                ).OnlyEnforceIf(a_bar[(i, U, k)])

    for k in range(K1):
        for i in V1:
            for U in string_sub_power_set_of_bsps_up_to_node[i]:
                sub_power_set = [int(u) for u in U.split("-")]
                model.Add(
                    sum((b[(i, s, k)] + nv[(s, k)]) for s in sub_power_set)
                    != len(sub_power_set) * 2
                ).OnlyEnforceIf(a_bar[(i, U, k)].Not())

    for k in range(K1):
        for i in V1:
            for U in string_sub_power_set_of_bsps_up_to_node[i]:
                cardinality: int = len(U.split("-"))
                model.Add(
                    ac[(i, U, k)]
                    == cardinality
                ).OnlyEnforceIf(a_bar[(i, U, k)])

    for k in range(K1):
        for i in V1:
            for U in string_sub_power_set_of_bsps_up_to_node[i]:
                model.Add(
                    ac[(i, U, k)]
                    == 0
                ).OnlyEnforceIf(a_bar[(i, U, k)].Not())

    for k in range(K1):
        for i in V1:
            num_of_bsps_visited_before_i = [ac[(i, U, k)] for U in string_sub_power_set_of_bsps_up_to_node[i]]
            if num_of_bsps_visited_before_i:
                model.AddMaxEquality(
                    acm[(i, k)],
                    num_of_bsps_visited_before_i
                )
            else:
                model.Add(
                    acm[(i, k)]
                    == 0
                )

    for k in range(K1):
        for i in V1:
            model.Add(
                acm[(i, k)]
                > 0
            ).OnlyEnforceIf(acmp[(i, k)])

    for k in range(K1):
        for i in V1:
            model.Add(
                acm[(i, k)]
                <= 0
            ).OnlyEnforceIf(acmp[(i, k)].Not())

    for k in range(K1):
        for i in V1:
            for U in string_sub_power_set_of_bsps_up_to_node[i]:
                model.Add(
                    a[(i, U, k)]
                    <= a_bar[(i, U, k)]
                )

    for i in N:
        model.Add(
            sum(x[(i, j, k)] for k in range(K1) for j in V1 if (i, j) in E1)
            == 1
        )

    for k in range(K1):
        for i in V1 - D:
            model.Add(
                sum(x[(i, j, k)] for j in V1 if (i, j) in E1) -
                sum(x[(j, i, k)] for j in V1 if (j, i) in E1)
                == 0
            )

    for k in range(K1):
        model.Add(
            sum(x[(i, j, k)] for i in D for j in N if (i, j) in E1)
            == 1
        )

    for k in range(K1):
        for i in N - B:
            for j in V1:
                if (i, j) in E1:
                    model.Add(
                        w[(i, k)] + t[(i, j)]
                        <= w[(j, k)]
                    ).OnlyEnforceIf(x[(i, j, k)])

    for k in range(K1):
        for i in B:
            for j in V1:
                if (i, j) in E1:
                    model.Add(
                        w[(i, k)] + t[(i, j)] + T * z[(i, k)]
                        <= w[(j, k)]
                    ).OnlyEnforceIf(x[(i, j, k)])

    for k in range(K1):
        for j in B:
            model.Add(
                sum(x[(i, j, k)] for i in V1 if (i, j) in E1)
                >= z[(j, k)]
            )

    for k in range(K1):
        for i in V1:
            model.Add(
                q[(i, k)]
                <= Q
            ).OnlyEnforceIf(nsv[k])

    for k in range(K1):
        for i in V1:
            if i in all_depot_neighbours:
                model.Add(
                    q[(i, k)]
                    <= Q
                ).OnlyEnforceIf(fnv[(i, k)])

    for k in range(K1):
        for s in B:
            model.Add(
                q[(s, k)]
                <= Q
            ).OnlyEnforceIf(fsv[(s, k)])

    for k in range(K1):
        for i in V1:
            for bsps in bsps_from_depot_to[i]:
                if bsps:
                    U = "-".join(map(str, [*bsps]))
                    model.Add(
                        q[(i, k)]
                        <= Q + Q * sum(z[(s, k)] for s in bsps)
                    ).OnlyEnforceIf(a[(i, U, k)])

    for k in range(K1):
        for i in B:
            for j in V1:
                if (i, j) in E1:
                    model.Add(
                        q[(i, k)] + c[(i, j)] + r[(i, k)]
                        <= q[(j, k)]
                    ).OnlyEnforceIf(x[(i, j, k)])

    for k in range(K1):
        for i in N - B:
            for j in V1:
                if (i, j) in E1:
                    model.Add(
                        q[(i, k)] + c[(i, j)]
                        <= q[(j, k)]
                    ).OnlyEnforceIf(x[(i, j, k)])

    for k in range(K1):
        for s in B:
            model.Add(
                Q * z[(s, k)] - q[(s, k)]
                <= r[(s, k)]
            ).OnlyEnforceIf(sfsv[(s, k)])

    for k in range(K1):
        for s in B:
            for bsps in bsps_from_depot_to[s]:
                if bsps:
                    U = "-".join(map(str, [*bsps]))

                    model.Add(
                        Q * z[(s, k)] + Q * sum(z[(i, k)] for i in bsps) - q[(s, k)]
                        <= r[(s, k)]
                    ).OnlyEnforceIf(a[(s, U, k)])

    for k in range(K1):
        for (i, j) in E1:
            if i in D:
                model.Add(
                    w[(j, k)]
                    != 0
                ).OnlyEnforceIf(x[(i, j, k)])

    for k in range(K1):
        for (i, j) in E1:
            if i in D:
                model.Add(
                    q[(j, k)]
                    != 0
                ).OnlyEnforceIf(x[(i, j, k)])

    for k in range(K1):
        for i in V1:
            model.Add(
                q[(i, k)]
                == 0
            ).OnlyEnforceIf(nv[(i, k)].Not())

    for k in range(K1):
        for i in V1:
            model.Add(
                w[(i, k)]
                == 0
            ).OnlyEnforceIf(nv[(i, k)].Not())

    for k in range(K1):
        model.Add(
            sum(sum(a[(i, U, k)] for U in string_sub_power_set_of_bsps_up_to_node[i]) for i in V1)
            == nvfsc[k]
        ).OnlyEnforceIf(nsv[k].Not())

    for k in range(K1):
        for i in V1:
            model.Add(
                sum(a[(i, U, k)] for U in string_sub_power_set_of_bsps_up_to_node[i])
                <= 1
            )

    for k in range(K1):
        for i in V1:
            for U in string_sub_power_set_of_bsps_up_to_node[i]:
                model.Add(
                    ac[(i, U, k)]
                    == acm[(i, k)]
                ).OnlyEnforceIf(a[(i, U, k)]).OnlyEnforceIf(acmp[(i, k)])

    for k in range(K1):
        for i in D:
            for j in V1:
                if (i, j) in E1:
                    model.Add(
                        c[(i, j)]
                        <= q[(j, k)]
                    ).OnlyEnforceIf(fnv[(j, k)])

    for k in range(K1):
        for i in D:
            for j in V1:
                if (i, j) in E1:
                    model.Add(
                        t[(i, j)]
                        <= w[(j, k)]
                    ).OnlyEnforceIf(fnv[(j, k)])

    objective = 0

    for k in range(K1):

        for (i, j) in E1:
            objective += c[(i, j)] * q[(i, k)]

    model.Minimize(objective)

    model.AddDecisionStrategy(
        [
            *x.values(),
            *w.values(),
            *z.values(),
            *q.values(),
            *r.values(),

            *nv.values(),
            *fnv.values(),
            *fsv.values(),
            *sfsv.values(),

            *b_bar.values(),
            *b.values(),
            *a_bar.values(),
            *ac.values(),
            *acm.values(),
            *acmp.values(),
            *a.values(),

            *nsv.values(),
            *svc.values(),
            *nvsc.values(),
            *nvfsc.values(),
        ],
        cp_model.CHOOSE_FIRST,
        cp_model.SELECT_MIN_VALUE)

    solver = cp_model.CpSolver()
    solver.parameters.search_branching = cp_model.FIXED_SEARCH
    solver.parameters.enumerate_all_solutions = True

    solution_callback = VarArraySolutionCallback([

    ])

    start_time = perf_counter()
    status = solver.Solve(model, solution_callback)
    end_time = perf_counter()

    if status == cp_model.OPTIMAL:
        objective_value = solver.ObjectiveValue()
        return True, objective_value, x, w, z, q, r

    elif status == cp_model.FEASIBLE:
        msg = "A feasible solution for UAV problem was found, but we don't know if it's optimal."
        return False, msg, None, None, None, None, None

    elif status == cp_model.INFEASIBLE:
        msg = "The UAV problem was proven infeasible."
        return False, msg, None, None, None, None, None

    else:
        msg = f"No solution was found for UAV problem. Status code: {status}"
        return False, msg, None, None, None, None, None


class VarArraySolutionCallback(cp_model.CpSolverSolutionCallback):

    def __init__(self, variables):
        cp_model.CpSolverSolutionCallback.__init__(self)
        self.__solution_counter = 0

    def on_solution_callback(self):
        self.__solution_counter += 1

    def solution_count(self):
        return self.__solution_counter
