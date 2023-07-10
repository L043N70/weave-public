from time import perf_counter
from ortools.linear_solver import pywraplp


def solve_bss_problem(dm, log):
    D = dm['D']
    S = dm['S']
    P = dm['P']
    V2 = dm['V2']
    A2 = dm['A2']

    d = dm['d']

    tw = dm['tw']
    K2 = dm['K2']
    T = dm['T']

    num_totale_archi = len(A2)
    num_max_swap = len(P)
    max_arco_tempo = max(d.values())
    MAX_TIME = (num_totale_archi * max_arco_tempo) + (num_max_swap * T)

    solver = pywraplp.Solver.CreateSolver('SCIP')

    y = {}
    for k in range(K2):
        for i in V2:
            for j in V2:
                if (i, j) in A2:
                    y[(i, j, k)] = solver.IntVar(0, 1, f"y({i},{j})[{k}]")

    v = {}
    for k in range(K2):
        for i in V2:
            v[(i, k)] = solver.IntVar(1, solver.infinity(), 'v({})[{}]'.format(i, k))

    additional_bss_cost = [1, 2, 4, 8, 16]

    objective = solver.Objective()
    objective.SetMinimization()
    for k in range(K2):
        for i in V2:
            objective.SetCoefficient(v[(i, k)], additional_bss_cost[k])
            for j in V2:
                if (i, j) in A2:
                    objective.SetCoefficient(y[(i, j, k)], d[(i, j)] * additional_bss_cost[k])

    for i in P:
        solver.Add(
            sum(y[(i, j, k)] for k in range(K2) for j in V2 if (i, j) in A2)
            >= 1
            , f"(9b)_visit_pit_stop_{i}"
        )

    solver.Add(
        sum(y[(i, j, k)] for i in D for k in range(K2) for j in S if (i, j) in A2)
        >= 1
        , "(9c)_bsss_from_depots"
    )

    for k in range(K2):
        for i in V2 - D:
            solver.Add(
                sum(y[(i, j, k)] for j in V2 if (i, j) in A2) - sum(y[(j, i, k)] for j in V2 if (j, i) in A2)
                == 0
                , f"(9d)_flow_node_{i}_bss_{k}"
            )

    solver.Add(
        sum(y[(i, j, k)] for k in range(K2) for i in D for j in S if (i, j) in A2)
        - sum(y[(j, i, k)] for k in range(K2) for i in D for j in S if (j, i) in A2)
        == 0
        , f"(9e)_flow_depots"
    )

    for k in range(K2):
        for i in S - P:
            for j in V2:
                if (i, j) in A2:
                    solver.Add(
                        v[(i, k)] + d[(i, j)] - v[(j, k)]
                        <= (1 - y[(i, j, k)]) * MAX_TIME
                        , f"(9f)_schedule_edge_{i}_{j}_bss_{k}"
                    )

    for k in range(K2):
        for i in P:
            for j in V2:
                if (i, j) in A2:
                    solver.Add(
                        v[(i, k)] + d[(i, j)] + T - v[(j, k)]
                        <= (1 - y[(i, j, k)]) * MAX_TIME
                        , f"(9g)_schedule_edge_{i}_{j}_bss_{k}"
                    )

    for k in range(K2):
        for i in P:
            solver.Add(
                tw[i][0]
                <= v[(i, k)]
                , f"(9h)_time_window_lower_bound_node_{i}_bss_{k}"
            )

    for k in range(K2):
        for i in P:
            solver.Add(
                v[(i, k)] <= tw[i][1]
                , f"(9i)_time_window_upper_bound_node_{i}_bss_{k}"
            )

    for k in range(K2):
        solver.Add(
            sum(y[(i, j, k)] for i in D for j in S if (i, j) in A2)
            <= 1
            , "(9j)_limit_use_bss_{}".format(k)
        )

    start_time = perf_counter()
    status = solver.Solve()
    end_time = perf_counter()

    if status == pywraplp.Solver.OPTIMAL:

        objective_value = solver.Objective().Value()

        y = three_index_var_to_named_tuple_converter(y, range(K2), A2)
        v = two_index_var_to_named_tuple_converter(v, range(K2), V2)
        return True, objective_value, y, v

    elif status == pywraplp.Solver.INFEASIBLE:
        msg = "Nessuna soluzione ottima per il problema BSS"
        return False, msg, None, None

    else:
        msg = f"Nessuna soluzione per il problema BSS. Status code: {status}"
        return False, msg, None, None


if __name__ == '__main__':
    pass
