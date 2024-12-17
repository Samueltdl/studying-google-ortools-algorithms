import matplotlib.pyplot as plt
import networkx as nx
import pandas as pd
import time
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def create_data_model():
    # Coordenadas simuladas (50 clientes)
    import random
    random.seed(42)
    data = {}
    data['locations'] = [(random.randint(0, 100), random.randint(0, 100)) for _ in range(50)]
    data['distance_matrix'] = [
        [
            int(((x1 - x2)**2 + (y1 - y2)**2)**0.5)  # Distância Euclidiana
            for x2, y2 in data['locations']
        ]
        for x1, y1 in data['locations']
    ]
    data['num_vehicles'] = 5
    data['depot'] = 0
    data['demands'] = [0] + [1] * (len(data['locations']) - 1)
    data['vehicle_capacities'] = [len(data['locations']) // data['num_vehicles']] * data['num_vehicles']
    return data

def plot_graph(locations, routes, title):
    G = nx.DiGraph()
    pos = {i: (locations[i][0], locations[i][1]) for i in range(len(locations))}
    colors = ['r', 'g', 'b', 'c', 'm', 'y']  # Cores para os veículos
    
    # Adicionar nós (clientes e depósito)
    for i in range(len(locations)):
        G.add_node(i)

    # Desenhar as rotas de cada veículo com cores distintas
    plt.figure(figsize=(10, 8))
    for vehicle_id, route in enumerate(routes):
        edges = [(route[i], route[i + 1]) for i in range(len(route) - 1)]
        nx.draw_networkx_edges(G, pos, edgelist=edges, edge_color=colors[vehicle_id % len(colors)], width=2, alpha=0.7)
        nx.draw_networkx_nodes(G, pos, nodelist=route, node_size=300, node_color=colors[vehicle_id % len(colors)], label=f"Veículo {vehicle_id + 1}")

    # Desenhar os rótulos e o depósito (nó inicial/final)
    nx.draw_networkx_labels(G, pos, font_size=10, font_color='black', font_weight="bold")
    nx.draw_networkx_nodes(G, pos, nodelist=[0], node_size=500, node_color="yellow", label="Depósito")

    # Configurar a legenda e o título
    plt.legend()
    plt.title(title)
    plt.show()

def solve_vrp(data, first_solution_strategy, metaheuristic):
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        return data['distance_matrix'][manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
   
    # Callback de demanda
    def demand_callback(from_index):
        return data['demands'][manager.IndexToNode(from_index)]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

    # Adiciona a dimensão de capacidade para forçar o balanceamento
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # Slack (folga)
        data['vehicle_capacities'],  # Capacidades dos veículos
        True,  # Inicia cumulativo no depósito
        "Capacity"
    )


    # Configuração de parâmetros
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = first_solution_strategy
    search_parameters.local_search_metaheuristic = metaheuristic
    search_parameters.time_limit.seconds = 1  # Limite de tempo de execução

    # Resolver o problema
    start_time = time.time()
    solution = routing.SolveWithParameters(search_parameters)
    execution_time = time.time() - start_time

    # Extrair rotas
    if solution:
        routes = []
        for vehicle_id in range(data['num_vehicles']):
            route = []
            index = routing.Start(vehicle_id)
            while not routing.IsEnd(index):
                route.append(manager.IndexToNode(index))
                index = solution.Value(routing.NextVar(index))
            route.append(manager.IndexToNode(index))  # Último nó (depósito)
            routes.append(route)
        return routes, solution.ObjectiveValue(), execution_time
    else:
        return None, None, execution_time

def main():
    data = create_data_model()
    print(data["vehicle_capacities"])
    print(len(data["demands"]))
    strategies = {
        "PATH_CHEAPEST_ARC": routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC,
        "PARALLEL_CHEAPEST_INSERTION": routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION,
        "SAVINGS": routing_enums_pb2.FirstSolutionStrategy.SAVINGS,
        "CHRISTOFIDES": routing_enums_pb2.FirstSolutionStrategy.CHRISTOFIDES
    }
    metaheuristics = {
        "GUIDED_LOCAL_SEARCH": routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH,
        "TABU_SEARCH": routing_enums_pb2.LocalSearchMetaheuristic.TABU_SEARCH,
        "SIMULATED_ANNEALING": routing_enums_pb2.LocalSearchMetaheuristic.SIMULATED_ANNEALING
    }

    results = []

    for strategy_name, strategy in strategies.items():
        for meta_name, meta in metaheuristics.items():
            print(f"Testando: {strategy_name} + {meta_name}")
            routes, distance, exec_time = solve_vrp(data, strategy, meta)
            if routes:
                title = f"{strategy_name} + {meta_name}\nDistância: {distance}, Tempo: {exec_time:.2f}s"
                plot_graph(data['locations'], routes, title)
                results.append({
                    "Strategy": strategy_name,
                    "Metaheuristic": meta_name,
                    "Distance": distance,
                    "Execution Time (s)": exec_time
                })
            else:
                print(f"Falha para {strategy_name} + {meta_name}")
                results.append({
                    "Strategy": strategy_name,
                    "Metaheuristic": meta_name,
                    "Distance": None,
                    "Execution Time (s)": exec_time
                })

    # Exportar resultados para uma tabela
    df = pd.DataFrame(results)
    print(df)
    df.to_csv("vrp_experiment_results.csv", index=False)

if __name__ == '__main__':
    main()
