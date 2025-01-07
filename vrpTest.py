import matplotlib.pyplot as plt
import networkx as nx
import pandas as pd
import time, math
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def create_data_model():
    """
    Responsável por gerar os dados do problema, incluindo:

        -> Localizações: Coordenadas aleatórias para clientes e o depósito.

        -> Matriz de Distâncias: Calculada usando a distância euclidiana entre as localizações.
        
        -> Número de Veículos: Define a quantidade de veículos disponíveis.
        
        -> Capacidades dos Veículos: Distribui igualmente a capacidade entre os veículos.
        
        -> Demandas: Define a demanda de cada cliente.
    """

    # Coordenadas simuladas
    import random
    random.seed(42)
    data = {}
    data['locations'] = [(random.randint(0, 300), random.randint(0, 300)) for _ in range(50)]
    data['distance_matrix'] = [
        [
            int(((x1 - x2)**2 + (y1 - y2)**2)**0.5)  # Distância Euclidiana
            for x2, y2 in data['locations']
        ]
        for x1, y1 in data['locations']
    ]
    data['num_vehicles'] = 3
    data['depot'] = 0
    
    # Demandas variáveis (exemplo: valores entre 1 e 5 para cada cliente)
    data['demands'] = [0] + [random.randint(1, 5) for _ in range(len(data['locations']) - 1)]
    
    total_demand = sum(data['demands'])
    print(f"\n\nTotal de demandas -> {total_demand}")
    data['vehicle_capacities'] = [math.ceil(total_demand*0.5), math.ceil(total_demand*0.35), math.ceil(total_demand*0.15)]
    total_capacity = sum(data['vehicle_capacities'])
    print(f"\n\nCapacidade total dos veículos -> {total_capacity}\n")
    for capacity, index in enumerate(data['vehicle_capacities']):
        print(f"\nCapacidade do veículo {index + 1} -> {capacity}")

    # Ajustar capacidade dos veículos para acomodar as demandas (veículos com mesma capacidade)
    # average_demand = math.ceil(total_demand / data['num_vehicles'])
    # data['vehicle_capacities'] = [average_demand] * data['num_vehicles']
    
    return data


def plot_graph(locations, routes, title, distance_matrix, save_path, vehicle_capacities):
    """
    Responsável por visualizar as rotas em um grafo:

        -> Criação do grafo com NetworkX.
        
        -> Adição dos nós (clientes e depósito).
        
        -> Desenho das rotas de cada veículo em cores distintas.
        
        -> Destaque do depósito com uma cor específica (amarelo).
    """

    G = nx.DiGraph()
    pos = {i: (locations[i][0], locations[i][1]) for i in range(len(locations))}
    colors = ['r', 'g', 'b', 'c', 'm', 'y']  # Cores para os veículos
    
    # Adicionar nós (clientes e depósito)
    for i in range(len(locations)):
        G.add_node(i)

    # Adicionar arestas com pesos baseados na matriz de distâncias
    for vehicle_id, route in enumerate(routes):
        edges = [(route[i], route[i + 1]) for i in range(len(route) - 1)]
        for edge in edges:
            G.add_edge(edge[0], edge[1], weight=distance_matrix[edge[0]][edge[1]])

    # Desenhar as rotas de cada veículo com cores distintas
    plt.figure(figsize=(10, 8))
    for vehicle_id, route in enumerate(routes):
        edges = [(route[i], route[i + 1]) for i in range(len(route) - 1)]
        nx.draw_networkx_edges(G, pos, edgelist=edges, edge_color=colors[vehicle_id % len(colors)], width=2, alpha=0.7)
        nx.draw_networkx_nodes(G, pos, nodelist=route, node_size=300, node_color=colors[vehicle_id % len(colors)], label=f"Veículo {vehicle_id + 1} ({vehicle_capacities[vehicle_id]})")

    # Adicionar rótulos para as arestas (pesos)
    edge_labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=8, font_color='black')

    # Desenhar os rótulos e o depósito (nó inicial/final)
    nx.draw_networkx_labels(G, pos, font_size=10, font_color='black', font_weight="bold")
    nx.draw_networkx_nodes(G, pos, nodelist=[0], node_size=500, node_color="yellow", label="Depósito")

    # Configurar a legenda e o título
    plt.legend()
    plt.title(title)

    if save_path:
            # Salvar o gráfico no caminho especificado
            plt.savefig(save_path, format='png')
            print(f"Gráfico salvo em: {save_path}")
    else:
        # Mostrar o gráfico
        plt.show()
        
    # Fechar a figura para evitar sobreposição entre gráficos
    plt.close()


def solve_vrp(data, first_solution_strategy, metaheuristic):
    """
    Esta função configura e resolve o problema utilizando o solver do OR-Tools:

        -> Gerenciamento de Índices: RoutingIndexManager associa índices dos nós do solver com as localizações.
        
        -> Callback de Distância: Função que calcula o custo entre dois nós (baseado na matriz de distâncias).
        
        -> Callback de Demanda: Retorna a demanda de cada cliente.
        
        -> Restrição de Capacidade: Garante que a capacidade de cada veículo não seja ultrapassada.
        
        -> Parâmetros de Busca:
            Estratégias de primeira solução: ex. PATH_CHEAPEST_ARC.
            Meta-heurísticas: ex. GUIDED_LOCAL_SEARCH, TABU_SEARCH.
    """

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
    search_parameters.time_limit.seconds = 5  # Limite de tempo de execução

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
    """
    Executa os experimentos:

        -> Gera os dados do problema.
        
        -> Itera por diferentes combinações de estratégias de primeira solução e meta-heurísticas.
        
        -> Resolve o problema para cada combinação e visualiza o resultado.
        
        -> Salva os resultados em um arquivo CSV.
    """

    data = create_data_model()

    strategies = {
        "AUTOMATIC": routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC,
        "PATH_CHEAPEST_ARC": routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC,
        "PATH_MOST_CONSTRAINED_ARC": routing_enums_pb2.FirstSolutionStrategy.PATH_MOST_CONSTRAINED_ARC,
        "SAVINGS": routing_enums_pb2.FirstSolutionStrategy.SAVINGS,
        "CHRISTOFIDES": routing_enums_pb2.FirstSolutionStrategy.CHRISTOFIDES,
        "PARALLEL_CHEAPEST_INSERTION": routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION,
        "SEQUENTIAL_CHEAPEST_INSERTION": routing_enums_pb2.FirstSolutionStrategy.SEQUENTIAL_CHEAPEST_INSERTION,
        "LOCAL_CHEAPEST_INSERTION": routing_enums_pb2.FirstSolutionStrategy.LOCAL_CHEAPEST_INSERTION,
        "LOCAL_CHEAPEST_COST_INSERTION": routing_enums_pb2.FirstSolutionStrategy.LOCAL_CHEAPEST_COST_INSERTION,
        "GLOBAL_CHEAPEST_ARC": routing_enums_pb2.FirstSolutionStrategy.GLOBAL_CHEAPEST_ARC,
        "LOCAL_CHEAPEST_ARC": routing_enums_pb2.FirstSolutionStrategy.LOCAL_CHEAPEST_ARC,
        "FIRST_UNBOUND_MIN_VALUE": routing_enums_pb2.FirstSolutionStrategy.FIRST_UNBOUND_MIN_VALUE
    }
    metaheuristics = {
        "AUTOMATIC": routing_enums_pb2.LocalSearchMetaheuristic.AUTOMATIC,
        "GREEDY_DESCENT": routing_enums_pb2.LocalSearchMetaheuristic.GREEDY_DESCENT,
        "GUIDED_LOCAL_SEARCH": routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH,
        "SIMULATED_ANNEALING": routing_enums_pb2.LocalSearchMetaheuristic.SIMULATED_ANNEALING,
        "TABU_SEARCH": routing_enums_pb2.LocalSearchMetaheuristic.TABU_SEARCH,
        "GENERIC_TABU_SEARCH": routing_enums_pb2.LocalSearchMetaheuristic.GENERIC_TABU_SEARCH
    }

    results = []

    for strategy_name, strategy in strategies.items():
        for meta_name, meta in metaheuristics.items():
            print(f"Testando: {strategy_name} + {meta_name}")
            routes, distance, exec_time = solve_vrp(data, strategy, meta)
            if routes:
                title = f"{strategy_name} + {meta_name}\nDistância: {distance}, Tempo: {exec_time:.2f}s"
                save_path = f"graphs/{strategy_name}_{meta_name}.png"
                plot_graph(data['locations'], routes, title, distance_matrix=data["distance_matrix"], save_path=save_path, vehicle_capacities=data['vehicle_capacities'])
                results.append({
                    "Strategy": strategy_name,
                    "Metaheuristic": meta_name,
                    "Distance": distance,
                    "Execution Time (s)": round(exec_time, 2)
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
