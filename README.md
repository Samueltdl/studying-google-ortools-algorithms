# studying-google-ortools-algorithms
Estudando as diferentes estratégias e meta-heurísticas da biblioteca OR-Tools da Google com foco nos problemas de roteamento de veículos (VRP).

## 1. Objetivo
O código resolve o problema de roteamento de veículos (VRP - Vehicle Routing Problem) com restrições de capacidade utilizando a biblioteca Google OR-Tools. O objetivo é minimizar a distância total percorrida pelos veículos ao visitar clientes, respeitando as seguintes condições:

- Cada cliente é visitado exatamente uma vez.

- Cada veículo tem uma capacidade máxima de carga.

- As demandas dos clientes devem ser atendidas sem exceder a capacidade dos veículos.

- Além disso, o código implementa visualização gráfica das rotas geradas utilizando as bibliotecas NetworkX e Matplotlib, facilitando a análise dos resultados.

## 2. Requisitos
- Python 3.x

Bibliotecas:
- ortools
- networkx
- matplotlib
- pandas

Instalação:

    pip install -r requirements.txt

## 3. Estrutura do Código
O código é organizado em cinco partes principais:

- Geração dos Dados:

- Criação das coordenadas de clientes.

- Geração da matriz de distâncias.

- Definição do número de veículos, capacidade dos veículos e demandas dos clientes.

Solução do Problema (OR-Tools):

- Configuração do modelo de roteamento.
- Implementação de callbacks de distância e demanda.
- Adição de restrições de capacidade.
- Configuração das estratégias de solução e meta-heurísticas.

Visualização das Rotas:

- Geração de um grafo direcionado representando as rotas.
- Desenho dos nós (clientes e depósito) e arestas (rotas percorridas) com cores distintas para cada veículo.

Execução dos Experimentos:

- Testes com diferentes combinações de estratégias de primeira solução e meta-heurísticas.
- Armazenamento e exportação dos resultados.

Exportação dos Resultados:

- Salvamento das métricas em um arquivo CSV para facilitar a análise posterior.

## 4. Estratégias Testadas

### 4.1. Estratégias de Primeira Solução

Define o método inicial de geração de rotas:

- PATH_CHEAPEST_ARC: Adiciona a aresta com menor custo.
- PARALLEL_CHEAPEST_INSERTION: Insere os clientes de forma paralela e econômica.
- SAVINGS: Aplica o algoritmo de Savings.
- CHRISTOFIDES: Solução baseada no Algoritmo de Christofides.

### 4.2. Meta-heurísticas

Refinam as soluções iniciais encontradas:

- GUIDED_LOCAL_SEARCH: Guia o solver para áreas promissoras da solução.
- TABU_SEARCH: Explora o espaço de busca impedindo repetições (tabu).
- SIMULATED_ANNEALING: Técnica probabilística baseada no recozimento.

## 5. Resultado Esperado

Gráficos: 
- Visualização das rotas para cada combinação de estratégia + meta-heurística.

CSV: 
- Estratégia de primeira solução.
- Meta-heurística utilizada.
- Distância total percorrida.
- Tempo de execução.

## 6. Exemplos de Saída Gráfica

- Cada rota de veículo é exibida em uma cor distinta.

- O depósito é destacado em amarelo.

- As rotas são visualizadas em um plano cartesiano.
