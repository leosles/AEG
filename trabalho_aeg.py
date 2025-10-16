import heapq
import random
from random import randrange
from collections import deque
import sys

# Função que gera um grafo conexo aleatório usnado o número de arestas de entrada
def gerar_grafo_conexo(num_vertices): 
    if num_vertices < 4:
        raise ValueError("O número de vértices deve ser pelo menos 4")
    
    # Garantir que o grafo seja conexo primeiro criando uma árvore geradora mínima
    arestas = []
    vertices = list(range(num_vertices))
    
    # Começa com um vértice aleatório
    conectados = set([random.choice(vertices)])
    naoconectados = set(vertices) - conectados
    
    # Algoritmo de Prim para garantir conexidade
    while naoconectados:
        u = random.choice(list(conectados))
        v = random.choice(list(naoconectados))

        peso = randrange(1, 21)
        arestas.append((u, v, peso))
        
        conectados.add(v)
        naoconectados.remove(v)
    
    # Adiciona arestas extras aleatorias
    # Número de arestas extras: entre numero_de_vertices - 1 e 2*numero_de_vertices
    max_arestas_extras = min(2 * num_vertices, (num_vertices * (num_vertices - 1)) // 2 - len(arestas))
    num_arestas_extras = random.randint(0, max_arestas_extras)
    
    arestas_possiveis = []
    for u in range(num_vertices):
        for v in range(u + 1, num_vertices):
            # Verifica se a aresta já existe
            existe = any((a == u and b == v) or (a == v and b == u) 
                            for a, b, _ in arestas)
            if not existe:
                arestas_possiveis.append((u, v))
    
    # Adiciona arestas extras aleatórias
    if arestas_possiveis:
        arestas_extras = random.sample(arestas_possiveis, min(num_arestas_extras, len(arestas_possiveis)))

        for u, v in arestas_extras:
            peso = randrange(1, 21)
            arestas.append((u, v, peso))
    
    # Embaralha as arestas
    random.shuffle(arestas)
    
    return len(arestas), arestas

# Função que escreve o arquivo de entrada(usa a função gerar_gravo_conexo)
def criar_arquivo_entrada():
    with open('entrada.txt', 'w') as arquivo:
        n = randrange(50, 100) #Número de arestas

        m, arestas = gerar_grafo_conexo(n) # Número de vértices e lista de vértices, respectivamente

        arquivo.write(str(n) + '\n')
        arquivo.write(str(m) + '\n')
        
        for u, v, w in arestas:
            arquivo.write(f"{u} {v} {w}\n")

        vertices_validos = list(range(1, n + 1)) # Lista de vértices válidos para a escolha de entrada, saída e o vértice que o minotauro começa

        entrada = random.choice(vertices_validos) # Escolhe vértice de entrada
        vertices_validos.remove(entrada)
        arquivo.write(str(entrada) + '\n')

        saida = random.choice(vertices_validos) # Escolhe vértice de saída
        vertices_validos.remove(saida)
        arquivo.write(str(saida) + '\n')

        pos_minotauro = random.choice(vertices_validos) # Escolhe vértice inicial do minotauro
        vertices_validos.remove(pos_minotauro)
        arquivo.write(str(pos_minotauro) + '\n')

        alcance_minotauro = 5 # Valor de distância máxima para que o minotauro detecte o intruso
        arquivo.write(str(alcance_minotauro) + '\n')

        duracao_recursos = randrange(300, 400) # Duração dos recursos do intruso
        arquivo.write(str(duracao_recursos) + '\n')

# Função que lê o arquivo de entrada e retorna os dados do labirinto
def ler_arquivo_entrada(nome_arquivo):
    with open(nome_arquivo, 'r') as f:
        n = int(f.readline().strip()) # Número de vértices
        m = int(f.readline().strip()) # Número de arestas
        
        # Lê lista de arestas
        arestas = []
        for _ in range(m):
            u, v, w = map(int, f.readline().strip().split())
            arestas.append((u, v, w))
        
        entrada = int(f.readline().strip()) # Vértice de entrada
        saida = int(f.readline().strip()) # Vértice de saída
        pos_minotauro = int(f.readline().strip()) # Posição inicial do minotauro
        percepcao_minotauro = int(f.readline().strip()) # Alcance da percepção deo minotauro
        tempo_maximo = int(f.readline().strip()) + 100 # Tempo máximo = duração dos recursos + tempo que consegue sobreviver sem recursos
    
    return n, m, arestas, entrada, saida, pos_minotauro, percepcao_minotauro, tempo_maximo

# Função que cria o grafo como uma lista de adjacências
def construir_grafo(n, arestas):
    grafo = [[] for _ in range(n)]

    for u, v, w in arestas:
        grafo[u].append((v, w))
        grafo[v].append((u, w))

    return grafo

# Função que calcula as menores distâncias da origem para todos os vértices
def dijkstra(grafo, origem):
    n = len(grafo)

    dist = [float('inf')] * n # Lista de todas as distâncias a partir da origem
    
    dist[origem] = 0

    heap = [(0, origem)]
    
    while heap:
        d, u = heapq.heappop(heap)

        if d > dist[u]:
            continue

        for v, w in grafo[u]:
            nova_dist = d + w

            if nova_dist < dist[v]:

                dist[v] = nova_dist

                heapq.heappush(heap, (nova_dist, v))
    
    return dist

# Função que encontra o caminho mínimo entre origem e destino
def caminho_minimo(grafo, origem, destino):
    n = len(grafo)

    dist = [float('inf')] * n # Lista das distâncias

    anterior = [-1] * n
    
    dist[origem] = 0

    heap = [(0, origem)]
    
    while heap:
        d, u = heapq.heappop(heap)

        if u == destino:
            break

        if d > dist[u]:
            continue

        for v, w in grafo[u]:
            nova_dist = d + w

            if nova_dist < dist[v]:
                dist[v] = nova_dist
                anterior[v] = u
                heapq.heappush(heap, (nova_dist, v))
    
    # Reconstrói o caminho
    caminho = []
    u = destino

    while u != -1:
        caminho.append(u)
        u = anterior[u]

    caminho.reverse()
    
    return caminho if caminho[0] == origem else []

# Função para mover o prisioneiro, retorna próximo vértice a ser visitado
def mover_prisioneiro(grafo, vertice_atual, visitados, caminho_percorrido):
    #Estratégia DFS: explora caminhos não visitados 
    
    # Verifica vértices adjacentes não visitados
    for vizinho, _ in grafo[vertice_atual]:
        if vizinho not in visitados:
            return vizinho
    
    # Se todos os vizinhos foram visitados, volta pelo caminho
    if len(caminho_percorrido) > 1:
        return caminho_percorrido[-2]  # Volta para o vértice anterior
    
    # Caso não haja para onde voltar, escolhe aleatoriamente
    vizinhos = [v for v, _ in grafo[vertice_atual]]

    return random.choice(vizinhos) if vizinhos else vertice_atual

# Função que move o minotauro aleatoriamente quando não está perseguindo o prisioneiro, retorn posição
def mover_minotauro_normal(grafo, pos_minotauro):
    vizinhos = [v for v, _ in grafo[pos_minotauro]]

    return random.choice(vizinhos) if vizinhos else pos_minotauro

# Função que move o minotauro em perseguição (2 movimentos por rodada),  retorn posição
def mover_minotauro_perseguicao(grafo, pos_minotauro, pos_prisioneiro, passos_restantes):
    pos_atual = pos_minotauro
    
    for _ in range(min(2, passos_restantes)):
        if pos_atual == pos_prisioneiro:
            break
        
        caminho = caminho_minimo(grafo, pos_atual, pos_prisioneiro)

        if len(caminho) > 1:
            pos_atual = caminho[1]  # Próximo vértice no caminho mínimo

        else:
            break
    
    return pos_atual

def obter_peso_aresta(grafo, u, v):
    """Retorna o peso da aresta entre u e v"""
    for vizinho, peso in grafo[u]:
        if vizinho == v:
            return peso
    return float('inf')  # Se não existe aresta

criar_arquivo_entrada() # Cria um arquivo de entrada com dados aleatórios

n, m, arestas, entrada, saida, pos_minotauro, percepcao_minotauro, tempo_maximo = ler_arquivo_entrada('entrada.txt') # Lê os dados

labirinto = construir_grafo(n, arestas) # Constroi o labirinto

# Inicializa os valores iniciais
pos_prisioneiro = entrada # Posição do prisioneiro
pos_minotauro = pos_minotauro # Posição do minotauro
tempo_restante = tempo_maximo # Tempo que o prisioneiro consegue sobreviver no labirinto com os recursos
visitados_prisioneiro = set([entrada]) # Vértices visitados pelo prisioneiro
caminho_prisioneiro = [entrada] # Caminho do prisioneiro
caminho_minotauro = [pos_minotauro] # Caminho do minotauro
caminho_perseguicao = [] # Caminho que o minotauro perseguiu o prisioneiro (quando acontecer)
perseguindo = False # Se o minotauro está perseguindo o prisioneiro
momento_deteccao = None # Momento em que o minotauro detectou o prisioneiro
momento_alcance = None # Momento em que o minotauro alcançou o prisioneiro
movimentos = 1 # Número de movimentos até o momento

while tempo_restante > 0:
    # Verifica se o prisioneiro chegou na saída
    if pos_prisioneiro == saida:
        break

    # Verifica se o prisioneiro encontrou o minotauro
    if pos_prisioneiro == pos_minotauro:
        # Verifica se o prisioneiro consegue escapar
        if random.random() > 0.01:  # 99% de chance de não conseguir fugir
            break

    # Calcula a distância entre o minotauro e o prisioneiro
    dist_minotauro = dijkstra(labirinto, pos_minotauro)
    distancia_atual = dist_minotauro[pos_prisioneiro]

    # Verifica se o prisioneiro está no alcance de detecção do minotauro
    if perseguindo == False and distancia_atual <= percepcao_minotauro:
        perseguindo = True

        momento_deteccao = movimentos

        caminho_perseguicao = [pos_minotauro]

    # Mover o prisioneiro (1 movimento por rodada)
    proximo_vertice_prisioneiro = mover_prisioneiro(labirinto, pos_prisioneiro, visitados_prisioneiro, caminho_prisioneiro)

    tempo_restante -= obter_peso_aresta(labirinto, pos_prisioneiro, proximo_vertice_prisioneiro)

    pos_prisioneiro = proximo_vertice_prisioneiro

    if not pos_prisioneiro in visitados_prisioneiro:
        visitados_prisioneiro.add(pos_prisioneiro)
    
    caminho_prisioneiro.append(pos_prisioneiro)

    # Mover o minotauro
    if perseguindo:
        # Perseguição: move 2 vértices
        nova_pos_minotauro = mover_minotauro_perseguicao(labirinto, pos_minotauro, pos_prisioneiro, 2)

        if nova_pos_minotauro != pos_minotauro:
            caminho_perseguicao.append(nova_pos_minotauro)

        pos_minotauro = nova_pos_minotauro

    else:
        # Movimento normal: move 1 vértice
        pos_minotauro = mover_minotauro_normal(labirinto, pos_minotauro)
        caminho_minotauro.append(pos_minotauro)

    # Verifica se o minotauro alcançou o prisioneiro
    if pos_prisioneiro == pos_minotauro and not momento_alcance:
        momento_alcance = movimentos

    movimentos += 1

print(f"Vértice de entrada: {entrada}")
print(f"Vértice de saída: {saida}")
print("")

# Resultado final
if pos_prisioneiro == saida:
    print("RESULTADO: Prisioneiro ESCAPOU com sucesso!")

elif pos_prisioneiro == pos_minotauro:
    print("RESULTADO: Prisioneiro foi ELIMINADO pelo Minotauro")

elif tempo_restante <= 0:
    print("RESULTADO: Tempo esgotou - Prisioneiro não conseguiu escapar")

else:
    print("RESULTADO: Simulação interrompida")
    
print(f"Tempo restante: {max(0, tempo_restante)}")
    
# Caminho do prisioneiro
print(f"\nCaminho percorrido pelo prisioneiro ({len(caminho_prisioneiro)} vértices):")
print(" -> ".join(map(str, caminho_prisioneiro)))
    
# Informações sobre perseguição
if perseguindo:
    print(f"\nDetecção ocorreu no momento: {momento_deteccao}")

    if momento_alcance:
        print(f"O prisioneiro foi alcançado no momento: {momento_alcance}")

    print(f"Caminho do Minotauro durante perseguição ({len(caminho_perseguicao)} vértices):")
    print(" -> ".join(map(str, caminho_perseguicao)))

else:
    print("\nMinotauro não detectou o prisioneiro durante a simulação")
    
# Estatísticas
print(f"\nESTATÍSTICAS:")
print(f"- Quantidade de vértices visitados pelo prisioneiro: {len(visitados_prisioneiro)}/{n}")
print(f"- Total de movimentos: {movimentos}")
print(f"- Perseguição ocorreu: {'SIM' if perseguindo else 'NÃO'}")