import heapq, math, sys

# Precalcular las distancias entre nodos para optimizar las consultas
def PrecomputeDistances(V, A):
    distances = {}
    for u in A:
        for v in A[u]:
            distances[(u, v)] = Distance(V[u], V[v])
    return distances

def ReadOBJ(fname):
    V = []
    A = {}
    with open(fname) as fstr:
        for line in fstr:
            d = line.rstrip().lstrip().split(' ')
            if d[0] == 'v':
                V += [tuple([float(c) for c in d[1:]])]

            elif d[0] == 'f':
                I = [int(i.split('/')[0]) - 1 for i in d[1:]]
                for i in range(len(I)):
                    s = I[i]
                    e = I[(i + 1) % len(I)]
                    if s not in A:
                        A[s] = []
                    if e not in A:
                        A[e] = []
                    A[s].append(e)
                    A[e].append(s)
    
    print("Vertices:", V)
    print("Aristas:", A)
    return (V, A)

def FormatPathAsOBJString(V, P):
    s = ''
    for i in P:
        s += f'v {V[i][0]} {V[i][1]} {V[i][2]}\n'
    s += '\nl'
    for i in range(len(P)):
        s += f' {i + 1}'
    return s

def Distance(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(len(a))))

def Kruskal(G):
    V, A = G
    edges = []
    # Crear una lista de todas las aristas con sus distancias
    for u in A:
        for v in A[u]:
            if u < v:  # Asegurar que cada arista se añade una sola vez
                edges.append((Distance(V[u], V[v]), u, v))

    # Ordenar las aristas por distancia
    edges.sort()

    # Estructura para encontrar y unir conjuntos
    parent = list(range(len(V)))
    rank = [0] * len(V)

    def find(u):
        if parent[u] != u:
            parent[u] = find(parent[u])
        return parent[u]

    def union(u, v):
        rootU = find(u)
        rootV = find(v)
        if rootU != rootV:
            if rank[rootU] > rank[rootV]:
                parent[rootV] = rootU
            elif rank[rootU] < rank[rootV]:
                parent[rootU] = rootV
            else:
                parent[rootV] = rootU
                rank[rootU] += 1

    # Crear el MST
    mst = [None] * len(V)  # Aquí cada nodo tendrá su predecesor
    for d, u, v in edges:
        if find(u) != find(v):
            union(u, v)
            mst[v] = u  # v tiene a u como predecesor

    return mst

def Dijkstra(G, s, distances):
    V, A = G
    dist = {i: math.inf for i in range(len(V))}
    dist[s] = 0
    prev = {i: None for i in range(len(V))}
    Q = [(0, s)]
    
    while Q:
        d, u = heapq.heappop(Q)
        if d > dist[u]:
            continue
        for v in A[u]:
            alt = dist[u] + distances[(u, v)]
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u
                heapq.heappush(Q, (alt, v))
    
    print("Árbol de Dijkstra (caminos más cortos):", prev)
    return prev

def SpanningTree_Backtrack(T, e):
    P = []
    if e is None or T[e] is None:  # Verifica si el nodo es alcanzable y si su predecesor es conocido
        return P  # Devuelve un camino vacío si no lo es

    j = e
    # Verifica que no esté en un bucle infinito y que el nodo tenga un predecesor válido
    while j is not None and T[j] != j:
        P = [j] + P
        j = T[j]  # Sigue al predecesor
    if j is not None:
        P = [j] + P  # Agrega el último nodo al camino
    return P


def ChooseMaxPath(G, T, distances):
    V, A = G
    max_path = []
    max_distance = -math.inf

    for i in range(len(V)):
        if T[i] is not None:
            for j in range(i + 1, len(V)):
                if T[j] is not None:
                    path = SpanningTree_Backtrack(T, j)
                    if len(path) > 1:
                        distance = sum(distances[(path[k], path[k + 1])] for k in range(len(path) - 1))
                        if distance > max_distance:
                            max_distance = distance
                            max_path = path

    print("Camino más largo:", max_path, "con distancia:", max_distance)
    return max_path

def process_models(models):
    for model in models:
        print(f"Procesando modelo: {model}")
        G = ReadOBJ(model)
        
        # Precomputar todas las distancias entre nodos conectados
        distances = PrecomputeDistances(*G)
        
        # Kruskal
        K = Kruskal(G)
        KP = ChooseMaxPath(G, K, distances)
        KS = FormatPathAsOBJString(G[0], KP)
        
        # Dijkstra
        D = Dijkstra(G, KP[0], distances)
        DP = ChooseMaxPath(G, D, distances)
        DS = FormatPathAsOBJString(G[0], DP)
        
        # Guardar resultados
        kruskal_file = model.replace('.obj', '_kruskal.obj')
        dijkstra_file = model.replace('.obj', '_dijkstra.obj')
        
        with open(kruskal_file, 'w') as fstr:
            fstr.write(KS)
        
        with open(dijkstra_file, 'w') as fstr:
            fstr.write(DS)

if __name__ == '__main__':
    models = ['Models_3d/f-16.obj', 'Models_3d/cruiser.obj', 'Models_3d/f16.obj']
    process_models(models)
