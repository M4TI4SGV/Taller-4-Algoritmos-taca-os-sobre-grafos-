import heapq, math, sys

def ReadOBJ( fname ):
    V = []
    A = {}
    fstr = open(fname)
    for line in fstr:
        d = line.rstrip().lstrip().split(' ')
        if d[0] == 'v':
            V += [tuple([float(c) for c in d[1:]])]

        elif d[0] == 'f':
            I = [int(i.split('/')[0]) - 1 for i in d[1:]]
            for i in range(len(I)):
                s = I[i]
                e = I[(i + 1) % len(I)]
                if not s in A:
                    A[s] = []
                if not e in A:
                    A[e] = []
                A[s] += [e]
                A[e] += [s]
    fstr.close()
    
    print("Vertices:", V)
    print("Aristas:", A)
    
    return (V, A)

def FormatPathAsOBJString( V, P ):
    s = ''
    for i in P:
        s += 'v ' + str(V[i][0]) + ' ' + str(V[i][1]) + ' ' + str(V[i][2]) + '\n'
    s += '\nl'
    for i in range(len(P)):
        s += ' ' + str(i + 1)
    return s

def Distance( a, b ):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(len(a))))

def Kruskal( G ):
    V, A = G
    e = (math.inf, -1, -1)
    for a in A:
        for n in A[a]:
            d = Distance(V[a], V[n])
            if d < e[0]:
                e = (d, a, n)
    Q = [e]
    M = [False for i in range(len(V))]
    T = [i for i in range(len(V))]
    start = True
    while Q:
        d, i, j = heapq.heappop(Q)
        if start:
            M[i] = M[j] = True
            start = False
            for k in A[i]:
                heapq.heappush(Q, (Distance(V[i], V[k]), i, k))
            for k in A[j]:
                heapq.heappush(Q, (Distance(V[j], V[k]), j, k))
        else:
            if M[i] and not M[j]:
                M[j] = True
                T[j] = i
                for k in A[j]:
                    heapq.heappush(Q, (Distance(V[j], V[k]), j, k))
    
    print("Arbol de Kruskal (MST):", T)
    return T

def Dijkstra( G, s ):
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
            alt = dist[u] + Distance(V[u], V[v])
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u
                heapq.heappush(Q, (alt, v))
    
    print("Arbol de Dijkstra (caminos más cortos):", prev)
    return prev

def SpanningTree_Backtrack(T, e):
    P = []
    if e is None:  # Verifica si el nodo es alcanzable
        return P  # Devuelve un camino vacío si no lo es

    j = e
    while T[j] is not None and T[j] != j:  # Asegúrate de que no es None
        P = [j] + P
        j = T[j]
    P = [j] + P  # Agregar el último nodo al camino
    return P


def ChooseMaxPath(G, T):
    V, A = G
    max_path = []
    max_distance = -math.inf

    for i in range(len(V)):
        if T[i] is not None:  # Solo considera los nodos alcanzables
            for j in range(i + 1, len(V)):
                if T[j] is not None:  # Solo considera nodos alcanzables
                    path = SpanningTree_Backtrack(T, j)
                    if len(path) > 1:  # Evita caminos vacíos o de un solo nodo
                        distance = sum(Distance(V[path[k]], V[path[k + 1]]) for k in range(len(path) - 1))
                        if distance > max_distance:
                            max_distance = distance
                            max_path = path

    print("Camino más largo:", max_path, "con distancia:", max_distance)
    return max_path



if __name__ == '__main__':
    G = ReadOBJ('f-16.obj')  # Asegúrate de que el archivo cruiser.obj esté en el mismo directorio
    K = Kruskal(G)
    KP = ChooseMaxPath(G, K)
    KS = FormatPathAsOBJString(G[0], KP)
    
    D = Dijkstra(G, KP[0])
    DP = ChooseMaxPath(G, D)
    DS = FormatPathAsOBJString(G[0], DP)
    
    fstr = open('kruskal.obj', 'w')
    fstr.write(KS)
    fstr.close()
    
    fstr = open('dijkstra.obj', 'w')
    fstr.write(DS)
    fstr.close()
