class Graph:
    def __init__(self, vertices):
        # Set up the graph specifying the total number of vertices
        self.V = vertices
        self.graph = [[] for _ in range(vertices)]
    
    def add_edge(self, u, v, w):
        # Add an edge with weight from vertex u to vertex v
        self.graph[u].append((v, w))

def dijkstra(graph, src):
    V = graph.V
    dist = [float("inf")] * V
    dist[src] = 0
    visited = [False] * V

    for _ in range(V):
        # Identify the closest vertex that has not yet been visited
        u = min((dist[i], i) for i in range(V) if not visited[i])[1]
        visited[u] = True

        # Update the distance for each neighbor
        for v, w in graph.graph[u]:
            if not visited[v] and dist[u] + w < dist[v]:
                dist[v] = dist[u] + w

    return dist

def bellman_ford(graph, src):
    V = graph.V
    dist = [float("inf")] * V
    dist[src] = 0

    for _ in range(V - 1):
        for u in range(V):
            for v, w in graph.graph[u]:
                if dist[u] != float("inf") and dist[u] + w < dist[v]:
                    dist[v] = dist[u] + w

    for u in range(V):
        for v, w in graph.graph[u]:
            if dist[u] != float("inf") and dist[u] + w < dist[v]:
                raise ValueError("Graph contains a negative weight cycle")

    return dist

def main():
    # User input for the number of vertices and edges
    vertices = int(input("Enter the number of vertices: "))
    g = Graph(vertices)
    
    edges = int(input("Enter the number of edges: "))
    for _ in range(edges):
        u, v, w = map(int, input("Enter edge (u v w): ").split())
        g.add_edge(u, v, w)

    # User selection of algorithm
    algorithm = input("Choose the algorithm (1 for Dijkstra/2 for Bellman-Ford): ").strip()
    src = int(input("Enter the source vertex: "))

    # Execute the selected algorithm
    if algorithm == "1":
        print("Dijkstra's algorithm: ", dijkstra(g, src))
    elif algorithm == "2":
        try:
            print("Bellman-Ford algorithm: ", bellman_ford(g, src))
        except ValueError as e:
            print(e)
    else:
        print("Invalid algorithm selected.")

if __name__ == "__main__":
    main()

input("Press Enter to exit")