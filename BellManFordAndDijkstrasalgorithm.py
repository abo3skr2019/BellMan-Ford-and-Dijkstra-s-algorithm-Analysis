import heapq
import time
import random
import csv


def relax_edges(graph, distances, previous_nodes):
    for node in graph:
        for neighbor, weight in graph[node].items():
            if (
                distances[node] != float("infinity")
                and distances[node] + weight < distances[neighbor]
            ):
                distances[neighbor] = distances[node] + weight
                previous_nodes[neighbor] = node
    return distances, previous_nodes


def shortest_path(node, previous_nodes):
    path = []
    while node is not None:
        path.append(node)
        node = previous_nodes[node]
    return path[::-1]


def update_distance(distances, previous_nodes, node, neighbor, weight):
    if (
        distances[node] != float("infinity")
        and distances[node] + weight < distances[neighbor]
    ):
        distances[neighbor] = distances[node] + weight
        previous_nodes[neighbor] = node
        return True
    return False


def dijkstra(graph, start):
    distances = {node: float("infinity") for node in graph}
    distances[start] = 0
    previous_nodes = {node: None for node in graph}
    priority_queue = [(0, start)]
    visited = set()

    while priority_queue:
        _, current_node = heapq.heappop(priority_queue)

        if current_node in visited:
            continue

        for neighbor, weight in graph[current_node].items():
            if neighbor not in visited:
                distance = distances[current_node] + weight
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous_nodes[neighbor] = current_node
                    heapq.heappush(priority_queue, (distance, neighbor))

        visited.add(current_node)

    return distances, {node: shortest_path(node, previous_nodes) for node in distances}


NEGATIVE_CYCLE_MESSAGE = "Graph contains a negative-weight cycle"


def bellman_ford(graph, start):
    distances = {node: float("infinity") for node in graph}
    distances[start] = 0
    previous_nodes = {node: None for node in graph}

    for _ in range(len(graph) - 1):
        if not relax_edges(graph, distances, previous_nodes):
            break

    if relax_edges(graph, distances, previous_nodes):
        return NEGATIVE_CYCLE_MESSAGE

    return distances, {node: shortest_path(node, previous_nodes) for node in distances}


def bellman_ford_optimized(graph, start):
    distances = {node: float("infinity") for node in graph}
    distances[start] = 0
    previous_nodes = {node: None for node in graph}

    edges = [(u, v, w) for u in graph for v, w in graph[u].items()]

    for _ in range(len(graph) - 1):
        changes_made = False
        for u, v, w in edges:
            if distances[u] != float("infinity") and distances[u] + w < distances[v]:
                distances[v] = distances[u] + w
                previous_nodes[v] = u
                changes_made = True
        if not changes_made:
            break

    for u, v, w in edges:
        if distances[u] != float("infinity") and distances[u] + w < distances[v]:
            return NEGATIVE_CYCLE_MESSAGE

    return distances, {node: shortest_path(node, previous_nodes) for node in distances}


def bellman_ford_with_end(graph, start, end):
    distances = {node: float("infinity") for node in graph}
    distances[start] = 0
    previous_nodes = {node: None for node in graph}
    end_updated = True

    for _ in range(len(graph) - 1):
        if not end_updated:
            break
        end_updated = update_distances(graph, distances, previous_nodes, end)

    if any(
        distances[node] != float("infinity")
        and distances[node] + weight < distances[neighbor]
        for node in graph
        for neighbor, weight in graph[node].items()
    ):
        return "Graph contains a negative-weight cycle"

    return distances[end], shortest_path(end, previous_nodes)


def update_distances(graph, distances, previous_nodes, end):
    end_updated = False
    for node in graph:
        for neighbor, weight in graph[node].items():
            if (
                distances[node] != float("infinity")
                and distances[node] + weight < distances[neighbor]
            ):
                distances[neighbor] = distances[node] + weight
                previous_nodes[neighbor] = node
                if neighbor == end:
                    end_updated = True
    return end_updated


def bellman_ford_optimized_with_end(graph, start, end):
    distances = {node: float("infinity") for node in graph}
    distances[start] = 0
    previous_nodes = {node: None for node in graph}

    edges = [(u, v, w) for u in graph for v, w in graph[u].items()]

    for _ in range(len(graph) - 1):
        changes_made, end_reached = relax_edges_with_end(edges, distances, previous_nodes, end)
        if not changes_made:
            break
        if end_reached:
            return distances[end], shortest_path(end, previous_nodes)

    if has_negative_cycle(edges, distances):
        return NEGATIVE_CYCLE_MESSAGE

    return distances[end], shortest_path(end, previous_nodes)


def relax_edges_with_end(edges, distances, previous_nodes, end):
    changes_made = False
    end_reached = False
    for u, v, w in edges:
        if distances[u] != float("infinity") and distances[u] + w < distances[v]:
            distances[v] = distances[u] + w
            previous_nodes[v] = u
            changes_made = True
            if v == end:
                end_reached = True
    return changes_made, end_reached


def has_negative_cycle(edges, distances):
    for u, v, w in edges:
        if distances[u] != float("infinity") and distances[u] + w < distances[v]:
            return True
    return False


def dijkstrawithend(graph, start, end):
    distances = {node: float("infinity") for node in graph}
    distances[start] = 0
    previous_nodes = {node: None for node in graph}
    priority_queue = [(0, start)]
    visited = set()

    while priority_queue:
        _, current_node = heapq.heappop(priority_queue)

        if current_node in visited:
            continue

        if current_node == end:
            break

        for neighbor, weight in graph[current_node].items():
            if neighbor not in visited:
                distance = distances[current_node] + weight
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous_nodes[neighbor] = current_node
                    heapq.heappush(priority_queue, (distance, neighbor))

        visited.add(current_node)

    return distances[end], shortest_path(end, previous_nodes)


# Function to generate a random graph
def generate_graph(num_vertices):
    graph = {
        i: {j: random.randint(1, 10) for j in range(num_vertices) if j != i}
        for i in range(num_vertices)
    }
    return graph


# Open New CSV File
# Open New CSV File
with open("Assignment2output.csv", "w", newline="") as file:
    writer = csv.writer(file)
    # Write the header row
    writer.writerow(
        [
            "Number of Vertices",
            "Dijkstra's Algorithm Time",
            "Dijkstra's_with_end Algorithm Time",
            "Bellman-Ford Algorithm Time",
            "Bellman-Ford Algorithm_with_end Time",
            "Bellman-Ford Optimized Algorithm Time",
            "Bellman-Ford Optimized_with_end Algorithm Time",
        ]
    )

    # Test the algorithms with varying sizes of vertices
    for num_vertices in range(5, 205, 5):
        graph = generate_graph(num_vertices)
        start = random.randint(0, num_vertices - 1)
        end = random.randint(0, num_vertices - 1)
        # Ensure the end vertex is not the same as the start vertex
        while end == start:
            end = random.randint(0, num_vertices - 1)

        start_time = time.perf_counter()
        dijkstra(graph, start)
        dijkstra_time = time.perf_counter() - start_time
        print(
            f"Dijkstra's algorithm for {num_vertices} vertices took {dijkstra_time} seconds."
        )

        start_time = time.perf_counter()
        dijkstrawithend(graph, start, end)
        dijkstra_with_end_time = time.perf_counter() - start_time
        print(
            f"Dijkstra's_with_end algorithm for {num_vertices} vertices took {dijkstra_with_end_time} seconds."
        )

        start_time = time.perf_counter()
        bellman_ford(graph, start)
        bellman_ford_time = time.perf_counter() - start_time
        print(
            f"Bellman-Ford algorithm for {num_vertices} vertices took {bellman_ford_time} seconds."
        )

        start_time = time.perf_counter()
        bellman_ford_with_end(graph, start, end)
        bellman_ford_with_end_time = time.perf_counter() - start_time
        print(
            f"Bellman-Ford_with_end algorithm for {num_vertices} vertices took {bellman_ford_with_end_time} seconds."
        )

        start_time = time.perf_counter()
        bellman_ford_optimized(graph, start)
        bellman_ford_optimized_time = time.perf_counter() - start_time
        print(
            f"Bellman-Ford optimized algorithm for {num_vertices} vertices took {bellman_ford_optimized_time} seconds."
        )

        start_time = time.perf_counter()
        bellman_ford_optimized_with_end(graph, start, end)
        bellman_ford_optimized_with_end_time = time.perf_counter() - start_time
        print(
            f"Bellman-Ford optimized_with_end algorithm for {num_vertices} vertices took {bellman_ford_optimized_with_end_time} seconds."
        )

        # Write the data row
        writer.writerow(
            [
                num_vertices,
                dijkstra_time,
                dijkstra_with_end_time,
                bellman_ford_time,
                bellman_ford_with_end_time,
                bellman_ford_optimized_time,
                bellman_ford_optimized_with_end_time,
            ]
        )

print("CSV file has been written successfully.")
