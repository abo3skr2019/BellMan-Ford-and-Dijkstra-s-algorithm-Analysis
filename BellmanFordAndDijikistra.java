import java.util.*;

class Graph {
    private Map<Integer, Map<Integer, Integer>> graph = new HashMap<>();

    public void addEdge(int node, int neighbor, int weight) {
        graph.putIfAbsent(node, new HashMap<>());
        graph.get(node).put(neighbor, weight);
    }

    public Map<Integer, Integer> getNeighbors(int node) {
        return graph.get(node);
    }

    public Set<Integer> getNodes() {
        return graph.keySet();
    }
}

class Main {
    private static final int INFINITY = Integer.MAX_VALUE;
    private static final String NEGATIVE_CYCLE_MESSAGE = "Graph contains a negative-weight cycle";

    public static boolean relaxEdges(Graph graph, Map<Integer, Integer> distances, Map<Integer, Integer> previousNodes) {
        boolean anyEdgeRelaxed = false;
        for (int node : graph.getNodes()) {
            for (Map.Entry<Integer, Integer> entry : graph.getNeighbors(node).entrySet()) {
                int neighbor = entry.getKey();
                int weight = entry.getValue();
                if (distances.get(node) != INFINITY && distances.get(node) + weight < distances.get(neighbor)) {
                    distances.put(neighbor, distances.get(node) + weight);
                    previousNodes.put(neighbor, node);
                    anyEdgeRelaxed = true;
                }
            }
        }
        return anyEdgeRelaxed;
    }

    public static List<Integer> shortestPath(int node, Map<Integer, Integer> previousNodes) {
        List<Integer> path = new ArrayList<>();
        while (node != -1) {
            path.add(node);
            node = previousNodes.get(node);
        }
        Collections.reverse(path);
        return path;
    }

    public static boolean updateDistance(Map<Integer, Integer> distances, Map<Integer, Integer> previousNodes, int node, int neighbor, int weight) {
        if (distances.get(node) != INFINITY && distances.get(node) + weight < distances.get(neighbor)) {
            distances.put(neighbor, distances.get(node) + weight);
            previousNodes.put(neighbor, node);
            return true;
        }
        return false;
    }

    // ... other methods ...

    public static Graph generateGraph(int numVertices) {
        Graph graph = new Graph();
        Random random = new Random();
        for (int i = 0; i < numVertices; i++) {
            for (int j = 0; j < numVertices; j++) {
                if (i != j) {
                    graph.addEdge(i, j, random.nextInt(10) + 1);
                }
            }
        }
        return graph;
    }

    public static Map<Integer, Integer> bellmanFord(Graph graph, int start) {
        Map<Integer, Integer> distances = new HashMap<>();
        Map<Integer, Integer> previousNodes = new HashMap<>();

        for (int node : graph.getNodes()) {
            distances.put(node, INFINITY);
            previousNodes.put(node, -1);
        }
        distances.put(start, 0);

        for (int i = 0; i < graph.getNodes().size() - 1; i++) {
            relaxEdges(graph, distances, previousNodes);
        }

        for (int node : graph.getNodes()) {
            for (Map.Entry<Integer, Integer> entry : graph.getNeighbors(node).entrySet()) {
                int neighbor = entry.getKey();
                int weight = entry.getValue();
                if (distances.get(node) != INFINITY && distances.get(node) + weight < distances.get(neighbor)) {
                    throw new RuntimeException(NEGATIVE_CYCLE_MESSAGE);
                }
            }
        }

        return distances;
    }

    public static Map<Integer, Integer> dijkstra(Graph graph, int start) {
        Map<Integer, Integer> distances = new HashMap<>();
        Map<Integer, Integer> previousNodes = new HashMap<>();
        PriorityQueue<Integer> queue = new PriorityQueue<>(Comparator.comparingInt(distances::get));

        for (int node : graph.getNodes()) {
            distances.put(node, INFINITY);
            previousNodes.put(node, -1);
        }
        distances.put(start, 0);
        queue.add(start);

        while (!queue.isEmpty()) {
            int node = queue.poll();
            for (Map.Entry<Integer, Integer> entry : graph.getNeighbors(node).entrySet()) {
                int neighbor = entry.getKey();
                int weight = entry.getValue();
                if (updateDistance(distances, previousNodes, node, neighbor, weight)) {
                    queue.remove(neighbor);
                    queue.add(neighbor);
                }
            }
        }

        return distances;
    }
    public static Map<Integer, Integer> bellmanFordWithEnd(Graph graph, int start, int end) {
        Map<Integer, Integer> distances = new HashMap<>();
        Map<Integer, Integer> previousNodes = new HashMap<>();
    
        for (int node : graph.getNodes()) {
            distances.put(node, INFINITY);
            previousNodes.put(node, -1);
        }
        distances.put(start, 0);
    
        for (int i = 0; i < graph.getNodes().size() - 1; i++) {
            if (!relaxEdges(graph, distances, previousNodes)) {
                break;
            }
        }

        for (int node : graph.getNodes()) {
            for (Map.Entry<Integer, Integer> entry : graph.getNeighbors(node).entrySet()) {
                int neighbor = entry.getKey();
                int weight = entry.getValue();
                if (distances.get(node) != INFINITY && distances.get(node) + weight < distances.get(neighbor)) {
                    throw new RuntimeException(NEGATIVE_CYCLE_MESSAGE);
                }
            }
        }
    
        return distances.containsKey(end) ? Collections.singletonMap(end, distances.get(end)) : Collections.emptyMap();
    }
    
    public static Map<Integer, Integer> dijkstraWithEnd(Graph graph, int start, int end) {
        Map<Integer, Integer> distances = new HashMap<>();
        Map<Integer, Integer> previousNodes = new HashMap<>();
        PriorityQueue<Integer> queue = new PriorityQueue<>(Comparator.comparingInt(distances::get));
    
        for (int node : graph.getNodes()) {
            distances.put(node, INFINITY);
            previousNodes.put(node, -1);
        }
        distances.put(start, 0);
        queue.add(start);
    
        while (!queue.isEmpty()) {
            int node = queue.poll();
            if (node == end) {
                break;
            }
            for (Map.Entry<Integer, Integer> entry : graph.getNeighbors(node).entrySet()) {
                int neighbor = entry.getKey();
                int weight = entry.getValue();
                if (updateDistance(distances, previousNodes, node, neighbor, weight)) {
                    queue.remove(neighbor);
                    queue.add(neighbor);
                }
            }
        }
    
        return distances.containsKey(end) ? Collections.singletonMap(end, distances.get(end)) : Collections.emptyMap();
    }

    public static void main(String[] args) {
        Graph graph = generateGraph(5);
        int start = 0;
        Map<Integer, Integer> distances = bellmanFord(graph, start);
        System.out.println("Bellman-Ford distances: " + distances);
        distances = dijkstra(graph, start);
        System.out.println("Dijkstra's distances: " + distances);
    }}