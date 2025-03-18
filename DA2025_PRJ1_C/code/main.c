#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

#define MAX_NODES 100

int graph[MAX_NODES][MAX_NODES];
int num_nodes = 0;

// Initialize the graph
void initialize_graph() {
    for (int i = 0; i < MAX_NODES; i++) {
        for (int j = 0; j < MAX_NODES; j++) {
            graph[i][j] = (i == j) ? 0 : INT_MAX;
        }
    }
}

// Add an edge to the graph
void add_edge(int from, int to, int driving_time) {
    graph[from][to] = driving_time;
    graph[to][from] = driving_time;
    if (from >= num_nodes) num_nodes = from + 1;
    if (to >= num_nodes) num_nodes = to + 1;
}

// Dijkstra's algorithm to find the shortest path
int dijkstra(int start, int end, int *path, int *total_time) {
    int dist[MAX_NODES], visited[MAX_NODES], prev[MAX_NODES];
    for (int i = 0; i < MAX_NODES; i++) {
        dist[i] = INT_MAX;
        visited[i] = 0;
        prev[i] = -1;
    }
    dist[start] = 0;

    for (int i = 0; i < num_nodes; i++) {
        int min_dist = INT_MAX, min_index = -1;
        for (int j = 0; j < num_nodes; j++) {
            if (!visited[j] && dist[j] < min_dist) {
                min_dist = dist[j];
                min_index = j;
            }
        }
        if (min_index == -1) break;

        visited[min_index] = 1;
        for (int j = 0; j < num_nodes; j++) {
            if (graph[min_index][j] != INT_MAX && dist[j] > dist[min_index] + graph[min_index][j]) {
                dist[j] = dist[min_index] + graph[min_index][j];
                prev[j] = min_index;
            }
        }
    }

    if (dist[end] == INT_MAX) {
        return 0; // No path found
    }

    *total_time = dist[end];
    int idx = 0, current = end;
    while (current != -1) {
        path[idx++] = current;
        current = prev[current];
    }
    // Reverse the path to get the correct order
    for (int i = 0; i < idx / 2; i++) {
        int temp = path[i];
        path[i] = path[idx - i - 1];
        path[idx - i - 1] = temp;
    }
    // Mark the end of the path with -1
    path[idx] = -1;
    return 1; // Path found
}

// Remove intermediate nodes and segments from the best route
void remove_intermediate_nodes_and_segments(int *path) {
    for (int i = 1; path[i] != -1 && path[i + 1] != -1; i++) {
        int node = path[i];
        // Remove all segments connected to the intermediate node
        for (int j = 0; j < MAX_NODES; j++) {
            graph[node][j] = INT_MAX;
            graph[j][node] = INT_MAX;
        }
    }
    // Remove the segments used in the best route
    for (int i = 0; path[i] != -1 && path[i + 1] != -1; i++) {
        int from = path[i];
        int to = path[i + 1];
        graph[from][to] = INT_MAX;
        graph[to][from] = INT_MAX;
    }
}

// Exclude specific nodes from the graph
void exclude_nodes(int *avoid_nodes, int count) {
    for (int i = 0; i < count; i++) {
        int node = avoid_nodes[i];
        for (int j = 0; j < MAX_NODES; j++) {
            graph[node][j] = INT_MAX;
            graph[j][node] = INT_MAX;
        }
    }
}

// Exclude specific segments from the graph
void exclude_segments(int (*avoid_segments)[2], int count) {
    for (int i = 0; i < count; i++) {
        int from = avoid_segments[i][0];
        int to = avoid_segments[i][1];
        graph[from][to] = INT_MAX;
        graph[to][from] = INT_MAX;
    }
}

// Ensure the route includes a specific node
int include_node(int start, int end, int include_node, int *path, int *total_time) {
    int path1[MAX_NODES], path2[MAX_NODES];
    int time1, time2;

    // Find the path from start to include_node
    int found1 = dijkstra(start, include_node, path1, &time1);
    // Find the path from include_node to end
    int found2 = dijkstra(include_node, end, path2, &time2);

    if (!found1 || !found2) {
        return 0; // No valid route
    }

    // Combine the two paths
    int idx = 0;
    for (int i = 0; path1[i] != -1; i++) {
        path[idx++] = path1[i];
    }
    for (int i = 1; path2[i] != -1; i++) { // Start from 1 to avoid duplicating include_node
        path[idx++] = path2[i];
    }
    path[idx] = -1;

    *total_time = time1 + time2;
    return 1; // Valid route found
}

// Load graph data from a CSV file
void load_graph(const char *filename) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        printf("Error opening file %s\n", filename);
        return;
    }

    char line[100];
    fgets(line, sizeof(line), file); // Skip the CSV header
    while (fgets(line, sizeof(line), file)) {
        int from, to, driving;
        sscanf(line, "%d,%d,%d", &from, &to, &driving);
        add_edge(from, to, driving);
    }
    fclose(file);
}

// Parse AvoidNodes string
void parse_avoid_nodes(char *str, int *avoid_nodes, int *count) {
    *count = 0;
    char *token = strtok(str, ",");
    while (token != NULL) {
        avoid_nodes[(*count)++] = atoi(token);
        token = strtok(NULL, ",");
    }
}

// Parse AvoidSegments string
void parse_avoid_segments(char *str, int (*avoid_segments)[2], int *count) {
    *count = 0;
    char *token = strtok(str, "(),");
    while (token != NULL) {
        int from = atoi(token);
        token = strtok(NULL, "(),");
        int to = atoi(token);
        avoid_segments[(*count)][0] = from;
        avoid_segments[(*count)][1] = to;
        (*count)++;
        token = strtok(NULL, "(),");
    }
}

int main() {
    initialize_graph();
    load_graph("data/Distances.csv");

    // Open input.txt for reading
    FILE *input_file = fopen("input.txt", "r");
    if (!input_file) {
        printf("Error opening input.txt\n");
        return 1;
    }

    // Read values from input.txt
    char mode[20], avoid_nodes_str[100] = "", avoid_segments_str[100] = "", include_node_str[20] = "";
    int start, end, include_node_id = -1;

    // Read the entire line and parse manually
    char line[256];
    while (fgets(line, sizeof(line), input_file)) {
        if (strstr(line, "Mode:") == line) {
            sscanf(line, "Mode:%19s", mode);
        } else if (strstr(line, "Source:") == line) {
            sscanf(line, "Source:%d", &start);
        } else if (strstr(line, "Destination:") == line) {
            sscanf(line, "Destination:%d", &end);
        } else if (strstr(line, "AvoidNodes:") == line) {
            sscanf(line, "AvoidNodes:%99[^\n]", avoid_nodes_str);
        } else if (strstr(line, "AvoidSegments:") == line) {
            sscanf(line, "AvoidSegments:%99[^\n]", avoid_segments_str);
        } else if (strstr(line, "IncludeNode:") == line) {
            sscanf(line, "IncludeNode:%19[^\n]", include_node_str);
        }
    }
    fclose(input_file);

    // Debug: Print parsed values
    printf("Mode: %s\n", mode);
    printf("Source: %d\n", start);
    printf("Destination: %d\n", end);
    printf("AvoidNodes: %s\n", avoid_nodes_str);
    printf("AvoidSegments: %s\n", avoid_segments_str);
    printf("IncludeNode: %s\n", include_node_str);

    // Check if AvoidNodes, AvoidSegments, and IncludeNode are provided
    int has_avoid_nodes = (strlen(avoid_nodes_str) > 0 && strcmp(avoid_nodes_str, "none") != 0);
    int has_avoid_segments = (strlen(avoid_segments_str) > 0 && strcmp(avoid_segments_str, "none") != 0);
    int has_include_node = (strlen(include_node_str) > 0 && strcmp(include_node_str, "none") != 0);

    // Debug: Print flags
    printf("has_avoid_nodes: %d\n", has_avoid_nodes);
    printf("has_avoid_segments: %d\n", has_avoid_segments);
    printf("has_include_node: %d\n", has_include_node);

    // Process AvoidNodes
    int avoid_nodes[MAX_NODES], avoid_nodes_count = 0;
    if (has_avoid_nodes) {
        parse_avoid_nodes(avoid_nodes_str, avoid_nodes, &avoid_nodes_count);
        exclude_nodes(avoid_nodes, avoid_nodes_count);
    }

    // Process AvoidSegments
    int avoid_segments[MAX_NODES][2], avoid_segments_count = 0;
    if (has_avoid_segments) {
        parse_avoid_segments(avoid_segments_str, avoid_segments, &avoid_segments_count);
        exclude_segments(avoid_segments, avoid_segments_count);
    }

    // Process IncludeNode
    if (has_include_node) {
        include_node_id = atoi(include_node_str);
    }

    // Debug: Print IncludeNode ID
    printf("IncludeNode ID: %d\n", include_node_id);

    // Compute the restricted route or the two best routes
    int best_path[MAX_NODES], alternative_path[MAX_NODES];
    for (int i = 0; i < MAX_NODES; i++) {
        best_path[i] = -1;
        alternative_path[i] = -1;
    }
    int best_time, alternative_time;
    int best_route_exists, alternative_route_exists;

    if (has_avoid_nodes || has_avoid_segments || has_include_node) {
        // Case 2: Restrictions provided
        printf("Smeagol\n"); // Debug: Ensure this is reached
        if (include_node_id != -1) {
            // Route must include a specific node
            best_route_exists = include_node(start, end, include_node_id, best_path, &best_time);
        } else {
            // Route without additional restrictions
            best_route_exists = dijkstra(start, end, best_path, &best_time);
        }

        // Write the result to output.txt
        FILE *output_file = fopen("output.txt", "w");
        if (!output_file) {
            printf("Error creating output.txt\n");
            return 1;
        }

        fprintf(output_file, "Source:%d\nDestination:%d\n", start, end);

        if (best_route_exists) {
            fprintf(output_file, "RestrictedDrivingRoute:");
            for (int i = 0; i < MAX_NODES; i++) {
                if (best_path[i] == -1) break;
                fprintf(output_file, "%d,", best_path[i]);
            }
            fprintf(output_file, "(%d)\n", best_time);
        } else {
            fprintf(output_file, "RestrictedDrivingRoute:none\n");
        }

        fclose(output_file);
    } else {
        // Case 1: Only Mode, Source, and Destination provided
        best_route_exists = dijkstra(start, end, best_path, &best_time);

        // Remove intermediate nodes and segments to compute the alternative route
        int temp_graph[MAX_NODES][MAX_NODES];
        memcpy(temp_graph, graph, sizeof(graph)); // Save the original graph
        remove_intermediate_nodes_and_segments(best_path);

        alternative_route_exists = dijkstra(start, end, alternative_path, &alternative_time);

        // Restore the original graph
        memcpy(graph, temp_graph, sizeof(graph));

        // Write the result to output.txt
        FILE *output_file = fopen("output.txt", "w");
        if (!output_file) {
            printf("Error creating output.txt\n");
            return 1;
        }

        fprintf(output_file, "Source:%d\nDestination:%d\n", start, end);

        if (best_route_exists) {
            fprintf(output_file, "BestDrivingRoute:");
            for (int i = 0; i < MAX_NODES; i++) {
                if (best_path[i] == -1) break;
                fprintf(output_file, "%d,", best_path[i]);
            }
            fprintf(output_file, "(%d)\n", best_time);
        } else {
            fprintf(output_file, "BestDrivingRoute:none\n");
        }

        if (alternative_route_exists) {
            fprintf(output_file, "AlternativeDrivingRoute:");
            for (int i = 0; i < MAX_NODES; i++) {
                if (alternative_path[i] == -1) break;
                fprintf(output_file, "%d,", alternative_path[i]);
            }
            fprintf(output_file, "(%d)\n", alternative_time);
        } else {
            fprintf(output_file, "AlternativeDrivingRoute:none\n");
        }

        fclose(output_file);
    }

    printf("Result saved to output.txt\n");

    return 0;
}