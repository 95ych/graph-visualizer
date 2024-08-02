#include "raylib.h"
#include "raymath.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MAX_NODES 100
#define MAX_NAME_LENGTH 20
#define ITERATIONS 10000          // Number of iterations for layout
#define SPRING_LENGTH 250.0f      // Desired length of springs (edges)
#define SPRING_CONSTANT 0.6f      // Spring constant
#define REPULSION_CONSTANT 10.0f  // Repulsion constant
#define DAMPING 0.5f              // Damping factor (0 to 1)
#define MIN_DISTANCE 20.0f        // Minimum distance between nodes
#define NODE_RADIUS 20.0f         // Radius of the node circle

typedef struct Node
{
    char    name[MAX_NAME_LENGTH];
    Color   color;
    int     neighbors[MAX_NODES];
    int     neighbor_count;
    bool    visited;
    Vector2 position;
    Vector2 velocity;
} Node;

Node nodes[MAX_NODES];
int  node_count      = 0;
int  traversal_steps = 0;

void addNode(const char* name)
{
    if (node_count < MAX_NODES)
    {
        Node* node = &nodes[node_count];
        strncpy(node->name, name, MAX_NAME_LENGTH - 1);
        node->name[MAX_NAME_LENGTH - 1] = '\0';
        node->color                     = GREEN;
        node->neighbor_count            = 0;
        node->position = (Vector2){ GetRandomValue(50, 750), GetRandomValue(50, 550) };
        node->velocity = (Vector2){ 0, 0 };
        node_count++;
    }
}

void addEdge(int node1, int node2)
{
    if (node1 < node_count && node2 < node_count)
    {
        nodes[node1].neighbors[nodes[node1].neighbor_count++] = node2;
        nodes[node2].neighbors[nodes[node2].neighbor_count++] = node1;
    }
}

Vector2 calculateSpringForce(Vector2 pos1, Vector2 pos2)
{
    Vector2 diff     = Vector2Subtract(pos2, pos1);
    float   distance = Vector2Length(diff);
    float   force    = SPRING_CONSTANT * (distance - SPRING_LENGTH);
    return Vector2Scale(Vector2Normalize(diff), force);
}


static Vector2 calculateRepulsionForce(Vector2 pos1, Vector2 pos2)
{
    Vector2 diff     = Vector2Subtract(pos1, pos2);
    float   distance = Vector2Length(diff);

    if (distance < 0.1f)
    {  // Prevent division by zero
        return (Vector2){ GetRandomValue(-1, 1), GetRandomValue(-1, 1) };
    }

    float force;
    if (distance < MIN_DISTANCE)
    {
        // Strong repulsion when nodes are too close
        force = REPULSION_CONSTANT * (MIN_DISTANCE - distance) / distance;
    }
    else
    {
        // Normal repulsion
        force = REPULSION_CONSTANT / (distance * distance);
    }

    return Vector2Scale(Vector2Normalize(diff), force);
}

void calculateLayout()
{
    for (int iter = 0; iter < ITERATIONS; iter++)
    {
        for (int i = 0; i < node_count; i++)
        {
            Vector2 totalForce = { 0, 0 };

            for (int j = 0; j < nodes[i].neighbor_count; j++)
            {
                int     neighbor = nodes[i].neighbors[j];
                Vector2 springForce
                    = calculateSpringForce(nodes[i].position, nodes[neighbor].position);
                totalForce = Vector2Add(totalForce, springForce);
            }

            for (int j = 0; j < node_count; j++)
            {
                if (i != j)
                {
                    Vector2 repulsionForce
                        = calculateRepulsionForce(nodes[i].position, nodes[j].position);
                    totalForce = Vector2Add(totalForce, repulsionForce);
                }
            }

            nodes[i].velocity = Vector2Scale(Vector2Add(nodes[i].velocity, totalForce), DAMPING);
            nodes[i].position = Vector2Add(nodes[i].position, nodes[i].velocity);

            // Keep nodes within the window
            nodes[i].position.x = Clamp(nodes[i].position.x, NODE_RADIUS, 800 - NODE_RADIUS);
            nodes[i].position.y = Clamp(nodes[i].position.y, NODE_RADIUS, 600 - NODE_RADIUS);
        }
    }
}

void drawGraph()
{
    for (int i = 0; i < node_count; i++)
    {
        const Node* node = &nodes[i];

        // Draw edges
        for (int j = 0; j < node->neighbor_count; j++)
        {
            int neighbor = node->neighbors[j];
            DrawLineV(node->position, nodes[neighbor].position, GRAY);
        }
    }

    for (int i = 0; i < node_count; i++)
    {
        const Node* node = &nodes[i];

        // Draw nodes
        DrawCircleV(node->position, 20, node->color);
        DrawText(node->name, node->position.x - 10, node->position.y - 10, 20, BLACK);
    }
}


// Queue for BFS
typedef struct
{
    int items[MAX_NODES];
    int front;
    int rear;
} Queue;

static Queue bfs_queue;
static int   bfs_order = 0;

static void initQueue(Queue* q)
{
    q->front = -1;
    q->rear  = -1;
}

static int isEmpty(Queue* q) { return q->front == -1; }

static void enqueue(Queue* q, int value)
{
    if (q->rear == MAX_NODES - 1)
        return;
    if (q->front == -1)
        q->front = 0;
    q->rear++;
    q->items[q->rear] = value;
}

static int dequeue(Queue* q)
{
    if (isEmpty(q))
        return -1;
    int item = q->items[q->front];
    q->front++;
    if (q->front > q->rear)
        initQueue(q);
    return item;
}

void initBFS(int start_node)
{
    initQueue(&bfs_queue);
    bfs_order = 0;

    for (int i = 0; i < node_count; i++)
    {
        // nodes[i].bfs_level = -1;
        // nodes[i].bfs_order = -1;
        nodes[i].visited = false;
    }

    enqueue(&bfs_queue, start_node);
    nodes[start_node].visited = true;
}

bool stepBFS()
{
    if (isEmpty(&bfs_queue))
    {
        return false;
    }

    int current_node = dequeue(&bfs_queue);

    for (int i = 0; i < nodes[current_node].neighbor_count; i++)
    {
        int neighbor = nodes[current_node].neighbors[i];
        if (!nodes[neighbor].visited)
        {
            enqueue(&bfs_queue, neighbor);
            nodes[neighbor].visited = true;
        }
    }

    nodes[current_node].color = BLUE;

    return true;  // We've processed the current node's neighbors
}
int main()
{
    InitWindow(800, 600, "Graph Visualization ");
    SetTargetFPS(60);

    // Add nodes
    addNode("0");
    addNode("1");
    addNode("2");
    addNode("3");
    addNode("4");
    addNode("5");
    addNode("6");

    // Add edges
    addEdge(1, 2);
    addEdge(2, 3);
    addEdge(3, 1);
    addEdge(4, 1);
    addEdge(5, 1);
    addEdge(5, 6);
    addEdge(0, 1);


    // Calculate static layout
    calculateLayout();
    initBFS(0);

    while (!WindowShouldClose())
    {
        BeginDrawing();
        ClearBackground(RAYWHITE);
        if (IsKeyPressed(KEY_RIGHT) || IsKeyPressed(KEY_L))
        {
            traversal_steps = (int) fmin(10, traversal_steps + 1);
            printf("Node: %d\n", bfs_queue.items[bfs_queue.front]);
            stepBFS();
        }

        drawGraph();


        EndDrawing();
    }

    CloseWindow();
    return 0;
}
