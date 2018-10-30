/**
 * CSCI 3302 Lab 5 October 2018
 * George Allison, Zander Louie, David Doan, Pierce Doogan
 */

#include <sparki.h>
#define TRUE 1
#define FALSE 0

#define CYCLE_TIME .100

// Number of vertices to discretize the map
#define NUM_X_CELLS 4
#define NUM_Y_CELLS 4

// Map is ~60cm x 42cm
#define MAP_SIZE_X 0.6
#define MAP_SIZE_Y 0.42

#define BIG_NUMBER 9999


// Grid map from Lab 4: values of 1 indicate free space, 0 indicates empty space
bool world_map[NUM_Y_CELLS][NUM_X_CELLS];

// Destination (I,J) grid coordinates
int goal_i = 0;
int goal_j = 0;

// Start (I,J) grid coordinates
int source_i = 0;
int source_j = 0;

// Pointer to ordered sequence of waypoints
int *path = NULL;


void setup() {   
  // Dijkstra Setup -- initialize empty world map
  for (int j = 0; j < NUM_Y_CELLS; ++j) {
    for (int i = 0; i < NUM_X_CELLS; ++i) {
      world_map[j][i] = 1;
    }
  }

  //TODO: Set up your map here by setting individual cells to 0 to indicate obstacles
  world_map[0][1] = 0; // Example of setup code to indicate an obstacle at grid position (1,0)
  world_map[1][1] = 0; // Example of setup code to indicate an obstacle at grid position (1,1)
}

/*****************************
 * Dijkstra Helper Functions *
 *****************************/

// Return 1 if there are entries in range [0,inf) in arr
// otherwise return 0, signifying empty queue
bool is_empty(int *arr, int len) {
  for (int i=0; i < len; ++i) {
    if (arr[i] >= 0) {
      return FALSE;
    }
  }
  return TRUE;
}

// Return the index with the minimum value in int array "arr" of length "len"
// Assumes positive values only, with values of "-1" indicating 'empty'
int get_min_index(int *arr, int len) {
  int min_val=-1, min_idx=-1;
  for (int i=0;i < len; ++i) {
    if (arr[i] < min_val || min_val == -1) {
      min_val = arr[i];
      min_idx = i;    
    }
  }
  return min_idx;
}


/**********************************
 * Coordinate Transform Functions *
 **********************************/

// Converts a vertex index into (I,J) coordinates
// Returns 0 if something went wrong -- assume invalid i and j values being set
bool vertex_index_to_ij_coordinates(int v_idx, int *i, int *j) {
  *i = v_idx % NUM_X_CELLS;
  *j = v_idx / NUM_X_CELLS;
  
  if (*i < 0 || *j < 0 || *i >= NUM_X_CELLS || *j >= NUM_Y_CELLS) return FALSE;
  return TRUE;
}

// Converts (I,J) coordinates into a vertex index
int ij_coordinates_to_vertex_index(int i, int j) {
  return j*NUM_X_CELLS + i;  
}

// Convert (i,j) coordinates into world coordinates
// Returns 0 if something went wrong -- assume invalid x and y values are being set
// Returns 1 otherwise. Assigned x and y values are the middle of cell (i,j)
bool ij_coordinates_to_xy_coordinates(int i, int j, float *x, float *y) {
  if (i < 0 || j < 0 || i >= NUM_X_CELLS || j >= NUM_Y_CELLS) return FALSE;
  
  *x = (i+0.5)*(MAP_SIZE_X/NUM_X_CELLS);
  *y = (j+0.5)*(MAP_SIZE_Y/NUM_Y_CELLS);
  return TRUE;  
}

// Convert (x,y) world coordinates into (i,j) grid coordinates
// Returns 0 if something went wrong -- assume invalid x and y values are being set
// Returns 1 otherwise. x and y values are the middle of cell (i,j)
bool xy_coordinates_to_ij_coordinates(float x, float y, int *i, int *j) {
  if (x < 0 || y < 0 || x >= MAP_SIZE_X || y >= MAP_SIZE_Y) return FALSE;
  
  *i = int((x/MAP_SIZE_X) * NUM_X_CELLS);
  *j = int((y/MAP_SIZE_Y) * NUM_Y_CELLS);

  return TRUE;  
}

/**********************************
 *      Core Dijkstra Functions   *
 **********************************/

// Returns the cost of moving from vertex_source to vertex_dest
int get_travel_cost(int vertex_source, int vertex_dest) {
  /* INSTRUCTIONS: 
      This function should return 1 if:
        vertex_source and vertex_dest are neighbors in a 4-connected grid (i.e., N,E,S,W of each other but not diagonal) and neither is occupied in world_map (i.e., world_map isn't 0 for either)

      This function shouuld return BIG_NUMBER if:
        vertex_source corresponds to (i,j) coordinates outside the map
        vertex_dest corresponds to (i,j) coordinates outside the map
        vertex_source and vertex_dest are not adjacent to each other (i.e., more than 1 move away from each other)
  */
  int s_i, s_j, d_i, d_j, distance_between_points;
  bool s_is_valid, d_is_valid;
  s_is_valid = vertex_index_to_ij_coordinates(vertex_source, &s_i, &s_j);
  d_is_valid = vertex_index_to_ij_coordinates(vertex_source, &d_i, &d_j);
  if (s_is_valid && d_is_valid && world_map[s_i, s_j] != 0 && world_map[d_i, d_j] != 0) {
    distance_between_points = sqrt(pow(s_i - d_i, 2) + pow(s_j - d_j, 2)); 
    if (distance_between_points == 1){
      return 1;
    }
    return BIG_NUMBER;
}
}


// Allocate and return a list of ints corresponding to the "prev" variable in Dijkstra's algorithm
// The returned array prev can be treated as a lookup table:  prev[vertex_index] = next vertex index on the path back to source_vertex
int *run_dijkstra(int source_vertex) {
  int total_cells = NUM_X_CELLS * NUM_Y_CELLS;
 
  // Array mapping vertex_index to distance of shortest path from source_vertex to vertex_index.
  int dist[total_cells];
  
  // Queue for identifying which vertices are still being explored -- Q_cost[vertex_index] = shortest known dist to get to vertex_index. 
  // Q_cost[vertex_index] = -1 if the vertex is no longer being considered.
  int Q_cost[total_cells]; 

  // Initialize memory for prev array
  int *prev = new int[total_cells];

  /**
   * TODO: Insert your Dijkstra's code here
   */
  dist[source_vertex] = 0;


  for (int i = 0; i < total_cells; i = i + 1){
    if (source_vertex != i){
      dist[i] = BIG_NUMBER;
      prev[i] = -1;
    }
    Q_cost[i] = -1;
  }

   Q_cost[source_vertex] = 0;
   while (is_empty(Q_cost, total_cells)){
    int min_index = get_min_index(Q_cost, total_cells);
    testLoop(min_index);
    int current_i, current_j;
    int neighbors[4] = {-1, -1, -1, -1};
    int j = 0;
    for (int i = 0; i < 16; i++) { //Find all of the neighbors to the current min_index
        if (get_travel_cost(min_index, i) == 1){
            neighbors[j] = i;
            j++;
        }
    }

    for (int i = 0; i < 4; i ++) {
        if (neighbors[i] != -1) { //For each neighbor that isn't -1, see if there is a better distance.
            int alt = dist[min_index] + 1;
            if (alt < dist[neighbors[i]]){
                dist[neighbors[i]] = alt;
                prev[neighbors[i]] = min_index;
                Q_cost[neighbors[i]] = alt;
            }
            
        }
        
    }
    Q_cost[min_index] = -1;
   }
  return prev;
}

// Given a populated 'prev' array, a source vertex, and destination vertex,
// allocate and return an integer array populated with the path from source to destination.
// The first entry of your path should be source_vertex and the last entry should be "-1" 
// to indicate the end of the array since paths can be variable length.
int *reconstruct_path(int *prev, int source_vertex, int dest_vertex) {
  int final_path[16];
  int *p;
  /**
   * TODO: Insert your code here
   */
  int current_vertex = dest_vertex;
  final_path[0] = -1;
  final_path[1] = current_vertex;
  int current_index = 2;
  while (current_vertex != source_vertex){
    testLoop(current_vertex);
    current_vertex = prev[current_vertex];
    final_path[current_index] = current_vertex;
    current_index ++;
  }
  p = new int[current_index];
  return p;
}

void testLoop(int i){
   sparki.clearLCD();
   sparki.println("Min_Index:");
   sparki.print(i);
   sparki.updateLCD();
  delay(1000);
}






void loop () {
  unsigned long begin_time = millis();
  unsigned long end_time = 0;
  unsigned long delay_time = 0;
  int *prev = NULL;
  int *path = NULL;


  /**
   * TODO: Populate prev with dijkstra's algorithm, then populate path with reconstruct_path
   */

  prev = run_dijkstra(0);
  path = reconstruct_path(prev, 0, 15);




  if (prev != NULL) {
    delete prev; 
    prev = NULL; // Once we have the path, don't need to keep prev around in memory anymore.
  }

  // TODO
  // Display the final path in the following format:
  //
  //  Source: (0,0)
  //  Goal: (3,1)
  //  0 -> 1 -> 2 -> 6 -> 7
  int s_i, s_j, d_i, d_j;
  vertex_index_to_ij_coordinates(0, &s_i, &s_j);
  vertex_index_to_ij_coordinates(5, &d_i, &d_j);
  char buf2[30];
  char buf3[100];
  
  sprintf(buf2, "Source: (%d,%d)\n", s_i, s_j);
  sprintf(buf3,"Goal: (%d,%d)\n", d_i, d_j);
  sparki.println();
  sparki.println(buf3);
  for (int i = sizeof(path) - 1; i >= 0; i --) {
      char buf[100];
      sparki.println(buf);
  }

  delay(10000);
  if (path != NULL) {
    delete path; 
    path=NULL; // Important! Delete the arrays returned from run_dijkstra and reconstruct_path when you're done with them!
  }
  ///////////////////////////////////////////////////  
 
  end_time = millis();
  delay_time = end_time - begin_time;
  if (delay_time < 1000*CYCLE_TIME)
    delay(1000*CYCLE_TIME - delay_time); // each loop takes CYCLE_TIME ms
  else
    delay(10); // Accept some error
}

