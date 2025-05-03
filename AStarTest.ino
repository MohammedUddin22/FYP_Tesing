#include <Arduino.h>

#define GRID_SIZE 6

struct Point { uint8_t x, y; };
using Node = Point;

bool     obstacleGrid[GRID_SIZE][GRID_SIZE];
bool     closedSet  [GRID_SIZE][GRID_SIZE];
bool     inOpenSet  [GRID_SIZE][GRID_SIZE];
uint16_t gCost      [GRID_SIZE][GRID_SIZE];
uint16_t fCost      [GRID_SIZE][GRID_SIZE];
Node     parent     [GRID_SIZE][GRID_SIZE];

inline uint16_t heuristic(const Node &a, const Node &b) {
  return abs(int(a.x) - int(b.x)) + abs(int(a.y) - int(b.y));
}

void printGrid(const Point &start, const Point &goal, Point path[], int pathLen) {
  char display[GRID_SIZE][GRID_SIZE];
  for (int y = 0; y < GRID_SIZE; y++)
    for (int x = 0; x < GRID_SIZE; x++)
      display[y][x] = '0';

  for (int y = 0; y < GRID_SIZE; y++)
    for (int x = 0; x < GRID_SIZE; x++)
      if (obstacleGrid[y][x])
        display[y][x] = 'x';

  for (int i = 0; i < pathLen; i++) {
    int x = path[i].x, y = path[i].y;
    display[y][x] = '1';
  }

  display[start.y][start.x] = '1';
  display[goal.y][goal.x]   = '1';

  Serial.print("   ");
  for (int x = 0; x < GRID_SIZE; ++x) {
    Serial.print(x);
    Serial.print(' ');
  }
  Serial.println();

  for (int y = GRID_SIZE - 1; y >= 0; --y) {
    //Serial.print(y);
    //Serial.print(": ");
    for (int x = 0; x < GRID_SIZE; ++x) {
      Serial.print(display[y][x]);
      Serial.print(' ');
    }
    Serial.println();
  }
  Serial.println();
}

int findPath(
  Point start,
  Point goal,
  Point obstacles[], int obstacleCount,
  Point pathOut[],     int &pathLenOut
) {
  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      obstacleGrid[y][x] = false;
      closedSet  [y][x] = false;
      inOpenSet  [y][x] = false;
      gCost      [y][x] = UINT16_MAX;
      fCost      [y][x] = UINT16_MAX;
    }
  }

  for (int i = 0; i < obstacleCount; i++) {
    int x = obstacles[i].x, y = obstacles[i].y;
    if (x < GRID_SIZE && y < GRID_SIZE) {
      obstacleGrid[y][x] = true;
    }
  }

  if (start.x >= GRID_SIZE || start.y >= GRID_SIZE ||
      goal.x  >= GRID_SIZE || goal.y  >= GRID_SIZE ||
      obstacleGrid[start.y][start.x] ||
      obstacleGrid[goal.y][goal.x]) {
    pathLenOut = 0;
    printGrid(start, goal, pathOut, 0);
    return -1;
  }

  Node openList[GRID_SIZE * GRID_SIZE];
  int  openCount = 0;
  gCost[start.y][start.x] = 0;
  fCost[start.y][start.x] = heuristic(start, goal);
  inOpenSet[start.y][start.x] = true;
  openList[openCount++] = start;

  const int dirs[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};
  bool found = false;
  Node current;

  while (openCount > 0) {
    int best = 0;
    for (int i = 1; i < openCount; i++) {
      Node &a = openList[i], &b = openList[best];
      if (fCost[a.y][a.x] < fCost[b.y][b.x]) best = i;
    }
    current = openList[best];

    if (current.x == goal.x && current.y == goal.y) {
      found = true;
      break;
    }

    openCount--;
    openList[best] = openList[openCount];
    inOpenSet[current.y][current.x] = false;
    closedSet  [current.y][current.x] = true;

    for (int d = 0; d < 4; d++) {
      int nx = current.x + dirs[d][0], ny = current.y + dirs[d][1];
      if (nx < 0 || nx >= GRID_SIZE || ny < 0 || ny >= GRID_SIZE) continue;
      if (obstacleGrid[ny][nx] || closedSet[ny][nx]) continue;

      uint16_t tg = gCost[current.y][current.x] + 1;
      if (!inOpenSet[ny][nx] || tg < gCost[ny][nx]) {
        parent[ny][nx] = current;
        gCost[ny][nx]  = tg;
        fCost[ny][nx]  = tg + heuristic({uint8_t(nx),uint8_t(ny)}, goal);
        if (!inOpenSet[ny][nx]) {
          openList[openCount++] = { uint8_t(nx), uint8_t(ny) };
          inOpenSet[ny][nx]     = true;
        }
      }
    }
  }

  if (!found) {
    pathLenOut = 0;
    printGrid(start, goal, pathOut, 0);
    return -1;
  }

  int len = 0;
  while (!(current.x == start.x && current.y == start.y)) {
    pathOut[len++] = current;
    current = parent[current.y][current.x];
  }
  pathOut[len++] = start;

  for (int i = 0; i < len/2; i++) {
    Point tmp = pathOut[i];
    pathOut[i] = pathOut[len - 1 - i];
    pathOut[len - 1 - i] = tmp;
  }

  pathLenOut = len;
  printGrid(start, goal, pathOut, len);
  return len;
}

bool done = false;

void setup() {
  Serial.begin(9600);
}

void loop() {
  if (!done) {
    Point start       = {4, 2};
    Point goal        = {1, 4};
    Point obstacles[] = { {0,4}, {0,3}, {1,3}, {2,3}, {3,3}, {4,3} };
    int   obsCount    = sizeof(obstacles)/sizeof(obstacles[0]);

    Point path[GRID_SIZE * GRID_SIZE];
    int   pathLen;

    int result = findPath(start, goal, obstacles, obsCount, path, pathLen);

    if (result > 0) {
      Serial.print("Path length = ");
      Serial.println(pathLen);
      Serial.println("Path coordinates:");
      for (int i = 0; i < pathLen; i++) {
        Serial.print('(');
        Serial.print(path[i].x);
        Serial.print(',');
        Serial.print(path[i].y);
        Serial.println(')');
      }
    } else {
      Serial.println("No path found.");
    }

    done = true;
  }
}
