Depth First Traversal
Breadth First Traversal
Detect cycle in undirected graph
Detect cycle in a directed graph
Topological sort
Find the number of islands
Implementing Dijkstra
Minimum Swaps
Strongly Connected Components
Shortest Source to Destination Path
<details>
<summary>Find whether path exist</summary>
<p>
https://practice.geeksforgeeks.org/problems/find-whether-path-exist5238/1

```java
package com.practice.graph;

public class Grid {

    public static void main(String[] args) {

        int[][] _grid = {
                {3, 3, 3, 3, 0, 0, 3, 0},
                {1, 3, 3, 3, 3, 3, 3, 2},
                {3, 3, 0, 3, 0, 3, 3, 3},
                {3, 3, 3, 0, 0, 3, 3, 0},
                {0, 3, 3, 3, 3, 3, 3, 3},
                {0, 0, 0, 3, 3, 0, 3, 3},
                {0, 3, 0, 3, 3, 3, 3, 0},
                {3, 3, 3, 0, 3, 3, 3, 3}};

        boolean isPossible = is_Possible(_grid);
        for(int i = 0; i < _grid.length; i++) {
            for (int j = 0; j < _grid[0].length; j++) {
                System.out.print(_grid[i][j] + " ");
            }
            System.out.println();
        }
        System.out.println("P " + isPossible);
    }

    public static boolean is_Possible(int[][] grid) {

        boolean flag = false;

        int U = grid.length;
        int V = grid[0].length;

        int x = 0;
        int y = 0;
        int _x = 0;
        int _y = 0;

        for (int i = 0; i < U; i++) {
            for (int j = 0; j < V; j++) {
                if (grid[i][j] == 1) {
                    x = i;
                    y = j;
                }
                if (grid[i][j] == 2) {
                    _x = i;
                    _y = j;
                }
            }
        }
        dfs(x, y, grid, U, V);
        return grid[_x][_y] == 0 ? true : false;
    }

    public static void dfs(int x, int y, int[][] grid, int U, int V) {
        if (!isValid(x, y, U, V, grid))
            return;

        if (grid[x][y] == 2 || grid[x][y] == 3 || grid[x][y] == 1)
            grid[x][y] = 0;

        dfs(x - 1, y, grid, U, V);
        dfs(x + 1, y, grid, U, V);
        dfs(x, y - 1, grid, U, V);
        dfs(x, y + 1, grid, U, V);
        return;
    }

    public static boolean isValid(int x, int y, int U, int V, int[][] grid) {
        if (x >= 0 && x < U && y >= 0 && y < V && grid[x][y] != 0) {
            return true;
        } else {
            return false;
        }
    }

}

```
</p>
</details>
Minimum Cost Path
Circle of Strings
Floyd Warshall
Alien Dictionary
Snake and Ladder Problem
