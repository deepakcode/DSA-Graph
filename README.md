# Graph: Must do coding questions
#### 1) Depth First Traversal
DFS can be sloved using recursion/stack.

In graph because node is repeated unlike trees so we need to keep track of visited nodes.

<p>
https://practice.geeksforgeeks.org/problems/depth-first-traversal-for-a-graph/1
    
```java
    public ArrayList<Integer> dfsOfGraph(int V, ArrayList<ArrayList<Integer>> adj) {
        boolean visited[] = new boolean[V];
        ArrayList<Integer> result = new ArrayList<>();
        // as start point is not given, lets assume it is '0'
        int s=0;
        dfs(adj,s,visited,result);
        return result;
    }
    
    public void dfs(ArrayList<ArrayList<Integer>> adj, int node, boolean[] visited, ArrayList<Integer> result){
        //mark start - as visited
        //Do dfs over each adjacent vertex only if it is not visited
        result.add(node);
        visited[node]=true;
        ArrayList<Integer> list= adj.get(node);
        for(int adjacentNode: list){
            if(!visited[adjacentNode]){
                dfs(adj,adjacentNode,visited,result);
            }
        }
    }
    
```
</p>

#### 2) Breadth First Traversal
    
BFS can be sloved using queue.
    
In graph because node is repeated unlike trees so we need to keep track of visited nodes.
    
``if (!visited[current]) ``- this is must !

<p>
https://practice.geeksforgeeks.org/problems/bfs-traversal-of-graph/1

```java    
     public ArrayList<Integer> bfsOfGraph(int V, ArrayList<ArrayList<Integer>> adj) {
        ArrayList<Integer> res = new ArrayList<>();
        boolean[] visited = new boolean[V];
        Arrays.fill(visited, false);
        int start = 0;//assuming it is 0 because it is not given
        LinkedList<Integer> queue = new LinkedList<>();
        queue.add(start);
        while (queue.size() > 0) {
            int current = queue.poll();
            if (!visited[current]) { //This check is required else some of the test cases are failing in gfg
                visited[current] = true;
                res.add(current);
                for (int connectedVertex : adj.get(current)) {
                    if (!visited[connectedVertex]) {
                        queue.add(connectedVertex);
                    }
                }
            }
        }//while
        return res;
    }
 ```
</p>

#### 3) Detect cycle in undirected graph

During DFS if somenode is already visited and adjacentNode is not equal to parent then there is cycle.

https://youtu.be/UPfUFoWjk5w?t=93

<p>
https://practice.geeksforgeeks.org/problems/detect-cycle-in-an-undirected-graph/1

```java    
    public boolean isCycle(int V, ArrayList<ArrayList<Integer>> adj) {

        for (int i = 0; i < adj.size(); i++) {
            boolean[] visited = visited = new boolean[V + 1];
            if (!visited[i])
                if (isCycleUtil(visited, i, V, adj, -1)) {
                    return true;
                }
        }
        return false;
    }

    public boolean isCycleUtil(boolean[] visited, int start, int V,
                               ArrayList<ArrayList<Integer>> adj, int parent) {
        Stack<Integer> stack = new Stack<>();
        stack.add(start);
        while (stack.size() > 0) {
            int current = stack.pop();
            if (!visited[current]) {
                visited[current] = true;
                parent = current;
                for (int adjacent : adj.get(current)) {
                    if (!visited[adjacent])
                        stack.add(adjacent);
                }
            } else if (current != parent) {
                return true;
            }
        }
        return false;
    }
 ```
</p>
5. Detect cycle in a directed graph
6. Topological sort
7. Find the number of islands
8. Implementing Dijkstra
9. Minimum Swaps
10. Strongly Connected Components
11. Shortest Source to Destination Path
<details>
<summary>11 Find whether path exist</summary>
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
12. Minimum Cost Path
13. Circle of Strings
14. Floyd Warshall
15. Alien Dictionary
16. Snake and Ladder Problem
