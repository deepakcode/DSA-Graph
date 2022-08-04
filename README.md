# Graph: Must do coding questions
#### 1) Depth First Traversal
DFS can be sloved using recursion/stack.

In graph because node is repeated unlike trees so we need to keep track of visited nodes.

<p>
https://practice.geeksforgeeks.org/problems/depth-first-traversal-for-a-graph/1
  
<details>
    
<summary>code</summary>    
      
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
    
</details>
    
</p>

#### 2) Breadth First Traversal
    
BFS can be sloved using queue.
    
In graph because node is repeated unlike trees so we need to keep track of visited nodes.
    
``if (!visited[current]) ``- this is must !

<p>
https://practice.geeksforgeeks.org/problems/bfs-traversal-of-graph/1
    
<details>
    
<summary>code</summary>    
    
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
    
 </details>
    
</p>

#### 3) Detect cycle in undirected graphDuring DFS if somenode is already visited and adjacentNode is not equal to parent then there is cycle.

Implementation note :
    
- Do DFS from each node.
    
- Neighbour node shuld not equals to parent.
    
- If DFS return true then only return true else false.
   ``if(isCyclicUtil(u,visited,-1,adj))
     return true;``
    
- In recursive call pass current as parent.
   ``if(isCyclicUtil(i,visited,i,adj))
 		return true;``

[https://youtu.be/UPfUFoWjk5w?t=93](https://tinyurl.com/isCyclePresent)

<p>
https://practice.geeksforgeeks.org/problems/detect-cycle-in-an-undirected-graph/1
 
<details>
    
<summary>code</summary>    
    
```java    
 public  boolean isCycle(int V, ArrayList<ArrayList<Integer>> adj) {
    boolean[] visited = new boolean[V];
    for (int u = 0; u < V; u++) {
        if (!visited[u])
            if(isCyclicUtil(u,visited,-1,adj))
                return true;
    }
    return false;
 }
 
 public boolean isCyclicUtil(int v, boolean[] visited,
                    int parent, ArrayList<ArrayList<Integer>> adj){
     visited[v]=true;
     for(int i: adj.get(v)){
        if(!visited[i]){
            if(isCyclicUtil(i,visited,v,adj)){
                 return true;
            }
        }else if(i != parent ){
            return true;
        }
     }
     return false;
 }
 ```
    
</details>
    
</p>

    
#### 4) Detect cycle in directed graph
    
    Do DFS traversal and take two visited array one for node visit and another one for recStack node visited.
    
    - Take two visited array one for node visit and another one for recStack node visited
    
    - Mark false for recStack after node process
    
    - Do not check for isVisited? for child as well as for parent node.

<p>
https://practice.geeksforgeeks.org/problems/detect-cycle-in-a-directed-graph/1
 
<details>
    
<summary>code</summary>    
    
```java    
class Solution {

    boolean isCyclicUtil(int i, boolean[] visited,boolean[] recStack,
                         ArrayList<ArrayList<Integer>> adj) {
        if(recStack[i])
            return true;

         if(visited[i])
            return false;

        visited[i] = true;
        recStack[i] = true;

        for (Integer c : adj.get(i)){
            //Do not check for isVisited - for `c`
                if (isCyclicUtil(c, visited,recStack, adj)) 
                    return true;
        }

        recStack[i] = false; // Mark false for recStack after node process

        return false;
    }

    public boolean isCyclic(int V, ArrayList<ArrayList<Integer>> adj) {

        boolean[] visited = new boolean[V];
        boolean[] recStack = new boolean[V];

        for (int i = 0; i < V; i++){
            //Do not check for isVisited - for `i`
             if (isCyclicUtil(i, visited,recStack,adj)) 
                return true;
        }
        return false;
    }
}
```
    
</details>
    
</p>

    
#### 5) Topological sort
    
    - Do dfs once node is processed store it into stack! that's it. Done!!!
        `` stack.add(i);``

<p>
https://practice.geeksforgeeks.org/problems/topological-sort/1
 
<details>
    
<summary>code</summary>    
    
```java    
class Solution
{
    //Function to return list containing vertices in Topological order. 
    static int[] topoSort(int V, ArrayList<ArrayList<Integer>> adj) 
    {
      //DFS and store nodes in stack and print stack
      Stack<Integer> stack = new Stack<>();// default '0'
      boolean[] visited = new boolean[V]; //default 'false'
      int[] res= new int[V];
      
      for(int i=0; i<V; i++){
          if(!visited[i])
          dfsUtil(i,adj,stack,visited);
      }
      int i=0;
     // System.out.println(stack.size());
      while(!stack.isEmpty()){
          res[i]=stack.pop();
          i++;
      }
      return res;
    }
    
    
    static public void dfsUtil(int i, ArrayList<ArrayList<Integer>> adj,Stack<Integer> stack, 
                boolean[] visited){
                    
        visited[i]=true;
        
        for(int child: adj.get(i)){
            if(!visited[child])
             dfsUtil(child,adj,stack,visited);
        }
    
        stack.add(i);
    }
```
    
</details>
    
</p>


#### 6) Find the number of islands

- Just return the count of total DFS traversal that's it.

<p>
https://practice.geeksforgeeks.org/problems/find-the-number-of-islands/1
  
<details>
    
<summary>code</summary>    
      
```java
class Solution {
    // Function to find the number of islands.
    public int numIslands(char[][] grid) {
        int count=0;
        int M=grid.length;
        int N=grid[0].length;
        for(int i=0; i<M; i++){
            for(int j=0; j<N; j++){
                if(grid[i][j]=='1'){
                     dfsUtil(grid,i,j,M,N);
                     count++;
                }
            }
        }
       dfsUtil(grid,0,1,M,N)
       return count;
    }
    
    public void dfsUtil(char[][] grid, int x,int y, int M, int N){
    
       if(isValid(x,y,grid,M,N)){

            if(grid[x][y]=='1'){
                
                grid[x][y]='2'; // Instead of visted mark is any other symbol except 1 or 0 
                
                dfsUtil(grid,x+1,y,M,N);
                dfsUtil(grid,x-1,y,M,N);
                dfsUtil(grid,x,y+1,M,N);
                dfsUtil(grid,x,y-1,M,N);
    
                dfsUtil(grid,x-1,y-1,M,N);
                dfsUtil(grid,x+1,y+1,M,N);
                dfsUtil(grid,x-1,y+1,M,N);
                dfsUtil(grid,x+1,y-1,M,N);
            }
       }

    }
    
    public boolean isValid(int i, int j, char[][] grid, int M,int N){
        if(i>=0 && i<M && j>=0 && j<N){
            return true;
        }
        return false;
    }
}
    
```
    
</details>
    
</p>

#### 7) Implementing Dijkstra

- Initialize weight of all the vertex from source to Infinite (Integer.MAX_VALUE) and source it self as '0'

- Pick up the min weight vertex(PriorityQueue sort by weight), because initialy source weight is '0' so we pick up it

        `PriorityQueue<Pair> pQueue= new PriorityQueue<>(V,Comparator.comparingInt(p->p.weight));`

- Now apply below logic for every adjacent vertex of min weight 

         `int v = adj.get(u).get(i).get(0);
          int edge_wt = adj.get(u).get(i).get(1);
          if(weight[v]>(weight[u]+edge_wt)){
               weight[v] = (weight[u]+edge_wt);
               pQueue.add(new Pair(weight[v],v));
          }`
<p>
https://practice.geeksforgeeks.org/problems/implementing-dijkstra-set-1-adjacency-matrix/1
  
<details>
    
<summary>code</summary>    
      
```java

class Solution
{
    //Function to find the shortest distance of all the vertices
    //from the source vertex S.
    static int[] dijkstra(int V, ArrayList<ArrayList<ArrayList<Integer>>> adj, int S)
    {
        
        int weight[] = new int[V];
        
        Arrays.fill(weight,Integer.MAX_VALUE);
        
        PriorityQueue<Pair> pQueue= new PriorityQueue<>(V,Comparator.comparingInt(p->p.weight));
        weight[S]=0;
       
        pQueue.add(new Pair(0,S));
        
        while(pQueue.size()>0){
           Pair node =  pQueue.poll();
           int u = node.vertex;
           
           for(int i=0; i<adj.get(u).size(); i++){
                  int v = adj.get(u).get(i).get(0);
                  int edge_wt = adj.get(u).get(i).get(1);
                  if(weight[v]>(weight[u]+edge_wt)){
                       weight[v] = (weight[u]+edge_wt);
                       pQueue.add(new Pair(weight[v],v));
                  }
           }
        }
        return weight;
    }
}

class Pair{
        
int weight;
    int vertex;

    public Pair(int _weight,int _vertex){
        weight=_weight;
        vertex=_vertex;
    }
}


```
</details>
    
</p>
  
8. Minimum Swaps

#### 9) Strongly Connected Components

- Perfom DFS and store the result in Stack (exactly like toposort) i:e first process the node then store it!

    `visited[u]=true;
    for(int v: adj.get(u)){
        if(!visited[v])
             dfsUtil(v,adj,visited,stack);
    }
    stack.add(u);`

- Transpose the Graph G'
    
    `for(int u=0; u<adj.size(); u++){
      for(int v=0; v<adj.get(u).size(); v++){
          tAdj.get(adj.get(u).get(v)).add(u);
      }
    }`

- Do DFS from stack nodes over G' (Transpose of graph), here each DFS is SCC so count it.
    `Arrays.fill(visited,false);
    while(!stack.isEmpty()){
        int node = stack.pop();
        if(!visited[node]){
             ++count;
             dfsUtil(node,tAdj,visited);
        }
    },


<p>
https://practice.geeksforgeeks.org/problems/strongly-connected-components-kosarajus-algo/1
  
<details>
    
<summary>code</summary>    
      
```java

class Solution
{
    //Function to find number of strongly connected components in the graph.
    public int kosaraju(int V, ArrayList<ArrayList<Integer>> adj)
    {
        //Do DFS
        //G1 - transpose
        //DGF of stack element
        int count = 0;
        boolean[] visited=new boolean[V];
        Stack<Integer> stack = new Stack<>();

        for(int u=0; u<V; u++){
            if(!visited[u]){
                dfsUtil(u,adj,visited,stack);
            }
        }
        
        ArrayList<ArrayList<Integer>> tAdj= new ArrayList<>();
        
        for(int i=0; i<adj.size(); i++)
            tAdj.add(new ArrayList<>());
        
        for(int u=0; u<adj.size(); u++){
              for(int v=0; v<adj.get(u).size(); v++){
                  tAdj.get(adj.get(u).get(v)).add(u);
              }
        }
        Arrays.fill(visited,false);
        while(!stack.isEmpty()){
            int node = stack.pop();
            if(!visited[node]){
                 ++count;
                 dfsUtil(node,tAdj,visited);
            }
        }
        return count;
    }
    
    public void dfsUtil(int u, ArrayList<ArrayList<Integer>> adj, 
            boolean[] visited, Stack<Integer> stack){
        visited[u]=true;
        for(int v: adj.get(u)){
            if(!visited[v])
                 dfsUtil(v,adj,visited,stack);
        }
        stack.add(u);
    } 
    
    public void dfsUtil(int u, ArrayList<ArrayList<Integer>> adj, 
            boolean[] visited){
        visited[u]=true;
        for(int v: adj.get(u)){
            if(!visited[v])
                 dfsUtil(v,adj,visited);
        }
    } 
    
}


```
</details>
    
</p>

  
10. Shortest Source to Destination Path
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
