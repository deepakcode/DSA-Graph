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
        stack.add(i);

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
     ```java
        PriorityQueue<Pair> pQueue= new PriorityQueue<>(V,Comparator.comparingInt(p->p.weight));
  ```

- Now apply below logic for every adjacent vertex of min weight 
     ```java
         int v = adj.get(u).get(i).get(0);
          int edge_wt = adj.get(u).get(i).get(1);
          if(weight[v]>(weight[u]+edge_wt)){
               weight[v] = (weight[u]+edge_wt);
               pQueue.add(new Pair(weight[v],v));
          }
  ```
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
    
    ```java
    visited[u]=true;
    for(int v: adj.get(u)){
        if(!visited[v])
             dfsUtil(v,adj,visited,stack);
    }
    stack.add(u);
    ```
  
- Transpose the Graph G'
  
    ```java
    for(int u=0; u<adj.size(); u++){
      for(int v=0; v<adj.get(u).size(); v++){
          tAdj.get(adj.get(u).get(v)).add(u);
      }
    }
  ```
  
- Do DFS from stack nodes over G' (Transpose of graph), here each DFS is SCC so count it.
    
    ```java
    Arrays.fill(visited,false);
    while(!stack.isEmpty()){
        int node = stack.pop();
        if(!visited[node]){
             ++count;
             dfsUtil(node,tAdj,visited);
        }
    }
  ```
  
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


#### 10) Shortest Source to Destination Path

- Do BFS 

- Corner case `-> If source node is not equal to 1 then return -1 because you can only move if it is '1'`
```java
    if(A[0][0]!=1)
        return -1; // corner case
```

- Mark it visited or processed at the time of adding into queue
    
    for example - 

```java
    if(isValid(cell.x+1,cell.y,N,M,A)){
        A[cell.x+1][cell.y]=2;
        queue.add(new Cell(cell.x+1,cell.y,cell.cost+1));
    }
```
<p>
https://practice.geeksforgeeks.org/problems/shortest-source-to-destination-path3544/1
  
<details>
    
<summary>code</summary>    
      
```java
class Solution {
    
    int shortestDistance(int N, int M, int A[][], int Dx, int Dy) {
        if(A[0][0]!=1)
            return -1; // corner case
            
        Queue<Cell> queue = new LinkedList<>();
        queue.add(new Cell(0,0,0));
        A[0][0]=2; // Mark it visited or processed at the time of adding into queue
        
        while(!queue.isEmpty()){
            Cell cell = queue.poll();
            if(cell.x== Dx  && cell.y==Dy)
                return cell.cost;
            else{
                if(isValid(cell.x+1,cell.y,N,M,A)){
                    A[cell.x+1][cell.y]=2;
                    queue.add(new Cell(cell.x+1,cell.y,cell.cost+1));
                }
                if(isValid(cell.x-1,cell.y,N,M,A)){
                    A[cell.x-1][cell.y]=2;
                    queue.add(new Cell(cell.x-1,cell.y,cell.cost+1));
                }
                if(isValid(cell.x,cell.y+1,N,M,A)){
                    A[cell.x][cell.y+1]=2;
                    queue.add(new Cell(cell.x,cell.y+1,cell.cost+1));
                    
                }
                if(isValid(cell.x,cell.y-1,N,M,A)){
                    A[cell.x][cell.y-1]=2;
                    queue.add(new Cell(cell.x,cell.y-1,cell.cost+1));
                }
             }
       }
       return -1;
    }
    
    private boolean isValid(int x,int y,int N,int M, int A[][]){
        if(x>=0 && x<N && y>=0 && y<M && A[x][y]==1)
            return true;
        return false;
    }
};

class Cell{
    int x;
    int y;
    int cost;
    Cell(int x, int y, int cost){
        this.x=x;
        this.y=y;
        this.cost=cost;
    }
}
```
</details>
    
</p>  
  

#### 11) Find whether path exist

- Do DFS 

<p>
https://practice.geeksforgeeks.org/problems/find-whether-path-exist5238/1
  
<details>
    
<summary>code</summary>    
      
```java
class Solution
{
    //Function to find whether a path exists from the source to destination.
    public boolean is_Possible(int[][] grid)
    {
        boolean flag = false;
        int U = grid.length;
        int V = grid[0].length;
        int x=0;
        int y=0;
        
        int _x=0;
        int _y=0;
        
        for (int i=0; i<U; i++){
            for(int j=0; j<V; j++){
                if(grid[i][j]==1){
                    x=i;
                    y=j;
                }
                if(grid[i][j]==2){
                    _x=i;
                    _y=j;
                }
            }
        }

        dfs(x,y,grid,U,V);
        
        return grid[_x][_y]==0 ?true:false;
    }
    
    public void dfs(int x,int y, int[][] grid,int U,int V){

        if(!isValid(x,y,U,V,grid))  // is valid
            return;
             
           if(grid[x][y]==2 || grid[x][y]==3 || grid[x][y]==1)
           grid[x][y]=0;
      
           dfs(x-1,y,grid,U,V);
           dfs(x+1,y,grid,U,V);
           dfs(x,y-1,grid,U,V);
           dfs(x,y+1,grid,U,V);
        return;
    }
    
    public boolean isValid(int x,int y, int U,int V,int[][] grid){
        if(x>=0 && x<U && y>=0 && y<V && grid[x][y] != 0 ){
           return true;
        }else{
           return false;
        }
    }
}
```
</details>
    
</p>
    
#### 12) Minimum Cost Path

- This question can be solved via Recursion (Time limit exception) - then you can optimized your Solution using DP Tabulation, here some of the test case are failing, but Dijkstra is working fine in this case.

So finally you can solve it using Dijkstra algo, it will works fine.

<p>
https://practice.geeksforgeeks.org/problems/minimum-cost-path3833/1

<details>
  
<summary>DP code</summary>    
  
```java
class Solution
{
    
int minimumCostPath(int cost[][])
{
    int m = cost.length;
    int n = cost[0].length;
    int[][] dp = new int[m][n];
     minimumCostPath(cost,m-1,n-1,dp);
     return dp[m-1][n-1];
}
 
 void minimumCostPath(int cost[][], int m, int n, int[][] dp){
   
    if (n < 0 || m < 0)
        return;
        
   dp[0][0]=cost[0][0];
   
   for(int i=1;i<=m; i++)
        dp[i][0]= dp[i-1][0]+cost[i][0];
   
   for(int j=1;j<=n; j++)
        dp[0][j]= dp[0][j-1]+cost[0][j];
        

     for(int i=1;i<=m; i++){
          for(int j=1;j<=n; j++){
            dp[i][j]=cost[i][j]+Math.min(dp[i-1][j],dp[i][j-1]);
          }
     }
 }

}
```
</details>
  
<details>
  
<summary>Dijkstra code - all test cases passed!</summary>    
  
```java
class Solution
{
    
    public int minimumCostPath(int[][] grid)
    {
        int row = grid.length;
        int col = grid[0].length;
        
        int[][] dp = new int[row][col];
        for(int i = 0; i < row; i++)
            for(int j = 0; j < col; j++)
                dp[i][j] = Integer.MAX_VALUE;
                
        dp[0][0] = grid[0][0];
        
        PriorityQueue<Cell> pq = new PriorityQueue<Cell>(row * col, Comparator.comparingInt(c->c.distance));
        pq.add(new Cell(0, 0, dp[0][0]));
        
        while (!pq.isEmpty())
        {
            Cell curr = pq.poll();
            
            for(int i = 0; i < 4; i++)
            {
                int rows = curr.x + dx[i];
                int cols = curr.y + dy[i];
                
                if (isValid(rows, cols, grid.length, grid[0].length)){
                    if (dp[rows][cols] >dp[curr.x][curr.y]+grid[rows][cols]){
                        
                        dp[rows][cols]=dp[curr.x][curr.y]+grid[rows][cols];
                        
                        pq.add(new Cell(rows, cols, dp[rows][cols]));
                    }
                }
            }
        }
        return dp[row - 1][col - 1];
    }
    
    static class Cell
    {
        int x;
        int y;
        
        int distance;
        
        Cell(int x, int y, int distance) 
        {
            this.x = x;
            this.y = y;
            this.distance = distance;
        }
    }
    
    static boolean isValid(int i, int j, int ROW, int COL){
        return (i >= 0 && i < ROW &&j >= 0 && j < COL);
    }
    ///up down left right
    static int[] dx = { -1, 0, 1, 0 };
    static int[] dy = { 0, 1, 0, -1 };
}
```
</details>
  
</p>
  
#### 13	) Circle of Strings

1. Create a graph considering edge for word'geek' as 'g' to 'k', total number of V would be 26, each vertext is ('g'-a)
2. Chain can only be formed if there is Elurian circuit is present in Graph G

  If Eulerian circuit is present in graph then 

  1. Graph is strongly connected (Kosaraju)
  2. In and Out degree of each vertex should be same. (check the length of adj node)


Is G is strongly connected if

	1. Define visited array  -  boolean[] visited = new boolean[V]; (defaut false)
	2. Find first node, with non zero degree vertex.  if (adj.get(n).size() > 0)
	3. Perform DFS
	4*. If DFS traversal doesn't visit all vertices, then return false.
	5. Create transpose of Graph - G' 
	6. Mark all node as not visited 
	7. Perform DFS over G' keep the start node as same as DFS of G
	8. If all vertices not visited, return false. 
	9. At he end return true

<p>
https://practice.geeksforgeeks.org/problems/minimum-cost-path3833/1

<details>
  
<summary>code</summary>    
  
```java


```
</details>

</p>
  
#### 14	) Floyd Warshall (DP)
	
1. In this algo we find shorted path from each vertex to all other vertices.
	
- Here DP can be applied -
	
	`A[i][j] = A[i][k]+A[K][j];`
	
- Simple Algo
	
```java
for(int k=1; k<n; k++) 

	for(int i=1; i<n; i++) 
		for(int j=1; j<n; j++) 
			A[i][j] = A[i][k]+A[K][j];
```

<p>
https://practice.geeksforgeeks.org/problems/minimum-cost-path3833/1

<details>
  
<summary>code</summary>    
  
```java
for(int k=1; k<n; k++) 

	for(int i=1; i<n; i++) 
		for(int j=1; j<n; j++) 
			A[i][j] = A[i][k]+A[K][j];

```
</details>

</p>
	
15. Alien Dictionary
	
16. Snake and Ladder Problem
