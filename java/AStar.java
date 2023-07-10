
public class AStar {

    public final int OBSTACLE = 1; 
    public final int PATH = 2; 

    
    public int DIRECT_VALUE = 1; 
    public int OBLIQUE_VALUE = 1; 

    MapInfo mapInfo;
    Queue<Node> openList; 
    List<Node> closeList;

    ArrayList<Integer[]> aStarPath;
    int totalCost;


    public AStar(
            int[][] maps,
            int[] startCoordinate,
            int[] endCoordinate
    ) {
        openList = new PriorityQueue<>();
        closeList = new ArrayList<>();
        aStarPath = new ArrayList<>();

        mapInfo = new MapInfo(
                MatrixCopier.deepCopyMatrix(maps),
                maps[0].length,
                maps.length,
                new Node(startCoordinate[1], startCoordinate[0]),
                new Node(endCoordinate[1], endCoordinate[0])
        );
    }

    
    private boolean isEndNode(Coordinates end, Coordinates coordinates) {
        return end.equals(coordinates);
    }

    
    private boolean canAddNodeToOpen(MapInfo mapInfo, int x, int y) {
        
        if (x < 0 || x >= mapInfo.width || y < 0 || y >= mapInfo.hight) return false;
        
        if (mapInfo.maps[y][x] == OBSTACLE) return false;
        
        return !isCoordInClose(x, y);
    }

    
    private boolean isCoordInClose(Coordinates coordinates) {
        return coordinates != null && isCoordInClose(coordinates.x, coordinates.y);
    }

    
    private boolean isCoordInClose(int x, int y) {
        if (closeList.isEmpty()) return false;
        for (Node node : closeList) {
            if (node.coordinates.x == x && node.coordinates.y == y) {
                return true;
            }
        }
        return false;
    }

    private int manhattanDistance(Coordinates end, Coordinates coordinates) {
        return Math.abs(end.x - coordinates.x) + Math.abs(end.y - coordinates.y);
    }

    private Node findNodeInOpen(Coordinates coordinates) {
        if (coordinates == null || openList.isEmpty()) return null;
        for (Node node : openList) {
            if (node.coordinates.equals(coordinates)) {
                return node;
            }
        }
        return null;
    }

    
    private void addNeighborNodeInOpen(MapInfo mapInfo, Node current) {
        int x = current.coordinates.x;
        int y = current.coordinates.y;
        
        addNeighborNodeInOpen(mapInfo, current, x - 1, y, DIRECT_VALUE);
        
        addNeighborNodeInOpen(mapInfo, current, x, y - 1, DIRECT_VALUE);
        
        addNeighborNodeInOpen(mapInfo, current, x + 1, y, DIRECT_VALUE);
        
        addNeighborNodeInOpen(mapInfo, current, x, y + 1, DIRECT_VALUE);
        
        addNeighborNodeInOpen(mapInfo, current, x - 1, y - 1, OBLIQUE_VALUE);
        
        addNeighborNodeInOpen(mapInfo, current, x + 1, y - 1, OBLIQUE_VALUE);
        
        addNeighborNodeInOpen(mapInfo, current, x + 1, y + 1, OBLIQUE_VALUE);
        
        addNeighborNodeInOpen(mapInfo, current, x - 1, y + 1, OBLIQUE_VALUE);
    }

    
    private void addNeighborNodeInOpen(MapInfo mapInfo, Node current, int x, int y, int value) {
        if (canAddNodeToOpen(mapInfo, x, y)) {
            Node end = mapInfo.end;
            Coordinates coordinates = new Coordinates(x, y);
            int G = current.G + value; 
            Node child = findNodeInOpen(coordinates);
            if (child == null) {
                int H = manhattanDistance(end.coordinates, coordinates); 
                if (isEndNode(end.coordinates, coordinates)) {
                    child = end;
                    child.parent = current;
                    child.G = G;
                    child.H = H;
                } else {
                    child = new Node(coordinates, current, G, H);
                }
                openList.add(child);
            } else if (child.G > G) {
                child.G = G;
                child.parent = current;
                
                openList.add(child);
            }
        }
    }

    private void drawPath(int[][] maps, Node end) {
        if (end == null || maps == null) return;
        totalCost = end.G;
        while (end != null) {
            Coordinates c = end.coordinates;
            maps[c.y][c.x] = PATH;
            aStarPath.add(new Integer[]{c.y, c.x});
            end = end.parent;
        }
    }

    public void start() {
        if (mapInfo == null) return;
        
        openList.clear();
        closeList.clear();
        
        openList.add(mapInfo.start);
        moveNodes(mapInfo);
    }

    
    private void moveNodes(MapInfo mapInfo) {
        while (!openList.isEmpty()) {
            if (isCoordInClose(mapInfo.end.coordinates)) {
                drawPath(mapInfo.maps, mapInfo.end);
                break;
            }
            Node current = openList.poll();
            closeList.add(current);
            addNeighborNodeInOpen(mapInfo, current);
        }
    }

    
    public ArrayList<Integer[]> getAStarPath() {
        Collections.reverse(aStarPath);
        return aStarPath;
    }

    public int[][] getMapWithPath() {
        return mapInfo.maps;
    }

    public int getTotalCost() {
        return totalCost;
    }

    private static class Coordinates {
        public int x;
        public int y;

        public Coordinates(int x, int y) {
            this.x = x;
            this.y = y;
        }

        @Override
        public boolean equals(Object obj) {
            if (obj == null) return false;
            if (obj instanceof Coordinates) {
                Coordinates c = (Coordinates) obj;
                return x == c.x && y == c.y;
            }
            return false;
        }

        @Override
        public String toString() {
            return getClass().getSimpleName() + "{" +
                    "x=" + x +
                    ", y=" + y +
                    '}';
        }
    }

    private static class Node implements Comparable<Node> {

        public Coordinates coordinates; 
        public Node parent; 
        public int G; 
        public int H; 

        
        public Node(int x, int y) {
            this.coordinates = new Coordinates(x, y);
        }

        public Node(Coordinates coordinates, Node parent, int g, int h) {
            this.coordinates = coordinates;
            this.parent = parent;
            G = g;
            H = h;
        }

        @Override
        public int compareTo(Node o) {
            if (o == null) return -1;
            if (G + H > o.G + o.H)
                return 1;
            else if (G + H < o.G + o.H) return -1;
            return 0;
        }

        @Override
        public String toString() {
            return getClass().getSimpleName() + " {" +
                    "coord=" + coordinates +
                    ", parent=" + parent +
                    ", G=" + G +
                    ", H=" + H +
                    '}';
        }
    }

    private static class MapInfo {
        public int[][] maps; 
        public int width; 
        public int hight; 
        public Node start; 
        public Node end; 

        public MapInfo(int[][] maps, int width, int hight, Node start, Node end) {
            this.maps = maps;
            this.width = width;
            this.hight = hight;
            this.start = start;
            this.end = end;
        }
    }
}

