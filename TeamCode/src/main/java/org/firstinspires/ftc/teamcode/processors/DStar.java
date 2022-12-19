package org.firstinspires.ftc.teamcode.processors;

import java.util.ArrayList;
import java.util.List;

public class DStar {
    public Node[] nodeArray;
    private List<Integer> openList;
    public List<Integer> obstacles;

    public final int GRID_X;
    public final int GRID_Y;

    private int start;
    private int end;

    public DStar(int gridX, int gridY, int start, int end) {
        this.GRID_X = gridX;
        this.GRID_Y = gridY;
        nodeArray = new Node[gridX * gridY];
        this.start = start;
        this.end = end;

        // Find and cache all neighbors
        for(int i = 0; i < gridX * gridY; i++) {
            Node node = nodeArray[i];
            List<Integer> neighbors = new ArrayList<>();

            node.ownIndex = i;

            int nodeX = i % gridX;
            int nodeY = i / gridX;

            if(nodeX - 1 >= 0) {
                neighbors.add(i-1);
            }
            if(nodeX + 1 < gridX) {
                neighbors.add(i+1);
            }
            if(nodeY - 1 >= 0) {
                neighbors.add(i-gridX);
            }
            if(nodeY + 1 < gridY) {
                neighbors.add(i+gridX);
            }

            node.neighbors = (Integer[]) neighbors.toArray();
        }
    }

    public List<Integer> getCompressedPath() {
        List<Integer> compressedPath = new ArrayList<>();

        int tileX;
        int tileY;
        int prevTileX = -1;
        int prevTileY = -1;
        int prevDirection = 0; // 1 for X, 2 for Y

        List<Integer> fullPath = getFullPath();

        // Add the first node
        compressedPath.add(fullPath.get(0));

        for (int i = 0; i < fullPath.size(); i++) {
            int nodeCheck = fullPath.get(i);

            tileX = nodeCheck % 6;
            tileY = (int) Math.floor(nodeCheck / 6f);

            // Check if the direction of the path changed, and if so add the corner node
            if (tileX == prevTileX) {
                if (prevDirection == 2) {
                    compressedPath.add(fullPath.get(i - 1));
                }
                prevDirection = 1;
            } else if (tileY == prevTileY) {
                if (prevDirection == 1) {
                    compressedPath.add(fullPath.get(i - 1));
                }
                prevDirection = 2;
            }

            prevTileX = tileX;
            prevTileY = tileY;
        }

        // Add the last node
        compressedPath.add(fullPath.get(fullPath.size() - 1));

        return compressedPath;
    }

    public List<Integer> getFullPath() {
        processOpenList();

        List<Integer> path = new ArrayList<>();
        int currentNode = start;

        while(true) {
            path.add(currentNode);

            if(getNodeCost(nodeArray[currentNode]) == 0) {
                break;
            }

            currentNode = nodeArray[currentNode].nextNode;
        }

        return path;
    }

    public void updateStart(int start) {
        nodeArray[this.start].cost = -1;
        nodeArray[this.start].state = Node.NodeState.OPEN;
        openList.add(this.start);
        this.start = start;
        nodeArray[start].cost = 0;
        processOpenList();
    }

    public void updateEnd(int end) {
        nodeArray[this.end].cost = -1;
        nodeArray[this.end].state = Node.NodeState.OPEN;
        openList.add(this.end);
        this.end = end;
        nodeArray[end].cost = 0;
        processOpenList();
    }

    public void markBlocked(int node) {
        nodeArray[node].obstacle = true;
        nodeArray[node].state = Node.NodeState.OPEN;
        openList.add(node);
        obstacles.add(node);
        processOpenList();
    }

    public void markOpen(int node) {
        nodeArray[node].cost = -1;
        nodeArray[node].obstacle = false;
        nodeArray[node].state = Node.NodeState.OPEN;
        openList.add(node);
        obstacles.remove((Object)node);
        processOpenList();
    }

    public void processOpenList() {
        while(openList.size() > 0) {
            List<Integer> processList = new ArrayList<>(openList);
            for(Integer nodeIndex : processList) {
                Node node = nodeArray[nodeIndex];
                expand(node);
                openList.remove(nodeIndex);
                node.state = Node.NodeState.CLOSED;
            }
        }
    }

    private void expand(Node node) {
        boolean raise = checkRaise(node);
        int costThroughCurrent = getNodeCost(node) + 1;
        for(Integer neighborIdx : node.neighbors) {
            Node neighbor = nodeArray[neighborIdx];
            if(raise) {
                if(neighbor.nextNode == node.ownIndex) {
                    neighbor.state = Node.NodeState.OPEN;
                    neighbor.cost = costThroughCurrent;
                    openList.add(neighborIdx);
                } else if(costThroughCurrent < getNodeCost(neighbor)) {
                    neighbor.nextNode = node.ownIndex;
                    neighbor.state = Node.NodeState.OPEN;
                    neighbor.cost = costThroughCurrent;
                    openList.add(neighborIdx);
                }
            } else {
                if(costThroughCurrent < getNodeCost(neighbor)) {
                    neighbor.nextNode = node.ownIndex;
                    neighbor.state = Node.NodeState.OPEN;
                    neighbor.cost = costThroughCurrent;
                    openList.add(neighborIdx);
                }
            }
        }
    }

    private int getNodeCost(Node node) {
        if(node.obstacle) return node.cost = nodeArray.length;

        if(node.state == Node.NodeState.CLOSED) return node.cost;

        if(node.cost != 0) {
            return node.cost = getNodeCost(nodeArray[node.nextNode]) + 1;
        } else {
            return 0;
        }
    }

    private boolean checkRaise(Node node) {
        int cost = getNodeCost(node);
        boolean costChanged = false;

        for(Integer neighborIdx : node.neighbors) {
            Node neighbor = nodeArray[neighborIdx];
            int neighborCost = getNodeCost(neighbor);
            if(neighborCost < cost) {
                cost = neighborCost;
                node.cost = cost;
                node.nextNode = neighborIdx;
                node.state = Node.NodeState.RAISE;
                costChanged = true;
                break;
            }
        }

        return costChanged;
    }
}

class Node {
    enum NodeState {
        NEW,
        OPEN,
        CLOSED,
        RAISE,
        LOWER
    }

    int ownIndex = -1;
    int cost = -1;
    int nextNode = -1;
    boolean obstacle = false;

    Integer[] neighbors;

    NodeState state = NodeState.NEW;
}
