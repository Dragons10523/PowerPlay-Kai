package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;

public class DStar {
    Node[] nodeArray;
    List<Integer> openList;

    private int gridX;
    private int gridY;

    private int start;
    private int end;

    public DStar(int gridX, int gridY, int start, int end) {
        this.gridX = gridX;
        this.gridY = gridY;
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
