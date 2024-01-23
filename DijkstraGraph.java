// --== CS400 File Header Information ==--
// Name: Akshay Gona
// Email: gona@wisc.edu
// Group and Team: BH Blue
// Group TA: Naman Gupta
// Lecturer: Gary Dahl
// Notes to Grader: <optional extra notes>

import org.junit.jupiter.api.Test;

import java.util.*;

import static org.junit.jupiter.api.Assertions.*;

/**
 * This class extends the BaseGraph data structure with additional methods for
 * computing the total cost and list of node data along the shortest path
 * connecting a provided starting to ending nodes.  This class makes use of
 * Dijkstra's shortest path algorithm.
 */
public class DijkstraGraph<NodeType, EdgeType extends Number>
    extends BaseGraph<NodeType,EdgeType>
    implements GraphADT<NodeType, EdgeType> {

    /**
     * While searching for the shortest path between two nodes, a SearchNode
     * contains data about one specific path between the start node and another
     * node in the graph.  The final node in this path is stored in it's node
     * field.  The total cost of this path is stored in its cost field.  And the
     * predecessor SearchNode within this path is referened by the predecessor
     * field (this field is null within the SearchNode containing the starting
     * node in it's node field).
     * <p>
     * SearchNodes are Comparable and are sorted by cost so that the lowest cost
     * SearchNode has the highest priority within a java.util.PriorityQueue.
     */
    protected class SearchNode implements Comparable<SearchNode> {
        public Node node;
        public double cost;
        public SearchNode predecessor;

        public SearchNode(Node node, double cost, SearchNode predecessor) {
            this.node = node;
            this.cost = cost;
            this.predecessor = predecessor;
        }

        public int compareTo(SearchNode other) {
            if (cost > other.cost)
                return +1;
            if (cost < other.cost)
                return -1;
            return 0;
        }
    }

    /**
     * This helper method creates a network of SearchNodes while computing the
     * shortest path between the provided start and end locations.  The
     * SearchNode that is returned by this method is represents the end of the
     * shortest path that is found: it's cost is the cost of that shortest path,
     * and the nodes linked together through predecessor references represent
     * all of the nodes along that shortest path (ordered from end to start).
     *
     * @param start the data item in the starting node for the path
     * @param end   the data item in the destination node for the path
     * @return SearchNode for the final end node within the shortest path
     * @throws NoSuchElementException when no path from start to end is found
     *                                or when either start or end data do not correspond to a graph node
     */
    protected SearchNode computeShortestPath(NodeType start, NodeType end) {
        // TODO: implement in step 6
        Node startNode = nodes.get(start);
        Node endNode = nodes.get(end);
        if (startNode == null || endNode == null) {
            throw new NoSuchElementException("There are no such nodes in the graph");
        }
        //make new nodes for start and end, and check if they are null or not
        SearchNode startSearch = new SearchNode(startNode, 0, null);
        //create a new SearchNode for the start node
        PriorityQueue<SearchNode> searchQueue = new PriorityQueue<>();
        //create a new priority queue to store the nodes
        searchQueue.add(startSearch);
        //add the start node to the queue


        Hashtable<NodeType, Double> visitedNodes = new Hashtable<>();
        //make a new Hashtable to store the visited nodes

        //exploring until we find the end node or exhaust all possibilities, at which point we throw an exception
        while (!searchQueue.isEmpty()) {
            // get the neighbor nodes from the priority queue
            SearchNode current = searchQueue.poll();
            //skip if node is already in the set of explored nodes
            if (visitedNodes.contains(current.node)) {
                continue;
            }

            // return the current node if it is the end node
            if (current.node == endNode) {
                return current;
            }

            // otherwise, add this node to the set of explored nodes
            visitedNodes.put(current.node.data, current.cost);

            // add all neighboring nodes to the priority queue
            for (Edge edge : current.node.edgesLeaving) {
                Node neighbor = edge.successor;
                double totalCost = current.cost + edge.data.doubleValue();
                SearchNode neighborSearchNode = new SearchNode(neighbor, totalCost, current);
                searchQueue.add(neighborSearchNode);
            }
        }
        // if we've exhausted all possibilities, throw an exception
        throw new NoSuchElementException("There is no path between the given start and end nodes");

    }

    /**
     * Returns the list of data values from nodes along the shortest path
     * from the node with the provided start value through the node with the
     * provided end value.  This list of data values starts with the start
     * value, ends with the end value, and contains intermediary values in the
     * order they are encountered while traversing this shorteset path.  This
     * method uses Dijkstra's shortest path algorithm to find this solution.
     *
     * @param start the data item in the starting node for the path
     * @param end   the data item in the destination node for the path
     * @return list of data item from node along this shortest path
     */
    public List<NodeType> shortestPathData(NodeType start, NodeType end) {
        // TODO: implement in step 7
        SearchNode endNode = computeShortestPath(start, end);
        //get the end node from the computeShortestPath method
        LinkedList<NodeType> path = new LinkedList<NodeType>();
        //create a new linked list to store the path
        while (endNode != null) {
            path.addFirst(endNode.node.data);
            endNode = endNode.predecessor;
        }
        //add the nodes to the linked list by working backwards from the end node, through the .predecessor field
        return path;
    }

    /**
     * Returns the cost of the path (sum over edge weights) of the shortest
     * path freom the node containing the start data to the node containing the
     * end data.  This method uses Dijkstra's shortest path algorithm to find
     * this solution.
     *
     * @param start the data item in the starting node for the path
     * @param end   the data item in the destination node for the path
     * @return the cost of the shortest path between these nodes
     */
    public double shortestPathCost(NodeType start, NodeType end) {
        // TODO: implement in step 7
        SearchNode endNode = computeShortestPath(start, end);
        //get the end node from the computeShortestPath method
        return endNode.cost;
        //return the cost of the end node, which is updated in the computeShortestPath method
    }

    // TODO: implement 3+ tests in step 8.

    /**
     * Test method for DijkstraGraph, uses the graph from lecture. This Test method specifically tests the graph path and weight from
     * Node D to Node I
     */
    @Test public void test1() {
        DijkstraGraph<String, Integer> graph = new DijkstraGraph<>();
        graph.insertNode("A");
        graph.insertNode("B");
        graph.insertNode("D");
        graph.insertNode("E");
        graph.insertNode("F");
        graph.insertNode("G");
        graph.insertNode("H");
        graph.insertNode("I");
        graph.insertNode("L");
        graph.insertNode("M");
        graph.insertEdge("D", "A", 7);
        graph.insertEdge("D", "G", 2);
        graph.insertEdge("A", "B", 1);
        graph.insertEdge("A", "H", 8);
        graph.insertEdge("A", "M", 5);
        graph.insertEdge("G", "L", 7);
        graph.insertEdge("B", "M", 3);
        graph.insertEdge("H", "B", 6);
        graph.insertEdge("H", "I", 2);
        graph.insertEdge("M", "E", 3);
        graph.insertEdge("M", "F", 4);
        graph.insertEdge("I", "H", 2);
        graph.insertEdge("I", "D", 1);
        graph.insertEdge("I", "L", 5);
        graph.insertEdge("F", "G", 9);
        //create the graph from the lecture

        //D to I was the example in the lecture
        assertEquals(graph.shortestPathCost("D", "I"), 17);
        //cost should be equal to 17
        assertEquals(graph.shortestPathData("D", "I"), Arrays.asList("D", "A", "H", "I"));
        //path should be equal to D->A->H->I
    }

    /**
     * Test method for DijkstraGraph, uses the graph from lecture. This Test method specifically tests the graph path and weight from
     * more nodes in the graph.
     */
    @Test public void test2() {
        DijkstraGraph<String, Integer> graph = new DijkstraGraph<>();
        graph.insertNode("A");
        graph.insertNode("B");
        graph.insertNode("D");
        graph.insertNode("E");
        graph.insertNode("F");
        graph.insertNode("G");
        graph.insertNode("H");
        graph.insertNode("I");
        graph.insertNode("L");
        graph.insertNode("M");
        graph.insertEdge("D", "A", 7);
        graph.insertEdge("D", "G", 2);
        graph.insertEdge("A", "B", 1);
        graph.insertEdge("A", "H", 8);
        graph.insertEdge("A", "M", 5);
        graph.insertEdge("G", "L", 7);
        graph.insertEdge("B", "M", 3);
        graph.insertEdge("H", "B", 6);
        graph.insertEdge("H", "I", 2);
        graph.insertEdge("M", "E", 3);
        graph.insertEdge("M", "F", 4);
        graph.insertEdge("I", "H", 2);
        graph.insertEdge("I", "D", 1);
        graph.insertEdge("I", "L", 5);
        graph.insertEdge("F", "G", 9);

        assertEquals(graph.shortestPathCost("A", "D"), 11);
        assertEquals(graph.shortestPathData("A", "D"), Arrays.asList("A", "H", "I", "D"));
        //test1, checks if the path is correct, and if the cost is correct from A to D
        assertEquals(graph.shortestPathCost("D", "L"), 9);
        assertEquals(graph.shortestPathData("D", "L"), Arrays.asList("D", "G", "L"));
        //test2, checks if the path is correct, and if the cost is correct from D to L
        assertEquals(graph.shortestPathCost("D", "B"), 8);
        assertEquals(graph.shortestPathData("D", "B"), Arrays.asList("D", "A", "B"));
        //test3, checks if the path is correct, and if the cost is correct from D to B
    }

    /**
     * Test method for DijkstraGraph, uses the graph from lecture. This Test method specifically tests the case where there is no path,
     * and an exception is expected to be thrown.
     */
    @Test public void test3() {
        DijkstraGraph<String, Integer> graph = new DijkstraGraph<>();
        graph.insertNode("A");
        graph.insertNode("B");
        graph.insertNode("D");
        graph.insertNode("E");
        graph.insertNode("F");
        graph.insertNode("G");
        graph.insertNode("H");
        graph.insertNode("I");
        graph.insertNode("L");
        graph.insertNode("M");
        graph.insertEdge("D", "A", 7);
        graph.insertEdge("D", "G", 2);
        graph.insertEdge("A", "B", 1);
        graph.insertEdge("A", "H", 8);
        graph.insertEdge("A", "M", 5);
        graph.insertEdge("G", "L", 7);
        graph.insertEdge("B", "M", 3);
        graph.insertEdge("H", "B", 6);
        graph.insertEdge("H", "I", 2);
        graph.insertEdge("M", "E", 3);
        graph.insertEdge("M", "F", 4);
        graph.insertEdge("I", "H", 2);
        graph.insertEdge("I", "D", 1);
        graph.insertEdge("I", "L", 5);
        graph.insertEdge("F", "G", 9);

        Exception e = assertThrows(NoSuchElementException.class, () -> graph.shortestPathData("M", "A"));
        //store the exception in a variable

        String expectedMessage = "There is no path between the given start and end nodes";
        //store the expected message in a variable
        String actualMessage = e.getMessage();
        //store the actual message from exception in a variable
        assertTrue(actualMessage.contains(expectedMessage));
        //check if the actual message contains(is equal to) the expected message
    }
}
