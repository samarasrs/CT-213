from grid import Node, NodeGrid
from math import inf
import heapq


class PathPlanner(object):
    """
    Represents a path planner, which may use Dijkstra, Greedy Search or A* to plan a path.
    """
    def __init__(self, cost_map):
        """
        Creates a new path planner for a given cost map.

        :param cost_map: cost used in this path planner.
        :type cost_map: CostMap.
        """
        self.cost_map = cost_map
        self.node_grid = NodeGrid(cost_map)

    @staticmethod
    def construct_path(goal_node):
        """
        Extracts the path after a planning was executed.

        :param goal_node: node of the grid where the goal was found.
        :type goal_node: Node.
        :return: the path as a sequence of (x, y) positions: [(x1,y1),(x2,y2),(x3,y3),...,(xn,yn)].
        :rtype: list of tuples.
        """
        node = goal_node
        # Since we are going from the goal node to the start node following the parents, we
        # are transversing the path in reverse
        reversed_path = []
        while node is not None:
            reversed_path.append(node.get_position())
            node = node.parent
        return reversed_path[::-1]  # This syntax creates the reverse list

    def dijkstra(self, start_position, goal_position):
        """
        Plans a path using the Dijkstra algorithm.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        # Todo: implement the Dijkstra algorithm
        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path
        self.node_grid.reset()
        pq = []
        node = self.node_grid.get_node(start_position[0], start_position[1])
        node.f = 0
        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])
        heapq.heappush(pq, (node.f, node))
        while len(pq) > 0:
            f, node = heapq.heappop(pq)
            node.closed = True
            if node == goal_node:
                return self.construct_path(goal_node), goal_node.f
            for successor in self.node_grid.get_successors(node.i, node.j):
                node_suc = self.node_grid.get_node(successor[0], successor[1])
                if not node_suc.closed:
                    cost = f + self.cost_map.get_edge_cost((node.i, node.j), (node_suc.i, node_suc.j))
                    if node_suc.f > cost:
                        node_suc.f = cost
                        heapq.heappush(pq, (node_suc.f, node_suc))
                        node_suc.parent = node


    def greedy(self, start_position, goal_position):
        """
        Plans a path using greedy search.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        # Todo: implement the Greedy Search algorithm
        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path
        self.node_grid.reset()
        pq = []
        node = self.node_grid.get_node(start_position[0], start_position[1])
        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])
        start_cost = node.distance_to(goal_position[0], goal_position[1])
        node.f = 0
        heapq.heappush(pq, (start_cost, node))
        node.closed = True
        while len(pq) > 0:
            f, node = heapq.heappop(pq)
            for successor in self.node_grid.get_successors(node.i, node.j):
                node_suc = self.node_grid.get_node(successor[0], successor[1])
                if not node_suc.closed:
                    node_suc.parent = node
                    h = node_suc.distance_to(goal_position[0], goal_position[1])
                    node_suc.f = node.f + self.cost_map.get_edge_cost((node.i, node.j), (node_suc.i, node_suc.j))
                    if node_suc == goal_node:
                        return self.construct_path(goal_node), goal_node.f
                    heapq.heappush(pq, (h, node_suc))
                    node_suc.closed = True

    def a_star(self, start_position, goal_position):
        """
        Plans a path using A*.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        # Todo: implement the A* algorithm
        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path
        self.node_grid.reset()
        pq = []
        node = self.node_grid.get_node(start_position[0], start_position[1])
        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])
        node.g = 0
        node.f = node.distance_to(goal_position[0], goal_position[1])
        heapq.heappush(pq, (node.f, node))
        while len(pq) > 0:
            f, node = heapq.heappop(pq)
            node.closed = True
            if node == goal_node:
                return self.construct_path(goal_node), goal_node.f
            for successor in self.node_grid.get_successors(node.i, node.j):
                node_suc = self.node_grid.get_node(successor[0], successor[1])
                if not node_suc.closed:
                    h = node_suc.distance_to(goal_position[0], goal_position[1])
                    cost = node.g + self.cost_map.get_edge_cost((node.i, node.j), (node_suc.i, node_suc.j)) + h
                    if node_suc.f > cost:
                        node_suc.g = node.g + self.cost_map.get_edge_cost((node.i, node.j), (node_suc.i, node_suc.j))
                        node_suc.f = cost
                        node_suc.parent = node
                        heapq.heappush(pq, (node_suc.f, node_suc))


    def a_star_weight(self, start_position, goal_position):
        """
        Plans a path using A*.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        # Todo: implement the A* algorithm
        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path

        w = 5

        self.node_grid.reset()
        pq = []
        node = self.node_grid.get_node(start_position[0], start_position[1])
        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])
        node.g = 0
        node.f = node.distance_to(goal_position[0], goal_position[1])
        heapq.heappush(pq, (node.f, node))
        while len(pq) > 0:
            f, node = heapq.heappop(pq)
            node.closed = True
            if node == goal_node:
                return self.construct_path(goal_node), goal_node.f
            for successor in self.node_grid.get_successors(node.i, node.j):
                node_suc = self.node_grid.get_node(successor[0], successor[1])
                if not node_suc.closed:
                    h = node_suc.distance_to(goal_position[0], goal_position[1])
                    cost = node.g + self.cost_map.get_edge_cost((node.i, node.j), (node_suc.i, node_suc.j)) + w * h
                    if node_suc.f > cost:
                        node_suc.g = node.g + self.cost_map.get_edge_cost((node.i, node.j), (node_suc.i, node_suc.j))
                        node_suc.f = cost
                        node_suc.parent = node
                        heapq.heappush(pq, (node_suc.f, node_suc))

    def a_star_dynamic_weight(self, start_position, goal_position):
        """
        Plans a path using A*.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        # Todo: implement the A* algorithm
        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path



        self.node_grid.reset()
        pq = []
        node = self.node_grid.get_node(start_position[0], start_position[1])
        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])
        node.g = 0

        n = node.distance_to(goal_position[0], goal_position[1])

        node.f = n
        heapq.heappush(pq, (node.f, node))

        e = 5


        while len(pq) > 0:
            f, node = heapq.heappop(pq)
            node.closed = True
            if node == goal_node:
                return self.construct_path(goal_node), goal_node.f
            for successor in self.node_grid.get_successors(node.i, node.j):
                node_suc = self.node_grid.get_node(successor[0], successor[1])
                if not node_suc.closed:
                    h = node_suc.distance_to(goal_position[0], goal_position[1])

                    d = h

                    w = e * (1 - d / n)

                    cost = node.g + self.cost_map.get_edge_cost((node.i, node.j), (node_suc.i, node_suc.j)) + w * h
                    if node_suc.f > cost:
                        node_suc.g = node.g + self.cost_map.get_edge_cost((node.i, node.j), (node_suc.i, node_suc.j))
                        node_suc.f = cost
                        node_suc.parent = node
                        heapq.heappush(pq, (node_suc.f, node_suc))

    def a_star_pxWU(self, start_position, goal_position):
        """
        Plans a path using A*.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        # Todo: implement the A* algorithm
        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path

        w = 2

        self.node_grid.reset()
        pq = []
        node = self.node_grid.get_node(start_position[0], start_position[1])
        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])
        node.g = 0
        n = node.distance_to(goal_position[0], goal_position[1])

        node.f = n
        heapq.heappush(pq, (node.f, node))

        e = 5
        while len(pq) > 0:
            f, node = heapq.heappop(pq)
            node.closed = True
            if node == goal_node:
                return self.construct_path(goal_node), goal_node.f
            for successor in self.node_grid.get_successors(node.i, node.j):
                node_suc = self.node_grid.get_node(successor[0], successor[1])
                if not node_suc.closed:
                    h = node_suc.distance_to(goal_position[0], goal_position[1])
                    g = node.g + self.cost_map.get_edge_cost((node.i, node.j), (node_suc.i, node_suc.j))

                    d = h

                    w = e * (1 - d / n)

                    if g < ((2 * w - 1) * h):
                        cost = g/(2 * w - 1) + h
                    else:
                        cost = (g + h) / w
                    if node_suc.f > cost:
                        node_suc.g = node.g + self.cost_map.get_edge_cost((node.i, node.j), (node_suc.i, node_suc.j))
                        node_suc.f = cost
                        node_suc.parent = node
                        heapq.heappush(pq, (node_suc.f, node_suc))

    def a_star_pxWD(self, start_position, goal_position):
        """
        Plans a path using A*.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        # Todo: implement the A* algorithm
        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path

        w = 10

        self.node_grid.reset()
        pq = []
        node = self.node_grid.get_node(start_position[0], start_position[1])
        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])
        node.g = 0
        n = node.distance_to(goal_position[0], goal_position[1])

        node.f = n
        heapq.heappush(pq, (node.f, node))

        e = 5
        while len(pq) > 0:
            f, node = heapq.heappop(pq)
            node.closed = True
            if node == goal_node:
                return self.construct_path(goal_node), goal_node.f
            for successor in self.node_grid.get_successors(node.i, node.j):
                node_suc = self.node_grid.get_node(successor[0], successor[1])
                if not node_suc.closed:
                    h = node_suc.distance_to(goal_position[0], goal_position[1])
                    g = node.g + self.cost_map.get_edge_cost((node.i, node.j), (node_suc.i, node_suc.j))

                    d = h

                    w = e * (1 - d / n)

                    if g < h:
                        cost = g + h
                    else:
                        cost = (g + (2 * w - 1) * h) / w

                    if node_suc.f > cost:
                        node_suc.g = node.g + self.cost_map.get_edge_cost((node.i, node.j), (node_suc.i, node_suc.j))
                        node_suc.f = cost
                        node_suc.parent = node
                        heapq.heappush(pq, (node_suc.f, node_suc))