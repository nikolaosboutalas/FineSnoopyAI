import heapq


def main():
    n = int(input("Missionaries/Cannibals: "))
    boat_capacity = int(input("Boat's capacity: "))
    max_traversals = int(input("Maximum # of traversals: "))
    neighbour_generator = LeftBankStateNeighbourGenerator(n, boat_capacity)
    initial_left_bank_state = LeftBankStateBuilder(neighbour_generator) \
        .with_missionaries(n) \
        .with_cannibals(n) \
        .with_boat_position(True) \
        .build()
    goal_left_bank_state = LeftBankStateBuilder(neighbour_generator).build()
    AStarAlgorithm(initial_left_bank_state, goal_left_bank_state, max_traversals) \
        .traverse() \
        .print()


class LeftBankState:
    def __init__(self, missionaries, cannibals, boat_position, predecessor, neighbour_generator):
        self.__missionaries = missionaries
        self.__cannibals = cannibals
        self.__boat_position = boat_position
        self.__predecessor = predecessor
        self.__neighbour_generator = neighbour_generator
        self.__cost = 0 if self.__predecessor is None else self.__predecessor.cost() + 1
        self.__heuristic = self.__missionaries * self.__missionaries + self.__cannibals * self.__cannibals + 2 * self.__missionaries * self.__cannibals

    def missionaries(self):
        return self.__missionaries

    def cannibals(self):
        return self.__cannibals

    def boat_position(self):
        return self.__boat_position

    def predecessor(self):
        return self.__predecessor

    def neighbour_generator(self):
        return self.__neighbour_generator

    def cost(self):
        return self.__cost

    def heuristic(self):
        return self.__heuristic

    def neighbours(self):
        return self.__neighbour_generator.generate(self)

    def __eq__(self, to):
        return self.__missionaries == to.__missionaries and self.__cannibals == to.__cannibals and self.__boat_position == to.__boat_position

    def __hash__(self):
        return hash((self.__missionaries, self.__cannibals, self.__boat_position))

    def __str__(self):
        return 'State of left bank is: Missionaries:{0} Cannibals:{1} Boat:{2}'.format(self.__missionaries,
                                                                                       self.__cannibals,
                                                                                       self.__boat_position)

    def __lt__(self, other):
        return self.__cost + self.__heuristic < other.cost() + other.heuristic()


class LeftBankStateBuilder:
    def __init__(self, neighbour_generator):
        self.missionaries = 0
        self.cannibals = 0
        self.boat_position = False
        self.predecessor = None
        self.neighbour_generator = neighbour_generator

    def build(self):
        return LeftBankState(self.missionaries, self.cannibals, self.boat_position, self.predecessor,
                             self.neighbour_generator)

    def with_missionaries(self, missionaries):
        self.missionaries = missionaries
        return self

    def with_cannibals(self, cannibals):
        self.cannibals = cannibals
        return self

    def with_boat_position(self, boat_position):
        self.boat_position = boat_position
        return self

    def with_predecessor(self, predecessor):
        self.predecessor = predecessor
        return self


class LeftBankStateNeighbourGenerator:
    def __init__(self, n, boat_capacity):
        self.__n = n
        self.__actions = self.__generate_actions(boat_capacity)

    def generate(self, node):
        neighbours = []
        if node.boat_position():
            for action in self.__actions:
                missionaries = node.missionaries() - action[0]
                cannibals = node.cannibals() - action[1]
                if self.__is_valid_balance(missionaries, cannibals):
                    neighbour = LeftBankStateBuilder(node.neighbour_generator()) \
                        .with_missionaries(missionaries) \
                        .with_cannibals(cannibals) \
                        .with_boat_position(False) \
                        .with_predecessor(node) \
                        .build()
                    neighbours.append(neighbour)
        else:
            for action in self.__actions:
                missionaries = node.missionaries() + action[0]
                cannibals = node.cannibals() + action[1]
                if self.__is_valid_balance(missionaries, cannibals):
                    neighbour = LeftBankStateBuilder(node.neighbour_generator()) \
                        .with_missionaries(missionaries) \
                        .with_cannibals(cannibals) \
                        .with_boat_position(True) \
                        .with_predecessor(node) \
                        .build()
                    neighbours.append(neighbour)
        return neighbours

    def __is_valid_balance(self, missionaries, cannibals):
        return 0 <= missionaries <= self.__n and 0 <= cannibals <= self.__n and missionaries >= cannibals

    @staticmethod
    def __generate_actions(boat_capacity):
        actions = set()
        for i in range(1, boat_capacity + 1):
            actions.add((i, 0))
            actions.add((0, i))
            if i >= boat_capacity - i:
                actions.add((i, boat_capacity - i))
        return actions


class AStarAlgorithm:
    def __init__(self, start_node, finish_node, max_traversals):
        self.__unexamined_nodes = list([start_node])
        self.__examined_nodes = set()
        self.__finish_node = finish_node
        self.__max_traversals = max_traversals

    def traverse(self):
        while len(self.__unexamined_nodes) > 0 and self.__unexamined_nodes[0] != self.__finish_node:
            current_node = heapq.heappop(self.__unexamined_nodes)
            if current_node.cost() < self.__max_traversals:
                neighbours = current_node.neighbours()
                for neighbour in neighbours:
                    if neighbour not in self.__examined_nodes:
                        heapq.heappush(self.__unexamined_nodes, neighbour)
            self.__examined_nodes.add(current_node)
        return self

    def print(self):
        if len(self.__unexamined_nodes) == 0:
            print("No solution")
        else:
            node = self.__unexamined_nodes[0]
            print("Solution with cost: {0} and path: ".format(node.cost()))
            path = []
            while node.predecessor() is not None:
                path.insert(0, node)
                node = node.predecessor()
            path.insert(0, node)
            for node in path:
                print(node)


if __name__ == "__main__":
    main()
