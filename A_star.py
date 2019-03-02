import fileinput
from copy import deepcopy
import heapq

visited = set()


class Node():
    def __init__(self, stacks, sz):
        self.parent = None
        self.state = deepcopy(stacks)
        self.path_cost = 0
        self.heuristic_cost = 0
        self.f_cost = 0
        self.action = None
        self.children = []
        self.stacksize = sz
        self.key = ""
    def __lt__(self, other):
        return self.f_cost < other.f_cost


    def Action(self):
        move = self.action
        item = self.state[move[0]].pop()
        self.state[move[1]].append(item)
        self.path_cost = self.parent.path_cost + 1 + abs(move[1] - move[0])

    def setKey(self):
        for i in range(len(self.state)):
            for j in range(len(self.state[i])):
                self.key += str(j)
            self.key += ";"

    def Children(self,goal):
        for i in range(len(self.state)):
            for j in range(len(self.state)):
                if len(self.state[i]) > 0:
                    if i != j and len(self.state[j]) < self.stacksize:
                        auxNode = Node(self.state, self.stacksize)
                        auxNode.action = (i, j)
                        auxNode.parent = self
                        auxNode.Action()
                        auxNode.setKey()
                        self.children.append(auxNode)
                        auxNode.heuristic(goal)

    def heuristic(self, goal):
        for i in range(len(goal.state)):
            if(self.state[i] != list() and goal.state[i] != list()):
                for j in range(min(len(goal.state[i]), len(self.state[i]))):
                    if(self.state[i][j] != goal.state[i][j]):
                        self.heuristic_cost += 1
        self.f_cost = self.heuristic_cost + self.path_cost



def ordering(a):
    a = a.strip().lstrip('(').rstrip(')').split(',')
    a = list(map(lambda x: x.lstrip(), a))
    return a

def checkHeight(stacks,max_height):
    for stack in stacks:
        if len(stack) > max_height:
            return False
    return True


def print_action(node, actions):
    if (node.parent == None):
        return
    print_action(node.parent, actions)
    actions.append(node.action)


def check_goal(node, goal):
    for i in range(len(goal.state)):
        if(goal.state[i] != ['X']):
            if(goal.state[i] != node.state[i]):
                return False
    return True


if __name__ =="__main__":
    lines = []
    visited = []
    min_heap = []


    lines.append("2" )
    lines.append("(A); (B); (C)")
    lines.append("(A, C); X; X")

    #for line in fileinput.input():
     #   lines.append(line)

    height=int(lines[0])
    ini=lines[1]
    G=lines[2]

    goal=list(map(ordering,G.split(';')))
    stacks=list(map(ordering,ini.split(';')))


    initialNode = Node(stacks, height)
    goalNode = Node(goal, height)


    if not checkHeight(goal, height):
        print('No solution found')
        exit()

    if not checkHeight(initialNode.state, height):
        print('No solution found')
        exit()

    heapq.heappush(min_heap, initialNode)
    while(True):

        if(len(min_heap) == 0):
            print("No solution found")
            exit()

        currentNode = heapq.heappop(min_heap)
        visited.append(currentNode)

        currentNode.Children(goalNode)

        if check_goal(currentNode, goalNode):
            print(currentNode.path_cost)
            path = []
            print_action(currentNode, path)
            print("; ".join(map(str, path)))

            break

        for i in range(len(currentNode.children)):
            heapq.heappush(min_heap, currentNode.children[i])
