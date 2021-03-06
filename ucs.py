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
        self.action = None
        self.children = []
        self.stacksize = sz
        self.key = ""

    def __lt__(self, other):
        return self.path_cost < other.path_cost


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

    def Children(self):
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



def ordering(data):
    data = data.strip().lstrip('(').rstrip(')').split(',')
    data=list(filter(None, data))
    data = list(map(lambda x: x.lstrip(), data))
    return data

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
    minHeap = []
    count =0

    #lines.append("2" )
    #lines.append("(A,D); (B); (C)")
    #lines.append("(A, C); X; X")

    for line in fileinput.input():
        lines.append(line)

    height=int(lines[0])
    ini=lines[1]
    G=lines[2]

    goal=list(map(ordering,G.split(';')))
    stacks=list(map(ordering,ini.split(';')))


    initial = Node(stacks, height)
    goalNode = Node(goal, height)


    if not checkHeight(goal, height):
        print('No solution found')
        exit()

    if not checkHeight(initial.state, height):
        print('No solution found')
        exit()

    heapq.heappush(minHeap, initial)
    while(True):

        if(len(minHeap) == 0):
            print("No solution found")
            exit()

        current = heapq.heappop(minHeap)
        count += 1
        visited.append(current)

        current.Children()

        if check_goal(current, goalNode):
            print(current.path_cost)
            path = []
            print_action(current, path)
            print("; ".join(map(str, path)))

            break

        for i in range(len(current.children)):
            heapq.heappush(minHeap, current.children[i])
