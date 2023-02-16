#Node coverage algorithm similar to the EPC strategy
import jpype
import os
import copy
import heapq
import math
import numpy as np
jarpath = os.path.join(os.path.abspath("."), "dkbrics.jar")
jvmPath = jpype.getDefaultJVMPath()
# jpype.startJVM(jvmPath,"-ea", "-Djava.class.path=%s" % (jarpath))
RegExp = jpype.JClass("dk.brics.automaton.RegExp")
class transition:
    def __init__(self, char,toward):
        self.char =char
        self.toward=toward
class point:
    def __init__(self, state):
        self.state = state
        self.trans = []
class automata:
    def __init__(self):
        self.points = []
        self.init=-1
        self.acc = []
class edgePair:
    def __init__(self, state1,char1to2,state2):#,char2to3,state3):
        self.state1 =state1
        self.char1to2=char1to2
        self.state2 =state2
class edgePairPoint:
    def __init__(self, state):
        self.state =state
        self.edgePairs=[]
        self.visited=False
def getCharList(start,end,accept):
    charlist=[]
    char=start
    endplus=chr(ord(end)+1)
    while char!=endplus :
        if char in accept:
            charlist.append(char)
        char=chr(ord(char)+1) 
    return charlist
def checkCharList(charstr,accept):
    chars=[]
    if charstr.find('-')>=0 and len(charstr)>1:
            a=eval(repr(charstr.split('-')[0]).replace('\\\\','\\'))
            b=eval(repr(charstr.split('-')[1]).replace('\\\\', '\\'))
            chars.extend(getCharList(a,b,accept))
    else:
            chars.append(charstr)
    return chars
def getEPPoint(State,adjList):
    for epp in adjList:
        if epp.state==State:
            break
    return epp
def getPoint(State,auto):
    for point in auto.points:
        if point.state==State:
            break
    return point
def DFS(epp,road,adjList,edgePairCoverage):
    flag=False
    epp.visited=True
    for EdgePair in epp.edgePairs:
        nextPoint=getEPPoint(EdgePair.state2,adjList)
        if nextPoint.visited==False:
            road.append(EdgePair)
            DFS(nextPoint,road,adjList,edgePairCoverage)
            flag=True
    if len(road)>0:
        if flag==False:
            edgePairCoverage.append(copy.deepcopy(road))#不能继续前进，存储路径
        road.pop()
def DFSearch(epp,road,adjList,edgePairCoverage):
    DFS(epp,road,adjList,edgePairCoverage)
    for sp in adjList:
        DFS(sp,road,adjList,edgePairCoverage)


def generateEdgeAdjList(auto):
    G={}
    for Point in auto.points:
        G[Point.state]={}
        for Transition in Point.trans:
            G[Point.state][Transition.toward]=1
        G[Point.state][Point.state]=0
    return G

def init_distance(graph, s):
    distance = {s: 0}
    for vertex in graph:
        if vertex != s:
            distance[vertex] = math.inf
    return distance

def dijkstra(graph, s):
    pqueue = []
    heapq.heappush(pqueue, (0, s))
    seen = set()
    parent = {s: None}
    distance = init_distance(graph, s)
    while len(pqueue) > 0:
        pair = heapq.heappop(pqueue)
        dist = pair[0]
        vertex = pair[1]
        seen.add(s)
        nodes = graph[vertex].keys()
        for w in nodes:
            if w not in seen:
                if dist + graph[vertex][w] < distance[w]:
                    heapq.heappush(pqueue, (dist + graph[vertex][w], w))
                    parent[w] = vertex
                    distance[w] = dist + graph[vertex][w]
    return parent, distance

def getFirstEdge(state1,state2,auto):
    thisPoint=getPoint(state1,auto)
    for Transition in thisPoint.trans:
        if Transition.toward==state2:
            break
    new_edge=edge(state1,Transition.char,state2)
    return new_edge

def onMove(edgeRoad,auto,G):
    if edgeRoad[-1].state2 in auto.acc:
        return# edgeRoad
    else:
        parent_dict, distance_dict =dijkstra(G, edgeRoad[-1].state2)
        shortestAcc=-1
        minDistance=math.inf
        for Toward in distance_dict.keys():
            if distance_dict[Toward]<minDistance and Toward in auto.acc:
                shortestAcc=Toward
                minDistance=distance_dict[Toward]
        reachEdges=[]
        thisState=shortestAcc
        while parent_dict[thisState]!=None:
            new_edge=getFirstEdge(parent_dict[thisState],thisState,auto)
            reachEdges.append(new_edge)
            thisState=parent_dict[thisState]
        reachEdges.reverse()
        edgeRoad.extend(reachEdges)
        return
def generateEdgeInAdjList(auto):
    G={}
    for Point in auto.points:
        G[Point.state]={}
    for Point in auto.points:
        for Transition in Point.trans:
            G[Transition.toward][Point.state]=1
        G[Point.state][Point.state]=0
    return G
def onBack(edgeRoad,auto,InG):
    if edgeRoad[0].state1 == auto.init:
        return
    else:
        parent_dict, distance_dict =dijkstra(InG, edgeRoad[0].state1)
        #print(parent_dict)
        reachEdges=[]
        thisState=auto.init
        while parent_dict[thisState]!=None:
            #print(parent_dict[thisState])
            new_edge=getFirstEdge(thisState,parent_dict[thisState],auto)
            reachEdges.append(new_edge)
            thisState=parent_dict[thisState]
        reachEdges.reverse()
        for reachEdge in reachEdges:
            edgeRoad.insert(0 , reachEdge)
        return
def onExtend(edgeRoads,auto):
    CoveredAcc=[]
    for eachRoad in edgeRoads:
        if eachRoad[-1].state2 not in CoveredAcc:
            CoveredAcc.append(eachRoad[-1].state2)
    for eachRoad in edgeRoads:
        thisRoad=copy.deepcopy(eachRoad)
        while len(thisRoad)>0:
            thisRoad.pop()
            if len(thisRoad)>0:
                if thisRoad[-1].state2 not in CoveredAcc and thisRoad[-1].state2 in auto.acc:
                    thatRoad=copy.deepcopy(thisRoad)
                    CoveredAcc.append(thisRoad[-1].state2)
                    #print(generateString(thatRoad),thisRoad[-1].state2)
                    edgeRoads.append(thatRoad)
    #print(CoveredAcc,auto.acc)
def generateString(edgeRoad):
    string=''
    for Edge in edgeRoad:
        string=string+Edge.char1to2
    return string
def EPCoverage(regexStr,accChars):
    regex = RegExp(regexStr)
    automaton=regex.toAutomaton()
    automaton.determinize()
    automaton.minimize()
    comp=automaton.complement()
    dfa=comp.toString()
    dfastr=str(dfa)
    auto=automata()
    dfaList=dfastr.split('\n')
    auto.init=int(dfaList[0].split(': ')[1])
    del dfaList[0]
    for i in dfaList:
        if i[0:5]=='state':
            state_new=int(i.split(' ')[1])
            if state_new!=0:
                auto.points.append(point_new)
            point_new=point(state_new)
            if i.split(' ')[2]=='[accept]:':
                auto.acc.append(state_new)
        elif i=='':
            auto.points.append(point_new)
        else:
            temp=i.split(' -> ' )
            charString=temp[0].strip()
            charList=checkCharList(charString,accChars)
            for theChar in charList:
                transition_new=transition(theChar,int(temp[1]))
                point_new.trans.append(transition_new)
    adjList=[]
    for Point in auto.points:
        new_epp=edgePairPoint(Point.state)
        adjList.append(new_epp)
    for Point in auto.points:
        for Transition in Point.trans:
            nextPoint=getPoint(Transition.toward,auto)
            newEdgePair= edgePair(Point.state, Transition.char, Transition.toward)
            epp = getEPPoint(Point.state,adjList)
            epp.edgePairs.append(newEdgePair)
    road=[]
    edgePairCoverage=[]
    startPoint=getEPPoint(auto.init,adjList)
    DFSearch(startPoint,road,adjList,edgePairCoverage)
    edgeRoads=edgePairCoverage

    G=generateEdgeAdjList(auto)
    InG=generateEdgeInAdjList(auto)
    
    onExtend(edgeRoads,auto)
    
    roadNum=len(edgeRoads)
    delNum=0
    for n in range(0,roadNum):
        #print(edgeRoads)
        try:
            onMove(edgeRoads[n-delNum],auto,G)
            onBack(edgeRoads[n-delNum],auto,InG)
        except:
            #print('remov')
            edgeRoads.remove(edgeRoads[n-delNum])
            #print(edgeRoads)
            delNum+=1
    
    
    Strings=[]
    
    if auto.init in auto.acc:
        Strings.append('')
        
    for eachRoad in edgeRoads:
        Strings.append(generateString(eachRoad))
    return Strings

