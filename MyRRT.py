import pygame
from MyRRT_base import RRTGraph
from MyRRT_base import RRTMap
import time

def main():
    dimensions =(512,512)
    start=(50,50)
    goal=(450,450)
    obsdim=30
    obsnum=30
    iteration=0
    num = 0
    t1=0
    pygame.init()
    map=RRTMap(start,goal,dimensions,obsdim,obsnum)
    dmax = map.nodeRad*10
    graph=RRTGraph(start,goal,dmax,dimensions,obsdim,obsnum)

    #obstacles=graph.makeRandomObs()
    obstacles=graph.makeStaticObs()
    map.drawMap(obstacles)
    font = pygame.font.Font('freesansbold.ttf', 16)
    textRect = pygame.Rect((map.MapDimensions[0] / 2, 15), (0, 200))

    iteration = 0
    t1=time.time()
    pygame.time.delay(9000)
    while (not graph.path_to_goal()):
        pygame.time.delay(20)
        elapsed=time.time()-t1
        t1=time.time()
        #raise exception if timeout
        if elapsed > 10000:
            print('timeout re-initiating the calculations')
            raise

        n_old = graph.number_of_nodes()
        X, Y, Parent = graph.extend()
        n_new = graph.number_of_nodes()

        text = font.render(str(iteration), True, map.black, map.white)
        map.map.blit(text, textRect)

        for i in range(n_old, n_new):
            pygame.draw.line(map.map, map.Blue, (X[i], Y[i]), (X[Parent[i]], Y[Parent[i]]),
                             map.edgeThickness)

        pygame.display.update()
        iteration += 1

    map.drawPath(graph.getPathCoords())
    pygame.display.update()
    print(graph.cost(graph.number_of_nodes()-1))
    pygame.event.clear()
    pygame.event.wait(0)

if __name__ == '__main__':
    result=False
    while not result:
        try:
            main()
            result=True
        except Exception as e:
            result=False
            print(e)
