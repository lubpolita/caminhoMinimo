import random
from math import inf
import time

#Nomes: Ismael Prado, Luiza Borges Polita
#Matrícula: 18.1.8022, 18.1.8168

#Preencher dados
def preencher():
    V = int(input('Digite o numero de vertices: '))
    E = int(input('Digite o numero de arestas: '))
    W_max = int(input('Digite o peso máximo: '))
    Origem = int(input('Digite o vertice de origem: '))
    Destino =int(input('Digite o vertice de destino: '))
    return V,E,W_max,Origem,Destino

#Cria o grafo 
def Criar(V, E, W):
    G = [[0 for i in range(V)] for i in range(V)]
    i = 0
    while i < E:
        u = random.randint(0, V - 1) 
        v = random.randint(0, V - 1) 
        w = random.randint(1,W) 
        if u != v and G[u][v] == 0:
            G[u][v] = w
            G[v][u] = w
            i += 1
    return G

# converte de matriz de adjacência para Lista de Adjacência 
def converter(a): 
    ListaAdj = [] 
    for i in range(len(a)): 
        for j in range(len(a[i])): 
                       if a[i][j]!= 0: 
                           w = a[i][j]
                           ListaAdj.append((i,j,w)) #Cria uma lista com os dados dos vertices e o peso
    #print(ListaAdj)
    #print('Criou Lista')
    return ListaAdj 

#bellman ford
def bellman_ford(G, Origem, Destino):
    
    distance = [[] for _ in range(len(G))] #Vetor que armazena a distância de s a cada vértice
    predecessor = [[] for _ in range(len(G))] #Vetor que armazena o predecessor de cada vértice
    #Variaveis para clcular o caminho e o custo
    L_caminho = []
    count = Destino
    custo = 0

    ListaAdj = converter(G)

    #Inicia a distância para todos os vertices como infinito e predecessor vazio
    for v in range(len(G)):
        distance[v] = inf        
        predecessor[v] = None         
    distance[Origem] = 0   

    for i in range(len(G)-1):
        Trocou = False
        for u, v, w in ListaAdj:
            if distance[u] + w < distance[v]:
                distance[v] = distance[u] + w
                predecessor[v] = u 
                Trocou = True
        if(Trocou == False):
            break 
    i = -1
    L_caminho.append(Destino)       #Adiciona o vértice destino na lista
    while i != Origem:              #Realizar o loop enquanto i não for o vertice destino
        i = predecessor[count]      #i recebe o valor do predecessor do vertice 
        custo += distance[count]    #Custo armazena o valor da distancia ate o vertice
        count = i                   #Count passa a ter o valor do predecessor
        L_caminho.append(i)         #Adiciona o predecessor na lista
        
    L_caminho.reverse() #Inverte a ordem da lista
    #prints para ver se o algoritmo está funcionando coretamente 
    #print('Distancia: ' , distance , '\nPredecessor: ' , predecessor)
    print('Algoritmo: Bellman-Ford')
    print('Origem: %d' % (Origem))
    print('Destino: %d' %(Destino))
    print('Caminho: ' , L_caminho)
    print('Custo: ' , custo)

#Dijkstra
def dijkstra(G, origem,Destino):
    dist = [[] for _ in range(len(G))] #Vetor que armazena a distância de s a cada vértice
    pred = [[] for _ in range(len(G))] #Vetor que armazena o predecessor de cada vértice
    restantes = []
    vertices = []
    #Variaveis para calcular o caminho e o custo
    L_caminho = []
    count = Destino
    custo = 0

    listaAdj = converter(G)
    #print(listaAdj)
    #encontrando vertices
    for i in range(len(G)):
        restantes.append(i)
        vertices.append(i)
    #inicialização das distancias e predecessores
    for v in restantes:
        dist[v] = float('inf')
        pred[v] = None
    dist[origem] = 0

    #laço principal
    while restantes:
        menor = None

        #analisando qual vertice tem a menor distancia
        for v in restantes:
            if menor == None:
                menor = v
            elif dist[v] < dist[menor]:
                menor = v

        #analisando qual adj teria menor distancia
        for v, u, w in listaAdj:
            if v == menor:
                if dist[u] > dist[menor] + w:
                    dist[u] = dist[menor] + w
                    pred[u] = menor
        restantes.remove(menor)

    i = -1 

    L_caminho.append(Destino)       #Adiciona o vértice destino na lista
    while i != origem:              #Realizar o loop enquanto i não for o vertice destino
        i = pred[count]             #i recebe o valor do predecessor do vertice
        custo += dist[count]        #Custo armazena o valor da distancia ate o vertice
        count = i                   #Count passa a ter o valor do predecessor
        L_caminho.append(i)         #Adiciona o predecessor na lista     

    L_caminho.reverse() #Inverte a ordem da lista
    #prints para ver se o algoritmo está funcionando coretamente 
    #print("DISTANCIA:     ", dist)
    #print("PREDECESSORES: ",pred)
    print('Algoritmo: Dijkstra')
    print('Origem: %d' % (origem))
    print('Destino: %d' %(Destino))
    print('Caminho: ' , L_caminho)
    print('Custo: ' , custo)

#Imprimir dist e pred Floyd Warshall(apenas para melhor visualização para testar o algoritmo)
#def imprimir(matriz):
#    print("-------------IMPRIMINDO MATRIZ-------------")
#    for i in range(len(matriz)):
#        print(matriz[i])

#Floyd Warshall
def floyd(G, origem, Destino):
    dist= [[0 for i in range(len(G))] for j in range(len(G))] #Matriz que armazena a distância de s a cada vértice
    pred= [[0 for i in range(len(G))] for j in range(len(G))] #Matriz que armazena o predecessor de cada vértice
    #Variaveis para clcular o caminho e o custo
    L_caminho = []
    count = Destino
    custo = 0

    #inicialização
    for i in range(len(G)):
        for j in range(len(G)):
            if i == j:
                dist[i][j] = 0
                pred[i][j] = None
            elif G[i][j] != 0:
                dist[i][j] = G[i][j]
                pred[i][j] = i
            elif G[i][j] == 0:
                dist[i][j] = float('inf')
                pred[i][j] = None

    for k in range(len(G)):
        for i in range(len(G)):
            for j in range(len(G)):
                if dist[i][j] > dist[i][k] + dist[k][j]:
                    dist[i][j] = dist[i][k] + dist[k][j]
                    pred[i][j] = pred [k][j]

    #Achar o caminho e o Custo               
    i = -1
    L_caminho.append(Destino)           #Adiciona o vértice destino na lista
    while i != origem:                  #Realizar o loop enquanto i não for o vertice destino
        i = pred[origem][count]         #i recebe o valor do predecessor do vertice
        custo += dist[origem][count]    #Custo armazena o valor da distancia ate o vertice
        count = i                       #Count passa a ter o valor do predecessor
        L_caminho.append(i)             #Adiciona o predecessor na lista  
        
    L_caminho.reverse()#Inverte a ordem da lista
    #prints para ver se o algoritmo está funcionando coretamente 
    #print("DISTANCIA:")
    #imprimir(dist)
    #print("PREDECESSOR:")
    #imprimir(pred)
    print('Algoritmo: Floyd Warshall')
    print('Origem: %d' % (origem))
    print('Destino: %d' %(Destino))
    print('Caminho: ' , L_caminho)
    print('Custo: ' , custo)
    

#Menu
Algo = 0
while Algo != 4:
    print('''    [ 1 ] Algoritmo Dijkstra
    [ 2 ] Algoritmo Bellman-Ford
    [ 3 ] Algoritmo Floyd-Warshall
    [ 4 ] Sair do Programa''')
    Algo = int(input('>>>>>Qual sua opção? '))

    if Algo == 1:
        #Dijkstra
        V,E,W,Vorigem,Vdestino = preencher()
        print('=-=' *13)
        inicio = time.time()
        Grafo = Criar(V, E, W)
        dijkstra(Grafo,Vorigem,Vdestino)
        fim = time.time()
        print('Tempo: %.3f' % (fim-inicio))
    elif Algo == 2:
        #Bellman-Ford
        V,E,W,Vorigem,Vdestino = preencher()
        print('=-=' *13)
        inicio = time.time()
        Grafo = Criar(V, E, W)
        Lista = converter(Grafo)
        bellman_ford(Grafo, Vorigem, Vdestino)
        fim = time.time()
        print('Tempo: %.3f' % (fim-inicio))
    elif Algo == 3:
        #Floyd-Warshall
        V,E,W,Vorigem,Vdestino = preencher()
        print('=-=' *13)
        inicio = time.time()
        Grafo = Criar(V, E, W)
        floyd(Grafo,Vorigem, Vdestino)
        fim = time.time()
        print('Tempo: %.3f' % (fim-inicio))
    elif Algo == 4:
        print('Finalizando...')
    else:
        print('Você digitou um número inválido!')
    print('=-=' *13)


