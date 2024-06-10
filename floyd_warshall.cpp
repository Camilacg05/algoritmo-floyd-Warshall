#include <bits/stdc++.h>  // Inclui todas as bibliotecas padrão do C++

using namespace std;  

void leituraGrafo(vector<vector<double>> &, int, bool);
void escritaGrafo(vector<vector<double>> &);
void floydWarshall(vector<vector<double>> &);
void imprimirMenoresDistancias(vector<vector<double>> &);

int main()
{
    int n, m;  // Declaração das variáveis para número de vértices (n) e arestas (m)
    cin >> n >> m;  
    vector<vector<double>> Grafo;  // Declara uma matriz de adjacência para o grafo
    Grafo.assign(n, vector<double>(n, numeric_limits<double>::infinity()));  // Inicializa a matriz com "infinito" para representar a ausência de arestas
    for (int i = 0; i < n; ++i) {
        Grafo[i][i] = 0; // Distância de um vértice para si mesmo é 0
    }
    bool direcionado;
    cout << "O grafo é direcionado? (0 para não, 1 para sim): ";
    cin >> direcionado;
    cout << "Digite as arestas no formato ( vértice A, vértice B, Peso da aresta AB):" << endl;
    leituraGrafo(Grafo, m, direcionado);  
    cout << endl <<endl << n << " vértices e " << m << " arestas." << endl; 
    cout << "Grafo Inicial:" << endl;
    escritaGrafo(Grafo);  // Imprime a matriz de adjacência inicial do grafo

    floydWarshall(Grafo);  
    cout << endl << "Grafo Após Floyd-Warshall:" << endl;
    escritaGrafo(Grafo);  // Imprime a matriz de adjacência após executar Floyd-Warshall

    cout << endl << "Menores distâncias:" << endl;
    imprimirMenoresDistancias(Grafo);  // Imprime as menores distâncias entre todos os pares de vértices que possuem arestas entre si

    return 0;  
}

// Função para ler as arestas do grafo
void leituraGrafo(vector<vector<double>> &G, int m, bool direcionado)
{
    int a, b;  // Declaração das variáveis para os vértices de uma aresta
    double c;  // Declaração da variável para o peso da aresta
    for (int i = 0; i < m; i++)
    {
        cin >> a >> b >> c;  
        G[a][b] = c;  // Define o peso da aresta na matriz de adjacência
        if (!direcionado) {
            G[b][a] = c;  // Define o peso da aresta bidirecional (grafos não direcionados)
        }
    }
}

// Função para imprimir a matriz de adjacência do grafo
void escritaGrafo(vector<vector<double>> &G)
{
    int n = G.size();  // Obtém o número de vértices no grafo
    cout << "Matriz de Adjacência:" << endl;
    cout << "   ";
    for (int u = 0; u < n; u++)
    {
        cout << u << " ";  // Imprime o cabeçalho da matriz (números dos vértices)
    }
    cout << endl;
    for (int u = 0; u < n; u++)
    {
        cout << u << ": ";  // Imprime o índice do vértice atual
        for (int v = 0; v < n; v++)
        {
            if (G[u][v] == numeric_limits<double>::infinity()) {
                cout << "inf ";  // Se a distância for infinita, imprime "inf"
            } else {
                cout << G[u][v] << " ";  // Caso contrário, imprime a distância
            }
        }
        cout << endl;  // Nova linha após cada linha da matriz
    }
}

// Implementação do algoritmo de Floyd-Warshall
void floydWarshall(vector<vector<double>> &G)
{
    int n = G.size();  // Obtém o número de vértices no grafo
    for (int k = 0; k < n; k++)  // Itera sobre todos os vértices como vértices intermediários
    {
        for (int i = 0; i < n; i++)  // Itera sobre todos os pares de vértices
        {
            for (int j = 0; j < n; j++)
            {
                if (G[i][k] < numeric_limits<double>::infinity() && G[k][j] < numeric_limits<double>::infinity())  // Verifica se existe um caminho via k
                {
                    G[i][j] = min(G[i][j], G[i][k] + G[k][j]);  // Atualiza a menor distância
                }
            }
        }
    }
}

// Função para imprimir as menores distâncias entre todos os pares de vértices
void imprimirMenoresDistancias(vector<vector<double>> &G)
{
    int n = G.size();  // Obtém o número de vértices no grafo
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (i != j)  // Não imprime a distância do vértice para si mesmo
            {
                if (G[i][j] == numeric_limits<double>::infinity()) {
                    cout << "vertice " << i <<" à "<< j << " distância inf" << endl;  // Se a distância for infinita, imprime "inf"
                } else {
                    cout << "vertice " << i <<" à " << j << " distância " << G[i][j] << endl;  // Caso contrário, imprime a distância
                }
            }
        }
    }
}
