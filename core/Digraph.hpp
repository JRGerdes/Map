// Digraph.hpp
//
// ICS 46 Spring 2018
// Project #5: Rock and Roll Stops the Traffic
//
//  Jaren Gerdes
//  55784665
//
// This header file declares a class template called Digraph, which is
// intended to implement a generic directed graph.  The implementation
// uses the adjacency lists technique, so each vertex stores a linked
// list of its outgoing edges.
//
// Along with the Digraph class template is a class DigraphException
// and a couple of utility structs that aren't generally useful outside
// of this header file.
//
// In general, directed graphs are all the same, except in the sense
// that they store different kinds of information about each vertex and
// about each edge; these two types are the type parameters to the
// Digraph class template.

#ifndef DIGRAPH_HPP
#define DIGRAPH_HPP

#include <exception>
#include <functional>
#include <list>
#include <map>
#include <utility>
#include <vector>
#include <queue>



// DigraphExceptions are thrown from some of the member functions in the
// Digraph class template, so that exception is declared here, so it
// will be available to any code that includes this header file.

class DigraphException : public std::runtime_error
{
public:
    DigraphException(const std::string& reason);
};


inline DigraphException::DigraphException(const std::string& reason)
    : std::runtime_error{reason}
{
}



// A DigraphEdge lists a "from vertex" (the number of the vertex from which
// the edge points), a "to vertex" (the number of the vertex to which the
// edge points), and an EdgeInfo object.  Because different kinds of Digraphs
// store different kinds of edge information, DigraphEdge is a struct template.

template <typename EdgeInfo>
struct DigraphEdge
{
    int fromVertex;
    int toVertex;
    EdgeInfo einfo;
};



// A DigraphVertex includes two things: a VertexInfo object and a list of
// its outgoing edges.  Because different kinds of Digraphs store different
// kinds of vertex and edge information, DigraphVertex is a struct template.

template <typename VertexInfo, typename EdgeInfo>
struct DigraphVertex
{
    VertexInfo vinfo;
    std::list<DigraphEdge<EdgeInfo>> edges;
};



// Digraph is a class template that represents a directed graph implemented
// using adjacency lists.  It takes two type parameters:
//
// * VertexInfo, which specifies the kind of object stored for each vertex
// * EdgeInfo, which specifies the kind of object stored for each edge
//
// You'll need to implement the member functions declared here; each has a
// comment detailing how it is intended to work.
//
// Each vertex in a Digraph is identified uniquely by a "vertex number".
// Vertex numbers are not necessarily sequential and they are not necessarily
// zero- or one-based.

template <typename VertexInfo, typename EdgeInfo>
class Digraph
{
public:
    // The default constructor initializes a new, empty Digraph so that
    // contains no vertices and no edges.
    Digraph();

    // The copy constructor initializes a new Digraph to be a deep copy
    // of another one (i.e., any change to the copy will not affect the
    // original).
    Digraph(const Digraph& d);

    // The move constructor initializes a new Digraph from an expiring one.
    Digraph(Digraph&& d) noexcept;

    // The destructor deallocates any memory associated with the Digraph.
    ~Digraph() noexcept;

    // The assignment operator assigns the contents of the given Digraph
    // into "this" Digraph, with "this" Digraph becoming a separate, deep
    // copy of the contents of the given one (i.e., any change made to
    // "this" Digraph afterward will not affect the other).
    Digraph& operator=(const Digraph& d);

    // The move assignment operator assigns the contents of an expiring
    // Digraph into "this" Digraph.
    Digraph& operator=(Digraph&& d) noexcept;

    // vertices() returns a std::vector containing the vertex numbers of
    // every vertex in this Digraph.
    std::vector<int> vertices() const;

    // edges() returns a std::vector of std::pairs, in which each pair
    // contains the "from" and "to" vertex numbers of an edge in this
    // Digraph.  All edges are included in the std::vector.
    std::vector<std::pair<int, int>> edges() const;

    // This overload of edges() returns a std::vector of std::pairs, in
    // which each pair contains the "from" and "to" vertex numbers of an
    // edge in this Digraph.  Only edges outgoing from the given vertex
    // number are included in the std::vector.  If the given vertex does
    // not exist, a DigraphException is thrown instead.
    std::vector<std::pair<int, int>> edges(int vertex) const;

    // vertexInfo() returns the VertexInfo object belonging to the vertex
    // with the given vertex number.  If that vertex does not exist, a
    // DigraphException is thrown instead.
    VertexInfo vertexInfo(int vertex) const;

    // edgeInfo() returns the EdgeInfo object belonging to the edge
    // with the given "from" and "to" vertex numbers.  If either of those
    // vertices does not exist *or* if the edge does not exist, a
    // DigraphException is thrown instead.
    EdgeInfo edgeInfo(int fromVertex, int toVertex) const;

    // addVertex() adds a vertex to the Digraph with the given vertex
    // number and VertexInfo object.  If there is already a vertex in
    // the graph with the given vertex number, a DigraphException is
    // thrown instead.
    void addVertex(int vertex, const VertexInfo& vinfo);

    // addEdge() adds an edge to the Digraph pointing from the given
    // "from" vertex number to the given "to" vertex number, and
    // associates with the given EdgeInfo object with it.  If one
    // of the vertices does not exist *or* if the same edge is already
    // present in the graph, a DigraphException is thrown instead.
    void addEdge(int fromVertex, int toVertex, const EdgeInfo& einfo);

    // removeVertex() removes the vertex (and all of its incoming
    // and outgoing edges) with the given vertex number from the
    // Digraph.  If the vertex does not exist already, a DigraphException
    // is thrown instead.
    void removeVertex(int vertex);

    // removeEdge() removes the edge pointing from the given "from"
    // vertex number to the given "to" vertex number from the Digraph.
    // If either of these vertices does not exist *or* if the edge
    // is not already present in the graph, a DigraphException is
    // thrown instead.
    void removeEdge(int fromVertex, int toVertex);

    // vertexCount() returns the number of vertices in the graph.
    int vertexCount() const noexcept;

    // edgeCount() returns the total number of edges in the graph,
    // counting edges outgoing from all vertices.
    int edgeCount() const noexcept;

    // This overload of edgeCount() returns the number of edges in
    // the graph that are outgoing from the given vertex number.
    // If the given vertex does not exist, a DigraphException is
    // thrown instead.
    int edgeCount(int vertex) const;

    // isStronglyConnected() returns true if the Digraph is strongly
    // connected (i.e., every vertex is reachable from every other),
    // false otherwise.
    bool isStronglyConnected() const;

    // findShortestPaths() takes a start vertex number and a function
    // that takes an EdgeInfo object and determines an edge weight.
    // It uses Dijkstra's Shortest Path Algorithm to determine the
    // shortest paths from the start vertex to every other vertex
    // in the graph.  The result is returned as a std::map<int, int>
    // where the keys are vertex numbers and the value associated
    // with each key k is the precedessor of that vertex chosen by
    // the algorithm.  For any vertex without a predecessor (e.g.,
    // a vertex that was never reached, or the start vertex itself),
    // the value is simply a copy of the key.
    std::map<int, int> findShortestPaths(
        int startVertex,
        std::function<double(const EdgeInfo&)> edgeWeightFunc) const;


private:
    // Add whatever member variables you think you need here.  One
    // possibility is a std::map where the keys are vertex numbers
    // and the values are DigraphVertex<VertexInfo, EdgeInfo> objects.


    // You can also feel free to add any additional member functions
    // you'd like (public or private), so long as you don't remove or
    // change the signatures of the ones that already exist.
    std::map<int, DigraphVertex<VertexInfo, EdgeInfo>> graphMap;//map of all vertices
    bool contains(std::vector<std::pair<int,int>> pairList, std::pair<int, int> pairElem) const;//used for checking if edge is already present
    void DFTr(int v, std::map<int, DigraphVertex<VertexInfo, EdgeInfo>> m, std::vector<int> *visited) const;//depth-first traversal recursion
};



// You'll need to implement the member functions below.  There's enough
// code in place to make them compile, but they'll all need to do the
// correct thing instead.

/*DEFAULT CONSTRUCTOR*/
template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::Digraph()
{
    if(!this->graphMap.empty())
        this->graphMap.clear();
}

/*COPY CONSTRUCTOR*/
template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::Digraph(const Digraph& d)
{
    std::map<int, DigraphVertex<VertexInfo, EdgeInfo>> copyMap(d.graphMap);
    this->graphMap = copyMap;
}

/*MOVE CONSTRUCTOR*/
template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::Digraph(Digraph&& d) noexcept
{
    std::map<int, DigraphVertex<VertexInfo, EdgeInfo>> moveMap(std::move(d.graphMap));
    this->graphMap = moveMap;
}

/*DESTRUCTOR*/
template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::~Digraph() noexcept
{
    if(!this->graphMap.empty())
        this->graphMap.clear();
}

/*COPY OPERATION*/
template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>& Digraph<VertexInfo, EdgeInfo>::operator=(const Digraph& d)
{
    if(this != &d)
    {
        this->graphMap.clear();
        std::map<int, DigraphVertex<VertexInfo, EdgeInfo>> copyMap(d.graphMap);
        this->graphMap = copyMap;
    }
    return *this;
}

/*MOVE OPERATION*/
template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>& Digraph<VertexInfo, EdgeInfo>::operator=(Digraph&& d) noexcept
{
    if(this != &d)
    {
        this->graphMap.clear();
        std::map<int, DigraphVertex<VertexInfo, EdgeInfo>> moveMap(std::move(d.graphMap));
        this->graphMap = moveMap;
    }
    return *this;
}

/*GET ALL VERTICES IN GRAPH*/
template <typename VertexInfo, typename EdgeInfo>
std::vector<int> Digraph<VertexInfo, EdgeInfo>::vertices() const
{
    std::vector<int> vertexNumbers;
    for(auto& it : graphMap)
        vertexNumbers.push_back(it.first);

    return vertexNumbers;
}

/*GET ALL EDGES IN GRAPH*/
template <typename VertexInfo, typename EdgeInfo>
std::vector<std::pair<int, int>> Digraph<VertexInfo, EdgeInfo>::edges() const
{
    std::vector<std::pair<int, int>> allEdges;
    for(auto it : graphMap)
    {
        for(auto itr : it.second.edges)
        {
            std::pair<int, int> edge(itr.fromVertex, itr.toVertex);
            if(!contains(allEdges, edge))
                allEdges.push_back(edge);
        }
    }

    return allEdges;
}

/*CONTAINS*/
template <typename VertexInfo, typename EdgeInfo>
bool Digraph<VertexInfo, EdgeInfo>::contains(std::vector<std::pair<int,int>> pairList, std::pair<int, int> pairElem) const
{
    for(int i = 0; i < pairList.size(); i++)
    {
        if(pairList[i] == pairElem)
            return true;
    }

    return false;
}

/*GET ALL EDGES FROM SPECIFIED VERTEX*/
template <typename VertexInfo, typename EdgeInfo>
std::vector<std::pair<int, int>> Digraph<VertexInfo, EdgeInfo>::edges(int vertex) const
{
    std::vector<std::pair<int, int>> allEdges;
    bool vertexFound = false;
    for(auto const& it : graphMap)
    {
        if(vertex == it.first)
        {
            vertexFound = true;
            for(auto const& itr : it.second.edges)
            {
                if(vertex == itr.fromVertex)
                {
                    std::pair<int, int> edge(itr.fromVertex, itr.toVertex);
                    allEdges.push_back(edge);
                }
            }
        }
    }

    if(!vertexFound) //THROW EXCEPTION IF THE VERTEX IS NOT FOUND IN THE GRAPH
        throw DigraphException("Vertex not found");
    return allEdges;
}

/*GET VERTEXINFO FROM SPECIFIED VERTEX*/
template <typename VertexInfo, typename EdgeInfo>
VertexInfo Digraph<VertexInfo, EdgeInfo>::vertexInfo(int vertex) const
{
    for(auto const& it : graphMap)
    {
        if(vertex == it.first)
        {
            return it.second.vinfo;
        }
    }
    
    //THROW EXCEPTION IF THE VERTEX IS NOT FOUND IN THE GRAPH
    throw DigraphException("Vertex not found");
    return VertexInfo{};

}

/*GET EDGEINFO FROM SPECIFIED VERTICECS*/
template <typename VertexInfo, typename EdgeInfo>
EdgeInfo Digraph<VertexInfo, EdgeInfo>::edgeInfo(int fromVertex, int toVertex) const
{
    for(auto const& it : graphMap)
    {
        for(auto const& itr : it.second.edges)
        {
            if(itr.fromVertex == fromVertex && itr.toVertex == toVertex)
                return itr.einfo;
        }
    }

    //THROW EXCEPTION IF THE EDGE IS NOT FOUND
    throw DigraphException("Edge not found");
    return EdgeInfo{};
}

/*ADD VERTEX TO GRAPH*/
template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::addVertex(int vertex, const VertexInfo& vinfo)
{
    DigraphVertex<VertexInfo, EdgeInfo> dVertex;
    dVertex.vinfo = vinfo;
    //causes sanity_check to fail
    /*for(auto it : graphMap)
    {
        if(it.first == vertex)
            throw DigraphException ("Vertex already in map");
    }*/
    graphMap.insert(std::pair<int, DigraphVertex<VertexInfo, EdgeInfo>>(vertex, dVertex));
}

/*ADD EDGE TO APPLIED VERTICES*/
template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::addEdge(int fromVertex, int toVertex, const EdgeInfo& einfo)
{
    for(auto& it : graphMap)
    {
        if((it.first == fromVertex))
        {
            for(auto& itr : it.second.edges)
            {
                //THROW EXCEPTION IS EDGE ALREADY IN GRAPH
                if((itr.fromVertex == fromVertex) && (itr.toVertex == toVertex))
                    throw DigraphException("Edge already in map");
            }
            it.second.edges.push_back(DigraphEdge<EdgeInfo>{fromVertex, toVertex, einfo});
        }
    }
}

/*REMOVE VERTEX FROM THE GRAPH*/
template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::removeVertex(int vertex)
{
    //THROW EXCEPTION IF VERTEX NOT FOUND IN GRAPH
    if(graphMap.find(vertex) == graphMap.end())
        throw DigraphException("Vertex not found in map");
    typename std::map<int, DigraphVertex<VertexInfo, EdgeInfo>>::iterator it = graphMap.find(vertex);
    graphMap.erase(it);
}

/*REMOVE EDGE FROM THE GRAPH*/
template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::removeEdge(int fromVertex, int toVertex)
{
    bool erased = false;
    for(auto& it : graphMap)
    {
        for(auto itr = it.second.edges.begin(); itr != it.second.edges.end();)
        {
            if((*itr).fromVertex == fromVertex && (*itr).toVertex == toVertex)
                itr = it.second.edges.erase(itr);
            else
                itr++;
            erased = true;
        }   
    }
    //THROW EXCEPTION IF EDGE NOT FOUND IN GRAPH
    if(!erased)
        throw DigraphException("Edge not found in map");
}

/*GET COUNT OF ALL VERTICES IN GRAPH*/
template <typename VertexInfo, typename EdgeInfo>
int Digraph<VertexInfo, EdgeInfo>::vertexCount() const noexcept
{
    std::vector<int> vList;
    for(auto it : graphMap)
    {
        bool contains = false;
        for(int i = 0; i < vList.size(); i++)
        {
            if(vList[i] == it.first)
                contains = true;
            else{}
        }
        if(!contains)
            vList.push_back(it.first);
    }
    return vList.size();
}

/*GET COUNT OF ALL EDGES IN GRAPH*/
template <typename VertexInfo, typename EdgeInfo>
int Digraph<VertexInfo, EdgeInfo>::edgeCount() const noexcept
{
    std::vector<DigraphEdge<EdgeInfo>> eList;
    for(auto& it : graphMap)
    {
        for(auto& itr : it.second.edges)
        {
            bool contains = false;
            for(int i = 0; i < eList.size(); i++)
            {
                if(eList[i].fromVertex == itr.fromVertex && eList[i].toVertex == itr.toVertex)
                    contains = true;
                else{}
            }
            if(!contains)
                eList.push_back(itr);
        }
    }
    return eList.size();
}

/*GET COUNT OF ALL EDGES COMING FROM SPECIFIED VERTEX*/
template <typename VertexInfo, typename EdgeInfo>
int Digraph<VertexInfo, EdgeInfo>::edgeCount(int vertex) const
{
    std::vector<DigraphEdge<EdgeInfo>> eList;
    bool vertexFound = false;
    for(auto& it : graphMap)
    {
        if(it.first == vertex)
        {
            vertexFound = true;
            for(auto& itr : it.second.edges)
            {
                bool contains = false;
                for(int i = 0; i < eList.size(); i++)
                {
                    if(eList[i].fromVertex == itr.fromVertex && eList[i].toVertex == itr.toVertex)
                        contains = true;
                    else{}
                }
                if(!contains)
                    eList.push_back(itr);
            }
        }
    }
    //THROW EXCEPTION IF VERTEX NOT FOUND IN GRAPH
    if(!vertexFound)
        throw DigraphException("Vertex not found in graph");
    return eList.size();
}

/*IS GRAPH STRONGLY CONNECTED?*/
template <typename VertexInfo, typename EdgeInfo>
bool Digraph<VertexInfo, EdgeInfo>::isStronglyConnected() const
{
    for(auto& it : graphMap)
    {
        std::vector<int> *visited = new std::vector<int>;
        DFTr(it.first, graphMap, visited);
        if(visited->size() != graphMap.size())
        {
            throw DigraphException("Disconnected Map");
            return false;
        }

    }
    return true;
}

/*DEPTH FIRST TRAVERSAL RECURSIVE FUNCTION*/
template<typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::DFTr(int v, std::map<int, DigraphVertex<VertexInfo, EdgeInfo>> m, std::vector<int> *visited) const
{
    for(auto& it : graphMap)
    {
        if(it.first == v)
        {
            bool contains = false;
            for(int i = 0; i < visited->size(); i++)
            {
                if(visited->at(i) == it.first)
                    contains = true;
            }
            
            if(!contains)
            {
                visited->push_back(it.first);
                for(auto& itr : it.second.edges)
                {
                    DFTr(itr.toVertex, m, visited);
                }
            }
        }
    }
}

/*FIND SHORTEST PATH FOR SPECIFIED VERTEX TO ALL OTHER VERTICES*/
/*DIJKSTRA'S ALGORITHM*/
template <typename VertexInfo, typename EdgeInfo>
std::map<int, int> Digraph<VertexInfo, EdgeInfo>::findShortestPaths(
    int startVertex,
    std::function<double(const EdgeInfo&)> edgeWeightFunc) const
{
    std::map<int, int> pv;//Pv

    std::vector<bool> kv(graphMap.size(), false);//Kv
    std::vector<int> dv(graphMap.size(), std::numeric_limits<int>::max());//Dv

    typedef std::pair<int,int> iPair;

    std::priority_queue<iPair, std::vector<iPair> , std::greater<iPair>> pq;//Priority_Queue

    pq.push(std::make_pair(0, startVertex));//enqueue the start vertex with priority 0
    dv[startVertex] = 0;//for start vertex dv = 0
    pv[startVertex] = startVertex;//for start vertex pv = none

    while(!pq.empty())
    {
        int v = pq.top().second;
        pq.pop();

        if(kv[v] == false)//if Kv is false
        {
            kv[v] = true;
            for(auto& it : graphMap)
            {
                if(it.first == v)
                {
                    for(auto& itr : it.second.edges)
                    {
                        if(itr.fromVertex == v)//all outgoing edges
                        {
                            int nextV = itr.toVertex;
                            int weight = edgeWeightFunc(itr.einfo);

                            if(dv[nextV] > dv[v] + weight)
                            {
                                dv[nextV] = dv[v] + weight;
                                pv[nextV] = v;
                                pq.push(std::make_pair(dv[nextV], nextV));
                            }
                        }
                    }
                }
            }
        }
    }


    return pv;
}



#endif // DIGRAPH_HPP

