#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <vector>
#include <math.h>

namespace globalplanner{    
    class Node{
        public:
            Node* parent; //set parent --> *parent = Node;
            Point position;
            double g;
            double h;
            double f;                       

            Node(){
                // parent = nullptr;
                // position = Point(0,0);
                // g = 0; //G is the distance between the current node and the start node.
                // h = 0; //H is the heuristic : estimated distance from the current node to the end node.
                // f = 0; //F is the total cost of the node. F= G + H
            }

            ~Node(){

            }
            // //Init node parameters from two points
            // void init_node(Node* par, Point pos){
            //     parent = par;
            //     position = pos;                 
            //     g = 0; //G is the distance between the current node and the start node.
            //     h = 0; //H is the heuristic : estimated distance from the current node to the end node.
            //     f = 0; //F is the total cost of the node. F= G + H                          
            // }
            //Create node
            Node *create_node(Node* par, Point pos){
                Node *result = new Node;
                result->parent = par;
                result->position = pos;
                result->g = 0; //G is the distance between the current node and the start node.
                result->h = 0; //H is the heuristic : estimated distance from the current node to the end node.
                result->f = 0; //F is the total cost of the node. F= G + H 

                return result;
            }
            // Node create_node(Node* par, Point pos){
            //     Node result;
            //     result.parent = par;
            //     result.position = pos;
            //     result.g = 0; //G is the distance between the current node and the start node.
            //     result.h = 0; //H is the heuristic : estimated distance from the current node to the end node.
            //     result.f = 0; //F is the total cost of the node. F= G + H 

            //     return result;
            // }
            //Set one node equal to another
            void eq(Node *N){
                parent = N->parent;
                position = N->position;
                g = N->g;
                h = N->h;
                f= N->f;
            }
            //check if one node is equal to another
            bool ifeq(Node *N){
                return position.x == N->position.x && position.y == N->position.y 
                        ? true:false;
            }
    };  

    //Check if one Point is equal to another
    bool ifeq_point(Point pt1, Point pt2){
        return pt1.x == pt2.x && pt1.y == pt2.y ? true:false;
    }

    std::vector<Point> astar(std::vector<std::pair<Point, std::vector<Point> >> graph, 
                                    Point start, Point end){
        
        //Define all variables
        std::vector<Point> path;
        Node N;
        //Node pointers start_node, end_node, current_node, current, new_node;
        Node *start_node, *end_node, *current_node, *current, *new_node;
        int current_index;
        int graph_index;
        std::vector<Point> V_vector; //vector of vertexes of graph
        std::vector<std::vector<Point>> E_vector; //vector of vectors of edges of graph
        std::vector<Point> current_node_edges; //edges of current node
        bool next_child; //flag for closed list check

        //Initialize vertex and edges vector
        for(int i=0;i<graph.size();i++){
            V_vector.push_back(graph[i].first); //Vertex
            std::cout << "V_vector: " << V_vector[i].x << ", " << V_vector[i].y << std::endl;
            E_vector.push_back(graph[i].second); //Edges            
        }        

        //Method tests       
        start_node = start_node->create_node(nullptr,start);
        std::cout << "START: " << start_node->position.x << ", " << start_node->position.y << std::endl;
        std::cout << "ADDRESS: " << start_node << std::endl;
        end_node = end_node->create_node(nullptr,end);
        std::cout << "END_PRE: " << end_node->position.x << ", " << end_node->position.y << std::endl;
        std::cout << "ADDRESS: " << end_node << std::endl;
        std::cout << "PARENT: " << end_node->parent << std::endl;
        end_node = end_node->create_node(start_node,Point(0,0));
        std::cout << "END_POS: " << end_node->position.x << ", " << end_node->position.y << std::endl;
        std::cout << "ADDRESS: " << end_node << std::endl;
        std::cout << "PARENT: " << end_node->parent << std::endl;
        std::cout << "PARENT_pos: " << end_node->parent->position.x << std::endl;
        //Check eq function
        end_node->eq(start_node);
        std::cout << "END_POS: " << end_node->position.x << ", " << end_node->position.y << std::endl;
        std::cout << "ADDRESS: " << end_node << std::endl;
        std::cout << "PARENT: " << end_node->parent << std::endl;
        //Check if equal
        if(start_node->ifeq(end_node)){
            std::cout << "True" << std::endl;
        } else{
            std::cout << "False" << std::endl;
        }

        //Create start and end node
        start_node = start_node->create_node(nullptr,start);
        end_node = end_node->create_node(nullptr,end);


        //TO MODIFY
        //Initialize both open and closed list
        std::vector<Node> open_list;  //include adjacent nodes that has to be evaluated (f value)
        std::vector<Node> closed_list;  //include nodes from the open_list which has been evaluated (lowest f value)
       
        //Add the start node
        open_list.push_back(start_node);

        //Loop until you find the end
        // //******DELETE TEMP*************
        // int loop = 0;
        // while(loop == 0){
        //     loop = 1;
        // //******DELETE TEMP*************
        while (open_list.size() > 0){ 
            for(int i=0;i<open_list.size();i++){
                if(open_list[i].parent != nullptr){
                std::cout << "start open list node parent: " << open_list[i].parent->position.x << ", " << open_list[i].parent->position.y << std::endl;
                std::cout << "open list parent address: " << open_list[i].parent << std::endl;                
                }
                std::cout << "start open list node pos: " << open_list[i].position.x << ", " << open_list[i].position.y << std::endl;
            }          
            //Get the current node
            current_node.eq(open_list[0]); //current node = open_list[0]
            if(current_node.parent != nullptr){
                std::cout << "current node parent: " << current_node.parent->position.x << ", " << current_node.parent->position.y << std::endl;
                std::cout << "current node parent adress: " << current_node.parent << std::endl;
            }            
            std::cout << "current node pos: " << current_node.position.x << ", " << current_node.position.y << std::endl;
            std::cout << "open list size: " << open_list.size() << std::endl;
            current_index = 0;

            // //test eq function
            // std::cout << "f,g,h current node: " << current_node.f << ", " << current_node.g 
            //           << ", " << current_node.h << std::endl;
            // std::cout << "Parent current node: " << current_node.parent << std::endl;
            // std::cout << "position current node: " << current_node.position.x << ", " << current_node.position.y << std::endl;
            
            for(int index=0;index<open_list.size();index++){ //search for the minimum f value among all nodes in open_list
                if (open_list[index].f < current_node.f){
                    current_node.eq(open_list[index]); //current node = open_list[index]
                    current_index = index;
                }
            }

            //Pop current (lowest f value node) off open list, add to closed list
            open_list.erase(open_list.begin() + current_index);
            // std::cout << "open list size: " << open_list.size() << std::endl;
            closed_list.push_back(current_node);

            //Found the goal
            if(current_node.ifeq(end_node)){
                current.eq(current_node);                      
                std::cout << "Found goal parent: " << current.parent->position.x << ", " << current.parent->position.y << std::endl;
                std::cout << "Found goal position: " << current.position.x << ", " << current.position.y << std::endl;
                while (!current.ifeq(start_node)){
                    path.insert(path.begin(), current.position); //add mid points of A* path
                    //std::cout << "path: " << current.position.x << ", " << current.position.y << std::endl;
                    current.eq(*current.parent);
                    //std::cout << "current parent pos: " << current.position.x << ", " << current.position.y << std::endl;
                }
                path.insert(path.begin(), current.position); //add start point
                std::cout << "path: " << current.position.x << ", " << current.position.y << std::endl;

                return path; //return path
            }
            //Generate children
            std::vector<Node> children;

            //Check adjacent nodes (edges)

            //find index of current node inside the graph
            graph_index = 0;
            for (Point p : V_vector){ 
                if(ifeq_point(p,current_node.position)){
                    break;
                }
                graph_index++;
            }
            std::cout << "Index: " << graph_index << std::endl;

            //Loop through edges for current node
            current_node_edges = E_vector[graph_index];
            for(int i=0;i<current_node_edges.size();i++){
                //create new node
                //new_node.init_node(&current_node,current_node_edges[i]);
                new_node.create_node(&current_node,current_node_edges[i]);
                std::cout << "New node parent: " << new_node.parent->position.x << ", " << new_node.parent->position.y << std::endl;
                std::cout << "New node pos: " << new_node.position.x << ", " << new_node.position.y << std::endl;
                
                //Append
                children.push_back(new_node);
            }

            //Loop through children            
            for(int i=0;i<children.size();i++){                
                //Child is on the closed list
                next_child = false; //reset flag
                for (Node closed_list_node : closed_list){ 
                    if(children[i].ifeq(closed_list_node)){
                        next_child = true;
                        break;
                    }
                }
                if(!next_child){
                    //Create the f, g, and h values
                    children[i].g = current_node.g + 1;
                    //H: Euclidean distance to end point
                    children[i].h = sqrt(pow(children[i].position.x-end_node.position.x,2)
                                    + pow(children[i].position.y-end_node.position.y,2));
                    // F: G + H
                    children[i].f = children[i].g + children[i].h;
                    std::cout << "f,g,h child node: " << children[i].f << ", " 
                                << children[i].g << ", " << children[i].h << std::endl;

                    //Child is already in open list
                    next_child = false; //reset flag 
                    for (Node open_list_node : open_list){ 
                        //check if the new path to children is worst or equal than
                        //one already in the open_list (by measuring g value)
                        if(children[i].ifeq(open_list_node) && 
                                children[i].g >= open_list_node.g){
                            next_child = true;
                            break;
                        }
                    }
                    if(!next_child){
                        //Add the child to the open list
                        open_list.push_back(children[i]);
                        std::cout << "open list node parent: " << children[i].parent->position.x << ", " << children[i].parent->position.y << std::endl;
                        std::cout << "open list node pos: " << children[i].position.x << ", " << children[i].position.y << std::endl;
                    }
                }

            }
        }
        
        // //DELETE TEMP
        ////return path; 
        
    }
}