/*******************************************************************************
*                                                                              *
* Author    :  Alvaro Rubio Gomez                                              *
* Version   :  1.0                                                             *
*                                                                              *
*  TBC                                                                         * 
*                                                                              *
*******************************************************************************/

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
            }

            ~Node(){

            }           
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
            //check if one node is equal to another
            bool ifeq(Node *N){
                float epsilon = 0.00001;
                return std::fabs(position.x - N->position.x) < epsilon 
                        && std::fabs(position.y - N->position.y) < epsilon
                        ? true:false;
            }
    };  

    //Check if one Point is equal to another
    bool ifeq_point(Point pt1, Point pt2){
        float epsilon = 0.00001;
        return std::fabs(pt1.x - pt2.x) < epsilon && std::fabs(pt1.y - pt2.y) < epsilon 
                    ? true:false;
    }   

    std::vector<Point> astar(std::vector<std::pair<Point, std::vector<Point> >> graph, 
                                    Point start, Point end){
        
        //Define all variables
        std::vector<Point> path;        
        //Node pointers start_node, end_node, current_node, current, new_node;
        Node *start_node, *end_node, *current_node, *current, *new_node;
        int current_index;
        int graph_index;
        std::vector<Point> V_vector; //vector of vertexes of graph
        std::vector<std::vector<Point>> E_vector; //vector of vectors of edges of graph
        std::vector<Point> current_node_edges; //edges of current node
        bool next_child; //flag for closed list check

        //Initialize vertex and edges vector
        std::cout << "graph size: " << graph.size() << std::endl;
        for(int i=0;i<graph.size();i++){
            V_vector.push_back(graph[i].first); //Vertex
            std::cout << "V_vector: " << V_vector[i].x << ", " << V_vector[i].y << std::endl;
            E_vector.push_back(graph[i].second); //Edges            
        }            

        //Create start and end node
        start_node = start_node->create_node(nullptr,start);
        end_node = end_node->create_node(nullptr,end);

        // //Method tests       
        // //start_node = start_node->create_node(nullptr,start);
        // std::cout << "START: " << start_node->position.x << ", " << start_node->position.y << std::endl;
        // std::cout << "ADDRESS: " << start_node << std::endl;
        // //end_node = end_node->create_node(nullptr,end);
        // std::cout << "END_PRE: " << end_node->position.x << ", " << end_node->position.y << std::endl;
        // std::cout << "ADDRESS: " << end_node << std::endl;
        // std::cout << "PARENT: " << end_node->parent << std::endl;
        // end_node = end_node->create_node(start_node,Point(0,0));
        // std::cout << "END_POS: " << end_node->position.x << ", " << end_node->position.y << std::endl;
        // std::cout << "ADDRESS: " << end_node << std::endl;
        // std::cout << "PARENT: " << end_node->parent << std::endl;
        // std::cout << "PARENT_pos: " << end_node->parent->position.x << std::endl;
        // //Check eq function
        // //end_node->eq(start_node);
        // end_node = start_node;
        // std::cout << "END_POS: " << end_node->position.x << ", " << end_node->position.y << std::endl;
        // std::cout << "ADDRESS: " << end_node << std::endl;
        // std::cout << "PARENT: " << end_node->parent << std::endl;
        // //Check if equal
        // if(start_node->ifeq(end_node)){
        //     std::cout << "True" << std::endl;
        // } else{
        //     std::cout << "False" << std::endl;
        // }
        
        //Initialize both open and closed list
        std::vector<Node*> open_list;  //include adjacent nodes that has to be evaluated (f value)
        std::vector<Node*> closed_list;  //include nodes from the open_list which has been evaluated (lowest f value)
       
        //Add the start node
        open_list.push_back(start_node); 
        

        //Loop until you find the end        
        while (open_list.size() > 0){                    
            //Get the current node            
            current_node = open_list[0];           
            current_index = 0;           
            for(int index=0;index<open_list.size();index++){ //search for the minimum f value among all nodes in open_list
                if (open_list[index]->f < current_node->f){
                    current_node = open_list[index]; //current node = open_list[index]
                    current_index = index;
                }
            }

            //Pop current (lowest f value node) off open list, add to closed list
            open_list.erase(open_list.begin() + current_index);            
            closed_list.push_back(current_node);

            //Found the goal
            if(current_node->ifeq(end_node)){
                current = current_node;               
                while (!current->ifeq(start_node)){
                    path.insert(path.begin(), current->position); //add mid points of A* path                    
                    current = current->parent;                    
                }
                path.insert(path.begin(), current->position); //add start point
                
                return path; //return path
            }
            //Generate children
            std::vector<Node*> children;  

            //Check adjacent nodes (edges)

            //find index of current node inside the graph
            graph_index = 0;            
            for (Point p : V_vector){                
                if(ifeq_point(p, current_node->position)){                    
                    break; //Found
                }else{
                graph_index++;
                }
            }
            if(graph_index >= V_vector.size()){   
                std::cout << "V not found: " << std::endl;                           
                return path; //Error
            }

            //Loop through edges for current node
            current_node_edges = E_vector[graph_index];            
            for(int i=0;i<current_node_edges.size();i++){
                //create new node                
                new_node = new_node->create_node(current_node,current_node_edges[i]);
               
                //Append
                children.push_back(new_node);
            }

            //Loop through children            
            for(int i=0;i<children.size();i++){                
                //Child is on the closed list
                next_child = false; //reset flag
                for (Node *closed_list_node : closed_list){ 
                    if(children[i]->ifeq(closed_list_node)){
                        next_child = true;
                        break;
                    }
                }
                if(!next_child){
                    //Create the f, g, and h values
                    children[i]->g = current_node->g + 1;
                    //H: Euclidean distance to end point
                    children[i]->h = sqrt(pow(children[i]->position.x-end_node->position.x,2)
                                    + pow(children[i]->position.y-end_node->position.y,2));
                    // F: G + H
                    children[i]->f = children[i]->g + children[i]->h;
                    
                    //Child is already in open list
                    next_child = false; //reset flag 
                    for (Node *open_list_node : open_list){ 
                        //check if the new path to children is worst or equal than
                        //one already in the open_list (by measuring g value)
                        if(children[i]->ifeq(open_list_node) && 
                                children[i]->g >= open_list_node->g){
                            next_child = true;
                            break;
                        }
                    }
                    if(!next_child){
                        //Add the child to the open list
                        open_list.push_back(children[i]);                       
                    }
                }

            }
        }
        
        // //DELETE TEMP
        printf("path not found");
        return path; 
        
    }
}