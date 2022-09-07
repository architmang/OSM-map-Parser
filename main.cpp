#include <bits/stdc++.h>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <cmath>
#include <climits>
#include <set>
#include"rapidxml\rapidxml_utils.hpp"  // including the necessary header files

using namespace std;
using namespace rapidxml;

unordered_map<string,int> map_nodes;
// it is the map with key as node id(string) and value as the position of corresponding id in the nodes vector
unordered_map<string,pair<double,double>> map_cordis;
// it is the map with key as node id(string) and value as the pair of latitude and longitude of corresponding node

void options()
{
    cout<<"\nEnter 1 to find the elements which contain the given string as a substring in their name are \n";
    cout<<"Enter 2 to find the k-closest nodes to a given node \n";
    cout<<"Enter 3 to find the the shortest path between two node elements, through the way elements \n";
    cout<<"Enter 4 to quit\n";
    // printing the options menu
}

int check(string y, string x)
{
    // checks if the string x is a substring of the string y or not
    int size1 = x.length();
    int size2 = y.length();
    if (size2 < size1) return 0;
    string name = y;
    string input = x;
    for(int i=0;i<size1;i++){
        input[i] = tolower(x[i]);
    }
    for(int i=0;i<size2;i++){
        name[i] = tolower(y[i]);
    }
    int t;
    for (t = 0; t <= size2 - size1; t++) 
    {
        int g;
        for (g = 0; g < size1; g++)
        {
            if (name[t+g] != input[g])
            {
                break;
            }
        }
        if (g-size1==0)
        {
            return 1;
            // returns the index t if x is a substring of y
        }
    }
    return 0;
    // returns -1 if x is not a substring of y
}

double dist(string a,string b)
{
    double dist,lat,lon,lat2,lon2;
    lat = map_cordis[a].first;
    lon = map_cordis[a].second;          // latitude and longitude of first node with node_id as string a
    lat2 = map_cordis[b].first;
    lon2 = map_cordis[b].second;         // latitude and longitude of second node with node_id as string b
    dist=0;
                                         // initializing the distance between nodes as 0
    double pi=2*acos(0.0);
    lat=lat*pi/180;
    lon=lon*pi/180;
    lon2=lon2*pi/180;
    lat2=lat2*pi/180;

    double diff1 = lon2 - lon;
    double diff2 = lat2 - lat;
    dist = pow(sin(diff2/2),2)+cos(lat)*cos(lat2)*pow(sin(diff1/2),2);         // using haver-sine formula for distance between two points
    dist = 2*asin(sqrt(dist));
    double earth = 6371;
    dist= dist*earth;
    // finding the distance between the given nodewith each node element and storing it in d
    return dist;
}

void algo(vector<pair<double,string>> v[],string node_id_1,string node_id_2,vector<string> nodes)
{
    vector<double> meas(43993,100000);
    // we are initializing the vector of distances from source with a large value
    meas[map_nodes[node_id_1]]=0;
    // defining source dist as 0
    priority_queue < pair<double,string> , vector< pair<double,string> >,greater< pair<double,string> > > pq; //implementing a prioority queue using min-heap.......pair has dist,index
    pq.push(make_pair(0,node_id_1));
    // push the source
    
    while(!pq.empty())
    {
        //iterating until the priority queue is empty 
        string x_=pq.top().second;
        int x=map_nodes[x_];
        pq.pop();
        // finding the node with minimum distance from the source in each iteration and popping it from the priority queue

        if(x_ == node_id_2) break;
        // we break when the popped node is our destination node (its minm distance has been finalized)
        vector<pair<double,string>>::iterator y;
        // we define an iterator for  vector of pairs 

        for(y=v[x].begin();y!=v[x].end();y++)
        {
            double dist=(*y).first;
            // dist has distance
            string id=(*y).second;
            // id has node id 
            if(meas[x]+ dist<meas[map_nodes[id]])
            {
                // we update the distance of y 
                meas[map_nodes[id]]=meas[x]+ dist;
                pq.push(make_pair(meas[map_nodes[id]],id));
                // we add the updated distance, node id pair to the priority queue
            }
        }
    }
    if (meas[map_nodes[node_id_2]] == 100000){
        cout<<"No path exists\n";
    }
    else cout<<"The distance is "<<meas[map_nodes[node_id_2]]<<" km !"<<endl;
    // printing distance
}

int main()
{
    rapidxml::file<> xmlFile("map.osm");
    rapidxml::xml_document<> doc;
    doc.parse<0>(xmlFile.data());
    //node points to the first occurence of element "node"

    // vector<string> nodes;
    int indice = 0;
    vector<string> nodes;
    xml_node<> *node = doc.first_node();
    for(xml_node<> *child = node->first_node("node"); child; child = child->next_sibling("node"))
    {
        string s = child->first_attribute("id")->value();
        double lat = stod(child->first_attribute("lat")->value());
        double lon = stod(child->first_attribute("lon")->value());
        map_cordis[s] = {lat,lon};
        nodes.push_back(s);
        map_nodes[s] = indice;
        indice++;
        // cout << child->name() << "\n";
    }
    
    int n_nodes = 0;
    int n_ways = 0;// initializing number of nodes and ways to 0

    // we are accessing the other elements using next_sibling() funciton
    for(xml_node<> *child = node->first_node(); child; child = child->next_sibling())
    {
        if(strcmp(child->name(), "node") == 0) n_nodes++;
        else if(strcmp(child->name(), "way") == 0) n_ways++;
    }
    //updating number of nodes and ways

    vector<pair<double,string>> v[43993];
    // array of vector of pairs
    //dist,node_id

    for(xml_node<> *child = node->first_node("way"); child; child = child->next_sibling("way"))
    {
        //iterating over all the ways
        for(xml_node<> *child2 = child->first_node(); child2->next_sibling(); child2 = child2->next_sibling())
        {
             xml_attribute<> *a = child2->first_attribute("ref");
             xml_attribute<> *b = child2->next_sibling()->first_attribute("ref");
             if(!a) continue;
             if(!b) continue;
             string ida = a->value();
             string idb = b->value();
             double disto=dist(a->value(),b->value());
             // ida is the id of current way node, idb is the id of next way node, disto is the distance between them

             // updating the vector of pairs for consecutive way nodes
             v[map_nodes[ida]].push_back(make_pair(disto,idb));
             v[map_nodes[idb]].push_back(make_pair(disto,ida));
        }
    }
    int ds=1;
    while(ds!=4)
    {
        int d;
        options();
        cout<<"\n";
        cin>>d;
        ds=d;

        // infinite switch-case
        switch(ds)
        {
            case 1:
            {
                cout << "nodes : " << n_nodes << "\n";
                cout << "ways : " << n_ways << "\n";

                string x;
                cout<<"Enter string"<<endl;
                cin>>x;
                int cnt=0;

                for(xml_node<> *child2 = node->first_node("node"); child2; child2 = child2->next_sibling("node"))
                {
                    for(xml_node<> *child3 = child2->first_node();child3;child3 = child3->next_sibling()){
                        xml_attribute<> *ptrk = child3->first_attribute("k");
                        if (strcmp(ptrk->value(),"name") == 0 ){
                            ptrk = child3->first_attribute("v");
                            if (check(ptrk->value(),x) == 1)
                            {
                                cnt++;
                                std::cout<<cnt<<". Node name: "<<ptrk->value()<<"\tNode id: "<<child2->first_attribute("id")->value()<<"\n";
                            }
                        }
                    }
                }

                for(xml_node<> *child2 = node->first_node("way"); child2; child2 = child2->next_sibling("way"))
                {
                    for(xml_node<> *child3 = child2->first_node("tag");child3;child3 = child3->next_sibling("tag")){
                        xml_attribute<> *ptrk = child3->first_attribute("k");
                        if (strcmp(ptrk->value(),"name") == 0 ){
                            ptrk = child3->first_attribute("v");
                            if (check(ptrk->value(),x) == 1){ 
                            cnt++;
                            std::cout<<cnt<<". Way name: "<<ptrk->value()<<"\tWay id: "<<child2->first_attribute("id")->value()<<"\n";
                            }
                        }
                    }
                }

                
                cout<<"\n\nNumber of elements which contain the given string as a substring in their name are "<<cnt<<endl;
            }
            break;
            case 2:
            {
                int k;
                std::cout<<"\n";
                std::cout<<"Enter k\n";
                std::cin>>k;
                std::string node_id;
                std::cout<<"Enter node_id\n";
                std::cin>>node_id;                          //taking user input for k and node id
                int cnt=0;     
                float lat,lon;

                for(rapidxml::xml_node<> *child = node->first_node(); child; child = child->next_sibling())
                {
                    if(strcmp(child->name(), "node") == 0)
                    {
                        if(child->first_attribute("id")->value() == node_id)
                        {
                            lat=std::stof(child->first_attribute("lat")->value());
                            lon=std::stof(child->first_attribute("lon")->value());
                            //finding latitude and longitude of given node
                        }
                    }
                }

                double pi=2*acos(0.0); //defining pi
                lat=lat*pi/180;
                lon=lon*pi/180;        // converting latitude and longitude to radians

                float lat2,lon2,min=INT_MAX;
                xml_node<> *node_min=NULL;
                std::vector<std::string> v;

                // to find the k closest nodes to the given node, we are parsing the document k number of times and in each iteration of document parsing, 
                // we are storing the reference to the node with the minimum distance to the given node
                std::cout<<"\n";
                while(cnt<k)
                {
                    for(xml_node<> *child = node->first_node(); child; child = child->next_sibling())
                    {
                        if(strcmp(child->name(), "node") == 0)
                        {
                            lat2=std::stof(child->first_attribute("lat")->value());
                            lon2=std::stof(child->first_attribute("lon")->value());
                            lon2=lon2*pi/180;
                            lat2=lat2*pi/180;
                            double diff1 = lon2 - lon;
                            double diff2 = lat2 - lat;
                            double d = pow(sin(diff2/2),2) + cos(lat)*cos(lat2)*pow(sin(diff1/2),2);
                            d = 2 * asin(sqrt(d));
                            double earth = 6371;
                            d= d*earth;
                            // finding the distance between the given nodewith each node element and storing it in d
                            
                            if(d<min && d!=0 && find(v.begin(),v.end(),child->first_attribute("id")->value())==v.end())
                            {
                                //stores the minumum distance in current iteration
                                // we are checking whether the vector v of minm distance nodes contains the current minm distance node or not
                                //if not, we make node_min refer to this node 
                                min=d;
                                node_min=child;
                            }
                        }
                    }
                    v.push_back(node_min->first_attribute("id")->value());
                    //adding the current minimum to the vector v
                    cnt++;
                    std::cout<<"The "<<cnt<<" th min dist node is "<<v[cnt-1]<<" with distance "<<min<<" km\n";
                    min=INT_MAX;
                    //updating the min to INT_MAX for next iteration
                }
                std::cout<<"\n";
            }
            break;
            case 3:
            {
                //our source
                cout<<"Enter node-1 id\n";
                string node_id_1;
                cin>>node_id_1;            
                
                //our destination
                cout<<"Enter node-2 id\n";
                string node_id_2;
                cin>>node_id_2;

                algo(v,node_id_1,node_id_2,nodes);   // calculating the minimum distance between 
                // djikstra's algorithm
            }
            break;
            case 4:
            {
                return 0;
            }
            default:
            {
                cout<<"\nWrong input\n";
            }
        }
    }
    return 0;
}