// main.cpp
//
//  Jaren Gerdes
//  55784665
//
// ICS 46 Spring 2018
// Project #5: Rock and Roll Stops the Traffic
//
// This is the program's main() function, which is the entry point for your
// console user interface.
#include "InputReader.hpp"
#include <iostream>
#include "Digraph.hpp"
#include "RoadMapReader.hpp"
#include "RoadMapWriter.hpp"
#include "RoadSegment.hpp"
#include "TripReader.hpp"
#include <iomanip>

int main()
{
    //Declarations
    InputReader ir(std::cin);
    RoadMapReader rmr;
    RoadMapWriter rmw;
    TripReader tr;

    //create digraph
    RoadMap rm = rmr.readRoadMap(ir);
    
    //write out the digraph
    rmw.writeRoadMap(std::cout, rm);

    //get all the trips from input
    std::vector<Trip> trips = tr.readTrips(ir);
    
    //variables to store the trips
    std::map<int, std::map<int, int>> shortestTripsDistance;
    std::map<int, std::map<int, int>> shortestTripsTime;

    //get all the trips paths
    for(int i = 0; i < trips.size(); i++)
    {
        std::map<int, int> shortest;

        //DISTANCE TRIPS
        if(trips[i].metric == TripMetric::Distance)
        {
            shortest = rm.findShortestPaths(
            trips[i].startVertex,
            [](RoadSegment segment)
            {
                return segment.miles;
            });
            shortestTripsDistance.emplace(trips[i].startVertex, shortest);
        }
        //TIME TRIPS
        if(trips[i].metric == TripMetric::Time)
        {
            shortest = rm.findShortestPaths(
            trips[i].startVertex,
            [](RoadSegment segment)
            {
                return (segment.miles/segment.milesPerHour)*60;
            });
            shortestTripsTime.emplace(trips[i].startVertex, shortest);
        }
    }

    //OUTPUT ALL TRIPS
    for(int i = 0; i < trips.size(); i++)
    {
        //variables
        std::vector<int> path;
        std::vector<double> distances;
        std::vector<std::pair<double, double>> time;
        if(trips[i].metric == TripMetric::Distance)
        {
            std::cout << "Shortest Distance from " << rm.vertexInfo(trips[i].startVertex) << " to " << rm.vertexInfo(trips[i].endVertex) << std::endl;
            std::cout << "\tBegin at " << rm.vertexInfo(trips[i].startVertex) << std::endl;

            std::map<int, std::map<int, int>>::iterator it = shortestTripsDistance.find(trips[i].startVertex);

            int unravel = trips[i].endVertex;

            while(unravel != trips[i].startVertex)
            {
                std::map<int, int>::iterator itr = it->second.find(unravel);
                unravel = itr->second;
                path.push_back(itr->first);
                RoadSegment segment = rm.edgeInfo(itr->second, itr->first);
                distances.push_back(segment.miles);
            }            
        }
        if(trips[i].metric == TripMetric::Time)
        {
            std::cout << "Shortest driving time from " << rm.vertexInfo(trips[i].startVertex) << " to " << rm.vertexInfo(trips[i].endVertex) << std::endl;
            std::cout << "\tBegin at " << rm.vertexInfo(trips[i].startVertex) << std::endl;

            std::map<int, std::map<int, int>>::iterator it2 = shortestTripsTime.find(trips[i].startVertex);

            int unravel = trips[i].endVertex;

            while(unravel != trips[i].startVertex)
            {
                std::map<int, int>::iterator itr2 = it2->second.find(unravel);
                unravel = itr2->second;
                path.push_back(itr2->first);
                RoadSegment segment = rm.edgeInfo(itr2->second, itr2->first);
                time.push_back(std::make_pair(segment.miles, segment.milesPerHour));
            }
    
        }

        //reverse all the paths
        std::reverse(path.begin(), path.end());
        std::reverse(distances.begin(), distances.end());
        std::reverse(time.begin(), time.end());

        if(trips[i].metric == TripMetric::Distance)
        {
            for(int j = 0; j < path.size(); j++)
                std::cout << "\tContinue to " << rm.vertexInfo(path[j]) << " (" << std::setprecision(1) << std::fixed << distances[j] << " miles)" << std::endl;
            double totalDist = 0;
            for(int k = 0; k < distances.size(); k++)
                totalDist += distances[k];

            std::cout << "Total distance: " << std::setprecision(1) << std::fixed << totalDist << " miles\n" << std::endl;
        }
        if(trips[i].metric == TripMetric::Time)
        {
            for(int l = 0; l < path.size(); l++)
            {
                double avgTime = (time[l].first/time[l].second)*60;
                int hours = 0;
                int minutes = 0;
                double seconds;
                while(avgTime > 60)
                {
                    hours++;
                    avgTime -= 60;
                }
                while(avgTime > 1)
                {
                    minutes++;
                    avgTime -= 1;
                }
                seconds = avgTime*60;
                

                std::cout << "\tContinue to " << rm.vertexInfo(path[l]) << " (" << time[l].first << " miles @ " << time[l].second << "mph = ";
                if(hours != 0)
                    std::cout << hours << "hours ";
                if(minutes != 0)
                    std::cout << minutes << "mins ";
                if(seconds != 0)
                    std::cout << seconds << "secs ";
                std::cout << ")" << std::endl;
                               
            }
            double totalTime = 0;
            for(int m = 0; m < time.size(); m++)
                totalTime += (time[m].first/time[m].second)*60;
            int hours = 0;
            int minutes = 0;
            double seconds;
            while(totalTime > 60)
            {
                hours++;
                totalTime -= 60;
            }
            while(totalTime > 1)
            {
                minutes++;
                totalTime -= 1;
            }
            seconds = totalTime*60;

            std::cout << "Total time: ";
            if(hours != 0)
                std::cout << hours << "hours ";
            if(minutes != 0)
                std::cout << minutes << "mins ";
            if(seconds != 0)
                std::cout << seconds << "secs ";
            std::cout << "\n" << std::endl;
        }
    }


    return 0;
}

