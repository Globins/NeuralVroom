#include "include/grid.hpp"
#include "include/pathfinder/hybridAStar.hpp"
#include "include/pathfinder/reedsShepps.hpp"

int main()
{
    Grid grid = Grid(12, 12);
    grid.addObstacle(0, 0);
    cout << "---------------------------------------------------"<<endl;
    grid.addObstacle(11, 11);
    cout << "---------------------------------------------------"<<endl;
    grid.addObstacle(6, 0);
    vector<vector<float>> distMap = grid.returnDistMap();
    vector<vector<float>> voroDistMap = grid.returnVoroDistMap();
    for (vector<vector<float>>::size_type i = 0; i < distMap.size(); i++ )
    {
        for ( vector<int>::size_type j = 0; j < distMap[i].size(); j++ )
        {
            cout << distMap[i][j] << ' ';
        }
        cout << endl;
    }
    cout << endl;
    for (vector<vector<float>>::size_type i = 0; i < voroDistMap.size(); i++ )
    {
        for ( vector<int>::size_type j = 0; j < voroDistMap[i].size(); j++ )
        {
            cout << voroDistMap[i][j] << ' ';
        }
        cout << endl;
    }
    return 0;
}