#include "include/grid.hpp"
#include "include/pathfinder/hybridAStar.hpp"

int main()
{
    Grid grid = Grid(12, 12);
    cout << "ADDING" << endl;
    grid.addObstacle(0, 0);
    cout << "ADDING" << endl;
    grid.addObstacle(11, 11);
    cout << "ADDING THIRD" << endl;
    grid.addObstacle(6, 0);
    cout << "REMOVING" << endl;
    grid.removeObstacle(11, 11);
    grid.removeObstacle(6, 0);
    //grid.removeObstacle(0, 0);
    vector<vector<bool>> voro = grid.returnVoroMap();
    for (vector<vector<float>>::size_type i = 0; i < voro.size(); i++ )
    {
        for ( vector<int>::size_type j = 0; j < voro[i].size(); j++ )
        {
            cout << (int)voro[i][j] << ' ';
        }
        cout << endl;
    }
    cout << "------------------------------------" << endl;
    vector<vector<float>> dist = grid.returnVoroDistMap();
    for (vector<vector<float>>::size_type i = 0; i < dist.size(); i++ )
    {
        for ( vector<int>::size_type j = 0; j < dist[i].size(); j++ )
        {
            if(dist[i][j] < 10)
            {
                cout << "0" <<(int)dist[i][j] << ' ';
            }
            else
            {
                cout << (int)dist[i][j] << ' ';
            }
            
        }
        cout << endl;
    }

    vector<vector<Coordinates2D>> nearest = grid.returnNearest();
    for (vector<vector<Coordinates2D>>::size_type i = 0; i < nearest.size(); i++ )
    {
        for ( vector<Coordinates2D>::size_type j = 0; j < nearest[i].size(); j++ )
        {
            cout << "(" << nearest[i][j].x << ", " << nearest[i][j].y << ") ";
            
        }
        cout << endl;
    }
    return 0;
}