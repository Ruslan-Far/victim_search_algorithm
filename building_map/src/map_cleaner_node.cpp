#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "building_map/CleanedMap.h"
#include <vector>

using namespace std;

vector<vector<bool>> grid;

void convertToGrid(const nav_msgs::OccupancyGrid& map)
{
	int rows;
	int cols;
	int currCell;

	rows = map.info.height;
	cols = map.info.width;
	currCell = 0;
    grid.resize(rows);
    for (int i = 0; i < rows; i++)
	{
		grid[i].resize(cols);
	}
    for (int i = 0; i < rows; i++)
	{
        for(int j = 0; j < cols; j++)
		{
            if (map.data[currCell] == 0)
                grid[i][j] = false;
            else
                grid[i][j] = true;
            currCell++;
        }
    }
}

void convertToArray(nav_msgs::OccupancyGrid& cleanedMap)
{
	int rows;
	int cols;
	int currCell;

	rows = cleanedMap.info.height;
	cols = cleanedMap.info.width;
	currCell = 0;
    for (int i = 0; i < rows; i++)
	{
        for(int j = 0; j < cols; j++)
		{
			cleanedMap.data[currCell] = (grid[i][j] ? 100 : 0);
            currCell++;
        }
    }
}

void fillCleanedMap(const nav_msgs::OccupancyGrid& map, nav_msgs::OccupancyGrid& cleanedMap)
{
	cleanedMap.header = map.header;
	cleanedMap.info = map.info;
	cleanedMap.data.resize(cleanedMap.info.width * cleanedMap.info.height);
	convertToArray(cleanedMap);
}

bool cleanClusterSize1(int i, int j, bool target)
{
	for (int ii = i - 1; ii < i + 2; ii++)
	{
		for (int jj = j - 1; jj < j + 2; jj++)
		{
			if (ii == i && jj == j)
			{
				continue;
			}
			if (target != grid[ii][jj])
			{
				return false;
			}
		}
	}
	grid[i][j] = target;
	return true;
}

bool cleanClusterSizeHor2(int i, int j, bool target)
{
	if (j + 2 == grid.size())
	{
		return false;
	}
	for (int ii = i - 1; ii < i + 2; ii++)
	{
		for (int jj = j - 1; jj < j + 3; jj++)
		{
			if (ii == i && jj == j || ii == i && jj == j + 1)
			{
				continue;
			}
			if (target != grid[ii][jj])
			{
				return false;
			}
		}
	}
	grid[i][j] = target;
	grid[i][j + 1] = target;
	return true;
}

bool cleanClusterSizeVer2(int i, int j, bool target)
{
	if (i + 2 == grid.size())
	{
		return false;
	}
	for (int ii = i - 1; ii < i + 3; ii++)
	{
		for (int jj = j - 1; jj < j + 2; jj++)
		{
			if (ii == i && jj == j || ii == i + 1 && jj == j)
			{
				continue;
			}
			if (target != grid[ii][jj])
			{
				return false;
			}
		}
	}
	grid[i][j] = target;
	grid[i + 1][j] = target;
	return true;
}

bool cleanClusterSize4(int i, int j, bool target)
{
	if (i + 2 == grid.size() || j + 2 == grid.size())
	{
		return false;
	}
	for (int ii = i - 1; ii < i + 3; ii++)
	{
		for (int jj = j - 1; jj < j + 3; jj++)
		{
			if (ii == i && jj == j || ii == i && jj == j + 1 || ii == i + 1 && jj == j || ii == i + 1 && jj == j + 1)
			{
				continue;
			}
			if (target != grid[ii][jj])
			{
				return false;
			}
		}
	}
	grid[i][j] = target;
	grid[i][j + 1] = target;
	grid[i + 1][j] = target;
	grid[i + 1][j + 1] = target;
	return true;
}

void clean()
{
	bool target;

	for (int i = 1; i < grid.size() - 1; i++)
	{
		target = grid[i][0];
		for (int j = 1; j < grid[i].size() - 1; j++)
		{
			if (target != grid[i][j])
			{
				if (!cleanClusterSize1(i, j, target))
				{
					if (!cleanClusterSizeHor2(i, j, target))
					{
						if (!cleanClusterSizeVer2(i, j, target))
						{
							if (!cleanClusterSize4(i, j, target))
							{
								target = grid[i][j];
							}
						}
					}
				}
			}
		}
	}
}

bool cleanMap(building_map::CleanedMap::Request &req, building_map::CleanedMap::Response &res)
{
	convertToGrid(req.map);
	clean();
	fillCleanedMap(req.map, res.cleaned_map);
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_cleaner_node");
	ros::NodeHandle nh;
	ros::ServiceServer server = nh.advertiseService("cleaned_map", cleanMap);

	ROS_INFO("SERVER");
	ros::spin();
	return 0;
}
