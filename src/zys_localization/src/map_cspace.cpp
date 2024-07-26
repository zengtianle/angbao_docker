/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <queue>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "zys_localization/map.h"


class CellData
{
  public:
    map_t* map_;
    unsigned int i_, j_;
    unsigned int src_i_, src_j_;
};

class CachedDistanceMap
{
  public:
    CachedDistanceMap(double scale, double max_dist) : 
      distances_(NULL), scale_(scale), max_dist_(max_dist) 
    {
      cell_radius_ = max_dist / scale;
      distances_ = new double *[cell_radius_+2];
      for(int i=0; i<=cell_radius_+1; i++)
      {
	distances_[i] = new double[cell_radius_+2];
        for(int j=0; j<=cell_radius_+1; j++)
	{
	  distances_[i][j] = sqrt(i*i + j*j);
	}
      }
    }
    ~CachedDistanceMap()
    {
      if(distances_)
      {
	for(int i=0; i<=cell_radius_+1; i++)
	  delete[] distances_[i];
	delete[] distances_;
      }
    }
    double** distances_;
    double scale_;
    double max_dist_;
    int cell_radius_;
};


bool operator<(const CellData& a, const CellData& b)
{
  return a.map_->cells[MAP_INDEX(a.map_, a.i_, a.j_)].occ_dist > a.map_->cells[MAP_INDEX(b.map_, b.i_, b.j_)].occ_dist;
}

CachedDistanceMap*
get_distance_map(double scale, double max_dist)
{
  static CachedDistanceMap* cdm = NULL;

  if(!cdm || (cdm->scale_ != scale) || (cdm->max_dist_ != max_dist))
  {
    if(cdm)
      delete cdm;
    cdm = new CachedDistanceMap(scale, max_dist);
  }

  return cdm;
}
//判断位移后的格子距离原来的距离是否超出cell_radius_
void enqueue(map_t* map, unsigned int i, unsigned int j, 
	     unsigned int src_i, unsigned int src_j,
	     std::priority_queue<CellData>& Q,
	     CachedDistanceMap* cdm,
	     unsigned char* marked)
{
  if(marked[MAP_INDEX(map, i, j)])
    return;

  unsigned int di = fabs(i - src_i);
  unsigned int dj = fabs(j - src_j);
  double distance = cdm->distances_[di][dj];

  if(distance > cdm->cell_radius_)
    return;

  map->cells[MAP_INDEX(map, i, j)].occ_dist = distance * map->scale;

  CellData cell;
  cell.map_ = map;
  cell.i_ = i;
  cell.j_ = j;
  cell.src_i_ = src_i;
  cell.src_j_ = src_j;

  Q.push(cell);

  marked[MAP_INDEX(map, i, j)] = 1;
}

// Update the cspace distance values
void map_update_cspace(map_t *map, double max_occ_dist)
{
  unsigned char* marked;
  std::priority_queue<CellData> Q;

  marked = new unsigned char[map->size_x*map->size_y];
  memset(marked, 0, sizeof(unsigned char) * map->size_x*map->size_y);

  map->max_occ_dist = max_occ_dist;

  CachedDistanceMap* cdm = get_distance_map(map->scale, map->max_occ_dist);

  // Enqueue all the obstacle cells
  CellData cell;
  cell.map_ = map;
  for(int i=0; i<map->size_x; i++)
  {
    cell.src_i_ = cell.i_ = i;
    for(int j=0; j<map->size_y; j++)
    {
      if(map->cells[MAP_INDEX(map, i, j)].occ_state == +1)
      {
        map->cells[MAP_INDEX(map, i, j)].occ_dist = 0.0;
        cell.src_j_ = cell.j_ = j;
        marked[MAP_INDEX(map, i, j)] = 1;
        Q.push(cell);
      }
      else
      	map->cells[MAP_INDEX(map, i, j)].occ_dist = max_occ_dist;
    }
  }

  while(!Q.empty())
  {
    CellData current_cell = Q.top();
    if(current_cell.i_ > 0)
      enqueue(map, current_cell.i_-1, current_cell.j_, 
	      current_cell.src_i_, current_cell.src_j_,
	      Q, cdm, marked);
    if(current_cell.j_ > 0)
      enqueue(map, current_cell.i_, current_cell.j_-1, 
	      current_cell.src_i_, current_cell.src_j_,
	      Q, cdm, marked);
    if((int)current_cell.i_ < map->size_x - 1)
      enqueue(map, current_cell.i_+1, current_cell.j_, 
	      current_cell.src_i_, current_cell.src_j_,
	      Q, cdm, marked);
    if((int)current_cell.j_ < map->size_y - 1)
      enqueue(map, current_cell.i_, current_cell.j_+1, 
	      current_cell.src_i_, current_cell.src_j_,
	      Q, cdm, marked);

    Q.pop();
  }


    //   // cout << "Writing Likelihood Field Map to " << filename << "..." <<endl;
    // FILE* out = fopen("/home/zhang/map/test.pgm", "w");
    // if (!out) { 
    //     // std::cout << "Could not save likelihood field map." << endl;
    //     return;
    // }
    // int maxGray = 127;
    // fprintf(out, "P5\n%d %d\n%d\n", (int) map->size_x, (int) map->size_y, maxGray);
    //         // std::cout<<"--------------------"<<endl;
    // for (int y = map->size_y; y > 0; --y) {

    //     for (int x = 0; x < map->size_x; ++x) {
    //         int output = map->cells[MAP_INDEX(map, x, y)].occ_dist * maxGray;
    //         fputc(output, out);
    //     }
    // }
    // fclose(out);
    // cout << "... Finished!" << endl;
  delete[] marked;
}

void map_update_cspace_changed_cells(const int min_x, const int max_x, const int min_y, const int max_y, const double max_occ_dist, map_t *map)
{
   unsigned char* marked;
   marked = new unsigned char[map->size_x*map->size_y];
   memset(marked, 1, sizeof(unsigned char) * map->size_x*map->size_y);

   std::priority_queue<CellData> Q;
   CachedDistanceMap* cdm = get_distance_map(map->scale, map->max_occ_dist);

   // Enqueue all the obstacle cells
   CellData cell;
   cell.map_ = map;
   for(int i=min_x; i<max_x; i++)
   {
      cell.src_i_ = cell.i_ = i;
      for(int j=min_y; j<max_y; j++)
      {
         marked[MAP_INDEX(map, i, j)] = 0;
         if(map->cells[MAP_INDEX(map, i, j)].occ_state == +1)
         {
            map->cells[MAP_INDEX(map, i, j)].occ_dist = 0.0;
            cell.src_j_ = cell.j_ = j;
            marked[MAP_INDEX(map, i, j)] = 1;
            Q.push(cell);
         }
         else {
            map->cells[MAP_INDEX(map, i, j)].occ_dist = max_occ_dist;
         }
      }
   }

   while(!Q.empty())
   {
     //循环判断每个格子向周围扩散最远的距离是多远,越远得分越接近2.0; map->cells[MAP_INDEX(map, i, j)].occ_dist 为得分
      CellData current_cell = Q.top();
      if(current_cell.i_ > 0)
         enqueue(map, current_cell.i_-1, current_cell.j_, 
               current_cell.src_i_, current_cell.src_j_,
               Q, cdm, marked);
      if(current_cell.j_ > 0)
         enqueue(map, current_cell.i_, current_cell.j_-1, 
               current_cell.src_i_, current_cell.src_j_,
               Q, cdm, marked);
      if((int)current_cell.i_ < map->size_x - 1)
         enqueue(map, current_cell.i_+1, current_cell.j_, 
               current_cell.src_i_, current_cell.src_j_,
               Q, cdm, marked);
      if((int)current_cell.j_ < map->size_y - 1)
         enqueue(map, current_cell.i_, current_cell.j_+1, 
               current_cell.src_i_, current_cell.src_j_,
               Q, cdm, marked);
      Q.pop();
   }
   
   delete[] marked;
}

void map_update_cspace_changed_cells1(const int min_x, const int max_x, const int min_y, const int max_y, const double max_occ_dist, map_t *map)
{
   // init distance;
   for(int x=min_x+1; x<max_x-1; x++){
      for(int y=min_y+1; y<max_y-1; y++){
         if(map->cells[MAP_INDEX(map, x, y)].occ_state == +1)
         {
            map->cells[MAP_INDEX(map, x, y)].occ_dist = 0.0;
         }
         else {
            map->cells[MAP_INDEX(map, x, y)].occ_dist = max_occ_dist;
         }
      }
   }
   // BUG FixMe;
   double ds=sqrt(2)*0.05; //resolution;
   double ls=0.05; //resolution;
  
   // distance propagation
   for(int x=min_x+1; x<=max_x-1; x++) {
      for(int y=min_y+1; y<=max_y-1; y++){
         double mval=max_occ_dist;
         int idx0 = MAP_INDEX(map, x, y);
         int idx1 = MAP_INDEX(map, x-1, y-1);
         int idx2 = MAP_INDEX(map, x-1, y);
         int idx3 = MAP_INDEX(map, x-1, y+1);
         int idx4 = MAP_INDEX(map, x, y-1);
         int idx5 = MAP_INDEX(map, x, y+1);
         int idx6 = MAP_INDEX(map, x+1, y-1);
         int idx7 = MAP_INDEX(map, x+1, y);
         int idx8 = MAP_INDEX(map, x+1, y+1);
         mval = mval < map->cells[idx1].occ_dist + ds? mval: map->cells[idx1].occ_dist + ds; // x-1, y-1 
         mval = mval < map->cells[idx2].occ_dist + ls? mval: map->cells[idx2].occ_dist + ls; // x-1, y
         mval = mval < map->cells[idx3].occ_dist + ds? mval: map->cells[idx3].occ_dist + ds; // x-1, y+1
         mval = mval < map->cells[idx4].occ_dist + ls? mval: map->cells[idx4].occ_dist + ls; // x, y-1
         mval = mval < map->cells[idx5].occ_dist + ls? mval: map->cells[idx5].occ_dist + ls; // x, y+1
         mval = mval < map->cells[idx6].occ_dist + ds? mval: map->cells[idx6].occ_dist + ds; // x+1, y-1
         mval = mval < map->cells[idx7].occ_dist + ls? mval: map->cells[idx7].occ_dist + ls; // x+1, y
         mval = mval < map->cells[idx8].occ_dist + ds? mval: map->cells[idx8].occ_dist + ds; // x+1, y+1
         mval = mval < max_occ_dist ? mval:max_occ_dist;
         map->cells[idx0].occ_dist = map->cells[idx0].occ_dist < mval ? map->cells[idx0].occ_dist : mval;
      }
   }
   
   for(int x=max_x-1; x>=min_x+1; x--) {
      for(int y=max_y-1; y>=min_y+1; y--) {
         double mval=max_occ_dist;
         int idx0 = MAP_INDEX(map, x, y);
         int idx1 = MAP_INDEX(map, x-1, y-1);
         int idx2 = MAP_INDEX(map, x-1, y);
         int idx3 = MAP_INDEX(map, x-1, y+1);
         int idx4 = MAP_INDEX(map, x, y-1);
         int idx5 = MAP_INDEX(map, x, y+1);
         int idx6 = MAP_INDEX(map, x+1, y-1);
         int idx7 = MAP_INDEX(map, x+1, y);
         int idx8 = MAP_INDEX(map, x+1, y+1);
         mval = mval < map->cells[idx1].occ_dist + ds? mval: map->cells[idx1].occ_dist + ds; // x-1, y-1 
         mval = mval < map->cells[idx2].occ_dist + ls? mval: map->cells[idx2].occ_dist + ls; // x-1, y
         mval = mval < map->cells[idx3].occ_dist + ds? mval: map->cells[idx3].occ_dist + ds; // x-1, y+1
         mval = mval < map->cells[idx4].occ_dist + ls? mval: map->cells[idx4].occ_dist + ls; // x, y-1
         mval = mval < map->cells[idx5].occ_dist + ls? mval: map->cells[idx5].occ_dist + ls; // x, y+1
         mval = mval < map->cells[idx6].occ_dist + ds? mval: map->cells[idx6].occ_dist + ds; // x+1, y-1
         mval = mval < map->cells[idx7].occ_dist + ls? mval: map->cells[idx7].occ_dist + ls; // x+1, y
         mval = mval < map->cells[idx8].occ_dist + ds? mval: map->cells[idx8].occ_dist + ds; // x+1, y+1
         mval = mval < max_occ_dist ? mval:max_occ_dist;
         map->cells[idx0].occ_dist = map->cells[idx0].occ_dist < mval ? map->cells[idx0].occ_dist : mval;
      }
   }
}

void map_update_cspace_changed(const int x, const int y, const int state, const double max_occ_dist, map_t *map)
{
   int i, j;
   int ni, nj;
   int s;
   double d;
   map_cell_t *cell, *ncell;

   map->max_occ_dist = max_occ_dist;
   s = (int) ceil(map->max_occ_dist / map->scale);

   // Reset the distance values in the ratios (x, y) +- 2.0m / map_scale
   for (j = y-s; j < y+s; j++)
   {
     for (i = x-s; i < x+s; i++)
     {
        if(MAP_VALID(map, i, j)) {
           cell = map->cells + MAP_INDEX(map, i, j);
           cell->occ_dist = map->max_occ_dist;
        }
     }
   }

   // Find all the occupied cells and update their neighbours
   for (j = y-s; j < y+s; j++)
   {
      for (i = x-s; i < y+s; i++)
      {
         cell = map->cells + MAP_INDEX(map, i, j);
         if (cell->occ_state != +1)
            continue;
          
         cell->occ_dist = 0;

         // Update adjacent cells
         for (nj = -s; nj <= +s; nj++)
         {
            for (ni = -s; ni <= +s; ni++)
            {
               if (!MAP_VALID(map, i + ni, j + nj))
                  continue;
               
               ncell = map->cells + MAP_INDEX(map, i + ni, j + nj);
               d = map->scale * sqrt(ni * ni + nj * nj);
               
               if (d < ncell->occ_dist)
                  ncell->occ_dist = d;
            }
         }
      }
   }
}

#if 0
// TODO: replace this with a more efficient implementation.  Not crucial,
// because we only do it once, at startup.
void map_update_cspace(map_t *map, double max_occ_dist)
{
  int i, j;
  int ni, nj;
  int s;
  double d;
  map_cell_t *cell, *ncell;

  map->max_occ_dist = max_occ_dist;
  s = (int) ceil(map->max_occ_dist / map->scale);

  // Reset the distance values
  for (j = 0; j < map->size_y; j++)
  {
    for (i = 0; i < map->size_x; i++)
    {
      cell = map->cells + MAP_INDEX(map, i, j);
      cell->occ_dist = map->max_occ_dist;
    }
  }

  // Find all the occupied cells and update their neighbours
  for (j = 0; j < map->size_y; j++)
  {
    for (i = 0; i < map->size_x; i++)
    {
      cell = map->cells + MAP_INDEX(map, i, j);
      if (cell->occ_state != +1)
        continue;
          
      cell->occ_dist = 0;

      // Update adjacent cells
      for (nj = -s; nj <= +s; nj++)
      {
        for (ni = -s; ni <= +s; ni++)
        {
          if (!MAP_VALID(map, i + ni, j + nj))
            continue;

          ncell = map->cells + MAP_INDEX(map, i + ni, j + nj);
          d = map->scale * sqrt(ni * ni + nj * nj);

          if (d < ncell->occ_dist)
            ncell->occ_dist = d;
        }
      }
    }
  }
  
  return;
}


#endif
