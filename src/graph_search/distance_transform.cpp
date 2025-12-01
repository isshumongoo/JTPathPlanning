#include <iostream>
#include <vector>
#include <cmath>

#include <path_planning/utils/math_helpers.h>
#include <path_planning/utils/graph_utils.h>

#include <path_planning/graph_search/distance_transform.h>

/**
 * These functions should calculate the distance transform on the given graph
 * and store it in graph.obstacle_distances.
 *
 * This is part of the Distance Transform advanced extensions for Project 3.
 **/


void distanceTransformSlow(GridGraph& graph)
{
    /**
     * TODO (P3 - Practice for Advanced Extension): Perform a distance transform
     * by finding the distance to the nearest occupied cell for each unoccupied
     * cell. Calculate the distance to the nearest cell by looping through all
     * the occupied cells in the graph.
     *
     * Store the result in the vector graph.obstacle_distances.
     **/

    int width  = graph.width;
    int height = graph.height;
    int num_cells = width * height;

    // Collect indices of all occupied cells
    std::vector<int> occupied_idxs;
    occupied_idxs.reserve(num_cells);
    for (int idx = 0; idx < num_cells; ++idx)
    {
        if (isIdxOccupied(idx, graph))
        {
            occupied_idxs.push_back(idx);
        }
    }

    graph.obstacle_distances.resize(num_cells);

    // For each cell, compute distance to nearest occupied cell
    for (int idx = 0; idx < num_cells; ++idx)
    {
        if (isIdxOccupied(idx, graph))
        {
            // Distance to obstacle at its own cell is zero
            graph.obstacle_distances[idx] = 0.0f;
            continue;
        }

        float best_dist = HIGH;

        Cell c = idxToCell(idx, graph);
        float cx = static_cast<float>(c.i) * graph.meters_per_cell;
        float cy = static_cast<float>(c.j) * graph.meters_per_cell;

        for (int obs_idx : occupied_idxs)
        {
            Cell o = idxToCell(obs_idx, graph);
            float ox = static_cast<float>(o.i) * graph.meters_per_cell;
            float oy = static_cast<float>(o.j) * graph.meters_per_cell;

            float dx = cx - ox;
            float dy = cy - oy;
            float d  = std::sqrt(dx * dx + dy * dy);

            if (d < best_dist)
            {
                best_dist = d;
            }
        }

        graph.obstacle_distances[idx] = best_dist;
    }
}


void distanceTransformManhattan(GridGraph& graph)
{
    /**
     * TODO (P3 - Advanced Extension): Perform a distance transform using the Manhattan distance
     * transform algorithm over a 2D grid.
     *
     * Store the result in the vector graph.obstacle_distances.
     **/

    int width  = graph.width;
    int height = graph.height;
    int num_cells = width * height;

    std::vector<float> dt(num_cells, HIGH);

    // Initialize: 0 at obstacles, HIGH elsewhere
    for (int idx = 0; idx < num_cells; ++idx)
    {
        if (isIdxOccupied(idx, graph))
        {
            dt[idx] = 0.0f;
        }
    }

    // Forward pass (top-left to bottom-right)
    for (int j = 0; j < height; ++j)
    {
        for (int i = 0; i < width; ++i)
        {
            int idx = cellToIdx(i, j, graph);
            float best = dt[idx];

            if (i > 0)
            {
                int left_idx = cellToIdx(i - 1, j, graph);
                best = std::min(best, dt[left_idx] + 1.0f);
            }
            if (j > 0)
            {
                int up_idx = cellToIdx(i, j - 1, graph);
                best = std::min(best, dt[up_idx] + 1.0f);
            }

            dt[idx] = best;
        }
    }

    // Backward pass (bottom-right to top-left)
    for (int j = height - 1; j >= 0; --j)
    {
        for (int i = width - 1; i >= 0; --i)
        {
            int idx = cellToIdx(i, j, graph);
            float best = dt[idx];

            if (i < width - 1)
            {
                int right_idx = cellToIdx(i + 1, j, graph);
                best = std::min(best, dt[right_idx] + 1.0f);
            }
            if (j < height - 1)
            {
                int down_idx = cellToIdx(i, j + 1, graph);
                best = std::min(best, dt[down_idx] + 1.0f);
            }

            dt[idx] = best;
        }
    }

    // Store result
    graph.obstacle_distances = dt;
}


std::vector<float> distanceTransformEuclidean1D(std::vector<float>& init_dt)
{
    std::vector<float> dt(init_dt.begin(), init_dt.end());

    /**
     * TODO (P3 - Advanced Extension): Perform a distance transform using the
     * Euclidean distance transform algorithm over a 1D vector using the initial
     * values provided in init_dt.
     *
     * Store the result in the vector dt.
     **/

    int n = static_cast<int>(init_dt.size());
    if (n == 0)
    {
        return dt;
    }

    std::vector<int> v(n);
    std::vector<float> z(n + 1);

    // v: indices of parabolas
    // z: locations of boundaries between parabolas
    int k = 0;
    v[0] = 0;
    z[0] = -HIGH;
    z[1] = HIGH;

    for (int q = 1; q < n; ++q)
    {
        float s = 0.0f;

        while (true)
        {
            float numerator   = (init_dt[q] + q * q) - (init_dt[v[k]] + v[k] * v[k]);
            float denominator = 2.0f * (q - v[k]);
            if (denominator == 0.0f)
            {
                s = HIGH;
            }
            else
            {
                s = numerator / denominator;
            }

            if (s <= z[k])
            {
                k--;
                if (k < 0)
                {
                    k = 0;
                    break;
                }
            }
            else
            {
                break;
            }
        }

        k++;
        v[k] = q;
        z[k] = s;
        z[k + 1] = HIGH;
    }

    k = 0;
    for (int q = 0; q < n; ++q)
    {
        while (z[k + 1] < q)
        {
            k++;
        }

        float diff = static_cast<float>(q - v[k]);
        dt[q] = diff * diff + init_dt[v[k]];  // squared distance in cells
    }

    return dt;
}


void distanceTransformEuclidean2D(GridGraph& graph)
{
    /**
     * TODO (P3 - Advanced Extension): Perform a distance transform using the
     * Euclidean distance transform algorithm over a 2D grid. Use the
     * distanceTransformEuclidean1D() function.
     *
     * Store the result in the vector graph.obstacle_distances.
     **/

         int width  = graph.width;
    int height = graph.height;
    int num_cells = width * height;

    if (width == 0 || height == 0)
    {
        graph.obstacle_distances.clear();
        return;
    }

    // Initial values: 0 at obstacles, HIGH elsewhere
    std::vector<float> init_dt(num_cells, HIGH);
    for (int idx = 0; idx < num_cells; ++idx)
    {
        if (isIdxOccupied(idx, graph))
        {
            init_dt[idx] = 0.0f;
        }
    }

    // First pass: vertical (per column)
    std::vector<float> temp(num_cells);
    for (int i = 0; i < width; ++i)
    {
        std::vector<float> column(height);
        for (int j = 0; j < height; ++j)
        {
            int idx = cellToIdx(i, j, graph);
            column[j] = init_dt[idx];
        }

        std::vector<float> col_dt = distanceTransformEuclidean1D(column);

        for (int j = 0; j < height; ++j)
        {
            int idx = cellToIdx(i, j, graph);
            temp[idx] = col_dt[j];
        }
    }

    // Second pass: horizontal (per row)
    std::vector<float> final_dt(num_cells);
    for (int j = 0; j < height; ++j)
    {
        std::vector<float> row(width);
        for (int i = 0; i < width; ++i)
        {
            int idx = cellToIdx(i, j, graph);
            row[i] = temp[idx];
        }

        std::vector<float> row_dt = distanceTransformEuclidean1D(row);

        for (int i = 0; i < width; ++i)
        {
            int idx = cellToIdx(i, j, graph);
            final_dt[idx] = row_dt[i];
        }
    }

    // Take square root to get Euclidean distance in cells
    graph.obstacle_distances.resize(num_cells);
    for (int idx = 0; idx < num_cells; ++idx)
    {
        if (final_dt[idx] >= HIGH)
        {
            graph.obstacle_distances[idx] = HIGH;
        }
        else
        {
            graph.obstacle_distances[idx] = std::sqrt(final_dt[idx]);
        }
    }
}
