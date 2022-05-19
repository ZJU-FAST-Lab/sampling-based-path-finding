/*
Copyright (C) 2022 Hongkai Ye (kyle_yeh@163.com), Longji Yin (ljyin6038@163.com )
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/
#ifndef BRRT_STAR_H
#define BRRT_STAR_H

#include "occ_grid/occ_map.h"
#include "visualization/visualization.hpp"
#include "sampler.h"
#include "node.h"
#include "kdtree.h"

#include <ros/ros.h>
#include <utility>
#include <queue>
#include <algorithm>

namespace path_plan
{
  class BRRTStar
  {
  public:
    BRRTStar(){};
    BRRTStar(const ros::NodeHandle &nh, const env::OccMap::Ptr &mapPtr) : nh_(nh), map_ptr_(mapPtr)
    {
      nh_.param("BRRT_Star/steer_length", steer_length_, 0.0);
      nh_.param("BRRT_Star/search_radius", search_radius_, 0.0);
      nh_.param("BRRT_Star/search_time", search_time_, 0.0);
      nh_.param("BRRT_Star/max_tree_node_nums", max_tree_node_nums_, 0);
      nh_.param("BRRT_Star/use_informed_sampling", use_informed_sampling_, false);
      ROS_WARN_STREAM("[BRRT*] param: steer_length: " << steer_length_);
      ROS_WARN_STREAM("[BRRT*] param: search_radius: " << search_radius_);
      ROS_WARN_STREAM("[BRRT*] param: search_time: " << search_time_);
      ROS_WARN_STREAM("[BRRT*] param: max_tree_node_nums: " << max_tree_node_nums_);
      ROS_WARN_STREAM("[BRRT*] param: use_informed_sampling: " << use_informed_sampling_);

      sampler_.setSamplingRange(mapPtr->getOrigin(), mapPtr->getMapSize());

      valid_tree_node_nums_ = 0;
      nodes_pool_.resize(max_tree_node_nums_);
      for (int i = 0; i < max_tree_node_nums_; ++i)
      {
        nodes_pool_[i] = new TreeNode;
      }
    }
    ~BRRTStar(){};

    bool plan(const Eigen::Vector3d &s, const Eigen::Vector3d &g)
    {
      reset();
      if (!map_ptr_->isStateValid(s))
      {
        ROS_ERROR("[BRRT*]: Start pos collide or out of bound");
        return false;
      }
      if (!map_ptr_->isStateValid(g))
      {
        ROS_ERROR("[BRRT*]: Goal pos collide or out of bound");
        return false;
      }
      /* construct start and goal nodes */
      start_node_ = nodes_pool_[1];
      start_node_->x = s;
      start_node_->cost_from_start = 0.0;
      goal_node_ = nodes_pool_[0];
      goal_node_->x = g;
      goal_node_->cost_from_start = 0.0; // important
      valid_tree_node_nums_ = 2;         // put start and goal in tree

      ROS_INFO("[BRRT*]: BRRT* starts planning a path");

      /* Init the sampler for informed sampling */
      sampler_.reset();
      if(use_informed_sampling_)
      {
        calInformedSet(10000000000.0, s, g, scale_, trans_, rot_);
        sampler_.setInformedTransRot(trans_, rot_);
      }
      return brrt_star(s, g);
    }

    vector<Eigen::Vector3d> getPath()
    {
      return final_path_;
    }

    vector<vector<Eigen::Vector3d>> getAllPaths()
    {
      return path_list_;
    }

    vector<std::pair<double, double>> getSolutions()
    {
      return solution_cost_time_pair_list_;
    }

    void setVisualizer(const std::shared_ptr<visualization::Visualization> &visPtr)
    {
      vis_ptr_ = visPtr;
    };

  private:
    // nodehandle params
    ros::NodeHandle nh_;

    BiasSampler sampler_;
    // for informed sampling
    Eigen::Vector3d trans_, scale_;
    Eigen::Matrix3d rot_;
    bool use_informed_sampling_;

    double steer_length_;
    double search_radius_;
    double search_time_;
    int max_tree_node_nums_;
    int valid_tree_node_nums_;
    double first_path_use_time_;
    double final_path_use_time_;
    double cost_best_;

    std::vector<TreeNode *> nodes_pool_;
    TreeNode *start_node_;
    TreeNode *goal_node_;
    vector<Eigen::Vector3d> final_path_;
    vector<vector<Eigen::Vector3d>> path_list_;
    vector<std::pair<double, double>> solution_cost_time_pair_list_;

    // environment
    env::OccMap::Ptr map_ptr_;
    std::shared_ptr<visualization::Visualization> vis_ptr_;

    void reset()
    {
      final_path_.clear();
      path_list_.clear();
      cost_best_ = DBL_MAX;
      solution_cost_time_pair_list_.clear();

      for (int i = 0; i < valid_tree_node_nums_; i++)
      {
        nodes_pool_[i]->parent = nullptr;
        nodes_pool_[i]->children.clear();
      }
      valid_tree_node_nums_ = 0;
    }

    double calDist(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2)
    {
      return (p1 - p2).norm();
    }

    RRTNode3DPtr addTreeNode(RRTNode3DPtr &parent, const Eigen::Vector3d &state,
                             const double &cost_from_start, const double &cost_from_parent)
    {
      RRTNode3DPtr new_node_ptr = nodes_pool_[valid_tree_node_nums_];
      valid_tree_node_nums_++;
      new_node_ptr->parent = parent;
      parent->children.push_back(new_node_ptr);
      new_node_ptr->x = state;
      new_node_ptr->cost_from_start = cost_from_start;
      new_node_ptr->cost_from_parent = cost_from_parent;
      return new_node_ptr;
    }

    void changeNodeParent(RRTNode3DPtr &node, RRTNode3DPtr &parent, const double &cost_from_parent)
    {
      if (node->parent)
        node->parent->children.remove(node); //DON'T FORGET THIS, remove it form its parent's children list
      node->parent = parent;
      node->cost_from_parent = cost_from_parent;
      node->cost_from_start = parent->cost_from_start + cost_from_parent;
      parent->children.push_back(node);

      // for all its descedants, change the cost_from_start and tau_from_start;
      RRTNode3DPtr descendant(node);
      std::queue<RRTNode3DPtr> Q;
      Q.push(descendant);
      while (!Q.empty())
      {
        descendant = Q.front();
        Q.pop();
        for (const auto &leafptr : descendant->children)
        {
          leafptr->cost_from_start = leafptr->cost_from_parent + descendant->cost_from_start;
          Q.push(leafptr);
        }
      }
    }

    void fillPath(const RRTNode3DPtr &node_A, const RRTNode3DPtr &node_B, vector<Eigen::Vector3d> &path)
    {
      path.clear();
      RRTNode3DPtr node_ptr = node_A;
      while (node_ptr->parent)
      {
        path.push_back(node_ptr->x);
        node_ptr = node_ptr->parent;
      }
      path.push_back(start_node_->x);
      std::reverse(std::begin(path), std::end(path));

      node_ptr = node_B;
      while (node_ptr->parent)
      {
        path.push_back(node_ptr->x);
        node_ptr = node_ptr->parent;
      }
      path.push_back(goal_node_->x);
    }

    inline void sortNbrSet( Neighbour &nbrSet, Eigen::Vector3d &x_rand )
    {
      std::sort(nbrSet.nearing_nodes.begin(), nbrSet.nearing_nodes.end(),
              [&x_rand](NodeWithStatus &node1, NodeWithStatus &node2){
                return node1.node_ptr->cost_from_start + (node1.node_ptr->x - x_rand).norm() < 
                       node2.node_ptr->cost_from_start + (node2.node_ptr->x - x_rand).norm();
              });
    }

    inline void rewireTree( Neighbour &nbrSet, RRTNode3DPtr &new_node, const Eigen::Vector3d &x_target)
    {
      for(auto curr_node : nbrSet.nearing_nodes)
      {
        double dist_to_potential_child = calDist(new_node->x, curr_node.node_ptr->x);
        bool not_consistent = new_node->cost_from_start + dist_to_potential_child < curr_node.node_ptr->cost_from_start ? true : false;
        bool promising = new_node->cost_from_start + dist_to_potential_child + calDist(curr_node.node_ptr->x, x_target) < cost_best_ ? true : false;
        if( not_consistent && promising )
        {
          bool connected(false);
          if (curr_node.is_checked)
            connected = curr_node.is_valid;
          else 
            connected = map_ptr_->isSegmentValid(new_node->x, curr_node.node_ptr->x);
          
          if(connected)
            changeNodeParent(curr_node.node_ptr, new_node, dist_to_potential_child);
        }
      }
    }

    inline void chooseBestNode( Neighbour &nbrSet, const Eigen::Vector3d &x_rand, RRTNode3DPtr &min_node, 
                                double &cost_start, double &cost_parent)
    {
      for( auto &curr_node : nbrSet.nearing_nodes)
      {
        curr_node.is_checked = true;
        if(map_ptr_->isSegmentValid(curr_node.node_ptr->x, x_rand))
        {
          curr_node.is_valid = true;
          min_node = curr_node.node_ptr;
          cost_parent = calDist(min_node->x, x_rand);
          cost_start = min_node->cost_from_start + cost_parent;
          break;
        }else{
          curr_node.is_valid = false;
          continue;
        }
      }
    }

    bool brrt_star(const Eigen::Vector3d &s, const Eigen::Vector3d &g)
    {
      ros::Time rrt_start_time = ros::Time::now();
      bool tree_connected = false;
      double c_square = (g - s).squaredNorm() / 4.0;

      /* kd tree init */
      kdtree *treeA = kd_create(3);
      kdtree *treeB = kd_create(3);
      //Add start and goal nodes to kd trees
      kd_insert3(treeA, start_node_->x[0], start_node_->x[1], start_node_->x[2], start_node_);
      kd_insert3(treeB, goal_node_->x[0], goal_node_->x[1], goal_node_->x[2], goal_node_);
      
      /* main loop */
      int idx = 0;
      for (idx = 0; (ros::Time::now() - rrt_start_time).toSec() < search_time_ && valid_tree_node_nums_ < max_tree_node_nums_; ++idx)
      {
        bool check_connect = false;
        bool selectTreeA = true;

        /* random sampling */
        Eigen::Vector3d x_rand;
        sampler_.samplingOnce(x_rand);

        if (!map_ptr_->isStateValid(x_rand))
        {
          continue;
        }

        /* Search neighbors in both treeA and treeB */
        Neighbour neighbour_nodesA, neighbour_nodesB;
        neighbour_nodesA.nearing_nodes.reserve(80);
        neighbour_nodesB.nearing_nodes.reserve(80);
        neighbour_nodesB.center = neighbour_nodesA.center = x_rand;
        struct kdres *nbr_setA = kd_nearest_range3(treeA, x_rand[0], x_rand[1], x_rand[2], search_radius_);
        struct kdres *nbr_setB = kd_nearest_range3(treeB, x_rand[0], x_rand[1], x_rand[2], search_radius_); 

        if ( nbr_setA == nullptr ) // TreeA
        {
          struct kdres *p_nearest = kd_nearest3(treeA, x_rand[0], x_rand[1], x_rand[2]);
          if (p_nearest == nullptr)
          {
            ROS_ERROR("nearest query error");
            continue;
          }
          RRTNode3DPtr nearest_node = (RRTNode3DPtr)kd_res_item_data(p_nearest);
          kd_res_free(p_nearest);
          neighbour_nodesA.nearing_nodes.emplace_back(nearest_node, false, false);

        }else{
          check_connect = true;
          while (!kd_res_end(nbr_setA)){
            RRTNode3DPtr curr_node = (RRTNode3DPtr)kd_res_item_data(nbr_setA);
            neighbour_nodesA.nearing_nodes.emplace_back(curr_node, false, false);
            // store range query result so that we dont need to query again for rewire;
            kd_res_next(nbr_setA); //go to next in kd tree range query result
          }
        }
        kd_res_free(nbr_setA); //reset kd tree range query

        if ( nbr_setB == nullptr )// TreeB
        {
          struct kdres *p_nearest = kd_nearest3(treeB, x_rand[0], x_rand[1], x_rand[2]);
          if (p_nearest == nullptr)
          {
            ROS_ERROR("nearest query error");
            continue;
          }
          RRTNode3DPtr nearest_node = (RRTNode3DPtr)kd_res_item_data(p_nearest);
          kd_res_free(p_nearest);
          neighbour_nodesB.nearing_nodes.emplace_back(nearest_node, false, false);

        }else{
          check_connect = true;
          while (!kd_res_end(nbr_setB)){
            RRTNode3DPtr curr_node = (RRTNode3DPtr)kd_res_item_data(nbr_setB);
            neighbour_nodesB.nearing_nodes.emplace_back(curr_node, false, false);
            // store range query result so that we dont need to query again for rewire;
            kd_res_next(nbr_setB); //go to next in kd tree range query result
          }
        }
        kd_res_free(nbr_setB); //reset kd tree range query

        /* Sort two neighbor sets */
        sortNbrSet(neighbour_nodesA, x_rand);
        sortNbrSet(neighbour_nodesB, x_rand);

        /* Get the best parent node in each tree */
        RRTNode3DPtr min_node_A(nullptr), min_node_B(nullptr);
        double min_cost_start_A(DBL_MAX), min_cost_start_B(DBL_MAX);
        double cost_parent_A(DBL_MAX), cost_parent_B(DBL_MAX);

        chooseBestNode(neighbour_nodesA, x_rand, min_node_A, min_cost_start_A, cost_parent_A);
        chooseBestNode(neighbour_nodesB, x_rand, min_node_B, min_cost_start_B, cost_parent_B);

        /* Select the best tree, insert the node and rewire the tree */
        RRTNode3DPtr new_node(nullptr);
        if( (min_node_A != nullptr) || (min_node_B != nullptr) )
        {
          if( min_cost_start_A < min_cost_start_B ){

            if(min_cost_start_A + calDist(x_rand, goal_node_->x) >= cost_best_)
              continue; // Sampling rejection

            selectTreeA = true;
            new_node = addTreeNode(min_node_A, x_rand, min_cost_start_A, cost_parent_A);
            kd_insert3(treeA, x_rand[0], x_rand[1], x_rand[2], new_node);
            rewireTree(neighbour_nodesA, new_node, goal_node_->x);

          }else{

            if(min_cost_start_B + calDist(x_rand, start_node_->x) >= cost_best_)
              continue; // Sampling rejection
            
            selectTreeA = false;
            new_node = addTreeNode(min_node_B, x_rand, min_cost_start_B, cost_parent_B);
            kd_insert3(treeB, x_rand[0], x_rand[1], x_rand[2], new_node);
            rewireTree(neighbour_nodesB, new_node, start_node_->x);
          }
        }
        if( (min_node_A == nullptr) || (min_node_B == nullptr) )
          check_connect = false; // No possible connection

        /* Check connection */
        if( check_connect )
        { 
          /* Accept connection if achieve better cost */
          double cost_curr =  min_cost_start_A + min_cost_start_B;
          if(cost_curr < cost_best_)
          {
            cost_best_ = cost_curr;
            tree_connected = true;
            vector<Eigen::Vector3d> curr_best_path;
            if( selectTreeA )
              fillPath(new_node, min_node_B, curr_best_path);
            else
              fillPath(min_node_A, new_node, curr_best_path);
            path_list_.emplace_back(curr_best_path);
            solution_cost_time_pair_list_.emplace_back(cost_best_, (ros::Time::now() - rrt_start_time).toSec());

            if(use_informed_sampling_)
            {
              /* Update informed set */
              scale_[0] = cost_best_ / 2.0;
              scale_[1] = sqrt(scale_[0] * scale_[0] - c_square);
              scale_[2] = scale_[1];
              sampler_.setInformedSacling(scale_);
              std::vector<visualization::ELLIPSOID> ellps;
              ellps.emplace_back(trans_, scale_, rot_);
              vis_ptr_->visualize_ellipsoids(ellps, "informed_set", visualization::yellow, 0.2);
            }
          }
        }
      }//End of one sampling iteration
        
      if (tree_connected)
      {
        final_path_use_time_ = (ros::Time::now() - rrt_start_time).toSec();
        ROS_INFO_STREAM("[BRRT*]: find_path_use_time: " << solution_cost_time_pair_list_.front().second << ", length: " << solution_cost_time_pair_list_.front().first);
        visualizeWholeTree();
        final_path_ = path_list_.back();
      
      }
      else if (valid_tree_node_nums_ == max_tree_node_nums_)
      {
        visualizeWholeTree();
        ROS_ERROR_STREAM("[BRRT*]: NOT CONNECTED TO GOAL after " << max_tree_node_nums_ << " nodes added to rrt-tree");
      }
      else
      {
        ROS_ERROR_STREAM("[BRRT*]: NOT CONNECTED TO GOAL after " << (ros::Time::now() - rrt_start_time).toSec() << " seconds");
      }
      return tree_connected;
    }

    void visualizeWholeTree()
    {
      // Sample and visualize the resultant tree
      vector<Eigen::Vector3d> vertice;
      vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> edges;
      vertice.clear(); edges.clear();
      sampleWholeTree(start_node_, vertice, edges);
      sampleWholeTree(goal_node_, vertice, edges);
      std::vector<visualization::BALL> tree_nodes;
      tree_nodes.reserve(vertice.size());
      visualization::BALL node_p;
      node_p.radius = 0.12;
      for (size_t i = 0; i < vertice.size(); ++i)
      {
        node_p.center = vertice[i];
        tree_nodes.push_back(node_p);
      }
      vis_ptr_->visualize_balls(tree_nodes, "tree_vertice", visualization::Color::blue, 1.0);
      vis_ptr_->visualize_pairline(edges, "tree_edges", visualization::Color::red, 0.06);
    }

    void sampleWholeTree(const RRTNode3DPtr &root, vector<Eigen::Vector3d> &vertice, vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &edges)
    {
      if (root == nullptr)
        return;

      // whatever dfs or bfs
      RRTNode3DPtr node = root;
      std::queue<RRTNode3DPtr> Q;
      Q.push(node);
      while (!Q.empty())
      {
        node = Q.front();
        Q.pop();
        for (const auto &leafptr : node->children)
        {
          vertice.push_back(leafptr->x);
          edges.emplace_back(std::make_pair(node->x, leafptr->x));
          Q.push(leafptr);
        }
      }
    }

    void calInformedSet(double a2, const Eigen::Vector3d &foci1, const Eigen::Vector3d &foci2,
                        Eigen::Vector3d &scale, Eigen::Vector3d &trans, Eigen::Matrix3d &rot)
    {
      trans = (foci1 + foci2) / 2.0;
      scale[0] = a2 / 2.0;
      Eigen::Vector3d diff(foci2 - foci1);
      double c_square = diff.squaredNorm() / 4.0;
      scale[1] = sqrt(scale[0] * scale[0] - c_square);
      scale[2] = scale[1];

      /* A generic implementation for SO(n) informed set */
      Eigen::Vector3d a1 = (foci2 - foci1) / calDist(foci1, foci2);
      Eigen::Matrix3d M = a1 * Eigen::MatrixXd::Identity(1,3);
      Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
      Eigen::Vector3d diag_v(1.0,svd.matrixU().determinant(),svd.matrixV().determinant());
      rot = svd.matrixU() * diag_v.asDiagonal() * svd.matrixV().transpose();
    } 


  public:
    void samplingOnce(Eigen::Vector3d &sample)
    {
      static int i = 0;
      sample = preserved_samples_[i];
      i++;
      i = i % preserved_samples_.size();
    }

    void setPreserveSamples(const vector<Eigen::Vector3d> &samples)
    {
      preserved_samples_ = samples;
    }
    vector<Eigen::Vector3d> preserved_samples_;
  };

} // namespace path_plan
#endif