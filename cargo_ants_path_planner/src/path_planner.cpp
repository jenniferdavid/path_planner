/* Cargo-ANTs software prototype.
 *
 * Copyright (C) 2014 Roland Philippsen. All rights reserved.
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "cargo_ants_msgs/Task.h"
#include "cargo_ants_msgs/TaskStatus.h"
#include "cargo_ants_msgs/Path.h"
#include "cargo_ants_msgs/MockupMap.h"
#include "cargo_ants_msgs/VehicleState.h"
#include "cargo_ants_util/util.h"
#include "sfl2/TraversabilityMap.h"
#include "sfl2/Estar.h"
#include <sfl/util/numeric.hpp>
#include <sfl/gplan/Mapper2d.hpp>
#include <estar2/estar.h>
#include <list>

using namespace cargo_ants_msgs;
using namespace cargo_ants_util;


namespace path_planner {
  
  
  class PathPlanner
  {
  public:
    explicit PathPlanner (std::string const & name)
      : name_ (name),
	path_id_ (0),
	pose_ (Eigen::Vector3d::Zero()),
	active_goal_ (0),
	prev_active_goal_ (0)
    {
      ros::NodeHandle node;
      vehicle_state_sub_ = node.subscribe (name + "/vehicle_state", 10,
					   &PathPlanner::vehicleStateCB, this);
      path_pub_ = node.advertise<Path> (name + "/path", 10);
      site_map_sub_ = node.subscribe ("/site_map", 10, &PathPlanner::siteMapCB, this);
      travmap_pub_ = node.advertise<sfl2::TraversabilityMap> (name + "/travmap", 10);
      estar_pub_ = node.advertise<sfl2::Estar> (name + "/estar", 10);
      
      // Notice the absolute topic names for task and task_status:
      // they are shared by all vehicles.
      //
      task_sub_ = node.subscribe ("/task", 10, &PathPlanner::taskCB, this);
      task_status_pub_ = node.advertise<TaskStatus> ("/task_status", 10);
    }
    
    
    void vehicleStateCB (VehicleState::ConstPtr const & msg)
    {
      pose_ = pose3Dto2D (msg->location, msg->orientation);
    }
    
    
    void taskCB (Task::ConstPtr const & msg)
    {
      if (msg->vehicle == name_) {
	task_queue_.push_back (msg);
      }
    }
    
    
    void siteMapCB (MockupMap::ConstPtr const & msg)
    {
      site_map_ = msg;
    }
    
    
    void update ()
    {
      TaskStatus status;
      
      if ( ! task_queue_.empty()) {
	
	if ( ! task_goals_.empty()) {
	  status.task_id = task_id_;
	  status.status = TaskStatus::PREEMPTED;
	  task_status_pub_.publish (status);
	  ROS_INFO ("task %llu preempted by %s", task_id_, name_.c_str());
	}
	task_goals_.clear();
	
	for (std::vector<Goal>::const_iterator ip (task_queue_.back()->goals.begin());
	     ip != task_queue_.back()->goals.end(); ++ip)
	  {
	    task_goals_.push_back (*ip);
	  }
	prev_active_goal_ = -1;
	active_goal_ = 0;
	task_id_ = task_queue_.back()->task_id;
	task_queue_.pop_back();
	
	for (task_queue_t::iterator ip (task_queue_.begin()); ip != task_queue_.end(); ++ip) {
	  status.task_id = (*ip)->task_id;
	  status.status = TaskStatus::SKIPPED;
	  task_status_pub_.publish (status);
	  ROS_INFO ("task %llu skipped by %s", (*ip)->task_id, name_.c_str());
	}
	
	ROS_INFO ("task %llu accepted by %s", task_id_, name_.c_str());
	
	task_queue_.clear();
      }
      
      while (active_goal_ < task_goals_.size()) {
	if (goalReached (task_goals_[active_goal_], pose_, true)) {
	  ROS_INFO ("goal %zu reached by %s", active_goal_, name_.c_str());
	  ++active_goal_;
	}
	else {
	  break;
	}
      }
      
      if (active_goal_ >= task_goals_.size()) {

	// Similarly to what can happen in path_adaptor, we might end
	// up at the final goal of a freshly requested task, while
	// there still is a path from a previous task running. In
	// that case, we'd need to tell the path_adaptor to stop as
	// well.
	
	if (prev_active_goal_ != active_goal_) {
	  status.task_id = task_id_;
	  status.status = TaskStatus::DONE;
	  task_status_pub_.publish (status);
	  
	  ROS_INFO ("task %llu finished by %s", task_id_, name_.c_str());
	  
	}
	// else { we've already handled this }
	
      }
      else {
	
	if (prev_active_goal_ != active_goal_) {
	  status.task_id = task_id_;
	  status.status = TaskStatus::PLANNING;
	  status.active_goal = active_goal_;
	  task_status_pub_.publish (status);
	  
	  ROS_INFO ("start planning for goal %zu of task %llu by %s",
		    active_goal_, task_id_, name_.c_str());
	  
	  Path path; // local var OK for mockup... will need smarter solution later
	  path.path_id = path_id_++;
	  path.goals = createPath (pose_, task_goals_[active_goal_], site_map_);
	  path_pub_.publish (path);
	  
	  ROS_INFO ("finished planning for goal %zu of task %llu by %s",
		    active_goal_, task_id_, name_.c_str());
	  
	  status.status = TaskStatus::ACTIVE;
	  task_status_pub_.publish (status);
	}
	// else {
	//   Do nothing. Unlike path_adaptor, which sends a trajectory
	//   at each update, the path_planner send a new path only
	//   when we transition to a new task_goal.
	// }	
	
      }
      
      prev_active_goal_ = active_goal_;
    }
    
    
    // "Obviously" this method shouldn't just create everything from
    // scratch each time it is called...
    //
    std::vector<Goal> createPath (Eigen::Vector3d pose,
				  Goal const & task_goal,
				  MockupMap::ConstPtr const & site_map)
    {
      ROS_INFO ("%s creating path (%g   %g   %g) to (%g   %g   %g) on map with %zu entries",
		name_.c_str(),
		pose[0], pose[1], pose[2],
		task_goal.gx, task_goal.gy, task_goal.gth,
		site_map->entries.size());
      
      //////////////////////////////////////////////////
      // create costmap
      
      // XXXX make these configurable...
      static double const map_resolution (1.0);
      static double const robot_radius (1.5);
      static double const buffer_zone (3.0);
      static double const decay_power (2.0);

      sfl::GridFrame const gf_sfl (pose_[0], pose_[1], pose_[2], map_resolution);
      
      boost::shared_ptr <sfl::Mapper2d>
	m2d (sfl::Mapper2d::Create (gf_sfl, robot_radius, buffer_zone, decay_power));
      boost::shared_ptr <sfl::TraversabilityMap> travmap (m2d->GetTravmap());
      int const freespace (travmap->freespace);
      travmap->Autogrow (pose[0], pose[1], freespace);
      travmap->Autogrow (task_goal.gx, task_goal.gy, freespace);
      
      for (size_t ie (0); ie < site_map->entries.size(); ++ie) {
	MockupMapEntry const & entry (site_map->entries[ie]);
	if (entry.name != name_) {
	  // blindly assuming y0, x1, y1 have same size...
	  for (size_t il (0); il < entry.x0.size(); ++il) {
	    m2d->AddObstacleLine (entry.x0[il], entry.y0[il], entry.x1[il], entry.y1[il], false);
	  }
	}
      }
      
      ROS_INFO ("%s created costmap %zd x %zd", name_.c_str(),
		travmap->grid.xend() - travmap->grid.xbegin(),
		travmap->grid.yend() - travmap->grid.ybegin());
      
      //////////////////////////////////////////////////
      // run E*
      //
      // some complications due to the "smart" nature of the flexgrid
      // used by traversability map... by construction, we know though
      // that grid.xbegin() and ybegin() are never positive.
      ssize_t const xbegin (travmap->grid.xbegin());
      ssize_t const ybegin (travmap->grid.ybegin());
      ssize_t const xend (travmap->grid.xend());
      ssize_t const yend (travmap->grid.yend());
      ssize_t const dimx (xend - xbegin);
      ssize_t const dimy (yend - ybegin);
      int const obstacle (travmap->obstacle);
      double const scale (1.0 / (obstacle - freespace));
      sfl::TraversabilityMap::grid_t const & grid (travmap->grid);
      
      estar_t estar;
      estar_init (&estar, dimx, dimy);
      for (ssize_t ix (xbegin); ix < xend; ++ix) {
	for (ssize_t iy (ybegin); iy < yend; ++iy) {
	  estar_set_speed (&estar, ix - xbegin, iy - ybegin,
			   sfl::boundval (0.0, scale * (obstacle - grid.at(ix, iy)), 1.0));
	}
      }
      
      ROS_INFO ("%s set E* speed map", name_.c_str());
      
      // it is easier to translater to and from travmap and E*
      // indexing by using a separate GridFrame.
      //
      sfl::GridFrame::position_t const org (gf_sfl.GlobalPoint (xbegin, ybegin));
      sfl::GridFrame const gf_estar (org.v0, org.v1, gf_sfl.Theta(), gf_sfl.Delta());
      sfl::GridFrame::index_t const
	goal_estar (gf_estar.GlobalIndex (task_goal.gx, task_goal.gy));
      
      ROS_INFO ("%s setting E* goal to %zd %zd", name_.c_str(), goal_estar.v0, goal_estar.v1);
      
      estar_set_goal (&estar, goal_estar.v0, goal_estar.v1);
      
      // Could propagate less, just until we hit the robot (or
      // actually a bit more just to be sure we can compute a nice
      // gradient), but keep it simple for now.
      //
      while (estar.pq.len != 0) {
	estar_propagate (&estar);
      }
      
      //////////////////////////////////////////////////
      // trace back the result
      
      static double const viadist (3.0); // XXXX make this configurable
      static double const viadist2 (pow (viadist, 2));
      
      sfl::GridFrame::index_t const poseidx (gf_estar.GlobalIndex (pose[0], pose[1]));
      estar_cell_t * cell (estar_grid_at (&estar.grid, poseidx.v0, poseidx.v1));
      std::vector<Goal> goals;
      
      ROS_INFO ("%s tracing back from start %zd %zd", name_.c_str(), poseidx.v0, poseidx.v1);
      
      if (std::isinf (cell->rhs)) {
	if (std::isinf (cell->phi)) {
	  ROS_WARN ("%s: E* start cell has infinite rhs and phi", name_.c_str());
	}
	else {
	  ROS_WARN ("%s: E* start cell has infinite rhs and phi = %g", name_.c_str(), cell->phi);
	}
      }
      else {
	double px (poseidx.v0);
	double py (poseidx.v1);
	Goal goal;
	
	static double const dmax (1.3 * cell->rhs); // hack to avoid limit cycling
	static double const stepsize (map_resolution / 4); // make this ocnfigurable?
	
	for (double dd (0.0); dd <= dmax; dd += stepsize) {
	  double gradx, grady;
	  if (0 == estar_cell_calc_gradient (cell, &gradx, &grady)) {
	    if (goals.empty()) {
	      ROS_WARN ("%s: E* could not compute gradient at start", name_.c_str());
	      fprintf (stderr, "  center rhs %g phi %g\n", cell->rhs, cell->phi);
	      estar_cell_t ** nn;
	      for (nn = cell->nbor; *nn != NULL; ++nn) {
		fprintf (stderr, "     nbr rhs %g phi %g\n", (*nn)->rhs, (*nn)->phi);
	      }
	    }
	    break;
	  }
	  double const gradmag (sqrt(pow(gradx, 2.0) + pow(grady, 2.0)));
	  gradx *= stepsize / gradmag;
	  grady *= stepsize / gradmag;
	  px += gradx;
	  py += grady;
	  
	  // Here, (px, py) is the sub-pixel-resolution E* grid
	  // coordinate of where we want to be next. This needs to get
	  // translated to global coordinates by taking into account
	  // the grid resolution and origin.
	  //
	  // If that is farther than viadist away from the
	  // previous goal, or if there is no goal yet and it is
	  // farther than viadist from the start, then a new goal
	  // gets appended to the goal list. In that case, also the
	  // current gradient is (gradx, grady) from which the goal
	  // heading needs to be computed, and that needs to be
	  // rotated by the grid frame orientation.
	  
	  goal.gx = px * map_resolution;
	  goal.gy = py * map_resolution;
	  gf_estar.To (goal.gx, goal.gy);
	  
	  double dist2;
	  if (goals.empty()) {
	    dist2 = pow (goal.gx - pose[0], 2) + pow (goal.gy - pose[1], 2);
	  }
	  else {
	    dist2 = pow (goal.gx - goals.back().gx, 2) + pow (goal.gy - goals.back().gy, 2);
	  }
	  if (dist2 >= viadist2) {
	    goal.gth = sfl::mod2pi (atan2 (grady, gradx) + gf_estar.Theta());
	    goal.dr = viadist;
	    goal.dth = M_PI;
	    goals.push_back (goal);
	  }
	  
	  //////////////////////////////////////////////////
	  // prepare for next step
	  
	  ssize_t const ix ((ssize_t) rint(px));
	  ssize_t const iy ((ssize_t) rint(py));
	  if (ix < 0 || ix >= dimx || iy < 0 || iy >= dimy) {
	    ROS_INFO ("  fell off the map at %zd %zd", ix, iy);
	    break;
	  }
	  cell = estar_grid_at (&estar.grid, ix, iy);
	  if (cell->flags & ESTAR_FLAG_GOAL) {
	    ROS_INFO ("  hit the goal at %zd %zd", ix, iy);
	    break;
	  }
	}
	
	goals.push_back (task_goal);
      }

      ROS_INFO ("%s traced back E*, path length %zu", name_.c_str(), goals.size());
      
      //////////////////////////////////////////////////
      // that's all
      
      ROS_INFO ("%s done creating path", name_.c_str());
      
      //////////////////////////////////////////////////
      // debug: publish travmap and E* for visualization in nepumuk
      // (should be configurable)
      
      sfl2::TraversabilityMap travmap_msg;
      travmap_msg.gx = gf_sfl.X();
      travmap_msg.gy = gf_sfl.Y();
      travmap_msg.gth = gf_sfl.Theta();
      travmap_msg.delta = gf_sfl.Delta();
      travmap_msg.xbegin = xbegin;
      travmap_msg.xend = xend;
      travmap_msg.ybegin = ybegin;
      travmap_msg.yend = yend;
      travmap_msg.freespace = freespace;
      travmap_msg.obstacle = obstacle;
      travmap_msg.grid.resize (dimx * dimy);
      for (ssize_t ix (0); ix < dimx; ++ix) {
	for (ssize_t iy (0); iy < dimy; ++iy) {
	  travmap_msg.grid[ix + dimx * iy] = travmap->grid.at (ix + xbegin, iy + ybegin);
	}
      }
      travmap_pub_.publish (travmap_msg);
      
      sfl2::Estar estar_msg;
      estar_msg.gx = gf_estar.X();
      estar_msg.gy = gf_estar.Y();
      estar_msg.gth = gf_estar.Theta();
      estar_msg.delta = gf_estar.Delta();
      estar_msg.xdim = xend - xbegin;
      estar_msg.ydim = yend - ybegin;
      size_t const dim (dimx * dimy);
      estar_msg.value.resize (dim);
      for (size_t ii (0); ii < dim; ++ii) {
	estar_msg.value[ii] = estar.grid.cell[ii].phi;
      }
      estar_pub_.publish (estar_msg);
      
      //////////////////////////////////////////////////
      // free up estar2 stuff (do this last so we can use it for debug
      // etc above).
      
      estar_fini (&estar);
      
      return goals;
    }
    
    
  private:
    std::string name_;
    uint64_t task_id_;
    Eigen::Vector3d pose_;
    ros::Subscriber vehicle_state_sub_;
    ros::Subscriber task_sub_;
    ros::Subscriber site_map_sub_;
    ros::Publisher path_pub_;
    ros::Publisher task_status_pub_;
    ros::Publisher travmap_pub_;
    ros::Publisher estar_pub_;
    typedef std::list <Task::ConstPtr> task_queue_t;
    task_queue_t task_queue_;
    std::vector <Goal> task_goals_;
    size_t active_goal_;
    size_t prev_active_goal_;
    uint64_t path_id_;
    MockupMap::ConstPtr site_map_;
  };
  
}

using namespace path_planner;


int main (int argc, char ** argv)
{
  ros::init (argc, argv, "path_planner");
  
  if (argc < 2) {
    ROS_FATAL ("Please specify the vehicle names on the command line");
    return 1;
  }
  
  typedef std::list <boost::shared_ptr <PathPlanner> > path_planners_t;
  path_planners_t path_planners;
  for (int ii (1); ii < argc; ++ii) {
    path_planners.push_back (boost::shared_ptr <PathPlanner> (new PathPlanner (argv[ii])));
  }
  
  ros::Rate idle_rate (10);
  while (ros::ok()) {
    for (path_planners_t::iterator ip (path_planners.begin()); ip != path_planners.end(); ++ip) {
      (*ip)->update();
    }
    idle_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
