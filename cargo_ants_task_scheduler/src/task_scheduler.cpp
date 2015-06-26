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
#include "cargo_ants_msgs/TaskRequest.h"
#include "cargo_ants_msgs/TaskRequestStatus.h"
#include "cargo_ants_msgs/Task.h"
#include <list>

using namespace cargo_ants_msgs;


static std::list<TaskRequest::ConstPtr> queue;


static void enqueue (TaskRequest::ConstPtr const & msg)
{
  queue.push_back (msg);
}


int main (int argc, char ** argv)
{
  ros::init (argc, argv, "task_scheduler");
  ros::NodeHandle node;
  ros::Subscriber request_sub (node.subscribe ("/task_request", 10, enqueue));
  ros::Publisher result_pub (node.advertise<Task> ("/task", 1));
  ros::Publisher status_pub (node.advertise<TaskRequestStatus> ("task_request_status", 1));
  
  ros::Rate idle_rate(10);
  TaskRequestStatus status;
  Task task;
  uint64_t task_id (0);
  
  while (ros::ok()) {
    
    while ( ! queue.empty()) {
      TaskRequest const & request (*queue.front());
      status.task_request_id = request.task_request_id;
      
      if (request.vehicles.empty()) {
	status.status = 42;
	status.message = "no vehicle specified";
      }
      else if (request.locations.empty()) {
	status.status = 42;
	status.message = "no location specified";
      }
      else {
	status.status = 0;
	status.message = "";
	task.task_id = ++task_id;
	task.vehicle = request.vehicles[0];
	// task.locations.clear();
	// task.locations.push_back ("foo");
	// task.locations.push_back (request.locations[0]);
	// task.locations.push_back ("bar");
	// for (size_t ii (1); ii < request.locations.size(); ++ii) {
	//   task.locations.push_back (request.locations[ii]);
	// }
	// task.locations.push_back ("baz");
	result_pub.publish (task);
      }
      
      status_pub.publish (status);
      queue.pop_front();
    }
    
    idle_rate.sleep();
    ros::spinOnce();
  }
  
  return 0;
}
