/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation, Inc. nor
 *     the names of its contributors may be used to endorse or promote
 *     products derived from this software without specific prior
 *     written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "gsoc_param_api/Set.h"
#include "gsoc_param_api/Get.h"
#include "gsoc_param_api/Has.h"
#include "gsoc_param_api/Del.h"
#include "gsoc_param_api/AddUpdater.h"
#include "gsoc_param_api/RemoveUpdater.h"
#include "gsoc_param_api/UpdateParameter.h"
#include "gsoc_param_api/AddAcceptor.h"
#include "gsoc_param_api/RemoveAcceptor.h"
#include "gsoc_param_api/AcceptParameter.h"

namespace gsoc {

  namespace param {

    namespace {

      ros::CallbackQueue g_param_callbackQueue;
      boost::shared_ptr<ros::AsyncSpinner> g_param_spinner;

    } // anonymous

    typedef boost::function<void (const std::vector<uint8_t>&, const std::vector<uint8_t>&)> UpdateCallback;
    typedef boost::function<bool (const std::vector<uint8_t>&, const std::vector<uint8_t>&)> AcceptCallback;

    class ParamServerProxy
    {
    public:
      ParamServerProxy(const std::string& namespc = "~")
      {
        ros::NodeHandle n(namespc);
        n.setCallbackQueue(&g_param_callbackQueue);
        updaterServer = n.advertiseService("update_parameter", &ParamServerProxy::update_parameter_callback, this);
        acceptorServer = n.advertiseService("accept_parameter", &ParamServerProxy::accept_parameter_callback, this);
        if (!g_param_spinner) {
          g_param_spinner.reset(new ros::AsyncSpinner(1, &g_param_callbackQueue));
          g_param_spinner->start();
        }
      }

      ~ParamServerProxy() 
      { }

      bool set(const std::string& name, const std::vector<uint8_t>& data)
      {
        gsoc_param_api::Set srv;
        srv.request.name = name;
        srv.request.data = data;
        std::string srv_name = "/param_server/set";
        if ( !ros::service::call(srv_name, srv) ) {
          ROS_ERROR_STREAM("Service " << srv_name << " is not available");
          return false;
        }
        return srv.response.accepted;
      }

      bool get(const std::string& name, std::vector<uint8_t>& data)
      {
        gsoc_param_api::Get srv;
        srv.request.name = name;
        std::string srv_name = "/param_server/get";
        if ( !ros::service::call(srv_name, srv) ) {
          ROS_ERROR_STREAM("Service " << srv_name << " is not available");
          return false;
        }
        if (srv.response.exist)
          data = srv.response.data;
        return srv.response.exist;
      }

      bool has(const std::string& name)
      {
        gsoc_param_api::Has srv;
        srv.request.name = name;
        std::string srv_name = "/param_server/has";
        if ( !ros::service::call(srv_name, srv) ) {
          ROS_ERROR_STREAM("Service " << srv_name << " is not available");
          return false;
        }
        return srv.response.exist;
      }

      bool del(const std::string& name)
      {
        gsoc_param_api::Del srv;
        srv.request.name = name;
        std::string srv_name = "/param_server/del";
        if ( !ros::service::call(srv_name, srv) ) {
          ROS_ERROR_STREAM("Service " << srv_name << " is not available");
          return false;
        }
        return srv.response.deleted;
      }


      bool addUpdater(const std::string& name, UpdateCallback cb)
      {
        if (!updateCallbacks.insert(make_pair(name, cb)).second)
          return false;

        gsoc_param_api::AddUpdater srv;
        srv.request.name = name;
        srv.request.updater = updaterServer.getService();
        std::string srv_name = "/param_server/add_updater";
        if ( !ros::service::call(srv_name, srv) ) {
          ROS_ERROR_STREAM("Service " << srv_name << " is not available");
          return false;
        }

        if (!srv.response.added)
          updateCallbacks.erase(name);

        return srv.response.added;
      }

      bool removeUpdater(const std::string& name)
      {
        if (updateCallbacks.erase(name) != 1)
          return false;

        gsoc_param_api::RemoveUpdater srv;
        srv.request.name = name;
        srv.request.updater = updaterServer.getService();
        std::string srv_name = "/param_server/remove_updater";
        if ( !ros::service::call(srv_name, srv) ) {
          ROS_ERROR_STREAM("Service " << srv_name << " is not available");
          return false;
        }
        return srv.response.removed;
      }

      bool addAcceptor(const std::string& name, AcceptCallback cb)
      {
        if (!acceptCallbacks.insert(make_pair(name, cb)).second)
          return false;

        gsoc_param_api::AddAcceptor srv;
        srv.request.name = name;
        srv.request.acceptor = acceptorServer.getService();
        std::string srv_name = "/param_server/add_acceptor";
        if ( !ros::service::call(srv_name, srv) ) {
          ROS_ERROR_STREAM("Service " << srv_name << " is not available");
          return false;
        }

        if (!srv.response.added)
          acceptCallbacks.erase(name);

        return srv.response.added;
      }

      bool removeAcceptor(const std::string& name)
      {
        if (acceptCallbacks.erase(name) != 1)
          return false;

        gsoc_param_api::RemoveAcceptor srv;
        srv.request.name = name;
        srv.request.acceptor = acceptorServer.getService();
        std::string srv_name = "/param_server/remove_acceptor";
        if ( !ros::service::call(srv_name, srv) ) {
          ROS_ERROR_STREAM("Service " << srv_name << " is not available");
          return false;
        }
        return srv.response.removed;
      }

    private:

      bool update_parameter_callback(gsoc_param_api::UpdateParameter::Request  &req,
                                     gsoc_param_api::UpdateParameter::Response &res)
      {
        updateCallbacks[req.name](req.old, req.actual);
        return true;
      }

      bool accept_parameter_callback(gsoc_param_api::AcceptParameter::Request  &req,
                                     gsoc_param_api::AcceptParameter::Response &res)
      {
        res.accepted = acceptCallbacks[req.name](req.actual, req.candidate);
        return true;
      }

      ros::ServiceServer updaterServer;
      std::map<std::string, UpdateCallback>  updateCallbacks;

      ros::ServiceServer acceptorServer;
      std::map<std::string, AcceptCallback> acceptCallbacks;
    };

    typedef boost::shared_ptr<ParamServerProxy> ParamServerProxyPtr;

  } // param

} // gsoc