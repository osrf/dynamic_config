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

  namespace param_server {

    namespace {

      typedef std::map<std::string, std::vector<uint8_t> > Parameters;
      Parameters parameters;

      typedef std::set<std::string> UpdatersSet;
      typedef std::map<std::string, UpdatersSet> Updaters;
      Updaters updaters;

      typedef UpdatersSet AcceptorsSet;
      typedef Updaters Acceptors;
      Acceptors acceptors;

      template <typename Iterator, typename Srv>
      bool call_acceptors_services(Iterator first, Iterator last, Srv& srv)
      {
        while (first != last) {
          if ( !ros::service::call(*first, srv) ) {
            ROS_ERROR_STREAM("Acceptor " << *first << " is not available");
          }
          if (!srv.response.accepted) return false;
          ++first;
        }
        return true;
      }

      bool notify_acceptors(const std::string& name, const std::vector<uint8_t>& candidate)
      {
        gsoc_param_api::AcceptParameter srv;
        srv.request.name = name;
        srv.request.actual = parameters[name];
        srv.request.candidate = candidate;
        return call_acceptors_services(acceptors[name].begin(), acceptors[name].end(), srv);
      }

      template <typename Iterator, typename Srv>
      void call_updaters_services(Iterator first, Iterator last, Srv& srv)
      {
        while (first != last) {
          if ( !ros::service::call(*first, srv) ) {
            ROS_ERROR_STREAM("Updater " << *first << " is not available");
          }
          ++first;
        }
      }

      void notify_updaters(const std::string& name, const std::vector<uint8_t>& actual)
      {
        gsoc_param_api::UpdateParameter srv;
        srv.request.name = name;
        srv.request.old = parameters[name];
        srv.request.actual = actual;
        call_updaters_services(updaters[name].begin(), updaters[name].end(), srv);
      }

    } // anonymous

    bool set_callback(gsoc_param_api::Set::Request  &req,
                      gsoc_param_api::Set::Response &res)
    {
      res.accepted = notify_acceptors(req.name, req.data);
      if (res.accepted) {
        notify_updaters(req.name, req.data);
        parameters[req.name] = req.data;
      }
      return true;
    }

    bool get_callback(gsoc_param_api::Get::Request  &req,
                      gsoc_param_api::Get::Response &res)
    {
      Parameters::iterator it = parameters.find(req.name);
      res.exist = it != parameters.end();
      if (res.exist)
        res.data = it->second;
      return true;
    }

    bool has_callback(gsoc_param_api::Has::Request  &req,
                      gsoc_param_api::Has::Response &res)
    {
      res.exist = parameters.find(req.name) != parameters.end();
      return true;
    }

    bool del_callback(gsoc_param_api::Del::Request  &req,
                      gsoc_param_api::Del::Response &res)
    {
      res.deleted = parameters.erase(req.name) == 1;
      return true;
    }

    bool addUpdater_callback(gsoc_param_api::AddUpdater::Request  &req,
                             gsoc_param_api::AddUpdater::Response &res)
    {
      res.added = updaters[req.name].insert(req.updater).second;
      return true;
    }

    bool removeUpdater_callback(gsoc_param_api::RemoveUpdater::Request  &req,
                                gsoc_param_api::RemoveUpdater::Response &res)
    {
      res.removed = updaters[req.name].erase(req.updater) == 1;
      if (updaters[req.name].empty())
        updaters.erase(req.name);
      return true;
    }

    bool addAcceptor_callback(gsoc_param_api::AddAcceptor::Request  &req,
                              gsoc_param_api::AddAcceptor::Response &res)
    {
      res.added = acceptors[req.name].insert(req.acceptor).second;
      return true;
    }

    bool removeAcceptor_callback(gsoc_param_api::RemoveAcceptor::Request  &req,
                                 gsoc_param_api::RemoveAcceptor::Response &res)
    {
      res.removed = acceptors[req.name].erase(req.acceptor) == 1;
      if (acceptors[req.name].empty())
        acceptors.erase(req.name);
      return true;
    }

    namespace {

      ros::ServiceServer setServer;
      ros::ServiceServer getServer;
      ros::ServiceServer hasServer;
      ros::ServiceServer delServer;
      ros::ServiceServer addUpdaterServer;
      ros::ServiceServer removeUpdaterServer;
      ros::ServiceServer addAcceptorServer;
      ros::ServiceServer removeAcceptorServer;

    } // anonymous

    void init(const std::string& namespc = "~")
    {
      ros::NodeHandle n(namespc);
      setServer = n.advertiseService("set", gsoc::param_server::set_callback);
      getServer = n.advertiseService("get", gsoc::param_server::get_callback);
      hasServer = n.advertiseService("has", gsoc::param_server::has_callback);
      delServer = n.advertiseService("del", gsoc::param_server::del_callback);
      addUpdaterServer = n.advertiseService("add_updater", gsoc::param_server::addUpdater_callback);
      removeUpdaterServer = n.advertiseService("remove_updater", gsoc::param_server::removeUpdater_callback);
      addAcceptorServer = n.advertiseService("add_acceptor", gsoc::param_server::addAcceptor_callback);
      removeAcceptorServer = n.advertiseService("remove_acceptor", gsoc::param_server::removeAcceptor_callback);
    }

  } // param_server

} // gsoc