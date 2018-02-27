#include <tf/tf.h>
#include <yocs_msgs/Trajectory.h>
#include <yocs_msgs/Waypoint.h>
#include "yocs_waypoint_provider/yaml_parser.hpp"

#include "json/json.h"

namespace yocs
{
  bool loadWaypointsAndTrajectoriesFromYaml(const std::string& filename,
                                            yocs_msgs::WaypointList& wps,
                                            yocs_msgs::TrajectoryList& trajs)
  {
    wps.waypoints.clear();
    trajs.trajectories.clear();

    // Yaml File Parsing
    try
    {
      YAML::Node doc;

      getYamlNode(filename, doc);
      parseWaypoints(doc, wps);
      parseTrajectories(doc, wps, trajs);
    }
    catch(YAML::ParserException& e)
    {
      ROS_ERROR("Parsing waypoints file failed: %s", e.what());
      return false;
    }
    catch(YAML::RepresentationException& e)
    {
      ROS_ERROR("Parsing waypoints file failed: %s", e.what());
      return false;
    }
    catch(std::string& e) {
      ROS_ERROR("Parsing waypoints file failed: %s",e.c_str());
      return false;
    }
    return true;
  }

  void getYamlNode(const std::string& filename, YAML::Node& node)
  {
    std::ifstream ifs(filename.c_str(), std::ifstream::in);
    if (ifs.good() == false)
    {
      throw std::string("Waypoints file not found");
    }

    #ifdef HAVE_NEW_YAMLCPP
      node = YAML::Load(ifs);
    #else
      YAML::Parser parser(ifs);
      parser.GetNextDocument(node);
    #endif
  }

  void parseWaypoints(const YAML::Node& node, yocs_msgs::WaypointList& wps)
  {
    #ifdef HAVE_NEW_YAMLCPP
      const YAML::Node& wp_node_tmp = node["waypoints"];
      const YAML::Node* wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
    #else
      const YAML::Node* wp_node = node.FindValue("waypoints");
    #endif

    if(wp_node != NULL)
    {
      int iter = 0;
      for(int i = 0; i < wp_node->size(); ++i)
      {
        // Parse waypoint entries on YAML
        yocs_msgs::Waypoint wp;
        (*wp_node)[i]["name"] >> wp.name;
        if ((*wp_node)[i]["pose"]) {
          (*wp_node)[i]["pose"]["position"]["x"] >> wp.pose.position.x;
          (*wp_node)[i]["pose"]["position"]["y"] >> wp.pose.position.y;
          (*wp_node)[i]["pose"]["position"]["z"] >> wp.pose.position.z;
          (*wp_node)[i]["pose"]["orientation"]["x"] >> wp.pose.orientation.x;
          (*wp_node)[i]["pose"]["orientation"]["y"] >> wp.pose.orientation.y;
          (*wp_node)[i]["pose"]["orientation"]["z"] >> wp.pose.orientation.z;
          (*wp_node)[i]["pose"]["orientation"]["w"] >> wp.pose.orientation.w;
        }

        Json::FastWriter json_writer;
        Json::Value json_value;
        for(YAML::const_iterator it = (*wp_node)[i].begin(); it != (*wp_node)[i].end(); ++it) {
          if ("name" != it->first.as<std::string>() && "pose" != it->first.as<std::string>() && "frame_id" != it->first.as<std::string>()) {
            json_value[it->first.as<std::string>()] = it->second.as<std::string>();
          }
        }
        if (!(*wp_node)[i]["type"] && (*wp_node)[i]["frame_id"]) {
          if ("map" == (*wp_node)[i]["frame_id"].as<std::string>()) {
            json_value["type"] = "goal";
            json_value["frame_id"] = (*wp_node)[i]["frame_id"].as<std::string>();
          } else {
            json_value["type"] = (*wp_node)[i]["frame_id"].as<std::string>();
            if ("initial_pose" == (*wp_node)[i]["frame_id"].as<std::string>()) {
              json_value["frame_id"] = "map";
            }
          }
        } else if ((*wp_node)[i]["frame_id"]) {
          json_value["frame_id"] = (*wp_node)[i]["frame_id"].as<std::string>();
        }
        if ((*wp_node)[i]["iter"]) {
          if (0 != iter) {
            json_value["iter"] = "0";
            double yaw = tf::getYaw(wp.pose.orientation);
            double cos_yaw = cos(yaw);
            double sin_yaw = sin(yaw);
          
            if ((*wp_node)[i]["iter_x"]) {
              double iter_x = iter * (*wp_node)[i]["iter_x"].as<double>();
              wp.pose.position.x += + iter_x * cos_yaw;
              wp.pose.position.y += + iter_x * sin_yaw;
            }
            if ((*wp_node)[i]["iter_y"]) {
              double iter_y = iter * (*wp_node)[i]["iter_y"].as<double>();
              wp.pose.position.x += - iter_y * sin_yaw;
              wp.pose.position.y += + iter_y * cos_yaw;
            }
            if ((*wp_node)[i]["iter_span"]) { //legacy
              double iter_y = iter * (*wp_node)[i]["iter_span"].as<double>();
              wp.pose.position.x += - iter_y * sin_yaw;
              wp.pose.position.y += + iter_y * cos_yaw;
            }
          }
          iter++;
          std::string str;
          std::stringstream sstr;
          sstr << iter;
          sstr >> str;
          wp.name = wp.name + "_" + str;
          if ((*wp_node)[i]["iter"].as<int>() == iter) {
            iter = 0;
          } else {
            i--;
          }
        }

        wp.header.frame_id = json_writer.write(json_value);

        wps.waypoints.push_back(wp);
      }
      ROS_INFO_STREAM("Parsed " << wps.waypoints.size() << " waypoints.");
    }
    else
    {
      ROS_WARN_STREAM("Couldn't find any waypoints in the provided yaml file.");
    }
  }

  void parseTrajectories(const YAML::Node& node,
                         const yocs_msgs::WaypointList& wps,
                         yocs_msgs::TrajectoryList& trajs)
  {
    unsigned int i;

    #ifdef HAVE_NEW_YAMLCPP
    const YAML::Node& wp_node_tmp = node["trajectories"];
    const YAML::Node* wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
    #else
    const YAML::Node* wp_node = node.FindValue("trajectories");
    #endif
    if(wp_node != NULL)
    {
      for(i = 0; i < wp_node->size(); ++i)
      {
        // Parse trajectory entries on YAML
        yocs_msgs::Trajectory traj;

        // check if all specified waypoints are configured
        bool all_waypoints_found = true;

        for(unsigned int wp = 0; wp < (*wp_node)[i]["waypoints"].size(); ++wp)
        {
          bool wp_found = false;
          std::string wp_name;
          (*wp_node)[i]["waypoints"][wp] >> wp_name;
          for(unsigned int known_wp = 0; known_wp < wps.waypoints.size(); ++known_wp)
          {
            if (wp_name == wps.waypoints[known_wp].name)
            {
              traj.waypoints.push_back(wps.waypoints[known_wp]);
              wp_found = true;
              break;
            }
          }
          if (!wp_found)
          {
            all_waypoints_found = false;
            break;
          }
        }
        if (all_waypoints_found)
        {
          (*wp_node)[i]["name"] >> traj.name;
  
          Json::FastWriter json_writer;
          Json::Value json_value;
          for(YAML::const_iterator it = (*wp_node)[i].begin(); it != (*wp_node)[i].end(); ++it) {
            if ("name" != it->first.as<std::string>() && "waypoints" != it->first.as<std::string>()) {
              json_value[it->first.as<std::string>()] = it->second.as<std::string>();
            }
          }
          traj.header.frame_id = json_writer.write(json_value);
  
          trajs.trajectories.push_back(traj);
        }
      }
      ROS_INFO_STREAM("Parsed " << trajs.trajectories.size() << " trajectories.");
    }
    else
    {
      ROS_WARN_STREAM("Couldn't find any trajectories in the provided yaml file.");
    }
  }

  bool saveWaypointsAndTrajectoriesToYaml(const std::string& filename,
                                            yocs_msgs::WaypointList& wps,
                                            yocs_msgs::TrajectoryList& trajs)
  {
      // Yaml File Parsing
    try
    {
        //      YAML::Node doc;

        //      setYamlNode(filename, doc);
        //      dumpWaypoints(filename, wps);
        //      dumpTrajectories(doc, wps, trajs);

        std::ofstream ofs(filename.c_str(), std::ofstream::out);

        ofs << "waypoints:" << std::endl;
        for(unsigned int i = 0; i < wps.waypoints.size(); ++i)
        {
          // Dump waypoint entries on YAML
          Json::Reader json_reader;
          Json::Value json_value;
          if (!json_reader.parse(wps.waypoints[i].header.frame_id, json_value, false))
          {
            return -1; //TODO: report error
          }

          std::string str = wps.waypoints[i].name;
          if (json_value.isMember("iter")) {
            if ("0" == json_value["iter"].asString()) {
              continue;
            } else {
              str = str.substr(0, str.rfind("_"));
            }
          }
          ofs << "  - name: "             << str << std::endl;

          Json::Value::Members json_member = json_value.getMemberNames();
          for (Json::Value::Members::iterator it = json_member.begin(); it != json_member.end(); it++)
          {
            ofs << "    " + (*it) + ": "    << json_value[(*it)].asString() << std::endl;
          }              

          if (wps.waypoints[i].pose.position.x || wps.waypoints[i].pose.position.y || wps.waypoints[i].pose.position.z || wps.waypoints[i].pose.orientation.x || wps.waypoints[i].pose.orientation.y || wps.waypoints[i].pose.orientation.z || wps.waypoints[i].pose.orientation.w) {
            ofs << "    pose:"              << std::endl;
            ofs << "      position:"        << std::endl;
            ofs << "        x: "            << wps.waypoints[i].pose.position.x << std::endl;
            ofs << "        y: "            << wps.waypoints[i].pose.position.y << std::endl;
            ofs << "        z: "            << wps.waypoints[i].pose.position.z << std::endl;
            ofs << "      orientation:"     << std::endl;
            ofs << "        x: "            << wps.waypoints[i].pose.orientation.x << std::endl;
            ofs << "        y: "            << wps.waypoints[i].pose.orientation.y << std::endl;
            ofs << "        z: "            << wps.waypoints[i].pose.orientation.z << std::endl;
            ofs << "        w: "            << wps.waypoints[i].pose.orientation.w << std::endl;
          }
        }
        ROS_INFO_STREAM("Dumped " << wps.waypoints.size() << " waypoints.");

        ofs << "trajectories:" << std::endl;
        for(unsigned int i = 0; i < trajs.trajectories.size(); ++i)
        {
            // Dump waypoint entries on YAML
            ofs << "  - name: "             << trajs.trajectories[i].name << std::endl;

            if ("" != trajs.trajectories[i].header.frame_id) {
              Json::Reader json_reader;
              Json::Value json_value;
              if (!json_reader.parse(trajs.trajectories[i].header.frame_id, json_value, false))
              {
                return -1; //TODO: report error
              }
              Json::Value::Members json_member = json_value.getMemberNames();
              for (Json::Value::Members::iterator it = json_member.begin(); it != json_member.end(); it++)
              {
                ofs << "    " + (*it) + ": "    << json_value[(*it)].asString() << std::endl;
              }
            }

            ofs << "    waypoints:"         << std::endl;
            for (unsigned int j = 0; j < trajs.trajectories[i].waypoints.size(); ++j)
            {
            ofs << "    - "                 << trajs.trajectories[i].waypoints[j].name << std::endl;
            }
        }
        ROS_INFO_STREAM("Dumped " << trajs.trajectories.size() << " trajectories.");

        ofs.close();
    }
    catch(YAML::ParserException& e)
    {
      ROS_ERROR("Dumping waypoints file failed: %s", e.what());
      return false;
    }
    catch(YAML::RepresentationException& e)
    {
      ROS_ERROR("Dumping waypoints file failed: %s", e.what());
      return false;
    }
    catch(std::string& e) {
      ROS_ERROR("Dumping waypoints file failed: %s",e.c_str());
      return false;
    }
    return true;
  }

  void setYamlNode(const std::string& filename, YAML::Node& node)
  {
      std::ofstream ofs(filename.c_str(), std::ofstream::out);
  }

  void dumpWaypoints(const std::string& filename, yocs_msgs::WaypointList& wps)
  {
      std::ofstream ofs(filename.c_str(), std::ofstream::out);

      ofs << "waypoints:" << std::endl;
      for(unsigned int i = 0; i < wps.waypoints.size(); ++i)
      {
          // Dump waypoint entries on YAML
          ofs << "  - name: "             << wps.waypoints[i].name << std::endl;
          ofs << "    frame_id: "         << wps.waypoints[i].header.frame_id << std::endl;
          ofs << "    close_enough: "     << wps.waypoints[i].close_enough << std::endl;
          ofs << "    goal_timeout: "     << wps.waypoints[i].goal_timeout << std::endl;
          ofs << "    failure_mode: "     << wps.waypoints[i].failure_mode << std::endl;
          ofs << "    pose:"              << std::endl;
          ofs << "      position:"        << std::endl;
          ofs << "        x: "            << wps.waypoints[i].pose.position.x << std::endl;
          ofs << "        y: "            << wps.waypoints[i].pose.position.y << std::endl;
          ofs << "        z: "            << wps.waypoints[i].pose.position.z << std::endl;
          ofs << "      orientation:"     << std::endl;
          ofs << "        x: "            << wps.waypoints[i].pose.orientation.x << std::endl;
          ofs << "        y: "            << wps.waypoints[i].pose.orientation.y << std::endl;
          ofs << "        z: "            << wps.waypoints[i].pose.orientation.z << std::endl;
          ofs << "        w: "            << wps.waypoints[i].pose.orientation.w << std::endl;
      }
      ofs.close();
      ROS_INFO_STREAM("Dumped " << wps.waypoints.size() << " waypoints.");
  }

  void dumpTrajectories(const YAML::Node& node,
                         const yocs_msgs::WaypointList& wps,
                         yocs_msgs::TrajectoryList& trajs)
  {
    unsigned int i;

    #ifdef HAVE_NEW_YAMLCPP
    const YAML::Node& wp_node_tmp = node["trajectories"];
    const YAML::Node* wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
    #else
    const YAML::Node* wp_node = node.FindValue("trajectories");
    #endif
    if(wp_node != NULL)
    {
      for(i = 0; i < wp_node->size(); ++i)
      {
        // Parse trajectory entries on YAML
        yocs_msgs::Trajectory traj;

        // check if all specified waypoints are configured
        bool all_waypoints_found = true;

        for(unsigned int wp = 0; wp < (*wp_node)[i]["waypoints"].size(); ++wp)
        {
          bool wp_found = false;
          std::string wp_name;
          (*wp_node)[i]["waypoints"][wp] >> wp_name;
          for(unsigned int known_wp = 0; known_wp < wps.waypoints.size(); ++known_wp)
          {
            if (wp_name == wps.waypoints[known_wp].name)
            {
              traj.waypoints.push_back(wps.waypoints[known_wp]);
              wp_found = true;
              break;
            }
          }
          if (!wp_found)
          {
            all_waypoints_found = false;
            break;
          }
        }
        if (all_waypoints_found)
        {
          (*wp_node)[i]["name"] >> traj.name;
          trajs.trajectories.push_back(traj);
        }
      }
      ROS_INFO_STREAM("Parsed " << trajs.trajectories.size() << " trajectories.");
    }
    else
    {
      ROS_WARN_STREAM("Couldn't find any trajectories in the provided yaml file.");
    }
  }
}
