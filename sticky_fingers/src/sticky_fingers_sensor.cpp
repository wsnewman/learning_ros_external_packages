#include <gazebo/sensors/sensors.hh>
#include <gazebo_ros_control/gazebo_ros_control_plugin.h>
#include <ros/ros.h>
#include <tf/tf.h>

//Auto-generated ROS service header
#include <sticky_fingers/StickyControl.h>

namespace gazebo{
	class StickyFingers : public gazebo::SensorPlugin{
		private:

			//State information
			bool sticky;
			physics::LinkPtr held_object;

			tf::Transform ho_transform;

			double max_mass;

			//A stack of objects relating to the sticky element-
			//	the world containing it, the link containing it,
			//	and finally the sensor object itself.
			sensors::ContactSensorPtr finger_sensor;
			physics::LinkPtr finger_link;
			physics::WorldPtr finger_world;
			
			//The joint that will connect to the held object
			physics::JointPtr fixedJoint;

			event::ConnectionPtr updateConnection;

			//ROS communication
			ros::NodeHandle nh;
			ros::ServiceServer service;
			bool ControlCallback(
				sticky_fingers::StickyControlRequest& request,
				sticky_fingers::StickyControlResponse& response
			){
				if(this->sticky && !request.sticky_status){//We are sticky and should stop being such.
					this->sticky = false;//Stop being sticky.
					if(this->held_object != NULL){
						this->held_object->SetCollideMode("all");
					}
					this->finger_link->SetCollideMode("all");//Resume collisionality
					this->fixedJoint->Detach();
					this->held_object = NULL;//Drop our held object (if any)
					response.new_status = false;//Report what we just did.
					return true;
				}
				else if(!this->sticky && request.sticky_status){//We are not sticky and should be sticky...
					this->sticky = true;
					response.new_status = true;
					return true;
				}
				//Otherwise, we really don't have anything in particular to DO...
				response.new_status = sticky;
				return true;
			}

		public:
			void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf){
				this->sticky = false;
				this->held_object = NULL;

				this->max_mass = sdf->GetElement("capacity")->Get<double>();
				
				//Get things that require effort to look up and can be kept persistant.
				this->finger_sensor = boost::dynamic_pointer_cast<sensors::ContactSensor>(sensor);
				this->finger_world = physics::get_world(finger_sensor->GetWorldName());
				this->finger_link = boost::dynamic_pointer_cast<physics::Link>(
					this->finger_world->GetEntity(finger_sensor->GetParentName())
				);
				
				//Initialize the joint.
				//We use a prismatic joint that will have limits of (0,0) because fixed joints are not supported in this version of Gazebo
				this->fixedJoint = this->finger_world->GetPhysicsEngine()->CreateJoint("prismatic", this->finger_link->GetModel());
				this->fixedJoint->SetName(this->finger_link->GetModel()->GetName() + "__sticking_joint__");

				//Set up ROS communication
				std::string fingername = finger_sensor->GetName();
				int a = 0;//No, it will NOT just accept an argument size of 0 without shenanigans. Annoying.
				ros::init(a, (char **) NULL, fingername+"_node");
				service = this->nh.advertiseService(
					"sticky_finger/" + fingername,
					&StickyFingers::ControlCallback,
					this
				);
				ROS_INFO(
					"Sticky finger node %s listening on topic [%s].",
					(fingername+"_node").c_str(),
					("sticky_finger/" + fingername).c_str()
				);

				//Activate the sensor.
				this->updateConnection = event::Events::ConnectWorldUpdateBegin(
					boost::bind(&StickyFingers::OnUpdate, this, _1)
				);
				this->finger_sensor->SetActive(true);
			}

			void OnUpdate(const common::UpdateInfo & info){
				if(this->sticky){//We don't really need to do anything at all unless we're in sticky mode...
					
					if(this->held_object == NULL){//Prospecting mode:

						msgs::Contacts allcon = this->finger_sensor->GetContacts();

						for(unsigned int i = 0; i < allcon.contact_size(); i++){

							//There seems to be no rhyme or reason as to which name Gazebo will register in the first slot.
							std::string cname;
							if(finger_sensor->GetCollisionName(0).compare(allcon.contact(i).collision1()) != 0){
								cname = allcon.contact(i).collision1();
							}
							else{
								cname = allcon.contact(i).collision2();
							}

							physics::LinkPtr candidate =
								boost::dynamic_pointer_cast<physics::Collision>(
									finger_world->GetEntity(cname)
								)
							->GetLink();

							if(!(candidate->GetModel()->IsStatic())){//Ignore static objects
								if(candidate->GetInertial()->GetMass() <= this->max_mass){//Ignore heavy objects
									ROS_INFO("Finger grabbing link %s.", candidate->GetName().c_str());

									this->held_object = candidate;
									
									this->finger_link->SetCollideMode("ghost");
									this->held_object->SetCollideMode("ghost");
									
									//Attach the joint
									this->fixedJoint->Load(this->finger_link, held_object, math::Pose());
									//The joint limits have to be set after attachment:
									// http://answers.gazebosim.org/question/2824/error-when-setting-dynamically-created-joints-axis-in-gazebo-180/
									this->fixedJoint->SetAxis(0, gazebo::math::Vector3(0.0, 0.0, 1.0));
									this->fixedJoint->SetLowStop(0, gazebo::math::Angle(0.0));
									this->fixedJoint->SetHighStop(0, gazebo::math::Angle(0.0));
									this->fixedJoint->Init();

									break;
								}
							}
						}
					}
				}
			}
	};

	GZ_REGISTER_SENSOR_PLUGIN(StickyFingers)
}
