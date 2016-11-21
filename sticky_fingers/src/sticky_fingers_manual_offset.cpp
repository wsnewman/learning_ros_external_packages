#include <gazebo/sensors/sensors.hh>
#include <gazebo_ros_control/gazebo_ros_control_plugin.h>
#include <ros/ros.h>
#include <tf/tf.h>
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
					held_object = NULL;//Drop our held object (if any)
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
									ROS_INFO("Finger rabbing link %s.", candidate->GetName().c_str());

									this->held_object = candidate;

									math::Pose finger_pose = finger_link->GetWorldCoGPose();
									math::Pose object_pose = held_object->GetWorldCoGPose();
									
									tf::Transform origin_finger_trans = tf::Transform(
										tf::Quaternion(finger_pose.rot.x, finger_pose.rot.y, finger_pose.rot.z, finger_pose.rot.w).normalize(),
										tf::Vector3(finger_pose.pos.x, finger_pose.pos.y, finger_pose.pos.z)
									);
									tf::Transform origin_obj_trans = tf::Transform(
										tf::Quaternion(object_pose.rot.x, object_pose.rot.y, object_pose.rot.z, object_pose.rot.w).normalize(),
										tf::Vector3(object_pose.pos.x, object_pose.pos.y, object_pose.pos.z)
									);
									
									ho_transform = origin_finger_trans.inverse() * origin_obj_trans;
									
									/*ROS_WARN("Offset is (%f, %f, %f)(%f, %f, %f, %f)",
										ho_transform.getOrigin().x(),
										ho_transform.getOrigin().y(),
										ho_transform.getOrigin().z(),
										
										ho_transform.getRotation().w(),
										ho_transform.getRotation().x(),
										ho_transform.getRotation().y(),
										ho_transform.getRotation().z()
									);*/

									this->finger_link->SetCollideMode("ghost");
									this->held_object->SetCollideMode("ghost");
									
									ROS_WARN("Finger pose is (%f, %f, %f)(%f, %f, %f, %f), Object pose is (%f, %f, %f)(%f, %f, %f, %f)",
										finger_pose.pos.x, finger_pose.pos.y, finger_pose.pos.z,
										finger_pose.rot.w, finger_pose.rot.x, finger_pose.rot.y, finger_pose.rot.z,
										
										object_pose.pos.x, object_pose.pos.y, object_pose.pos.z,
										object_pose.rot.w, object_pose.rot.x, object_pose.rot.y, object_pose.rot.z
									);
									
									tf::Transform object_transform = origin_finger_trans * ho_transform;
									tf::Vector3 oot_vec = object_transform.getOrigin();
									tf::Quaternion oot_qat = object_transform.getRotation().normalize();
									
									math::Pose origin_final_pose = math::Pose(
										math::Vector3(oot_vec.x(), oot_vec.y(), oot_vec.z()),
										math::Quaternion(oot_qat.w(), oot_qat.x(), oot_qat.y(), oot_qat.z())
									);
									this->held_object->SetWorldPose(origin_final_pose,true,true);
									this->held_object->SetWorldTwist(
										finger_link->GetWorldLinearVel(),
										math::Vector3(0.0, 0.0, 0.0)
									);
									
									/*ROS_WARN("Finger pose is (%f, %f, %f)(%f, %f, %f, %f), Object pose is (%f, %f, %f)(%f, %f, %f, %f)",
										finger_link->GetWorldCoGPose().pos.x,
										finger_link->GetWorldCoGPose().pos.y,
										finger_link->GetWorldCoGPose().pos.z,
										
										finger_link->GetWorldCoGPose().rot.w,
										finger_link->GetWorldCoGPose().rot.x,
										finger_link->GetWorldCoGPose().rot.y,
										finger_link->GetWorldCoGPose().rot.z,
										
										
										held_object->GetWorldCoGPose().pos.x,
										held_object->GetWorldCoGPose().pos.y,
										held_object->GetWorldCoGPose().pos.z,
										
										held_object->GetWorldCoGPose().rot.w,
										held_object->GetWorldCoGPose().rot.x,
										held_object->GetWorldCoGPose().rot.y,
										held_object->GetWorldCoGPose().rot.z
									);*/

									break;
								}
							}
						}
					}

					else{//Carrying mode
					
						math::Pose finger_pose = finger_link->GetWorldCoGPose();
					
						tf::Transform origin_finger_trans = tf::Transform(
							tf::Quaternion(finger_pose.rot.x, finger_pose.rot.y, finger_pose.rot.z, finger_pose.rot.w).normalize(),
							tf::Vector3(finger_pose.pos.x, finger_pose.pos.y, finger_pose.pos.z)
						);
					
						tf::Transform object_transform = origin_finger_trans * ho_transform;
						tf::Vector3 oot_vec = object_transform.getOrigin();
						tf::Quaternion oot_qat = object_transform.getRotation().normalize();
									
						math::Pose origin_final_pose = math::Pose(
							math::Vector3(oot_vec.x(), oot_vec.y(), oot_vec.z()),
							math::Quaternion(oot_qat.w(), oot_qat.x(), oot_qat.y(), oot_qat.z())
						);
						this->held_object->SetWorldPose(origin_final_pose,true,true);
						this->held_object->SetWorldTwist(
						
							finger_link->GetWorldLinearVel(),
							math::Vector3(0.0, 0.0, 0.0)
						);
						
						/*ROS_WARN("Finger pose is (%f, %f, %f)(%f, %f, %f, %f), Object pose is (%f, %f, %f)(%f, %f, %f, %f)",
							finger_pose.pos.x,
							finger_pose.pos.y,
							finger_pose.pos.z,
										
							finger_pose.rot.w,
							finger_pose.rot.x,
							finger_pose.rot.y,
							finger_pose.rot.z,
								
										
							held_object->GetWorldCoGPose().pos.x,
							held_object->GetWorldCoGPose().pos.y,
							held_object->GetWorldCoGPose().pos.z,
										
							held_object->GetWorldCoGPose().rot.w,
							held_object->GetWorldCoGPose().rot.x,
							held_object->GetWorldCoGPose().rot.y,
							held_object->GetWorldCoGPose().rot.z
						);*/
					}
				}
			}
	};

	GZ_REGISTER_SENSOR_PLUGIN(StickyFingers)
}
