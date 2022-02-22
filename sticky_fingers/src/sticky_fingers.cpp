#include <gazebo/gazebo_config.h>

#include <gazebo/sensors/sensors.hh>
#include <gazebo_ros_control/gazebo_ros_control_plugin.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <std_srvs/SetBool.h>

#if (GAZEBO_MAJOR_VERSION > 10)
#define NEW_GAZEBO_INTERFACE 0
#else
#define NEW_GAZEBO_INTERFACE 1
#endif

#define OLD_GAZEBO_UPDATE ((GAZEBO_MAJOR_VERSION == 9) || (GAZEBO_MINOR_VERSION >= 19))

#if OLD_GAZEBO_UPDATE
#include "ignition/math.hh"
#endif

namespace gazebo{
	class StickyFingers : public gazebo::ModelPlugin{
		private:

			//State information
			bool sticky;
			physics::LinkPtr held_object;

			double max_mass;

			//A stack of objects relating to the sticky element-
			//	the world containing it, the link containing it,
			//	and finally the sensor object itself.
			physics::ModelPtr finger_model;
			physics::LinkPtr finger_link;
			physics::WorldPtr finger_world;
			std::string finger_name;
			
			//The joint that will connect to the held object
			physics::JointPtr fixedJoint;
			
			
			//Something to do with contacts or something.
			transport::SubscriberPtr contact_sub;
			transport::NodePtr contact_node;
			//std::mutex mutex;
			
			event::ConnectionPtr updateConnection;
			
			void ContactCB(ConstContactsPtr &msg){
				if(this->sticky && this->held_object == NULL){
					for(int i = 0; i < msg->contact_size(); i++){
						physics::LinkPtr candidate = NULL;
						if(strcmp(msg->contact(i).collision1().c_str(), this->finger_name.c_str())){
							candidate =
								boost::dynamic_pointer_cast<physics::Collision>(
#if NEW_GAZEBO_INTERFACE
									this->finger_world->GetEntity(msg->contact(i).collision1())
#else
									this->finger_world->EntityByName(msg->contact(i).collision1())
#endif
								)
							->GetLink();

						}
						if(strcmp(msg->contact(i).collision2().c_str(), this->finger_name.c_str())){
							candidate =
								boost::dynamic_pointer_cast<physics::Collision>(
#if NEW_GAZEBO_INTERFACE
									this->finger_world->GetEntity(msg->contact(i).collision1())
#else
									this->finger_world->EntityByName(msg->contact(i).collision1())
#endif
								)
							->GetLink();
						}
						if(candidate != NULL){
							if(!candidate->IsStatic()){
#if NEW_GAZEBO_INTERFACE
								if(candidate->GetInertial()->GetMass() <= this->max_mass){//Ignore heavy objects
#else
								if(candidate->GetInertial()->Mass() <= this->max_mass){//Ignore heavy objects
#endif
									ROS_INFO("Finger grabbing link %s.", candidate->GetName().c_str());

									this->held_object = candidate;
									
									this->finger_link->SetCollideMode("ghost");
									this->held_object->SetCollideMode("ghost");
									
									//Attach the joint
#if NEW_GAZEBO_INTERFACE
									this->fixedJoint->Load(this->finger_link, held_object, math::Pose());
#else
									this->fixedJoint->Load(this->finger_link, held_object, ignition::math::Pose3d());
#endif
									//The joint limits have to be set after attachment:
									// http://answers.gazebosim.org/question/2824/error-when-setting-dynamically-created-joints-axis-in-gazebo-180/
#if NEW_GAZEBO_INTERFACE
									this->fixedJoint->SetAxis(0, gazebo::math::Vector3(0.0, 0.0, 1.0));
									this->fixedJoint->SetLowStop(0, gazebo::math::Angle(0.0));
									this->fixedJoint->SetHighStop(0, gazebo::math::Angle(0.0));
#else
#if OLD_GAZEBO_UPDATE
									this->fixedJoint->SetAxis(0, ignition::math::Vector3d(0.0, 0.0, 1.0));
#else
									this->fixedJoint->SetAxis(0, ignition::math::Vector3(0.0, 0.0, 1.0));
#endif
									this->fixedJoint->SetParam("lo_stop", 0, ignition::math::Angle(0.0));
									this->fixedJoint->SetParam("hi_stop", 0, ignition::math::Angle(0.0));
#endif
									this->fixedJoint->Init();

									break;
								}
							}
						}
					}
				}
			}

			//ROS communication
			ros::NodeHandle nh;
			ros::ServiceServer service;
			bool ControlCallback(
				std_srvs::SetBoolRequest& request,
				std_srvs::SetBoolResponse& response
			){
				if(this->sticky && !request.data){//We are sticky and should stop being such.
					this->sticky = false;//Stop being sticky.
					if(this->held_object != NULL){
						this->held_object->SetCollideMode("all");
					}
					this->finger_link->SetCollideMode("all");//Resume collisionality
					this->fixedJoint->Detach();
					this->held_object = NULL;//Drop our held object (if any)
					response.success = false;//Report what we just did.
					return true;
				}
				else if(!this->sticky && request.data){//We are not sticky and should be sticky...
					this->sticky = true;
					response.success = true;
					return true;
				}
				//Otherwise, we really don't have anything in particular to DO...
				response.success = sticky;
				return true;
			}

		public:
			void Load(physics::ModelPtr mod, sdf::ElementPtr sdf){
				printf("Loaded SF.");
				this->sticky = false;
				this->held_object = NULL;
				
				//Get things that require effort to look up and can be kept persistant.
				this->max_mass = sdf->GetElement("capacity")->Get<double>();
				this->finger_name = sdf->GetElement("link")->Get<std::string>();
				this->finger_model = mod;
				this->finger_world = finger_model->GetWorld();
				this->finger_link = boost::dynamic_pointer_cast<physics::Link>(
#if NEW_GAZEBO_INTERFACE
					this->finger_world->GetEntity(this->finger_name)
#else
					this->finger_world->EntityByName(this->finger_name)
#endif
				);
				
				//Initialize the joint.
				//We use a prismatic joint that will have limits of (0,0) because fixed joints are not natively supported in this version of Gazebo
#if NEW_GAZEBO_INTERFACE
				this->fixedJoint = this->finger_world->GetPhysicsEngine()->CreateJoint("prismatic", this->finger_model);
#else
				this->fixedJoint = this->finger_world->Physics()->CreateJoint("prismatic", this->finger_model);
#endif
				this->fixedJoint->SetName(this->finger_model->GetName() + "__sticking_joint__");
				
				//Pull out all possible collision objects from the link
				unsigned int ccount = this->finger_link->GetChildCount();
				std::map<std::string, physics::CollisionPtr> collisions;
				for(unsigned int i = 0; i < ccount; i++){
					 physics::CollisionPtr ccan = this->finger_link->GetCollision(i);
					 //What even is the point of this check? It SEEMS to look for the same collision appearing twice, but how would that even happen?
					 /*std::map<std::string, physics::CollisionPtr>::iterator collIter = this->dataPtr->collisions.find(collision->GetScopedName());
					 if (collIter != this->dataPtr->collisions.end()) continue;*/
					collisions[ccan->GetScopedName()] = ccan;
				}
				
				//Create a listener on those contacts
				this->contact_node = transport::NodePtr(new transport::Node());
#if NEW_GAZEBO_INTERFACE				
				this->contact_node->Init(this->finger_world->GetName());
#else
				this->contact_node->Init(this->finger_world->Name());
#endif
				if (!collisions.empty()){
					// Create a filter to receive collision information
#if NEW_GAZEBO_INTERFACE
					physics::ContactManager * mgr = this->finger_world->GetPhysicsEngine()->GetContactManager();
#else
					physics::ContactManager * mgr = this->finger_world->Physics()->GetContactManager();
#endif
					std::string topic = mgr->CreateFilter(finger_name, collisions);
					if (!this->contact_sub){
						this->contact_sub = this->contact_node->Subscribe(topic, &StickyFingers::ContactCB, this);
					}
				}
				
				//Set up ROS communication
				int a = 0;//No, it will NOT just accept an argument size of 0 without shenanigans. Annoying.
				ros::init(a, (char **) NULL, finger_name + "_node");
				service = this->nh.advertiseService(
					"sticky_finger/" + this->finger_link->GetName(),
					&StickyFingers::ControlCallback,
					this
				);
				ROS_INFO(
					"Sticky finger node %s listening on topic [%s].",
					(finger_name+"_node").c_str(),
					("sticky_finger/" + this->finger_link->GetName()).c_str()
				);
				
				this->updateConnection = event::Events::ConnectWorldUpdateEnd(boost::bind(&StickyFingers::OnUpdate, this));
			}

			void OnUpdate(){}
	};

	GZ_REGISTER_MODEL_PLUGIN(StickyFingers)
}
