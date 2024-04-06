// Copyright 2008 Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage, Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/** \author Wim Meeussen */
// stew modified.

#pragma once

#include <array>
#include <string>
#include <type_traits>

#include "kdl/frames.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "tf2/convert.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer_interface.h"

namespace nhk24_2nd_ws::r2::geometry_msgs_convertor::impl
{
	template<class Out, class Message>
	#ifdef __cpp_concepts
	requires rosidl_generator_traits::is_message<Message>::value
	#endif
	struct MsgConvertor_ final {
		static_assert([]{return false;}(), "MsgConvertor_ is not defined for this pair of types.");
	};

	template<class Out, class Message>
	#ifdef __cpp_concepts
	#ifdef __cpp_lib_concepts
	requires rosidl_generator_traits::is_message<Message>::value
		&& requires {
			{MsgConvertor_<Out, Message>::toMsg(std::declval<Out>())} -> std::same_as<Message>;
			{MsgConvertor_<Out, Message>::fromMsg(std::declval<Message>())} -> std::same_as<Out>;
		}
	#endif
	#endif
	struct MsgConvertor final {
		static auto toMsg(const Out& in) -> Message {
			return MsgConvertor_<Out, Message>::toMsg(in);
		}

		static auto fromMsg(const Message& msg) -> Out {
			return MsgConvertor_<Out, Message>::fromMsg(msg);
		}
	};


	/*****************/
	/** Identitical **/
	/*****************/
	template<class M>
	#ifdef __cpp_concepts
	requires rosidl_generator_traits::is_message<M>::value
	#endif
	struct MsgConvertor_<M, M> final {
		static auto toMsg(const M& in) -> M {
			return in;
		}

		static auto fromMsg(const M& msg) -> M {
			return msg;
		}
	};

	/***************/
	/** KDL Frame **/
	/***************/
	template<>
	struct MsgConvertor_<KDL::Frame, geometry_msgs::msg::TransformStamped> final {
		static auto toMsg(const KDL::Frame& in) -> geometry_msgs::msg::TransformStamped {
			geometry_msgs::msg::TransformStamped out;
			out.transform.translation.x = in.p[0];
			out.transform.translation.y = in.p[1];
			out.transform.translation.z = in.p[2];
			in.M.GetQuaternion (
				out.transform.rotation.x
				, out.transform.rotation.y
				, out.transform.rotation.z
				, out.transform.rotation.w
			);
			return out;
		}

		static auto fromMsg(const geometry_msgs::msg::TransformStamped& msg) -> KDL::Frame {
			return KDL::Frame (
				KDL::Rotation::Quaternion (
					msg.transform.rotation.x
					, msg.transform.rotation.y
					, msg.transform.rotation.z
					, msg.transform.rotation.w
				),
				KDL::Vector(
					msg.transform.translation.x
					, msg.transform.translation.y
					, msg.transform.translation.z
				)
			);
		}
	};

	/****************/
	/** [T]Stamped **/
	/****************/
	#define MYTF2_DEFINE_STAMPED_CONVERTOR(T, M, data_name)\
	template<>\
	struct MsgConvertor_<tf2::Stamped<T>, M> final {\
		static auto toMsg(const tf2::Stamped<T>& in) -> M {\
			M out;\
			out.header.stamp = tf2_ros::toMsg(in.stamp_);\
			out.header.frame_id = in.frame_id_;\
			out.data_name = MsgConvertor<T, decltype(M::data_name)>::toMsg(in);\
			return out;\
		}\
\
		static auto fromMsg(const M& msg) -> tf2::Stamped<T> {\
			tf2::Stamped<T> out;\
			out.stamp_ = tf2_ros::fromMsg(msg.header.stamp);\
			out.frame_id_ = msg.header.frame_id;\
			out.setData(MsgConvertor<T, decltype(M::data_name)>::fromMsg(msg.data_name));\
			return out;\
		}\
	};

	

	/*************/
	/** Vector3 **/
	/*************/
	template<>
	struct MsgConvertor_<tf2::Vector3, geometry_msgs::msg::Vector3> final {
		static auto toMsg(const tf2::Vector3& in) -> geometry_msgs::msg::Vector3 {
			geometry_msgs::msg::Vector3 out;
			out.x = in.getX();
			out.y = in.getY();
			out.z = in.getZ();
			return out;
		}

		static auto fromMsg(const geometry_msgs::msg::Vector3& msg) -> tf2::Vector3 {
			return tf2::Vector3(msg.x, msg.y, msg.z);
		}
	};

	MYTF2_DEFINE_STAMPED_CONVERTOR(tf2::Vector3, geometry_msgs::msg::Vector3Stamped, vector);


	/*************/
	/** Point32 **/
	/*************/
	template<>
	struct MsgConvertor_<tf2::Vector3, geometry_msgs::msg::Point32> final {
		static auto toMsg(const tf2::Vector3& in) -> geometry_msgs::msg::Point32 {
			geometry_msgs::msg::Point32 out;
			out.x = in.getX();
			out.y = in.getY();
			out.z = in.getZ();
			return out;
		}

		static auto fromMsg(const geometry_msgs::msg::Point32& msg) -> tf2::Vector3 {
			return tf2::Vector3(msg.x, msg.y, msg.z);
		}
	};


	/***********/
	/** Point **/
	/***********/
	template<>
	struct MsgConvertor_<tf2::Vector3, geometry_msgs::msg::Point> final {
		static auto toMsg(const tf2::Vector3& in) -> geometry_msgs::msg::Point {
			geometry_msgs::msg::Point out;
			out.x = in.getX();
			out.y = in.getY();
			out.z = in.getZ();
			return out;
		}

		static auto fromMsg(const geometry_msgs::msg::Point& msg) -> tf2::Vector3 {
			return tf2::Vector3(msg.x, msg.y, msg.z);
		}
	};

	MYTF2_DEFINE_STAMPED_CONVERTOR(tf2::Vector3, geometry_msgs::msg::PointStamped, point);


	/****************/
	/** Quaternion **/
	/****************/
	template<>
	struct MsgConvertor_<tf2::Quaternion, geometry_msgs::msg::Quaternion> final {
		static auto toMsg(const tf2::Quaternion& in) -> geometry_msgs::msg::Quaternion {
			geometry_msgs::msg::Quaternion out;
			out.x = in.getX();
			out.y = in.getY();
			out.z = in.getZ();
			out.w = in.getW();
			return out;
		}

		static auto fromMsg(const geometry_msgs::msg::Quaternion& msg) -> tf2::Quaternion {
			return tf2::Quaternion(msg.x, msg.y, msg.z, msg.w);
		}
	};

	MYTF2_DEFINE_STAMPED_CONVERTOR(tf2::Quaternion, geometry_msgs::msg::QuaternionStamped, quaternion);

	/***************/
	/** Transform **/
	/***************/
	template<>
	struct MsgConvertor_<tf2::Transform, geometry_msgs::msg::Transform> final {
		static auto toMsg(const tf2::Transform& in) -> geometry_msgs::msg::Transform {
			geometry_msgs::msg::Transform out;
			out.translation.x = in.getOrigin().getX();
			out.translation.y = in.getOrigin().getY();
			out.translation.z = in.getOrigin().getZ();
			out.rotation = MsgConvertor<tf2::Quaternion, geometry_msgs::msg::Quaternion>::toMsg(in.getRotation());
			return out;
		}

		static auto fromMsg(const geometry_msgs::msg::Transform& msg) -> tf2::Transform {
			return tf2::Transform {
				tf2::Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w)
				, tf2::Vector3(msg.translation.x, msg.translation.y, msg.translation.z)
			};
		}
	};

	MYTF2_DEFINE_STAMPED_CONVERTOR(tf2::Transform, geometry_msgs::msg::TransformStamped, transform);


	/**********/
	/** Pose **/
	/**********/
	template<>
	struct MsgConvertor_<tf2::Transform, geometry_msgs::msg::Pose> final {
		static auto toMsg(const tf2::Transform& in) -> geometry_msgs::msg::Pose {
			geometry_msgs::msg::Pose out;
			out.position.x = in.getOrigin().getX();
			out.position.y = in.getOrigin().getY();
			out.position.z = in.getOrigin().getZ();
			out.orientation = MsgConvertor<tf2::Quaternion, geometry_msgs::msg::Quaternion>::toMsg(in.getRotation());
			return out;
		}

		static auto fromMsg(const geometry_msgs::msg::Pose& msg) -> tf2::Transform {
			return tf2::Transform {
				tf2::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
				, tf2::Vector3(msg.position.x, msg.position.y, msg.position.z)
			};
		}
	};

	MYTF2_DEFINE_STAMPED_CONVERTOR(tf2::Transform, geometry_msgs::msg::PoseStamped, pose);


	/*******************************/
	/** PoseWithCovarianceStamped **/
	/*******************************/
	template<>
	struct MsgConvertor_<tf2::WithCovarianceStamped<tf2::Transform>, geometry_msgs::msg::PoseWithCovarianceStamped> final {
		static auto toMsg(const tf2::WithCovarianceStamped<tf2::Transform>& in) -> geometry_msgs::msg::PoseWithCovarianceStamped {
			geometry_msgs::msg::PoseWithCovarianceStamped out;
			out.header.stamp = tf2_ros::toMsg(in.stamp_);
			out.header.frame_id = in.frame_id_;
			out.pose.covariance = tf2::covarianceNestedToRowMajor(in.cov_mat_);
			out.pose.pose.orientation.x = in.getRotation().getX();
			out.pose.pose.orientation.y = in.getRotation().getY();
			out.pose.pose.orientation.z = in.getRotation().getZ();
			out.pose.pose.orientation.w = in.getRotation().getW();
			out.pose.pose.position.x = in.getOrigin().getX();
			out.pose.pose.position.y = in.getOrigin().getY();
			out.pose.pose.position.z = in.getOrigin().getZ();
			return out;
		}

		static auto fromMsg(const geometry_msgs::msg::PoseWithCovarianceStamped &in) -> tf2::WithCovarianceStamped<tf2::Transform> {
			tf2::WithCovarianceStamped<tf2::Transform> out;
			out.stamp_ = tf2_ros::fromMsg(in.header.stamp);
			out.frame_id_ = in.header.frame_id;
			out.cov_mat_ = tf2::covarianceRowMajorToNested(in.pose.covariance);
			tf2::Transform tmp = MsgConvertor<tf2::Transform, geometry_msgs::msg::Pose>::fromMsg(in.pose.pose);
			out.setData(tmp);
			return out;
		}
	};
}

namespace nhk24_2nd_ws::r2::geometry_msgs_convertor {
	using impl::MsgConvertor;
}


