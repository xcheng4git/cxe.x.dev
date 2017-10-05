#pragma once
#include "KeyframeSLAM.h"

#include <DenseVO\RgbdImagePyramid.h>
#include <DenseVO\DenseTracker.h>

#include "Datatypes.h"

#include "LocalMap.h"

#include <boost/scoped_ptr.hpp>
#include <boost/signals2.hpp>


namespace slam
{
	namespace internal
	{
		struct LocalTrackerImpl;
	} /* namespace internal */

	class LocalTracker
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		typedef vo::DenseTracker::Result TrackingResult;

		struct All
		{
			typedef bool result_type;

			template<typename InputIterator>
			bool operator()(InputIterator first, InputIterator last) const
			{
				int i = 0;
				bool result = true;
				//std::cerr << "votes: ";
				for (; first != last; ++first)
				{
					bool tmp = *first;
					result = result && tmp;

					//std::cerr << tmp << " ";
					i++;
				}

				//std::cerr << std::endl;

				return result;
			}
		};

		typedef boost::signals2::signal<bool(const slam::LocalTracker&, const slam::LocalTracker::TrackingResult&, const slam::LocalTracker::TrackingResult&), slam::LocalTracker::All> AcceptSignal;
		typedef AcceptSignal::slot_type AcceptCallback;
		typedef boost::signals2::signal<void(const slam::LocalTracker&, const slam::LocalMap::Ptr&, const slam::LocalTracker::TrackingResult&)> MapInitializedSignal;
		typedef MapInitializedSignal::slot_type MapInitializedCallback;
		typedef boost::signals2::signal<void(const slam::LocalTracker&, const slam::LocalMap::Ptr&)> MapCompleteSignal;
		typedef MapCompleteSignal::slot_type MapCompleteCallback;

		LocalTracker();
		virtual ~LocalTracker();

		slam::LocalMap::Ptr getLocalMap() const;

		void getCurrentPose(AffineTransformd& pose);

		void initNewLocalMap(const vo::RgbdImagePyramid::Ptr& keyframe, const vo::RgbdImagePyramid::Ptr& frame, const AffineTransformd& keyframe_pose = AffineTransformd::Identity());

		const vo::DenseTracker::Config& configuration() const;

		void configure(const vo::DenseTracker::Config& config);

		void update(const vo::RgbdImagePyramid::Ptr& image, AffineTransformd& pose);

		void forceCompleteCurrentLocalMap();

		boost::signals2::connection addAcceptCallback(const AcceptCallback& callback);
		boost::signals2::connection addMapInitializedCallback(const MapInitializedCallback& callback);
		boost::signals2::connection addMapCompleteCallback(const MapCompleteCallback& callback);
	private:
		boost::scoped_ptr<internal::LocalTrackerImpl> impl_;
		slam::LocalMap::Ptr local_map_;

		void initNewLocalMap(const vo::RgbdImagePyramid::Ptr& keyframe, const vo::RgbdImagePyramid::Ptr& frame, TrackingResult& r_odometry, const AffineTransformd& keyframe_pose);

	};
}

