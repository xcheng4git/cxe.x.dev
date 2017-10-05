#pragma once
#include <vector>
#include <boost/shared_ptr.hpp>

#include "Keyframe.h"

namespace slam
{
	namespace constraints
	{
		struct ConstraintProposal
		{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			struct Vote
			{
				enum Enum
				{
					Accept,
					Reject
				};

				// hard decision
				Enum Decision;

				// score to select better proposal
				double Score;

				// details for the decision
				std::string Reason;

				Vote() : Decision(Reject), Score(0.0) {}
			};
			typedef std::vector<Vote> VoteVector;

			static boost::shared_ptr<ConstraintProposal> createWithIdentity(const slam::KeyframePtr& reference, const slam::KeyframePtr& current);

			static boost::shared_ptr<ConstraintProposal> createWithRelative(const slam::KeyframePtr& reference, const slam::KeyframePtr& current);

			slam::KeyframePtr Reference, Current;
			Eigen::Affine3d InitialTransformation;
			vo::DenseTracker::Result TrackingResult;
			VoteVector Votes;

			ConstraintProposal();

			double TotalScore() const;

			bool Accept() const;

			bool Reject() const;

			void clearVotes();

			boost::shared_ptr<ConstraintProposal> createInverseProposal() const;

			bool isConstraintBetweenSameFrames(const ConstraintProposal& other);

			void printVotingResults(std::ostream& out, const std::string indent = "") const;
		};

		typedef boost::shared_ptr<ConstraintProposal> ConstraintProposalPtr;
		typedef std::vector<ConstraintProposalPtr> ConstraintProposalVector;
	}
}