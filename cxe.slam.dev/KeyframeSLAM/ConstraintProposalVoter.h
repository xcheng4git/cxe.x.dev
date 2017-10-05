#pragma once
#include "ConstraintProposal.h"

namespace slam
{
	namespace constraints
	{
		struct ConstraintProposalVoter
		{
			virtual ~ConstraintProposalVoter() {};

			/**
			* These methods allow voters to get tracking results for additional proposals, which they might need for their
			* decision.
			*/
			virtual void createAdditionalProposals(ConstraintProposalVector& proposals) {}
			virtual void removeAdditionalProposals(ConstraintProposalVector& proposals) {}

			/**
			* Vote for the proposal. Has to set the decision.
			* If asked for reason provide detailed explanation of decision.
			*/
			virtual ConstraintProposal::Vote vote(const ConstraintProposal& proposal, bool provide_reason) = 0;
		};

		typedef boost::shared_ptr<ConstraintProposalVoter> ConstraintProposalVoterPtr;
		typedef std::vector<ConstraintProposalVoterPtr> ConstraintProposalVoterVector;

		struct CrossValidationVoter : public ConstraintProposalVoter
		{
			double TranslationThreshold;

			CrossValidationVoter(double threshold);
			CrossValidationVoter(const CrossValidationVoter& other);
			virtual ~CrossValidationVoter();

			virtual void createAdditionalProposals(ConstraintProposalVector& proposals);

			virtual void removeAdditionalProposals(ConstraintProposalVector& proposals);

			virtual ConstraintProposal::Vote vote(const ConstraintProposal& proposal, bool provide_reason);
		private:
			typedef std::vector<std::pair<ConstraintProposal*, ConstraintProposal*> > ProposalPairVector;
			ProposalPairVector pairs_;

			ConstraintProposal* findInverse(const ConstraintProposal *proposal);
		};

		struct TrackingResultEvaluationVoter : public ConstraintProposalVoter
		{
			double RatioThreshold;

			TrackingResultEvaluationVoter(double threshold);
			virtual ~TrackingResultEvaluationVoter();

			virtual ConstraintProposal::Vote vote(const ConstraintProposal& proposal, bool provide_reason);
		};

		struct ConstraintRatioVoter : public ConstraintProposalVoter
		{
			double RatioThreshold;

			ConstraintRatioVoter(double threshold);
			virtual ~ConstraintRatioVoter();

			virtual ConstraintProposal::Vote vote(const ConstraintProposal& proposal, bool provide_reason);
		};

		struct NaNResultVoter : public ConstraintProposalVoter
		{
			NaNResultVoter();
			virtual ~NaNResultVoter();

			virtual ConstraintProposal::Vote vote(const ConstraintProposal& proposal, bool provide_reason);
		};

		struct OdometryConstraintVoter : public ConstraintProposalVoter
		{
			OdometryConstraintVoter();
			virtual ~OdometryConstraintVoter();

			virtual ConstraintProposal::Vote vote(const ConstraintProposal& proposal, bool provide_reason);
		};
	}
}
